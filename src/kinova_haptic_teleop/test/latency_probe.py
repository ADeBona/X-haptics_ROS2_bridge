#!/usr/bin/env python3
"""
Standalone command-path latency probe for the chi-Haptics pneumatic interface.

READ THIS FIRST
---------------
This tool does NOT import, modify, patch or wrap kinova_haptic_bridge.py, and
it does NOT require the ROS package to be rebuilt (it lives in test/, which
setup.py excludes from the install). It is a self-contained script: run it
directly with python3.

The only thing it shares with the live bridge is screw_torque.py, imported
read-only, so the torque->pressure mapping under test can never drift from the
one in production. Its 20 Hz output timer, its 2-decimal dedup and its serial
framing are deliberate re-implementations of the bridge's behaviour, kept
identical on purpose - see BRIDGE_MIRROR below. If you ever change the bridge's
timing, change it here too or the numbers stop describing the real system.

WHAT IT MEASURES
----------------
The chain from a torque reading landing on the Pi to the pump pin going high:

  [1] sensor_to_host    FT stamp        -> wrench callback entry
  [2] compute           screw-torque + pad mapping arithmetic
  [3] queue_wait        wrench received -> serial write begins   <-- dominant
  [4] serial_write      ser.write() + flush() duration
  [5] host_total        wrench received -> command bytes flushed  ([3]+[4])
  [6] link_ack_rtt      bytes flushed   -> Arduino acknowledged the command
  [7] link_oneway_est   estimated one-way host->MCU (half of [6])
  [8] cmd_to_inflate    bytes flushed   -> Arduino entered INFLATE
  [9] end_to_end        wrench received -> Arduino entered INFLATE ([5]+[8])
 [10] pneumatic_dead    INFLATE entered -> first measurable pressure rise

Stages 1-5 are exact: both endpoints are timestamps taken on this process's own
monotonic clock (stage 1 excepted - see the clock caveat in the report).

Stages 6-9 depend on the firmware:
  * stock pressure_tracker.ino  - the Arduino only speaks every LOG_INTERVAL_MS
    (50 ms), so these are quantised to 50 ms and biased high by ~25 ms on
    average. Usable as an upper bound, not as a precise figure.
  * pressure_tracker_latency.ino (optional, arduino/pressure_tracker_latency/)
    - emits an immediate EVT line the moment it parses a command and the moment
    it enters INFLATE, so these become millisecond-accurate. Auto-detected; you
    do not pass a flag for it.

Stage 10 is the pump + tubing + pad physics. It is reported separately and
deliberately kept out of the command-latency budget - it is not a control-path
delay, it is the actuator doing its job.

MODES
-----
  --mode live      (default) Subscribes to the real wrench topic and drives the
                   real hardware exactly as the bridge does. Full chain, real
                   conditions. THE BRIDGE MUST BE STOPPED - two processes cannot
                   own the serial port. Requires actual torque variation during
                   the run: if nothing moves, nothing is ever sent and there is
                   nothing to measure.

  --mode stimulus  No ROS, no robot, no wrench topic - plain python3 + pyserial.
                   Drives a square wave between two pressures and measures the
                   link, the MCU and the pneumatic dead time in isolation.
                   Repeatable, and the cleanest answer to "how long from command
                   to the start of the action". THE BRIDGE MUST BE STOPPED.

  --mode passive   Opens no serial port and publishes nothing. Observes a
                   RUNNING bridge over its own ROS topics. Safe to run against
                   live hardware mid-experiment, but it can only see stage 1 and
                   a coarse wrench->echo round trip, because everything else
                   happens inside the bridge process where it cannot reach.

STOPPING
--------
Ctrl+C. The probe prints a per-stage table (n / mean / p50 / p95 / max / min),
a "where the time goes" budget, and writes two files next to the CSV log:
a per-event CSV and a text copy of the report.
"""
import argparse
import csv
import math
import os
import signal
import statistics
import sys
import threading
import time
from collections import deque

# --- BRIDGE_MIRROR: these must match kinova_haptic_bridge.py exactly ---------
OUTPUT_PERIOD_S = 0.05          # bridge's 20 Hz push_to_arduino timer
KEEPALIVE_PERIOD_S = 1.0        # bridge's keepalive resend
COMMAND_FORMAT = '{:.2f}'       # bridge's f'{s.pA:.2f}' dedup granularity
SERIAL_TIMEOUT_S = 1.0
ARDUINO_BOOT_WAIT_S = 3.0       # DTR pulse -> reset -> tare settle
# ----------------------------------------------------------------------------

TELEMETRY_INTERVAL_MS = 50.0    # firmware LOG_INTERVAL_MS, for quantisation notes
PRESSURE_RISE_KPA = 0.5         # "the pad started moving" threshold for stage 10
PENDING_EXPIRY_S = 2.0          # give up matching a command after this long
MIN_N_FOR_CLAIMS = 20          # below this, report numbers but do not interpret

DEFAULT_PARAMS = {
    'axis_vertical': 2, 'axis_lateral': 0, 'lever_L': 0.20,
    'lever_sign': 1, 'tighten_sign': 1,
    'tau_deadband': 0.05, 'tau_max': 2.0,
    'pressure_bias': 0.0, 'pressure_span': 40.0,
}

STAGES = [
    ('sensor_to_host',  '[1] FT stamp -> wrench callback',        'exact*'),
    ('compute',         '[2] torque math + pad mapping',          'exact'),
    ('queue_wait',      '[3] wrench recv -> serial write begins', 'exact'),
    ('serial_write',    '[4] ser.write() + flush()',              'exact'),
    ('host_total',      '[5] wrench recv -> bytes flushed',       'exact'),
    ('link_ack_rtt',    '[6] bytes flushed -> command acked',     'fw'),
    ('link_oneway_est', '[7] one-way host->MCU (est, rtt/2)',     'fw'),
    ('cmd_to_inflate',  '[8] bytes flushed -> INFLATE entered',   'fw'),
    ('end_to_end',      '[9] wrench recv -> INFLATE entered',     'fw'),
    ('pneumatic_dead',  '[10] INFLATE -> pressure rises',         'phys'),
]

COMMAND_STAGES = {'sensor_to_host', 'compute', 'queue_wait', 'serial_write',
                  'host_total', 'link_oneway_est', 'cmd_to_inflate'}


# ============================================================================
# statistics
# ============================================================================

class Stage:
    """One measured step. Stores every sample so percentiles are exact."""

    def __init__(self, key, label, precision):
        self.key = key
        self.label = label
        self.precision = precision
        self.samples = []

    def add(self, seconds):
        self.samples.append(seconds * 1000.0)   # store milliseconds

    @property
    def n(self):
        return len(self.samples)

    def summary(self):
        if not self.samples:
            return None
        s = sorted(self.samples)
        return {
            'n': len(s),
            'mean': statistics.fmean(s),
            'p50': _percentile(s, 0.50),
            'p95': _percentile(s, 0.95),
            'max': s[-1],
            'min': s[0],
            'sd': statistics.pstdev(s) if len(s) > 1 else 0.0,
        }


def _percentile(sorted_ms, q):
    if len(sorted_ms) == 1:
        return sorted_ms[0]
    idx = q * (len(sorted_ms) - 1)
    lo = math.floor(idx)
    hi = math.ceil(idx)
    if lo == hi:
        return sorted_ms[int(idx)]
    return sorted_ms[lo] + (sorted_ms[hi] - sorted_ms[lo]) * (idx - lo)


# ============================================================================
# recorder: owns the stages, the pending-command matcher and the event CSV
# ============================================================================

class Recorder:
    def __init__(self, csv_path):
        self.stages = {k: Stage(k, lbl, p) for k, lbl, p in STAGES}
        self.lock = threading.Lock()

        self.t_start = time.monotonic()
        self.wrench_count = 0
        self.send_count = 0
        self.suppressed_count = 0     # dedup: pA formatted to the same string
        self.dropped_count = 0        # wrench samples never sent (decimation)
        self.expired_count = 0
        self.fw_instrumented = False
        self.stamp_valid = None       # None = no wrench seen yet
        self.clock_skew_warned = False

        self._pending = deque()
        self._events = []
        self._seq = 0

        self.csv_path = csv_path
        self._csv_file = None
        self._csv = None
        if csv_path:
            d = os.path.dirname(csv_path)
            if d:
                os.makedirs(d, exist_ok=True)
            self._csv_file = open(csv_path, 'w', newline='')
            self._csv = csv.writer(self._csv_file)
            self._csv.writerow([
                'seq', 't_wall', 'target_kpa',
                'sensor_to_host_ms', 'compute_ms', 'queue_wait_ms',
                'serial_write_ms', 'host_total_ms', 'link_ack_rtt_ms',
                'link_oneway_est_ms', 'cmd_to_inflate_ms', 'end_to_end_ms',
                'pneumatic_dead_ms', 'acked', 'inflated', 'source'])

    # -- stage helpers -------------------------------------------------------

    def add(self, key, seconds):
        with self.lock:
            self.stages[key].add(seconds)

    # -- command lifecycle ---------------------------------------------------

    def command_sent(self, target_str, t_write_start, t_write_done,
                     t_wrench_recv, sensor_to_host, compute_s, actual_at_cmd):
        """Register a command on the wire and open a matching window for it."""
        with self.lock:
            self._seq += 1
            self.send_count += 1
            rec = {
                'seq': self._seq,
                't_wall': time.time(),
                'key': target_str,
                'target': float(target_str),
                't_write_start': t_write_start,
                't_write_done': t_write_done,
                't_wrench_recv': t_wrench_recv,
                'sensor_to_host': sensor_to_host,
                'compute': compute_s,
                'serial_write': t_write_done - t_write_start,
                'queue_wait': (t_write_start - t_wrench_recv
                               if t_wrench_recv is not None else None),
                'actual_at_cmd': actual_at_cmd,
                'acked': False, 'inflated': False,
                'link_ack_rtt': None, 'cmd_to_inflate': None,
                'pneumatic_dead': None, 't_inflate': None,
                'baseline_kpa': None,
            }
            self._pending.append(rec)

            self.stages['serial_write'].add(rec['serial_write'])
            if compute_s is not None:
                self.stages['compute'].add(compute_s)
            if sensor_to_host is not None:
                self.stages['sensor_to_host'].add(sensor_to_host)
            if rec['queue_wait'] is not None:
                self.stages['queue_wait'].add(rec['queue_wait'])
                self.stages['host_total'].add(t_write_done - t_wrench_recv)
            self._expire_locked()
            return rec

    def command_acked(self, target_value, t_recv, source):
        """Arduino confirmed it parsed a command with this target value."""
        key = COMMAND_FORMAT.format(target_value)
        with self.lock:
            for rec in self._pending:
                if rec['acked'] or rec['key'] != key:
                    continue
                rec['acked'] = True
                rec['ack_source'] = source
                rtt = t_recv - rec['t_write_done']
                rec['link_ack_rtt'] = rtt
                self.stages['link_ack_rtt'].add(rtt)
                self.stages['link_oneway_est'].add(rtt / 2.0)
                return rec
            return None

    def inflate_started(self, t_recv, actual_kpa, source):
        """Arduino entered INFLATE. Attribute it to the oldest live command."""
        with self.lock:
            for rec in self._pending:
                if rec['inflated']:
                    continue
                rec['inflated'] = True
                rec['inflate_source'] = source
                d = t_recv - rec['t_write_done']
                rec['cmd_to_inflate'] = d
                rec['t_inflate'] = t_recv
                rec['baseline_kpa'] = actual_kpa
                self.stages['cmd_to_inflate'].add(d)
                if rec['t_wrench_recv'] is not None:
                    self.stages['end_to_end'].add(t_recv - rec['t_wrench_recv'])
                return rec
            return None

    def pressure_observed(self, t_recv, actual_kpa):
        """Close out stage 10 once the pad actually starts moving."""
        with self.lock:
            for rec in self._pending:
                if (rec['inflated'] and rec['pneumatic_dead'] is None
                        and rec['baseline_kpa'] is not None):
                    if actual_kpa >= rec['baseline_kpa'] + PRESSURE_RISE_KPA:
                        d = t_recv - rec['t_inflate']
                        rec['pneumatic_dead'] = d
                        self.stages['pneumatic_dead'].add(d)
                    return

    def _expire_locked(self):
        now = time.monotonic()
        while self._pending:
            rec = self._pending[0]
            done = rec['acked'] and (rec['inflated'] or rec['target'] <= 0.1)
            if not done and now - rec['t_write_done'] < PENDING_EXPIRY_S:
                break
            self._pending.popleft()
            if not rec['acked']:
                self.expired_count += 1
            self._flush_row(rec)

    def drain(self):
        with self.lock:
            while self._pending:
                rec = self._pending.popleft()
                if not rec['acked']:
                    self.expired_count += 1
                self._flush_row(rec)
            if self._csv_file:
                self._csv_file.flush()

    def _flush_row(self, rec):
        if not self._csv:
            return

        def ms(v):
            return '' if v is None else round(v * 1000.0, 3)

        e2e = None
        if rec['t_inflate'] is not None and rec['t_wrench_recv'] is not None:
            e2e = rec['t_inflate'] - rec['t_wrench_recv']
        host_total = None
        if rec['t_wrench_recv'] is not None:
            host_total = rec['t_write_done'] - rec['t_wrench_recv']
        oneway = (rec['link_ack_rtt'] / 2.0
                  if rec['link_ack_rtt'] is not None else None)

        self._csv.writerow([
            rec['seq'], round(rec['t_wall'], 6), rec['key'],
            ms(rec['sensor_to_host']), ms(rec['compute']), ms(rec['queue_wait']),
            ms(rec['serial_write']), ms(host_total), ms(rec['link_ack_rtt']),
            ms(oneway), ms(rec['cmd_to_inflate']), ms(e2e),
            ms(rec['pneumatic_dead']),
            int(rec['acked']), int(rec['inflated']),
            rec.get('ack_source', ''),
        ])
        self._csv_file.flush()

    def close(self):
        if self._csv_file:
            try:
                self._csv_file.close()
            except Exception:
                pass


# ============================================================================
# report
# ============================================================================

def build_report(rec, mode, args):
    L = []
    w = L.append
    dur = time.monotonic() - rec.t_start
    fw = ('instrumented (EVT lines, ms-accurate)' if rec.fw_instrumented
          else 'stock (telemetry-quantised: MCU stages +-%d ms, biased ~+%d ms)'
               % (TELEMETRY_INTERVAL_MS, TELEMETRY_INTERVAL_MS / 2))

    w('')
    w('=' * 78)
    w(' COMMAND-PATH LATENCY REPORT')
    w('=' * 78)
    w(' mode            : %s' % mode)
    w(' firmware        : %s' % fw)
    w(' duration        : %.1f s' % dur)
    if mode != 'stimulus':
        rate = rec.wrench_count / dur if dur > 0 else 0.0
        w(' wrench msgs     : %d  (%.1f Hz)' % (rec.wrench_count, rate))
    srate = rec.send_count / dur if dur > 0 else 0.0
    w(' commands sent   : %d  (%.1f Hz)' % (rec.send_count, srate))
    if mode == 'live':
        w(' dedup-suppressed: %d  (pA unchanged at 2 dp -> nothing on the wire)'
          % rec.suppressed_count)
        w(' samples dropped : %d  (arrived between sends, never transmitted)'
          % rec.dropped_count)
    w(' unmatched cmds  : %d  (expired before the Arduino answered)'
      % rec.expired_count)
    w('')

    hdr = ' %-38s %6s %8s %8s %8s %8s %8s' % (
        'STAGE', 'n', 'mean', 'p50', 'p95', 'max', 'min')
    w(hdr)
    w(' ' + '-' * 76)

    any_data = False
    for key, label, precision in STAGES:
        st = rec.stages[key]
        s = st.summary()
        if s is None:
            w(' %-38s %6s %8s %8s %8s %8s %8s' % (label, '-', '', '', '', '', ''))
            continue
        any_data = True
        w(' %-38s %6d %8.2f %8.2f %8.2f %8.2f %8.2f' % (
            label, s['n'], s['mean'], s['p50'], s['p95'], s['max'], s['min']))
    w(' ' + '-' * 76)
    w(' all values in milliseconds')
    w('')

    if not any_data:
        w(' NO DATA COLLECTED.')
        if mode == 'live':
            w('   Nothing was ever sent, which means pA never changed at 2 dp.')
            w('   The probe only sees a command when the torque actually moves:')
            w('   turn the nut driver during the run, or use --mode stimulus')
            w('   to drive a square wave without the robot.')
        else:
            w('   No commands completed. Check the serial port and that the')
            w('   bridge is stopped (two processes cannot hold the port).')
        w('=' * 78)
        return '\n'.join(L)

    # --- where the time goes -------------------------------------------------
    budget = []
    for key in ('sensor_to_host', 'compute', 'queue_wait', 'serial_write',
                'link_oneway_est', 'cmd_to_inflate'):
        s = rec.stages[key].summary()
        if s:
            budget.append((key, s['mean'], s['max']))

    # cmd_to_inflate already contains the one-way link time; do not double-count
    keys = [b[0] for b in budget]
    if 'cmd_to_inflate' in keys and 'link_oneway_est' in keys:
        budget = [b for b in budget if b[0] != 'link_oneway_est']

    total_mean = sum(b[1] for b in budget)
    if total_mean > 0:
        w(' WHERE THE TIME GOES (mean, torque on the Pi -> pump pin high)')
        w(' ' + '-' * 76)
        for key, mean_ms, max_ms in budget:
            share = 100.0 * mean_ms / total_mean
            bar = '#' * int(round(share / 2.5))
            w(' %-24s %7.2f ms  %5.1f%%  %s' % (key, mean_ms, share, bar))
        w(' %-24s %7.2f ms' % ('TOTAL (mean)', total_mean))
        w('')

    # --- interpretation ------------------------------------------------------
    w(' NOTES')
    w(' ' + '-' * 76)
    thin = [lbl for k, lbl, _ in STAGES
            if 0 < rec.stages[k].n < MIN_N_FOR_CLAIMS]
    if thin:
        w(' * SMALL SAMPLE: fewer than %d measurements for %s.'
          % (MIN_N_FOR_CLAIMS, ', '.join(l.split('] ')[1] for l in thin[:3])
             + (' and others' if len(thin) > 3 else '')))
        w('   Means and maxima from a handful of events are not a')
        w('   characterisation. Run for longer before quoting them.')
    if mode != 'stimulus' and dur > 0 and rec.wrench_count:
        in_hz = rec.wrench_count / dur
        out_hz = 1.0 / OUTPUT_PERIOD_S
        if in_hz > out_hz * 1.5:
            w(' * the wrench topic runs at %.0f Hz but the output timer is'
              % in_hz)
            w('   %.0f Hz, so %.1f%% of readings are overwritten before they'
              % (out_hz, 100.0 * (1.0 - out_hz / in_hz)))
            w('   ever reach the Arduino. Not a latency term by itself, but it')
            w('   bounds how fresh the data on the wire can possibly be.')
    qw = rec.stages['queue_wait'].summary()
    if qw:
        w(' * queue_wait is the %.0f Hz output timer in the bridge. It is'
          % (1.0 / OUTPUT_PERIOD_S))
        w('   unsynchronised with the wrench callback, so a fresh reading waits')
        w('   a uniform 0-%.0f ms before it reaches the wire, mean %.0f ms.'
          % (OUTPUT_PERIOD_S * 1000, OUTPUT_PERIOD_S * 1000 / 2))
        if qw['n'] >= MIN_N_FOR_CLAIMS:
            w('   Measured %.2f ms over %d commands. This is a design choice,'
              % (qw['mean'], qw['n']))
            w('   not a fault, and it is the one term here that can be removed')
            w('   outright.')
        else:
            w('   Only %d command(s) were sent, too few to compare against'
              % qw['n'])
            w('   that expectation - the arm has to move enough to change pA')
            w('   at 2 dp. Run longer, or work the nut driver through its range.')
    if not rec.fw_instrumented and (rec.stages['link_ack_rtt'].n
                                    or rec.stages['cmd_to_inflate'].n):
        w(' * MCU-side stages came from 50 ms telemetry ticks, so they are')
        w('   upper bounds inflated by ~25 ms of sampling delay on average.')
        w('   Flash arduino/pressure_tracker_latency/ for real numbers.')
    if rec.stages['link_oneway_est'].n:
        w(' * link_oneway_est halves the round trip, which assumes the USB CDC')
        w('   path is symmetric. Good to a millisecond or two; confirm once')
        w('   with a scope if the number ever matters to a conclusion.')
    if rec.stamp_valid is False:
        w(' * sensor_to_host unavailable: the wrench messages carry a zero')
        w('   header.stamp, so the driver never stamped them.')
    elif rec.clock_skew_warned:
        w(' * sensor_to_host went negative at least once: the FT publisher is')
        w('   on another machine and the clocks are not synchronised. Treat')
        w('   that row as meaningless unless you run PTP/chrony.')
    pn = rec.stages['pneumatic_dead'].summary()
    if pn:
        w(' * pneumatic_dead (%.1f ms mean) is pump spin-up plus tube and pad'
          % pn['mean'])
        w('   volume - the physics, not the control path. It is excluded from')
        w('   the budget above on purpose.')
    w('=' * 78)
    return '\n'.join(L)


# ============================================================================
# serial link shared by live + stimulus modes
# ============================================================================

class ArduinoLink:
    """Owns the port and the telemetry reader. Mirrors the bridge's framing."""

    def __init__(self, port, baud, recorder, logfn, boot_wait=ARDUINO_BOOT_WAIT_S):
        self.port = port
        self.baud = baud
        self.boot_wait = boot_wait
        self.rec = recorder
        self.log = logfn
        self.ser = None
        self.write_lock = threading.Lock()
        self._stop = False
        self.last_state = None
        self.last_target = None
        self.last_actual = 0.0
        self.telemetry_lines = 0

    def open(self):
        import serial
        self.ser = serial.Serial()
        self.ser.port = self.port
        self.ser.baudrate = self.baud
        self.ser.timeout = SERIAL_TIMEOUT_S
        try:
            self.ser.dtr = False
        except (OSError, IOError):
            pass
        self.ser.open()
        try:
            self.ser.dtr = False
            time.sleep(0.1)
            self.ser.dtr = True
        except (OSError, IOError):
            self.log('port does not support DTR - skipping the reset pulse')
        if self.boot_wait > 0:
            self.log('Waiting %.1fs for Arduino reset and tare...' % self.boot_wait)
            time.sleep(self.boot_wait)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.log('Connected on %s @ %d' % (self.port, self.baud))
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

    def write_command(self, target_str):
        """Returns (t_write_start, t_write_done) on this process's clock."""
        import serial as _s
        payload = (target_str + '\n').encode('utf-8')
        with self.write_lock:
            t0 = time.monotonic()
            try:
                self.ser.write(payload)
                self.ser.flush()
            except (_s.SerialException, OSError) as e:
                self.log('serial write failed: %s' % e)
                return None, None
            t1 = time.monotonic()
        return t0, t1

    def _read_loop(self):
        while not self._stop:
            try:
                raw = self.ser.readline()
            except Exception:
                time.sleep(0.2)
                continue
            t_recv = time.monotonic()
            line = raw.decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            self._handle(line, t_recv)

    def _handle(self, line, t_recv):
        # --- instrumented firmware: immediate, unquantised events ------------
        if line.startswith('EVT:'):
            self.rec.fw_instrumented = True
            parts = line[4:].split(',')
            kind = parts[0].strip().upper()
            try:
                value = float(parts[1])
            except (IndexError, ValueError):
                return
            if kind == 'ACK':
                self.rec.command_acked(value, t_recv, 'evt')
            elif kind == 'INFLATE':
                self.rec.inflate_started(t_recv, self.last_actual, 'evt')
            return

        if line.startswith('WATCHDOG') or line.startswith('VALVE'):
            self.log('Arduino: %s' % line)
            return

        parsed = parse_telemetry(line)
        if parsed is None:
            return
        target, actual, state = parsed
        self.telemetry_lines += 1
        self.last_actual = actual

        # Stock firmware fallback: infer the ack from the echoed Target and the
        # INFLATE entry from the State field. Both are quantised to 50 ms.
        if not self.rec.fw_instrumented:
            if self.last_target is None or abs(target - self.last_target) > 1e-9:
                self.rec.command_acked(target, t_recv, 'telemetry')
            if state == 'INFLATE' and self.last_state != 'INFLATE':
                self.rec.inflate_started(t_recv, actual, 'telemetry')

        self.rec.pressure_observed(t_recv, actual)
        self.last_target = target
        self.last_state = state

    def close(self, vent=True):
        self._stop = True
        if self.ser is not None and self.ser.is_open:
            try:
                if vent:
                    self.ser.write(b'0.00\n')      # leave the pads deflated
                    self.ser.flush()
                    time.sleep(0.2)
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass


def parse_telemetry(line):
    """'Target:12.34, Actual:11.98, State:HOLD' -> (12.34, 11.98, 'HOLD')."""
    try:
        parts = line.split(',')
        target = float(parts[0].split(':')[1])
        actual = float(parts[1].split(':')[1])
    except (IndexError, ValueError):
        return None
    state = ''
    if len(parts) > 2:
        bits = parts[2].split(':')
        if len(bits) > 1:
            state = bits[1].strip()
    return target, actual, state


# ============================================================================
# mode: stimulus  (no ROS, no robot - pure link/MCU/pneumatic characterisation)
# ============================================================================

def run_stimulus(args, rec):
    def log(m):
        print('[probe] %s' % m, flush=True)

    link = ArduinoLink(args.serial_port, args.baud, rec, log, args.boot_wait)
    link.open()

    log('Stimulus: square wave %.2f <-> %.2f kPa every %.1f s. Ctrl+C to stop.'
        % (args.stim_low, args.stim_high, args.stim_period))

    high = True
    try:
        while not STOP.is_set():
            target = args.stim_high if high else args.stim_low
            high = not high
            key = COMMAND_FORMAT.format(target)
            t0, t1 = link.write_command(key)
            if t0 is not None:
                rec.command_sent(key, t0, t1, None, None, None, link.last_actual)
            STOP.wait(args.stim_period)
    finally:
        link.close()
    return 'stimulus'


# ============================================================================
# mode: live  (instrumented twin of the bridge - drives the real hardware)
# ============================================================================

def run_live(args, rec, ros_args):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import WrenchStamped
    from kinova_haptic_teleop.screw_torque import (
        compute_screw_torque, compute_u, compute_pad_pressures)

    params = load_params(args.params)

    class LatencyProbeNode(Node):
        def __init__(self):
            super().__init__('latency_probe')
            self.rec = rec
            self.latest = None
            self.last_sent = None
            self.link = ArduinoLink(args.serial_port, args.baud, rec,
                                    lambda m: self.get_logger().info(m),
                                    args.boot_wait)
            self.link.open()
            self.create_subscription(WrenchStamped, args.wrench_topic,
                                     self.on_wrench, 10)
            self.create_timer(OUTPUT_PERIOD_S, self.push)
            self.create_timer(KEEPALIVE_PERIOD_S, self.keepalive)
            self.create_timer(5.0, self.progress)
            self.get_logger().info(
                'LIVE probe on %s -> %s. Apply torque now; Ctrl+C to report.'
                % (args.wrench_topic, args.serial_port))

        def on_wrench(self, msg):
            t_recv = time.monotonic()
            t_wall = time.time()
            rec.wrench_count += 1

            stamp = msg.header.stamp
            sensor_to_host = None
            if stamp.sec or stamp.nanosec:
                rec.stamp_valid = True
                delta = t_wall - (stamp.sec + stamp.nanosec * 1e-9)
                if delta < 0:
                    if not rec.clock_skew_warned:
                        rec.clock_skew_warned = True
                        self.get_logger().warn(
                            'header.stamp is ahead of this host clock - the FT '
                            'publisher is on an unsynchronised machine; stage 1 '
                            'is not meaningful.')
                else:
                    sensor_to_host = delta
            elif rec.stamp_valid is None:
                rec.stamp_valid = False

            t_c0 = time.monotonic()
            force = (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z)
            torque = (msg.wrench.torque.x, msg.wrench.torque.y,
                      msg.wrench.torque.z)
            st = compute_screw_torque(
                torque, force, params['axis_vertical'], params['axis_lateral'],
                params['lever_L'], params['lever_sign'])
            # No tare here on purpose: the probe measures timing, and a tare
            # offset shifts pA without changing any latency. Keeping it out
            # removes a piece of state that could differ from the bridge's.
            tau = params['tighten_sign'] * st.tau_screw
            u = compute_u(tau, params['tau_deadband'], params['tau_max'])
            pA, _pB = compute_pad_pressures(
                u, params['pressure_bias'], params['pressure_span'])
            compute_s = time.monotonic() - t_c0

            if self.latest is not None:
                rec.dropped_count += 1     # previous sample never made the wire
            self.latest = (pA, t_recv, sensor_to_host, compute_s)

        def push(self):
            s = self.latest
            if s is None:
                return
            pA, t_recv, sensor_to_host, compute_s = s
            # Cleared so queue_wait always measures a FRESH sample's age. The
            # bridge leaves self.latest in place and lets the dedup suppress the
            # resend; identical on the wire, but it would let a stale t_recv be
            # differenced against a much later tick and inflate stage 3.
            self.latest = None

            key = COMMAND_FORMAT.format(pA)
            if key == self.last_sent:
                rec.suppressed_count += 1
                return
            self.last_sent = key
            t0, t1 = self.link.write_command(key)
            if t0 is not None:
                rec.command_sent(key, t0, t1, t_recv, sensor_to_host,
                                 compute_s, self.link.last_actual)

        def keepalive(self):
            # Mirrors the bridge so the 15 s firmware watchdog never trips.
            # Not measured: it carries no new information.
            if self.last_sent is not None:
                self.link.write_command(self.last_sent)

        def progress(self):
            self.get_logger().info(
                'wrench=%d sent=%d suppressed=%d acked=%d inflate=%d'
                % (rec.wrench_count, rec.send_count, rec.suppressed_count,
                   rec.stages['link_ack_rtt'].n,
                   rec.stages['cmd_to_inflate'].n))

    rclpy.init(args=ros_args)
    node = LatencyProbeNode()
    try:
        while rclpy.ok() and not STOP.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.link.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 'live'


# ============================================================================
# mode: passive  (observes a running bridge, touches nothing)
# ============================================================================

def run_passive(args, rec, ros_args):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import WrenchStamped
    from std_msgs.msg import Float32

    class PassiveNode(Node):
        def __init__(self):
            super().__init__('latency_probe_passive')
            self.pending = deque()
            self.last_echo = None
            self.create_subscription(WrenchStamped, args.wrench_topic,
                                     self.on_wrench, 10)
            self.create_subscription(Float32, args.echo_topic, self.on_echo, 10)
            self.create_timer(5.0, self.progress)
            self.get_logger().info(
                'PASSIVE probe: %s + %s. No serial port opened, bridge '
                'untouched.' % (args.wrench_topic, args.echo_topic))

        def on_wrench(self, msg):
            t_recv = time.monotonic()
            rec.wrench_count += 1
            stamp = msg.header.stamp
            if stamp.sec or stamp.nanosec:
                rec.stamp_valid = True
                d = time.time() - (stamp.sec + stamp.nanosec * 1e-9)
                if d >= 0:
                    rec.add('sensor_to_host', d)
                elif not rec.clock_skew_warned:
                    rec.clock_skew_warned = True
            elif rec.stamp_valid is None:
                rec.stamp_valid = False
            self.pending.append(t_recv)
            while len(self.pending) > 200:
                self.pending.popleft()

        def on_echo(self, msg):
            # The bridge republishes the Arduino's Target readback. A change in
            # it means a new command completed the full round trip. Attribute
            # it to the oldest wrench still in flight: this is end-to-end
            # including the 50 ms telemetry tick, not a per-stage figure.
            t_recv = time.monotonic()
            if self.last_echo is not None and abs(msg.data - self.last_echo) < 1e-9:
                return
            self.last_echo = msg.data
            if self.pending:
                rec.add('end_to_end', t_recv - self.pending[-1])
                rec.send_count += 1

        def progress(self):
            self.get_logger().info(
                'wrench=%d echo-changes=%d' % (rec.wrench_count, rec.send_count))

    rclpy.init(args=ros_args)
    node = PassiveNode()
    try:
        while rclpy.ok() and not STOP.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 'passive'


# ============================================================================
# params + main
# ============================================================================

def load_params(path):
    """Read the bridge's own YAML so the mapping under test matches production."""
    p = dict(DEFAULT_PARAMS)
    if not path or not os.path.exists(path):
        print('[probe] params file not found, using bridge defaults', flush=True)
        return p
    try:
        import yaml
        with open(path) as f:
            doc = yaml.safe_load(f) or {}
        for node_params in doc.values():
            got = (node_params or {}).get('ros__parameters', {})
            for k in DEFAULT_PARAMS:
                if k in got:
                    p[k] = got[k]
        print('[probe] params from %s' % path, flush=True)
    except Exception as e:
        print('[probe] could not read %s (%s), using defaults' % (path, e),
              flush=True)
    return p


STOP = threading.Event()


def main():
    ap = argparse.ArgumentParser(
        description='Command-path latency probe (standalone; does not touch '
                    'the bridge).',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mode', choices=['live', 'stimulus', 'passive'],
                    default='live')
    ap.add_argument('--serial-port',
                    default='/dev/serial/by-id/usb-Arduino_UNO_WiFi_R4_'
                            'CMSIS-DAP_F412FA654890-if01')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--boot-wait', type=float, default=ARDUINO_BOOT_WAIT_S,
                    help='pause after the DTR reset pulse, for the MCU to tare')
    ap.add_argument('--wrench-topic', default='/ft_sensor_link/wrench')
    ap.add_argument('--echo-topic', default='/pressure/target_kpa_echo')
    ap.add_argument('--params', default='/repo/config/bridge_real.yaml',
                    help='bridge YAML, so pA is computed identically')
    ap.add_argument('--out-dir', default='/repo/logs',
                    help='where the event CSV and report are written')
    ap.add_argument('--stim-low', type=float, default=0.0)
    ap.add_argument('--stim-high', type=float, default=30.0)
    ap.add_argument('--stim-period', type=float, default=4.0,
                    help='seconds per half-cycle; must exceed the pad fill time')
    args, ros_args = ap.parse_known_args()

    stamp = time.strftime('%Y%m%d_%H%M%S')
    try:
        os.makedirs(args.out_dir, exist_ok=True)
        out_base = os.path.join(args.out_dir, 'latency_%s_%s' % (args.mode, stamp))
    except OSError:
        out_base = os.path.join('.', 'latency_%s_%s' % (args.mode, stamp))
        print('[probe] %s not writable, writing to CWD' % args.out_dir, flush=True)

    rec = Recorder(out_base + '_events.csv')

    def on_sigint(_sig, _frm):
        STOP.set()

    signal.signal(signal.SIGINT, on_sigint)

    mode = args.mode
    try:
        if args.mode == 'stimulus':
            mode = run_stimulus(args, rec)
        elif args.mode == 'passive':
            mode = run_passive(args, rec, ros_args)
        else:
            mode = run_live(args, rec, ros_args)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print('\n[probe] aborted: %s' % e, flush=True)
        if 'could not open port' in str(e).lower() or 'Permission' in str(e):
            print('[probe] Is the bridge still running? It holds the serial '
                  'port. Stop it, or use --mode passive.', flush=True)

    rec.drain()
    report = build_report(rec, mode, args)
    print(report, flush=True)

    try:
        with open(out_base + '_report.txt', 'w') as f:
            f.write(report + '\n')
        print(' events : %s' % rec.csv_path, flush=True)
        print(' report : %s' % (out_base + '_report.txt'), flush=True)
    except OSError as e:
        print(' (could not write report file: %s)' % e, flush=True)
    rec.close()


if __name__ == '__main__':
    main()
