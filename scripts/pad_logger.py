#!/usr/bin/env python3
"""
Pad characterisation logger.

Interactive front end for pad_characterisation.ino. Steps pressure manually
to find a safe limit, runs inflation and deflation tests, writes CSV, plots.

Usage:
    python3 pad_logger.py --port /dev/ttyACM0 --out ../pads/pad_v2_55x30_shore10
"""
import argparse
import csv
import os
import select
import subprocess
import sys
import termios
import time
import tty

import serial


def check_port_free(port):
    """Refuse to proceed if another process already holds the port.

    Two opens of the same tty both succeed silently on Linux unless the
    holder set TIOCEXCL, so a stuck bridge process otherwise looks just
    like a normal connection - commands land nowhere and nothing inflates.
    Tries sudo -n first since the bridge usually runs as root and a plain
    lsof can't see another user's open fds; falls back to a plain lsof
    (still catches a same-user holder) if sudo isn't cached.
    """
    for cmd in (['sudo', '-n', 'lsof', port], ['lsof', port]):
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
        except FileNotFoundError:
            continue
        lines = [l for l in out.stdout.splitlines() if l and not l.startswith('COMMAND')]
        if lines:
            print(f'\n{port} is already open by another process:')
            for l in lines:
                print(f'  {l}')
            print('\nMost likely the ROS bridge (kinova_haptic_bridge) is still running.')
            print('Stop it first (GUIDE.md §12), e.g.:')
            print('  sudo kill -INT <PID>')
            sys.exit(1)
        if out.returncode == 0:
            return  # authoritative "nothing has it open" - no need to try sudo


class PadRig:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 0.1
        self.ser.dtr = False
        self.ser.open()
        self.ser.dtr = False
        time.sleep(0.1)
        self.ser.dtr = True
        print('Waiting for board reset and tare (3 s)...')
        time.sleep(3.0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print('Connected.')

        self.trace = []          # (t_ms, target, actual)
        self.max_seen = 0.0
        self.last_result = None
        self.last_actual = 0.0
        self.recording = False

    def send(self, cmd):
        self.ser.write((cmd + '\n').encode())
        self.ser.flush()

    def pump_serial(self):
        """Read all pending lines, update state."""
        while self.ser.in_waiting:
            raw = self.ser.readline().decode(errors='ignore').strip()
            if not raw:
                continue
            if raw.startswith('D,'):
                try:
                    _, t, tgt, act, mode = raw.split(',')
                    t, tgt, act = int(t), float(tgt), float(act)
                except ValueError:
                    continue
                self.last_actual = act
                self.max_seen = max(self.max_seen, act)
                if self.recording:
                    self.trace.append((t, tgt, act))
            elif raw.startswith('RESULT,'):
                self.last_result = raw
                print(f'\n  {raw}')
            elif raw.startswith('EVT,') or raw.startswith('MAX,'):
                print(f'\n  {raw}')

    def run_test(self, cmd, timeout=90.0):
        """Send a test command, record the trace until RESULT arrives."""
        self.trace = []
        self.last_result = None
        self.recording = True
        self.send(cmd)
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pump_serial()
            if self.last_result:
                time.sleep(0.3)
                self.pump_serial()
                break
            time.sleep(0.02)
        self.recording = False
        return self.last_result, list(self.trace)

    def close(self):
        try:
            self.send('Z')
            time.sleep(0.5)
        finally:
            self.ser.close()


def save_csv(path, trace, header_note=''):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not trace:
        print('No samples recorded.')
        return
    t0 = trace[0][0]
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        if header_note:
            w.writerow([f'# {header_note}'])
        w.writerow(['time_s', 'target_kPa', 'actual_kPa'])
        for t, tgt, act in trace:
            w.writerow([f'{(t - t0) / 1000.0:.3f}', f'{tgt:.2f}', f'{act:.2f}'])
    print(f'Saved {len(trace)} samples -> {path}')


def plot(path_csv, path_png, title):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not installed - CSV saved, skipping plot.')
        print('  pip3 install matplotlib')
        return

    ts, tgts, acts = [], [], []
    with open(path_csv) as f:
        for row in csv.reader(f):
            if not row or row[0].startswith('#') or row[0] == 'time_s':
                continue
            ts.append(float(row[0])); tgts.append(float(row[1])); acts.append(float(row[2]))

    plt.figure(figsize=(9, 5.5))
    plt.plot(ts, tgts, 'r--', lw=2, label='Target (kPa)')
    plt.plot(ts, acts, 'b-', lw=2, label='Actual pressure (kPa)')
    plt.xlabel('Time (s)'); plt.ylabel('Pressure (kPa)')
    plt.title(title); plt.grid(True, ls=':', alpha=0.6); plt.legend()
    plt.tight_layout(); plt.savefig(path_png, dpi=150)
    print(f'Plot -> {path_png}')


def manual_stepping(rig):
    """Arrow-key stepping to find the safe limit."""
    print('\n--- MANUAL STEPPING ---')
    print('UP/DOWN: +/- 2 kPa   z: vent   q: done')
    print('Keep the pad shielded until you know where it fails.\n')

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            rig.pump_serial()
            sys.stdout.write(f'\r\033[K  actual {rig.last_actual:6.2f} kPa   '
                             f'max {rig.max_seen:6.2f} kPa  ')
            sys.stdout.flush()
            if select.select([fd], [], [], 0.05)[0]:
                data = os.read(fd, 8)
                if data.startswith(b'\x1b'):
                    # over SSH the escape sequence can arrive split across
                    # reads; give it a moment to complete
                    while len(data) < 3 and select.select([fd], [], [], 0.05)[0]:
                        data += os.read(fd, 8)
                    # CSI (ESC [ A/B) or SS3 (ESC O A/B, application cursor
                    # key mode) - terminals vary in which they send
                    if data[1:2] in (b'[', b'O'):
                        if data[2:3] == b'A':
                            rig.send('U')
                        elif data[2:3] == b'B':
                            rig.send('D')
                elif data in (b'z', b'Z'):
                    rig.send('Z')
                elif data in (b'q', b'\x03'):
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()
    rig.send('Z')
    time.sleep(0.5)
    rig.pump_serial()
    return rig.max_seen


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--out', default='../pads/pad_current',
                    help='output directory for CSV and plots')
    args = ap.parse_args()
    outdir = os.path.abspath(args.out)

    check_port_free(args.port)
    rig = PadRig(args.port)
    try:
        while True:
            print('\n=== PAD CHARACTERISATION ===')
            print('  1  manual stepping (find safe limit)')
            print('  2  inflation test')
            print('  3  single deflation test')
            print('  q  quit')
            choice = input('> ').strip().lower()

            if choice == '1':
                peak = manual_stepping(rig)
                print(f'\nMax pressure reached: {peak:.2f} kPa')
                print('Record this as your safe limit if the pad was undamaged.')

            elif choice == '2':
                tgt = float(input('  target kPa: ').strip())
                print('  running...')
                result, trace = rig.run_test(f'I,{tgt:.2f}')
                csv_path = os.path.join(outdir, 'inflation.csv')
                save_csv(csv_path, trace, f'inflation to {tgt} kPa; {result}')
                plot(csv_path, os.path.join(outdir, 'inflation.png'),
                     f'Inflation transient to {tgt:.0f} kPa')
                if result:
                    ms = int(result.split(',')[2])
                    print(f'\n  RISE TIME: {ms} ms ({ms/1000:.2f} s)')
                    print('  Use this as T_ref for the deflation optimiser.')

            elif choice == '3':
                tgt = float(input('  start kPa: ').strip())
                tmax = int(input('  T_max ms: ').strip())
                tmin = int(input('  T_min ms: ').strip())
                print('  running...')
                result, trace = rig.run_test(f'F,{tgt:.2f},{tmax},{tmin}', timeout=120)
                csv_path = os.path.join(outdir, f'deflation_{tmax}_{tmin}.csv')
                save_csv(csv_path, trace, f'deflation {tmax}/{tmin}; {result}')
                plot(csv_path, os.path.join(outdir, f'deflation_{tmax}_{tmin}.png'),
                     f'Deflation, T_max={tmax} ms, T_min={tmin} ms')
                if result:
                    ms = int(result.split(',')[2])
                    print(f'\n  FALL TIME: {ms} ms ({ms/1000:.2f} s)')

            elif choice == 'q':
                break
    finally:
        rig.close()
        print('Vented and disconnected.')


if __name__ == '__main__':
    main()