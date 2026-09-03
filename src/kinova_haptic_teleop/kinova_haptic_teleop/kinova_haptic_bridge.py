#!/usr/bin/env python3
"""
REAL-robot bridge for the chi-Haptics interface.

Subscribes to a geometry_msgs/WrenchStamped F/T topic and renders the screw
torque about the bolt axis (see screw_torque.py for the physics) as pad
pressure, at a fixed 20 Hz over USB serial to an Arduino. Wrench processing
is decimated to that same 20 Hz regardless of the sensor's own publish rate
- see WRENCH_PROCESS_PERIOD_S.

Only one pneumatic channel exists today (one pump, one valve, one sensor,
inflating a fixed pair of pads together), so only group_A - the tightening
direction - is actually sent over serial. group_B is computed and logged for
validation and is ready to be wired to a second channel later; nothing about
the mapping needs to change when that hardware arrives.

Published topics (all of it recordable into a rosbag on this side):
    /pressure/actual_kpa      std_msgs/Float32       measured pad pressure
    /pressure/target_kpa_echo std_msgs/Float32       target as the Arduino sees it
    /haptic/screw_torque      geometry_msgs/Vector3Stamped
                              (tau_raw, tau_screw, tau_tared) - the screw-axis
                              component extracted from the full F/T wrench
    /haptic/pad_target_kpa    geometry_msgs/Vector3Stamped
                              (pA, pB, unused) - what the mapping commands

The two Vector3Stamped topics carry the header stamp of the F/T sample they
were derived from, so in a bag they align with the source wrench rather than
with the moment this node got round to publishing them.

Includes automatic USB recovery. The UNO R4's native USB CDC endpoint can
stall under sustained traffic while the device remains enumerated; an MCU
reset does not clear it, only a bus-level re-enumeration does. When telemetry
stops arriving, this node issues USBDEVFS_RESET itself and reconnects.
"""
import fcntl
import os
import threading
import time
from collections import namedtuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Empty
from geometry_msgs.msg import WrenchStamped, Vector3Stamped
import serial

from kinova_haptic_teleop.screw_torque import (
    compute_screw_torque, compute_u, compute_pad_pressures, TareEstimator,
)
from kinova_haptic_teleop.pad_csv_logger import ScrewTorqueCsvLogger

KEEPALIVE_PERIOD_S = 1.0
OUTPUT_PERIOD_S = 0.05          # 20 Hz to the Arduino
SUPERVISE_PERIOD_S = 1.0
STALL_TIMEOUT_S = 3.0           # no telemetry for this long -> recover

# Some F/T sensors stream at 500 Hz-1 kHz regardless of what the robot is
# doing. Running the full extraction (and publishing /haptic/screw_torque)
# on every single sample buys nothing - push_to_arduino only ever acts at
# OUTPUT_PERIOD_S - but it does cost real CPU/GIL time in rclpy's per-message
# Python dispatch. At sustained 1 kHz that was enough to starve this node's
# own 1 Hz supervise() timer, which is what notices a stalled USB link and
# recovers it; with supervise() starved, a stall that would normally
# self-heal in ~3 s instead sits dead until someone restarts the node. Gate
# on_wrench to the same cadence we actually act on to keep that headroom.
WRENCH_PROCESS_PERIOD_S = OUTPUT_PERIOD_S

USBDEVFS_RESET = (ord('U') << 8) | 20    # _IO('U', 20)

Sample = namedtuple('Sample', [
    'stamp', 'force', 'torque', 'tau_raw', 'tau_screw', 'tau_tared',
    'tau_perp_mag', 'u', 'pA', 'pB'])

# Vector3Stamped has no field names of its own, so the meaning of x/y/z on
# the two derived topics is fixed here and documented in README.md. Do not
# reorder these: a recorded bag is only interpretable through this mapping.
TORQUE_FIELDS = ('tau_raw', 'tau_screw', 'tau_tared')   # /haptic/screw_torque
PAD_FIELDS = ('pA', 'pB', 'unused')                     # /haptic/pad_target_kpa


class KinovaHapticBridge(Node):
    def __init__(self):
        super().__init__('kinova_haptic_bridge')

        # --- parameters ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('usb_vendor_id', '2341')    # Arduino SA
        self.declare_parameter('wrench_topic', '/ft_sensor_link/wrench')
        self.declare_parameter('auto_recover', True)

        # Screw-torque extraction (see screw_torque.py). Which sensor axis is
        # vertical/lateral is not yet confirmed - tune empirically.
        self.declare_parameter('axis_vertical', 2)         # index into torque.{x,y,z}
        self.declare_parameter('axis_lateral', 0)          # index into force.{x,y,z}
        self.declare_parameter('lever_L', 0.20)             # metres
        self.declare_parameter('lever_sign', 1)             # +1 or -1, tunes lever-arm cancellation only
        self.declare_parameter('tighten_sign', 1)           # +1 or -1, flips which twist direction is "tightening" - see AXIS_CALIBRATION.md

        # Pad mapping
        self.declare_parameter('tau_deadband', 0.05)        # Nm
        self.declare_parameter('tau_max', 2.0)              # Nm
        self.declare_parameter('pressure_bias', 0.0)        # kPa (serial protocol units)
        self.declare_parameter('pressure_span', 40.0)       # kPa

        # Tare
        self.declare_parameter('tare_samples', 20)

        # Logging
        self.declare_parameter('csv_log_path', 'kinova_haptic_bridge_log.csv')

        # frame_id stamped on the derived Vector3Stamped topics. Empty by
        # default and deliberately so: x/y/z there are three packed scalars
        # (see TORQUE_FIELDS / PAD_FIELDS), not a geometric vector, so naming
        # a real frame would invite a TF transform that means nothing.
        self.declare_parameter('derived_frame_id', '')

        p = self.get_parameter
        self.port = p('serial_port').value
        self.baud = p('baud_rate').value
        self.usb_vendor = p('usb_vendor_id').value
        self.topic = p('wrench_topic').value
        self.auto_recover = bool(p('auto_recover').value)

        self.axis_vertical = int(p('axis_vertical').value)
        self.axis_lateral = int(p('axis_lateral').value)
        self.lever_L = float(p('lever_L').value)
        self.lever_sign = int(p('lever_sign').value)
        self.tighten_sign = int(p('tighten_sign').value)

        self.tau_deadband = float(p('tau_deadband').value)
        self.tau_max = float(p('tau_max').value)
        self.pressure_bias = float(p('pressure_bias').value)
        self.pressure_span = float(p('pressure_span').value)

        self.tare_samples = int(p('tare_samples').value)
        self.csv_log_path = p('csv_log_path').value
        self.frame_id = p('derived_frame_id').value

        # --- state ---
        self.ser = None
        self.connected = False
        self.recovering = False
        self.last_sent = None
        self.last_telemetry = time.time()
        self.write_lock = threading.Lock()
        self.recovery_lock = threading.Lock()

        self.tare = TareEstimator(self.tare_samples)
        self.latest = None   # Sample, set by on_wrench once data has arrived
        self.csv_logger = ScrewTorqueCsvLogger(self.csv_log_path)
        self._last_wrench_process_time = 0.0   # monotonic; see WRENCH_PROCESS_PERIOD_S

        self._open_serial()

        self.actual_pub = self.create_publisher(Float32, '/pressure/actual_kpa', 10)
        self.echo_pub = self.create_publisher(Float32, '/pressure/target_kpa_echo', 10)

        # Stamped derived signals, for rosbag correlation with the raw wrench.
        # Both carry the header stamp of the F/T sample they were computed
        # from, NOT the time they happened to be published, so the whole
        # chain (wrench -> screw torque -> pad command) lines up on one time
        # axis in the bag. See TORQUE_FIELDS / PAD_FIELDS below.
        self.torque_pub = self.create_publisher(
            Vector3Stamped, '/haptic/screw_torque', 10)
        self.pad_target_pub = self.create_publisher(
            Vector3Stamped, '/haptic/pad_target_kpa', 10)

        self.create_subscription(WrenchStamped, self.topic, self.on_wrench, 10)
        self.create_subscription(Empty, '~/tare', self.on_tare, 10)
        self.create_subscription(Empty, '~/reset_tare', self.on_reset_tare, 10)
        self.get_logger().info(
            f'REAL mode: {self.topic}, axis_vertical={self.axis_vertical}, '
            f'axis_lateral={self.axis_lateral}, lever_L={self.lever_L}, '
            f'lever_sign={self.lever_sign}, tighten_sign={self.tighten_sign}, '
            f'tau_deadband={self.tau_deadband}, '
            f'tau_max={self.tau_max}, auto_recover={self.auto_recover}, '
            f'csv_log_path={self.csv_log_path}')

        self.create_timer(OUTPUT_PERIOD_S, self.push_to_arduino)
        self.create_timer(KEEPALIVE_PERIOD_S, self.keepalive)
        self.create_timer(SUPERVISE_PERIOD_S, self.supervise)

        self._stop = False
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    # ---------------- serial / USB ----------------

    def _open_serial(self):
        self.ser = serial.Serial()
        self.ser.port = self.port
        self.ser.baudrate = self.baud
        self.ser.timeout = 1.0
        self.ser.dtr = False
        self.ser.open()
        # Explicit DTR low->high pulse
        self.ser.dtr = False
        time.sleep(0.1)
        self.ser.dtr = True

        self.get_logger().info('Waiting for Arduino reset and calibration...')
        time.sleep(3.0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.last_telemetry = time.time()
        self.connected = True
        self.get_logger().info(f'Connected on {self.port} @ {self.baud}')

    def _find_usb_device(self):
        """Locate /dev/bus/usb/BBB/DDD for the Arduino, via sysfs only."""
        base = '/sys/bus/usb/devices'
        try:
            entries = os.listdir(base)
        except OSError:
            return None
        for entry in entries:
            vid_path = os.path.join(base, entry, 'idVendor')
            if not os.path.exists(vid_path):
                continue
            try:
                with open(vid_path) as f:
                    if f.read().strip().lower() != self.usb_vendor.lower():
                        continue
                with open(os.path.join(base, entry, 'busnum')) as f:
                    bus = int(f.read().strip())
                with open(os.path.join(base, entry, 'devnum')) as f:
                    dev = int(f.read().strip())
                return f'/dev/bus/usb/{bus:03d}/{dev:03d}'
            except (OSError, ValueError):
                continue
        return None

    def _usb_reset(self):
        path = self._find_usb_device()
        if path is None:
            self.get_logger().error('Arduino not found on USB bus')
            return False
        try:
            fd = os.open(path, os.O_WRONLY)
            try:
                fcntl.ioctl(fd, USBDEVFS_RESET, 0)
            finally:
                os.close(fd)
            self.get_logger().warn(f'USB re-enumeration forced on {path}')
            return True
        except OSError as e:
            self.get_logger().error(f'USB reset failed ({e}) - needs root')
            return False

    def _recover(self):
        with self.recovery_lock:
            self.get_logger().warn('Telemetry stalled - recovering USB link')
            # close() and write() race on the same pyserial object: a write
            # already in flight on the executor thread reads from an
            # internal abort pipe that close() nulls out mid-call, raising
            # an uncaught TypeError that used to kill the whole process.
            # write_lock makes the two mutually exclusive; connected is
            # flipped to False inside it so a write that loses the race
            # blocks until close() has fully finished, then hits a clean
            # PortNotOpenError (a SerialException) instead.
            with self.write_lock:
                self.connected = False
                try:
                    if self.ser and self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass

            self._usb_reset()

            # Wait for the device node to reappear. connected is False for
            # this whole stretch, so concurrent send() calls return
            # immediately without touching write_lock - this loop does not
            # need to hold it.
            deadline = time.time() + 15.0
            while time.time() < deadline and not os.path.exists(self.port):
                time.sleep(0.5)
            if not os.path.exists(self.port):
                self.get_logger().error(f'{self.port} did not reappear')
                self.recovering = False
                return

            time.sleep(1.0)      # let udev settle
            try:
                with self.write_lock:
                    self._open_serial()   # sets self.connected = True
                self.last_sent = None       # force a resend of the target
                self.get_logger().info('USB link recovered')
            except Exception as e:
                self.get_logger().error(f'Reopen failed: {e}')
            finally:
                self.recovering = False

    def supervise(self):
        if not self.auto_recover or self.recovering or not self.connected:
            return
        if time.time() - self.last_telemetry > STALL_TIMEOUT_S:
            self.recovering = True
            threading.Thread(target=self._recover, daemon=True).start()

    def send(self, command, log=True):
        if not self.connected:
            return
        with self.write_lock:
            try:
                self.ser.write((command + '\n').encode('utf-8'))
                self.ser.flush()
                if log:
                    self.get_logger().info(f'TARGET -> {command} kPa')
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial write failed: {e}')
                self.connected = False
            except Exception as e:
                # Belt-and-suspenders: a torn-down pyserial object can raise
                # outside its own exception hierarchy (see the write_lock
                # note in _recover). Never let a write failure take the
                # whole node down.
                self.get_logger().error(f'Serial write failed unexpectedly: {e}')
                self.connected = False

    # ---------------- input (compute only) ----------------

    def on_wrench(self, msg: WrenchStamped):
        # Decimate: see WRENCH_PROCESS_PERIOD_S. A sensor streaming at
        # 500 Hz-1 kHz would otherwise run this (and a publish) on every
        # sample for no benefit, at real risk to the node's own timers.
        now_mono = time.monotonic()
        if now_mono - self._last_wrench_process_time < WRENCH_PROCESS_PERIOD_S:
            return
        self._last_wrench_process_time = now_mono

        force = (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z)
        torque = (msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z)

        st = compute_screw_torque(
            torque, force, self.axis_vertical, self.axis_lateral,
            self.lever_L, self.lever_sign)

        if self.tare.feed(st.tau_screw):
            self.get_logger().info(f'Tare captured: offset={self.tare.offset:.4f} Nm')

        # tighten_sign is applied last, after tare - it only decides which
        # physical twist direction counts as "tightening" (see
        # AXIS_CALIBRATION.md's open question). It must never be folded into
        # lever_sign: that one exists solely to cancel the false twist a
        # sideways push produces, and flipping it for this instead breaks
        # that cancellation.
        tau_tared = self.tighten_sign * self.tare.apply(st.tau_screw)
        u = compute_u(tau_tared, self.tau_deadband, self.tau_max)
        pA, pB = compute_pad_pressures(u, self.pressure_bias, self.pressure_span)

        stamp = self._source_stamp(msg.header.stamp)
        self.latest = Sample(
            stamp=stamp, force=force, torque=torque, tau_raw=st.tau_raw,
            tau_screw=st.tau_screw, tau_tared=tau_tared,
            tau_perp_mag=st.tau_perp_mag, u=u, pA=pA, pB=pB)

        # Published at the F/T sensor's own rate, one message per input
        # sample - the extracted screw component at full fidelity, not
        # decimated to the 20 Hz serial cadence.
        self.torque_pub.publish(self._vec3(stamp, st.tau_raw, st.tau_screw, tau_tared))

    def _source_stamp(self, stamp):
        """Header stamp of the incoming wrench, or now if it is unset.

        A publisher that leaves the stamp at zero (some sim sources do) would
        otherwise put every derived sample at the epoch in the bag.
        """
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self.get_clock().now().to_msg()
        return stamp

    def _vec3(self, stamp, x, y, z):
        m = Vector3Stamped()
        m.header.stamp = stamp
        m.header.frame_id = self.frame_id
        m.vector.x = float(x)
        m.vector.y = float(y)
        m.vector.z = float(z)
        return m

    def on_tare(self, _msg: Empty):
        self.tare.start_tare()
        self.get_logger().info(f'Tare requested: averaging next {self.tare_samples} samples')

    def on_reset_tare(self, _msg: Empty):
        self.tare.reset()
        self.get_logger().info('Tare offset reset to 0')

    # ---------------- fixed-rate writer ----------------

    def push_to_arduino(self):
        s = self.latest
        if s is None:
            return

        # Only group_A is wired to hardware today (one channel = the
        # tightening direction). group_B is logged for validation only.
        command = f'{s.pA:.2f}'
        if command != self.last_sent:
            self.last_sent = command
            self.send(command)

        # Published unconditionally at the 20 Hz output rate, including when
        # the command is unchanged and no serial write happens - a bag needs
        # the commanded value at every tick, not only at the transitions.
        self.pad_target_pub.publish(self._vec3(s.stamp, s.pA, s.pB, 0.0))

        fx, fy, fz = s.force
        tx, ty, tz = s.torque
        self.csv_logger.write_row(
            t=time.time(), force_xyz=(fx, fy, fz), torque_xyz=(tx, ty, tz),
            tau_raw=s.tau_raw, tau_screw=s.tau_screw, tau_perp_mag=s.tau_perp_mag,
            u=s.u, pA=s.pA, pB=s.pB)

    def keepalive(self):
        self.send(self.last_sent if self.last_sent is not None else '0.00',
                  log=False)

    # ---------------- telemetry reader ----------------

    def read_loop(self):
        while not self._stop and rclpy.ok():
            if not self.connected:
                time.sleep(0.2)
                continue
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            except (serial.SerialException, OSError, TypeError):
                self.connected = False
                time.sleep(0.2)
                continue
            if not line:
                continue

            self.last_telemetry = time.time()

            if line.startswith('WATCHDOG') or line.startswith('VALVE'):
                self.get_logger().warn(f'Arduino: {line}')
                continue

            parsed = self.parse_telemetry(line)
            if parsed is None:
                continue
            target, actual = parsed

            m = Float32(); m.data = actual
            self.actual_pub.publish(m)
            m2 = Float32(); m2.data = target
            self.echo_pub.publish(m2)

    @staticmethod
    def parse_telemetry(line):
        """Parse 'Target:12.34, Actual:11.98[, State:HOLD]' -> (12.34, 11.98)."""
        try:
            parts = line.split(',')
            return (float(parts[0].split(':')[1]),
                    float(parts[1].split(':')[1]))
        except (IndexError, ValueError):
            return None

    def destroy_node(self):
        self._stop = True
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write(b'0.00\n')
                self.ser.flush()
            except Exception:
                pass
            self.ser.close()
        self.csv_logger.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KinovaHapticBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
