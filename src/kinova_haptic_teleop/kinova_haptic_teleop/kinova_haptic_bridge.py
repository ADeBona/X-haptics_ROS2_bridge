#!/usr/bin/env python3
"""
REAL-robot bridge for the chi-Haptics interface.

Subscribes to a geometry_msgs/WrenchStamped F/T topic and renders only the
INTERACTION torque via gated-tare: the baseline (gravity / pose / mounting
offset) adapts ONLY when out of contact and freezes during a push, so a
sustained contact is rendered at true magnitude for its whole duration.

Includes automatic USB recovery. The UNO R4's native USB CDC endpoint can
stall under sustained traffic while the device remains enumerated; an MCU
reset does not clear it, only a bus-level re-enumeration does. When telemetry
stops arriving, this node issues USBDEVFS_RESET itself and reconnects.
"""
import fcntl
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import WrenchStamped
import serial

KEEPALIVE_PERIOD_S = 1.0
OUTPUT_PERIOD_S = 0.05          # 20 Hz to the Arduino
SUPERVISE_PERIOD_S = 1.0
STALL_TIMEOUT_S = 3.0           # no telemetry for this long -> recover

USBDEVFS_RESET = (ord('U') << 8) | 20    # _IO('U', 20)


class KinovaHapticBridge(Node):
    def __init__(self):
        super().__init__('kinova_haptic_bridge')

        # --- parameters ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('usb_vendor_id', '2341')    # Arduino SA
        self.declare_parameter('wrench_topic', '/ft_sensor_link/wrench')
        self.declare_parameter('torque_axis', 'y')         # 'x' | 'y' | 'z'
        self.declare_parameter('max_input', 6.0)           # torque at full scale (Nm)
        self.declare_parameter('max_pressure', 60.0)       # kPa at full scale
        self.declare_parameter('baseline_alpha', 0.995)    # closer to 1 = slower baseline
        self.declare_parameter('engage_threshold', 0.10)   # Nm above baseline -> contact
        self.declare_parameter('release_threshold', 0.15)  # Nm -> leave contact
        self.declare_parameter('auto_recover', True)

        p = self.get_parameter
        self.port = p('serial_port').value
        self.baud = p('baud_rate').value
        self.usb_vendor = p('usb_vendor_id').value
        self.topic = p('wrench_topic').value
        self.axis = p('torque_axis').value
        self.max_input = float(p('max_input').value)
        self.max_pressure = float(p('max_pressure').value)
        self.alpha = float(p('baseline_alpha').value)
        self.engage = float(p('engage_threshold').value)
        self.release = float(p('release_threshold').value)
        self.auto_recover = bool(p('auto_recover').value)

        # --- state ---
        self.ser = None
        self.connected = False
        self.recovering = False
        self.pending_pressure = 0.0
        self.last_sent = None
        self.last_telemetry = time.time()
        self.write_lock = threading.Lock()
        self.recovery_lock = threading.Lock()
        self.baseline = None
        self.in_contact = False

        self._open_serial()

        self.actual_pub = self.create_publisher(Float32, '/pressure/actual_kpa', 10)
        self.echo_pub = self.create_publisher(Float32, '/pressure/target_kpa_echo', 10)
        self.create_subscription(WrenchStamped, self.topic, self.on_wrench, 10)
        self.get_logger().info(
            f'REAL mode: {self.topic}, axis={self.axis}, max_input={self.max_input} Nm, '
            f'engage={self.engage}, release={self.release}, auto_recover={self.auto_recover}')

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
            self.connected = False
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass

            self._usb_reset()

            # Wait for the device node to reappear
            deadline = time.time() + 15.0
            while time.time() < deadline and not os.path.exists(self.port):
                time.sleep(0.5)
            if not os.path.exists(self.port):
                self.get_logger().error(f'{self.port} did not reappear')
                self.recovering = False
                return

            time.sleep(1.0)      # let udev settle
            try:
                self._open_serial()
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

    # ---------------- input (compute only) ----------------

    def on_wrench(self, msg: WrenchStamped):
        raw = getattr(msg.wrench.torque, self.axis)

        if self.baseline is None:
            self.baseline = raw

        mag = abs(raw - self.baseline)

        if mag > self.engage:
            self.in_contact = True
        elif mag < self.release:
            self.in_contact = False

        # Adapt baseline only out of contact; freeze during a push.
        if not self.in_contact:
            self.baseline = self.alpha * self.baseline + (1.0 - self.alpha) * raw

        interaction = mag if self.in_contact else 0.0
        self.pending_pressure = min(
            interaction / self.max_input * self.max_pressure, self.max_pressure)

    # ---------------- fixed-rate writer ----------------

    def push_to_arduino(self):
        command = f'{self.pending_pressure:.2f}'
        if command == self.last_sent:
            return
        self.last_sent = command
        self.send(command)

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