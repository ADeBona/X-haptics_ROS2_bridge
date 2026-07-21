#!/usr/bin/env python3
"""
Subscribes to /kinova/sim_torque, maps |torque| (0-5 Nm) to a pressure
target (0-50 kPa), and streams it to the Arduino over serial.

Continuously drains the Arduino's telemetry stream (required: the UNO R4's
USB CDC blocks if the host stops reading) and republishes it as
/pressure/actual_kpa and /pressure/target_kpa_echo.
"""
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial


class KinovaHapticBridge(Node):
    def __init__(self):
        super().__init__('kinova_haptic_bridge')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.max_torque = 5.0
        self.max_pressure = 50.0
        self.last_sent = None

        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        self.get_logger().info('Waiting for Arduino reset and calibration...')
        time.sleep(3.0)
        self.ser.reset_input_buffer()
        self.get_logger().info(f'Connected to Arduino on {port} @ {baud} baud')

        self.actual_pub = self.create_publisher(Float32, '/pressure/actual_kpa', 10)
        self.echo_pub = self.create_publisher(Float32, '/pressure/target_kpa_echo', 10)
        self.create_subscription(Float32, '/kinova/sim_torque',
                                 self.on_torque_received, 10)

        self._stop = False
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    def on_torque_received(self, msg: Float32):
        pressure = min(abs(msg.data) / self.max_torque * self.max_pressure,
                       self.max_pressure)
        command = f'{pressure:.2f}'

        # Only write on change: avoids flooding the link at 10 Hz and makes a
        # stray second publisher immediately visible in the log.
        if command == self.last_sent:
            return
        self.last_sent = command

        try:
            self.ser.write((command + '\n').encode('utf-8'))
            self.ser.flush()
            self.get_logger().info(f'TARGET -> {command} kPa')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def read_loop(self):
        """Drain telemetry. Must run: the R4 blocks if the host stops reading."""
        while not self._stop and rclpy.ok():
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read failed: {e}')
                break
            if not line:
                continue

            parsed = self.parse_telemetry(line)
            if parsed is None:
                continue
            target, actual = parsed

            m = Float32()
            m.data = actual
            self.actual_pub.publish(m)

            m2 = Float32()
            m2.data = target
            self.echo_pub.publish(m2)

    @staticmethod
    def parse_telemetry(line):
        """Parse 'Target:12.34, Actual:11.98' -> (12.34, 11.98)."""
        try:
            parts = line.split(',')
            return (float(parts[0].split(':')[1]),
                    float(parts[1].split(':')[1]))
        except (IndexError, ValueError):
            return None

    def destroy_node(self):
        self._stop = True
        if hasattr(self, 'ser') and self.ser.is_open:
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