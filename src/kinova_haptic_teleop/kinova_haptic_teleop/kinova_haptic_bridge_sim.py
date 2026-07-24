#!/usr/bin/env python3
"""
SIM bridge for the chi-Haptics interface (bench testing, no robot).

Subscribes to std_msgs/Float32 on /kinova/sim_torque (from fake_torque_pub)
and maps |torque| directly to a pressure target. No gated-tare: the fake
signal has no gravity baseline to reject.

Shares all serial handling with the real bridge: fixed-rate writes, DTR
reset pulse on connect, telemetry drain, keepalive, vent on shutdown.
"""
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

KEEPALIVE_PERIOD_S = 1.0
OUTPUT_PERIOD_S = 0.05          # 20 Hz to the Arduino


class KinovaHapticBridgeSim(Node):
    def __init__(self):
        super().__init__('kinova_haptic_bridge_sim')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('sim_topic', '/kinova/sim_torque')
        self.declare_parameter('sim_max_torque', 5.0)
        self.declare_parameter('max_pressure', 50.0)

        p = self.get_parameter
        port = p('serial_port').value
        baud = p('baud_rate').value
        self.topic = p('sim_topic').value
        self.sim_max_torque = float(p('sim_max_torque').value)
        self.max_pressure = float(p('max_pressure').value)

        self.pending_pressure = 0.0
        self.last_sent = None
        self.write_lock = threading.Lock()

        self._open_serial(port, baud)

        self.actual_pub = self.create_publisher(Float32, '/pressure/actual_kpa', 10)
        self.echo_pub = self.create_publisher(Float32, '/pressure/target_kpa_echo', 10)
        self.create_subscription(Float32, self.topic, self.on_sim, 10)
        self.get_logger().info(
            f'SIM mode: {self.topic}, sim_max_torque={self.sim_max_torque} Nm')

        self.create_timer(OUTPUT_PERIOD_S, self.push_to_arduino)
        self.create_timer(KEEPALIVE_PERIOD_S, self.keepalive)

        self._stop = False
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    def _open_serial(self, port, baud):
        try:
            self.ser = serial.Serial()
            self.ser.port = port
            self.ser.baudrate = baud
            self.ser.timeout = 1.0
            self.ser.dtr = False
            self.ser.open()
            self.ser.dtr = False
            time.sleep(0.1)
            self.ser.dtr = True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        self.get_logger().info('Waiting for Arduino reset and calibration...')
        time.sleep(3.0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.get_logger().info(f'Connected on {port} @ {baud}')

    def send(self, command, log=True):
        with self.write_lock:
            try:
                self.ser.write((command + '\n').encode('utf-8'))
                self.ser.flush()
                if log:
                    self.get_logger().info(f'TARGET -> {command} kPa')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write failed: {e}')

    def on_sim(self, msg: Float32):
        tau = msg.data
        self.pending_pressure = min(
            abs(tau) / self.sim_max_torque * self.max_pressure, self.max_pressure)

    def push_to_arduino(self):
        command = f'{self.pending_pressure:.2f}'
        if command == self.last_sent:
            return
        self.last_sent = command
        self.send(command)

    def keepalive(self):
        self.send(self.last_sent if self.last_sent is not None else '0.00',
                  log=False)

    def read_loop(self):
        while not self._stop and rclpy.ok():
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read failed: {e}')
                break
            if not line:
                continue
            if line.startswith('WATCHDOG'):
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
        try:
            parts = line.split(',')
            return (float(parts[0].split(':')[1]),
                    float(parts[1].split(':')[1]))
        except (IndexError, ValueError):
            return None

    def destroy_node(self):
        self._stop = True
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                self.ser.write(b'0.00\n')
                self.ser.flush()
            except serial.SerialException:
                pass
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KinovaHapticBridgeSim()
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