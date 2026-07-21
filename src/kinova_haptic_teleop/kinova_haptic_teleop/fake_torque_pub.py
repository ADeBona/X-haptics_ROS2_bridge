#!/usr/bin/env python3
"""
Publishes a simulated Force/Torque reading on /kinova/sim_torque.
Use the UP/DOWN arrow keys to change torque (-5.0 to +5.0 Nm).
Press 'q' or Ctrl+C to quit.
"""
import os
import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class FakeTorquePub(Node):
    def __init__(self):
        super().__init__('fake_torque_pub')
        self.publisher_ = self.create_publisher(Float32, '/kinova/sim_torque', 10)

        self.max_tau = 5.0
        self.min_tau = -5.0
        self.current_tau = 0.0
        self.step = 0.5

        self.fd = sys.stdin.fileno()
        self.settings = termios.tcgetattr(self.fd)
        self.running = True

        self.timer = self.create_timer(0.1, self.publish_torque)
        self.input_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.input_thread.start()

    def publish_torque(self):
        msg = Float32()
        msg.data = float(self.current_tau)
        self.publisher_.publish(msg)
        self.draw_bar()

    def draw_bar(self):
        mid = 20
        if self.current_tau >= 0:
            fill = int((self.current_tau / self.max_tau) * mid)
            left = " " * mid
            right = "#" * fill + " " * (mid - fill)
        else:
            fill = int((abs(self.current_tau) / abs(self.min_tau)) * mid)
            left = " " * (mid - fill) + "#" * fill
            right = " " * mid
        sys.stdout.write(
            f"\r\033[K Torque: [{left}|{right}] {self.current_tau:5.2f} Nm "
        )
        sys.stdout.flush()

    def keyboard_listener(self):
        try:
            tty.setraw(self.fd)
            while self.running and rclpy.ok():
                if not select.select([self.fd], [], [], 0.1)[0]:
                    continue

                # Read raw bytes straight from the fd (no Python buffering)
                data = os.read(self.fd, 8)
                if not data:
                    continue

                # Arrow keys arrive as the 3-byte sequence ESC [ A/B
                if data.startswith(b'\x1b['):
                    code = data[2:3]
                    if code == b'A':
                        self.current_tau = min(self.current_tau + self.step, self.max_tau)
                    elif code == b'B':
                        self.current_tau = max(self.current_tau - self.step, self.min_tau)
                elif data in (b'q', b'\x03'):  # 'q' or Ctrl+C
                    self.running = False
                    break
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.settings)
            self.running = False


def main(args=None):
    rclpy.init(args=args)
    node = FakeTorquePub()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        sys.stdout.write("\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()