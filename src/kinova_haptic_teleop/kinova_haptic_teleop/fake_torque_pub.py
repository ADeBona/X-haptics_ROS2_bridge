#!/usr/bin/env python3
"""
Publishes a simulated Force/Torque reading on /kinova/sim_torque.
Use the UP/DOWN arrow keys to increase/decrease torque (-5.0 to +5.0 Nm).
Press 'q' to quit.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import select
import termios
import tty
import threading


class FakeTorquePub(Node):
    def __init__(self):
        super().__init__('fake_torque_pub')
        self.publisher_ = self.create_publisher(Float32, '/kinova/sim_torque', 10)
        self.max_tau = 5.0
        self.min_tau = -5.0
        self.current_tau = 0.0
        self.step = 0.25
        self.timer = self.create_timer(0.1, self.publish_torque)
        self.get_logger().info("Use UP/DOWN arrows to change torque. Press 'q' to quit.")

        self.settings = termios.tcgetattr(sys.stdin)
        self.input_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.input_thread.start()

    def publish_torque(self):
        msg = Float32()
        msg.data = self.current_tau
        self.publisher_.publish(msg)
        self.draw_bar()

    def draw_bar(self):
        bar_len = 40
        mid = bar_len // 2
        if self.current_tau >= 0:
            fill = int((self.current_tau / self.max_tau) * mid)
            left_side = " " * mid
            right_side = "#" * fill + " " * (mid - fill)
        else:
            fill = int((abs(self.current_tau) / abs(self.min_tau)) * mid)
            left_side = " " * (mid - fill) + "#" * fill
            right_side = " " * mid
        sys.stdout.write(f"\r\033[K Torque: [{left_side}|{right_side}] {self.current_tau:5.2f} Nm ")
        sys.stdout.flush()

    def keyboard_listener(self):
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == '\x1b':
                        sys.stdin.read(1)
                        arrow = sys.stdin.read(1)
                        if arrow == 'A':
                            self.current_tau = min(self.current_tau + self.step, self.max_tau)
                        elif arrow == 'B':
                            self.current_tau = max(self.current_tau - self.step, self.min_tau)
                    elif key == 'q':
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FakeTorquePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()