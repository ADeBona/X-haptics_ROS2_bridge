#!/usr/bin/env python3
"""
Subscribes to /kinova/sim_torque, maps absolute torque (0-5 Nm) to a
pressure target (0-50 kPa), and streams the target to the Arduino
pressure tracker over serial.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time 


class KinovaHapticBridge(Node):
    def __init__(self):
        super().__init__('kinova_haptic_bridge')
        
        # HARDCODED. No ROS parameters to mess this up.
        port = '/dev/ttyACM0' 
        baud = 115200

        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            # Give the Arduino the exact time it needs to reboot
            time.sleep(3.0) 
            self.get_logger().info(f'CONNECTED TO HARDCODED PORT: {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        self.max_torque = 5.0
        self.max_pressure = 50.0

        self.create_subscription(Float32, '/kinova/sim_torque', self.on_torque_received, 10)
    def on_torque_received(self, msg: Float32):
        tau = msg.data
        self.get_logger().info(f'BRIDGE RECEIVED TORQUE: {tau}') # ADD THIS LINE
        # Ensure your write block looks like this
        target_pressure = (abs(tau) / self.max_torque) * self.max_pressure
        target_pressure = min(target_pressure, self.max_pressure)
        command = f"{target_pressure:.2f}\n" # Ensure the newline is here
        
        try: 
            self.ser.write(command.encode('utf-8'))
            self.ser.flush() # CRITICAL: Force the data out of the buffer immediately
            self.get_logger().info(f'WRITING TO ARDUINO: {command.strip()}') 
        except serial.SerialException as e: 
            self.get_logger().error(f'Serial write failed: {e}') 
    def destroy_node(self):
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()