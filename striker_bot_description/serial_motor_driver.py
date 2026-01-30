#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class SerialMotorDriver(Node):
    def __init__(self):
        super().__init__('serial_motor_driver')
        
        # --- CONFIGURATION ---
        self.serial_port = '/dev/ttyUSB0' # Check your port!
        self.baud_rate = 57600
        
        # Subscribe to Teleop Command
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
            
        # Initialize Serial Connection
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
            time.sleep(2) # Wait for Arduino reset
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            self.ser = None

    def listener_callback(self, msg):
        if not self.ser:
            return

        # 1. Get Linear and Angular speeds
        # Linear.x is m/s, Angular.z is rad/s
        linear = msg.linear.x
        angular = msg.angular.z
        
        # 2. Convert to Motor PWM (Simple Map)
        # We Map: -1.0 to 1.0 (ROS Speed)  ->  -255 to 255 (Arduino PWM)
        # You may need to tune the 'factor' multiplier based on how fast your bot goes
        factor = 200.0 
        
        left_val = (linear - angular) * factor
        right_val = (linear + angular) * factor
        
        # 3. Constrain values to -255 to 255
        left_pwm = int(max(min(left_val, 255), -255))
        right_pwm = int(max(min(right_val, 255), -255))
        
        # 4. Create String command: "Left,Right\n"
        command = f"{left_pwm},{right_pwm}\n"
        
        # 5. Send to Arduino
        self.ser.write(command.encode('utf-8'))
        # Optional: Print for debugging
        # self.get_logger().info(f"Sent: {command.strip()}")

    def stop(self):
        if self.ser:
            self.ser.write("0,0\n".encode('utf-8'))
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
