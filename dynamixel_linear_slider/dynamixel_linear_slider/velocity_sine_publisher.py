#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_handler_msgs.msg import DxlCommandsX
import math
import time
import signal
import sys

class VelocitySinePublisher(Node):
    def __init__(self):
        super().__init__('dynamixel_velocity_sine_publisher')

        # パラメータ宣言
        self.declare_parameter('id_list', [1])
        self.declare_parameter('amplitude', 160.0)  # deg/s
        self.declare_parameter('frequency', 0.08)    # Hz
        self.declare_parameter('publish_rate', 20.0) # Hz

        self.id_list = self.get_parameter('id_list').get_parameter_value().integer_array_value
        if not self.id_list:
            self.id_list = [1]
        self.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(DxlCommandsX, '/dynamixel/commands/x', 10)
        self.timer_ = self.create_timer(1.0 / self.publish_rate, self.publish_velocity)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.running = True

        self.get_logger().info(f"Start velocity sine publisher: id_list={self.id_list}, amplitude={self.amplitude}, frequency={self.frequency}, publish_rate={self.publish_rate}")

    def publish_velocity(self):
        if not self.running:
            return
            
        now = self.get_clock().now().nanoseconds * 1e-9
        t = now - self.start_time
        velocity = self.amplitude * math.sin(2 * math.pi * self.frequency * t)

        msg = DxlCommandsX()
        msg.velocity_control.id_list = list(self.id_list)
        msg.velocity_control.velocity_deg_s = [velocity for _ in self.id_list]
        msg.velocity_control.profile_acc_deg_ss = []  # 空配列

        self.publisher_.publish(msg)
        self.get_logger().info(f"Publish: id_list={msg.velocity_control.id_list}, velocity_deg_s={msg.velocity_control.velocity_deg_s}")

    def publish_zero_velocity(self):
        """Velocityを0にリセット"""
        msg = DxlCommandsX()
        msg.velocity_control.id_list = list(self.id_list)
        msg.velocity_control.velocity_deg_s = [0.0 for _ in self.id_list]
        msg.velocity_control.profile_acc_deg_ss = []

        self.publisher_.publish(msg)
        self.get_logger().info(f"Reset velocity to 0: id_list={msg.velocity_control.id_list}")

    def shutdown_callback(self):
        """終了時の処理"""
        self.get_logger().info('Shutting down...')
        self.running = False
        self.publish_zero_velocity()
        self.get_logger().info('Velocity reset to 0. Safe to stop.')

def main(args=None):
    rclpy.init(args=args)
    node = VelocitySinePublisher()
    
    def signal_handler(sig, frame):
        node.shutdown_callback()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown_callback()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 