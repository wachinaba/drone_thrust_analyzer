#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_handler_msgs.msg import DxlCommandsX, DxlStates
import math
import time
import signal
import sys

class DynamixelHandlerController(Node):
    def __init__(self):
        super().__init__('dynamixel_handler_controller')

        # パラメータ宣言
        self.declare_parameter('id_list', [1])
        self.declare_parameter('amplitude', 160.0)  # deg/s
        self.declare_parameter('frequency', 0.08)    # Hz
        self.declare_parameter('publish_rate', 20.0) # Hz
        self.declare_parameter('control_mode', 'velocity')  # velocity, position, current_base_position

        self.id_list = self.get_parameter('id_list').get_parameter_value().integer_array_value
        if not self.id_list:
            self.id_list = [1]
        self.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().string_value

        # パブリッシャーとサブスクライバーの設定
        self.command_publisher_ = self.create_publisher(DxlCommandsX, '/dynamixel/commands/x', 10)
        self.states_subscription_ = self.create_subscription(
            DxlStates, '/dynamixel/states', self.states_callback, 10
        )

        # タイマーの設定
        self.timer_ = self.create_timer(1.0 / self.publish_rate, self.control_timer_callback)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.running = True

        # 状態管理
        self.current_positions = {}
        self.current_velocities = {}
        self.current_currents = {}
        self.servo_status = {}

        self.get_logger().info(f"Start DynamixelHandler controller: id_list={self.id_list}, amplitude={self.amplitude}, frequency={self.frequency}, publish_rate={self.publish_rate}, control_mode={self.control_mode}")

    def states_callback(self, msg):
        """Dynamixelの状態を受信"""
        # 現在値の更新
        if msg.present.id_list:
            for i, id in enumerate(msg.present.id_list):
                self.current_positions[id] = msg.present.position_deg[i]
                self.current_velocities[id] = msg.present.velocity_deg_s[i]
                self.current_currents[id] = msg.present.current_ma[i]

        # サーボ状態の更新
        if msg.status.id_list:
            for i, id in enumerate(msg.status.id_list):
                self.servo_status[id] = {
                    'torque': msg.status.torque[i],
                    'error': msg.status.error[i],
                    'ping': msg.status.ping[i],
                    'mode': msg.status.mode[i]
                }

    def control_timer_callback(self):
        """制御タイマーコールバック"""
        if not self.running:
            return
            
        now = self.get_clock().now().nanoseconds * 1e-9
        t = now - self.start_time

        # 制御値の計算
        if self.control_mode == 'velocity':
            control_value = self.amplitude * math.sin(2 * math.pi * self.frequency * t)
            self.publish_velocity_control(control_value)
        elif self.control_mode == 'position':
            # 位置制御の場合は往復運動
            position_amplitude = 45.0  # deg
            control_value = position_amplitude * math.sin(2 * math.pi * self.frequency * t)
            self.publish_position_control(control_value)
        elif self.control_mode == 'current_base_position':
            # 電流ベース位置制御
            position_amplitude = 45.0  # deg
            control_value = position_amplitude * math.sin(2 * math.pi * self.frequency * t)
            self.publish_current_base_position_control(control_value, current_limit=300.0)

    def publish_velocity_control(self, velocity):
        """速度制御のパブリッシュ"""
        msg = DxlCommandsX()
        msg.velocity_control.id_list = list(self.id_list)
        msg.velocity_control.velocity_deg_s = [velocity for _ in self.id_list]
        msg.velocity_control.profile_acc_deg_ss = []  # 空配列

        self.command_publisher_.publish(msg)
        self.get_logger().info(f"Velocity control: id_list={msg.velocity_control.id_list}, velocity_deg_s={msg.velocity_control.velocity_deg_s}")

    def publish_position_control(self, position):
        """位置制御のパブリッシュ"""
        msg = DxlCommandsX()
        msg.position_control.id_list = list(self.id_list)
        msg.position_control.position_deg = [position for _ in self.id_list]
        msg.position_control.profile_vel_deg_s = []
        msg.position_control.profile_acc_deg_ss = []

        self.command_publisher_.publish(msg)
        self.get_logger().info(f"Position control: id_list={msg.position_control.id_list}, position_deg={msg.position_control.position_deg}")

    def publish_current_base_position_control(self, position, current_limit):
        """電流ベース位置制御のパブリッシュ"""
        msg = DxlCommandsX()
        msg.current_base_position_control.id_list = list(self.id_list)
        msg.current_base_position_control.position_deg = [position for _ in self.id_list]
        msg.current_base_position_control.current_ma = [current_limit for _ in self.id_list]
        msg.current_base_position_control.profile_vel_deg_s = []
        msg.current_base_position_control.profile_acc_deg_ss = []

        self.command_publisher_.publish(msg)
        self.get_logger().info(f"Current base position control: id_list={msg.current_base_position_control.id_list}, position_deg={msg.current_base_position_control.position_deg}, current_ma={msg.current_base_position_control.current_ma}")

    def publish_torque_control(self, enable=True):
        """トルク制御のパブリッシュ"""
        msg = DxlCommandsX()
        msg.status.id_list = list(self.id_list)
        msg.status.torque = [enable for _ in self.id_list]

        self.command_publisher_.publish(msg)
        self.get_logger().info(f"Torque control: id_list={msg.status.id_list}, torque={msg.status.torque}")

    def publish_zero_velocity(self):
        """Velocityを0にリセット"""
        self.publish_velocity_control(0.0)
        self.get_logger().info(f"Reset velocity to 0: id_list={self.id_list}")

    def shutdown_callback(self):
        """終了時の処理"""
        self.get_logger().info('Shutting down...')
        self.running = False
        self.publish_zero_velocity()
        self.get_logger().info('Velocity reset to 0. Safe to stop.')

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelHandlerController()
    
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