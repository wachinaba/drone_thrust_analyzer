#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
import math
import time
from dynamixel_handler_msgs.msg import DxlStates, DxlCommandsX

class DynamixelHandlerPositionControllerNode(Node):
    def __init__(self):
        super().__init__('dynamixel_handler_position_controller_node')

        # パラメータの取得
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('rack_pitch', 0.106214)
        self.declare_parameter('gear_ratio', 1.0)
        
        self.control_frequency = self.get_parameter('control_frequency').value

        self.rack_pitch = self.get_parameter('rack_pitch').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        # サブスクライバーの設定
        self.estimated_position_subscription = self.create_subscription(
            Float64,
            'estimated_position',
            self.estimated_position_callback,
            10
        )
        
        self.target_position_subscription = self.create_subscription(
            Float64,
            'target_position',
            self.target_position_callback,
            10
        )
        
        self.feedforward_velocity_subscription = self.create_subscription(
            Float64,
            'feedforward_velocity',
            self.feedforward_velocity_callback,
            10
        )

        self.dynamixel_present_state_subscription = self.create_subscription(
            DxlStates,
            '/dynamixel/states',
            self.dynamixel_present_state_callback,
            10
        )
        
        # パブリッシャーの設定
        self.dynamixel_command_publisher = self.create_publisher(
            DxlCommandsX,
            '/dynamixel/commands/x',
            10
        )

        # 制御用の変数
        self.estimated_position = 0.0
        self.target_position = 0.0
        self.feedforward_velocity = 0.0

        self.current_dynamixel_position_deg = 0.0

        self.m_to_deg_factor = 360.0 / self.rack_pitch * self.gear_ratio

        # データの有効性フラグ
        self.estimated_position_valid = False
        self.target_position_valid = False
        self.feedforward_velocity_valid = False
        self.dynamixel_present_state_valid = False
        
        # タイマーの設定
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_timer_callback)
        
        
        
    def estimated_position_callback(self, msg):
        self.estimated_position = msg.data
        self.estimated_position_valid = True
    
    def target_position_callback(self, msg):
        self.target_position = msg.data
        self.target_position_valid = True
    
    def feedforward_velocity_callback(self, msg):
        self.feedforward_velocity = msg.data
        self.feedforward_velocity_valid = True
    
    def dynamixel_present_state_callback(self, msg):
        present_state = msg.present
        if len(present_state.id_list) > 0:
            self.current_dynamixel_position_deg = present_state.position_deg[0]
            self.dynamixel_present_state_valid = True

    def calculate_position_error(self):
        if not self.estimated_position_valid or not self.target_position_valid or not self.dynamixel_present_state_valid:
            return 0.0
        
        error = self.target_position - self.estimated_position
        return error
    
    def control_timer_callback(self):
        if not self.estimated_position_valid or not self.target_position_valid or not self.feedforward_velocity_valid or not self.dynamixel_present_state_valid:
            return
        
        position_error = self.calculate_position_error()
        degree_error = position_error * self.m_to_deg_factor

        feedforward_velocity_deg_s = self.feedforward_velocity * self.m_to_deg_factor

        self.publish_position_command(degree_error, feedforward_velocity_deg_s)
        
    def publish_position_command(self, position_error, feedforward_velocity_deg_s):
        dxl_msg = DxlCommandsX()
        dxl_msg.extended_position_control.id_list = [1]
        dxl_msg.extended_position_control.position_deg = [position_error + self.current_dynamixel_position_deg]

        # 少し余裕を持たせる
        feedforward_velocity_deg_s_compensated = math.fabs(feedforward_velocity_deg_s) * 1.3

        if (feedforward_velocity_deg_s_compensated < 20.0):
            feedforward_velocity_deg_s_compensated = 20.0
        
        dxl_msg.extended_position_control.profile_vel_deg_s = [feedforward_velocity_deg_s_compensated]

        self.dynamixel_command_publisher.publish(dxl_msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = DynamixelHandlerPositionControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()