#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Bool
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
        self.declare_parameter('profile_velocity_deg_s', 1000.0)  # プロファイル速度（度/秒）: 高め
        self.declare_parameter('profile_accel_deg_ss', 200.0)     # プロファイル加速度（度/秒^2）: ゆっくり
        
        self.control_frequency = self.get_parameter('control_frequency').value
        self.rack_pitch = self.get_parameter('rack_pitch').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.profile_velocity_deg_s = self.get_parameter('profile_velocity_deg_s').value
        self.profile_accel_deg_ss = self.get_parameter('profile_accel_deg_ss').value
        
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

        self.dynamixel_present_state_subscription = self.create_subscription(
            DxlStates,
            '/dynamixel/states',
            self.dynamixel_present_state_callback,
            10
        )
        self.origin_offset_subscription = self.create_subscription(
            Float64,
            'origin_offset_deg',
            self.origin_offset_callback,
            10
        )
        
        # パブリッシャーの設定
        self.dynamixel_command_publisher = self.create_publisher(
            DxlCommandsX,
            '/dynamixel/commands/x',
            10
        )
        
        # 移動状態パブリッシャー（bool型）
        self.movement_status_publisher = self.create_publisher(
            Bool,
            'movement_status',
            10
        )

        # 制御用の変数
        self.estimated_position = 0.0
        self.target_position = 0.0
        self.current_dynamixel_position_deg = 0.0
        self.origin_offset_deg = 0.0
        self.origin_offset_valid = False

        self.m_to_deg_factor = 360.0 / self.rack_pitch * self.gear_ratio

        # データの有効性フラグ
        self.estimated_position_valid = False  # 原点復帰の確認用
        self.target_position_valid = False
        self.dynamixel_present_state_valid = False
        
        # 移動状態管理（簡易版：目標位置と推定位置の差で判定）
        self.is_moving = False
        
        # 警告フラグ（初回のみ警告を表示）
        self.origin_warning_issued = False
        
        # タイマーの設定
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_timer_callback)
        
        self.get_logger().info('DynamixelHandlerPositionControllerNode initialized (simplified mode)')
        self.get_logger().info(f'Control frequency: {self.control_frequency} Hz')
        self.get_logger().info(f'Rack pitch: {self.rack_pitch} m/rev')
        self.get_logger().info(f'Gear ratio: {self.gear_ratio}')
        self.get_logger().info(f'Profile velocity: {self.profile_velocity_deg_s} deg/s')
        self.get_logger().info(f'Profile acceleration: {self.profile_accel_deg_ss} deg/s^2')
        self.get_logger().info('Control mode: Simple position command (control handled by Dynamixel)')
        
        
    def estimated_position_callback(self, msg):
        self.estimated_position = msg.data
        self.estimated_position_valid = True
    
    def target_position_callback(self, msg):
        self.target_position = msg.data
        self.target_position_valid = True
    
    def dynamixel_present_state_callback(self, msg):
        present_state = msg.present
        if len(present_state.id_list) > 0:
            self.current_dynamixel_position_deg = present_state.position_deg[0]
            self.dynamixel_present_state_valid = True
    
    def origin_offset_callback(self, msg):
        self.origin_offset_deg = msg.data
        self.origin_offset_valid = True

    def calculate_position_error(self):
        """位置偏差を計算（移動状態判定用）"""
        if not self.estimated_position_valid or not self.target_position_valid:
            return 0.0
        
        error = self.target_position - self.estimated_position
        return error
    
    def update_movement_status(self):
        """移動状態を更新"""
        if not self.estimated_position_valid or not self.target_position_valid:
            self.is_moving = False
            return
        
        position_error = abs(self.calculate_position_error())
        # 位置偏差が0.001m以上の場合、移動中と判定
        self.is_moving = position_error > 0.001
        
        status_msg = Bool()
        status_msg.data = self.is_moving
        self.movement_status_publisher.publish(status_msg)
    
    def control_timer_callback(self):
        """
        制御タイマーコールバック
        常に目標位置をDynamixelに送信する（制御はDynamixel側で行う）
        """
        # 目標位置が有効でない場合は送信しない
        if not self.target_position_valid:
            return
        
        # 原点復帰が完了していない場合の警告（初回のみ）
        if not self.estimated_position_valid and not self.origin_warning_issued:
            self.get_logger().warn('Warning: Origin not reset yet. Position control may be inaccurate.')
            self.origin_warning_issued = True
        
        # 移動状態を更新
        self.update_movement_status()
        
        # 目標位置をそのまま送信（制御はDynamixel側で行う）
        self.publish_position_command()
        
    def publish_position_command(self):
        """
        位置制御コマンドを送信
        
        単純に目標位置を度に変換してDynamixelに送信するだけ。
        制御はDynamixel側で行う。
        """
        dxl_msg = DxlCommandsX()
        dxl_msg.extended_position_control.id_list = [1]
        
        # 目標位置を度に変換して送信
        target_position_deg = self.target_position * self.m_to_deg_factor
        # ARマーカー原点オフセットを加味（原点未設定時は0）
        if self.origin_offset_valid:
            target_position_deg += self.origin_offset_deg
        dxl_msg.extended_position_control.position_deg = [target_position_deg]
        
        # プロファイル速度を設定
        dxl_msg.extended_position_control.profile_vel_deg_s = [self.profile_velocity_deg_s]
        # プロファイル加速度も設定
        dxl_msg.extended_position_control.profile_acc_deg_ss = [self.profile_accel_deg_ss]

        self.dynamixel_command_publisher.publish(dxl_msg)

        
def main(args=None):
    rclpy.init(args=args)
    node = DynamixelHandlerPositionControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()