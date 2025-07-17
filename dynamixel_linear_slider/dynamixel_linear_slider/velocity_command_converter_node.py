#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from dynamixel_handler_msgs.msg import DxlCommandsX
import math


class VelocityCommandConverterNode(Node):
    """
    速度指令値の変換とDynamixelHandlerへの送信を行うノード
    
    機能:
    - 速度指令値（m/s）の受信
    - deg/sへの変換
    - DynamixelHandlerへの送信
    """
    
    def __init__(self):
        super().__init__('velocity_command_converter_node')
        
        # パラメータの取得
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('rack_pitch', 0.106214)  # m/rev
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('max_velocity_deg_s', 100.0)
        self.declare_parameter('min_velocity_deg_s', -100.0)
        
        self.motor_id = self.get_parameter('motor_id').value
        self.rack_pitch = self.get_parameter('rack_pitch').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.max_velocity_deg_s = self.get_parameter('max_velocity_deg_s').value
        self.min_velocity_deg_s = self.get_parameter('min_velocity_deg_s').value
        
        # 変換係数の計算
        # 1 rev = rack_pitch m
        # 1 rev = 360 deg
        # 1 m/s = (360 / rack_pitch) * gear_ratio deg/s
        self.mps_to_deg_s_factor = (360.0 / self.rack_pitch) * self.gear_ratio
        
        # サブスクライバーの設定
        self.velocity_command_subscription = self.create_subscription(
            Float64,
            'velocity_command',
            self.velocity_command_callback,
            10
        )
        
        # DynamixelHandler用のパブリッシャー
        self.dynamixel_command_publisher = self.create_publisher(
            DxlCommandsX,
            '/dynamixel/commands/x',
            10
        )
        
        # デバッグ用パブリッシャー
        self.converted_velocity_publisher = self.create_publisher(
            Float64,
            'converted_velocity_deg_s',
            10
        )
        
        self.get_logger().info(f'VelocityCommandConverterNode initialized: motor_id={self.motor_id}, conversion_factor={self.mps_to_deg_s_factor:.2f}')
    
    def velocity_command_callback(self, msg):
        """速度指令値のコールバック"""
        velocity_mps = msg.data
        
        # m/s から deg/s への変換
        velocity_deg_s = self.convert_mps_to_deg_s(velocity_mps)
        
        # 速度制限の適用
        velocity_deg_s = max(self.min_velocity_deg_s, 
                           min(self.max_velocity_deg_s, velocity_deg_s))
        
        # DynamixelHandler用のメッセージを作成
        dxl_msg = DxlCommandsX()
        dxl_msg.velocity_control.id_list = [self.motor_id]
        dxl_msg.velocity_control.velocity_deg_s = [velocity_deg_s]
        dxl_msg.velocity_control.profile_acc_deg_ss = []  # 空配列
        
        # 送信
        self.dynamixel_command_publisher.publish(dxl_msg)
        
        # デバッグ用に変換後の速度をパブリッシュ
        converted_msg = Float64()
        converted_msg.data = velocity_deg_s
        self.converted_velocity_publisher.publish(converted_msg)
        
        # デバッグ情報の出力
        self.get_logger().debug(
            f'Velocity conversion: {velocity_mps:.4f} m/s -> {velocity_deg_s:.2f} deg/s'
        )
    
    def convert_mps_to_deg_s(self, velocity_mps):
        """m/s から deg/s への変換"""
        return velocity_mps * self.mps_to_deg_s_factor
    
    def convert_deg_s_to_mps(self, velocity_deg_s):
        """deg/s から m/s への変換"""
        return velocity_deg_s / self.mps_to_deg_s_factor
    
    def set_velocity_limits(self, max_velocity_deg_s, min_velocity_deg_s):
        """速度制限を設定"""
        self.max_velocity_deg_s = max_velocity_deg_s
        self.min_velocity_deg_s = min_velocity_deg_s
        self.get_logger().info(f'Velocity limits updated: max={max_velocity_deg_s}, min={min_velocity_deg_s}')
    
    def set_conversion_parameters(self, rack_pitch, gear_ratio):
        """変換パラメータを設定"""
        self.rack_pitch = rack_pitch
        self.gear_ratio = gear_ratio
        self.mps_to_deg_s_factor = (360.0 / self.rack_pitch) * self.gear_ratio
        self.get_logger().info(f'Conversion parameters updated: rack_pitch={rack_pitch}, gear_ratio={gear_ratio}, factor={self.mps_to_deg_s_factor:.2f}')


def main(args=None):
    rclpy.init(args=args)
    
    node = VelocityCommandConverterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 