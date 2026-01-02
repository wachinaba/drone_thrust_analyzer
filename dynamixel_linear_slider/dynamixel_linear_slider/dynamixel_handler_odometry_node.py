#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from dynamixel_handler_msgs.msg import DxlStates
import math
import time


class DynamixelHandlerOdometryNode(Node):
    """
    DynamixelHandler_ros2を使用したオドメトリ計算ノード
    
    機能:
    - DynamixelHandler_ros2の状態取得を利用
    - スライダ軸方向の位置・速度の計算
    - ドリフト補正
    """
    
    def __init__(self):
        super().__init__('dynamixel_handler_odometry_node')
        
        # パラメータの取得
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('rack_pitch', 0.106214)  # m/rev
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('control_frequency', 100.0)
        
        self.motor_id = self.get_parameter('motor_id').value
        self.rack_pitch = self.get_parameter('rack_pitch').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.control_frequency = self.get_parameter('control_frequency').value
        
        # オドメトリ計算用の変数
        self.prev_encoder_position_deg = 0.0
        self.current_position = 0.0  # m
        self.current_velocity = 0.0  # m/s
        self.is_initialized = False
        
        # サブスクライバーの設定
        self.states_subscription = self.create_subscription(
            DxlStates,
            '/dynamixel/states',
            self.states_callback,
            10
        )
        
        # パブリッシャーの設定
        self.odometry_position_publisher = self.create_publisher(
            Float64,
            'odometry_position',
            10
        )
        
        self.odometry_velocity_publisher = self.create_publisher(
            Float64,
            'odometry_velocity',
            10
        )
        
        # タイマーの設定
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.odometry_timer_callback
        )
        
        self.get_logger().info('DynamixelHandlerOdometryNode initialized')
    
    def states_callback(self, msg):
        """DynamixelHandler_ros2の状態を受信"""
        # 指定されたモーターIDのデータを探す
        target_id = self.motor_id
        
        # presentデータから位置と速度を取得
        if msg.present.id_list:
            for i, id in enumerate(msg.present.id_list):
                if id == target_id:
                    # エンコーダー位置（degree）を取得
                    encoder_position_deg = msg.present.position_deg[i]
                    encoder_velocity_deg_s = msg.present.velocity_deg_s[i]
                    
                    # 初期化処理
                    if not self.is_initialized:
                        self.prev_encoder_position_deg = encoder_position_deg
                        self.is_initialized = True
                        self.get_logger().info(f'Odometry initialized for motor ID {target_id}')
                    
                    # 位置計算（相対変位の累積）
                    encoder_delta_deg = encoder_position_deg - self.prev_encoder_position_deg
                    position_delta = self.encoder_to_position(encoder_delta_deg)
                    self.current_position += position_delta
                    
                    # 速度計算
                    self.current_velocity = self.encoder_to_velocity(encoder_velocity_deg_s)
                    
                    # 前回値を更新
                    self.prev_encoder_position_deg = encoder_position_deg
                    
                    # デバッグ情報
                    self.get_logger().debug(f'Motor {target_id}: encoder_pos_deg={encoder_position_deg:.2f}, pos={self.current_position:.6f}m, vel={self.current_velocity:.6f}m/s')
                    break
    
    def encoder_to_position(self, encoder_delta_deg):
        """エンコーダー値（degree）を位置（m）に変換"""
        # degree -> rev -> m
        rev_delta = encoder_delta_deg / 360.0
        position_delta = rev_delta * self.rack_pitch / self.gear_ratio
        return position_delta
    
    def encoder_to_velocity(self, encoder_velocity_deg_s):
        """エンコーダー速度値を速度（m/s）に変換"""
        # degree/s -> rpm -> m/s
        rpm = encoder_velocity_deg_s / 6.0  # degree/s -> rpm
        mps = (rpm / 60.0) * self.rack_pitch / self.gear_ratio
        return mps
    
    def odometry_timer_callback(self):
        """オドメトリタイマーコールバック"""
        if not self.is_initialized:
            return
        
        # 位置をパブリッシュ
        position_msg = Float64()
        position_msg.data = self.current_position
        self.odometry_position_publisher.publish(position_msg)
        
        # 速度をパブリッシュ
        velocity_msg = Float64()
        velocity_msg.data = self.current_velocity
        self.odometry_velocity_publisher.publish(velocity_msg)
        
        # デバッグ情報
        self.get_logger().debug(f'Published: position={self.current_position:.6f}m, velocity={self.current_velocity:.6f}m/s')


def main(args=None):
    rclpy.init(args=args)
    
    node = DynamixelHandlerOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 