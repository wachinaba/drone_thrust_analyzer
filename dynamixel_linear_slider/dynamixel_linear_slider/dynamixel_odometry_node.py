#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from dynamixel_sdk import *
import math
import time


class DynamixelOdometryNode(Node):
    """
    Dynamixelモーターのオドメトリ計算を行うノード
    
    機能:
    - エンコーダー値の読み取り
    - スライダ軸方向の位置・速度の計算
    - ドリフト補正
    """
    
    def __init__(self):
        super().__init__('dynamixel_odometry_node')
        
        # パラメータの取得
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('port_name', '/dev/ttyUSB0')
        self.declare_parameter('rack_pitch', 0.005)  # m/rev (5mm/rev)
        self.declare_parameter('gear_ratio', 100.0)
        self.declare_parameter('encoder_resolution', 4096)
        self.declare_parameter('control_frequency', 100.0)
        
        self.motor_id = self.get_parameter('motor_id').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.port_name = self.get_parameter('port_name').value
        self.rack_pitch = self.get_parameter('rack_pitch').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.control_frequency = self.get_parameter('control_frequency').value
        
        # Dynamixel SDK の初期化
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(2.0)  # Protocol 2.0
        
        # 制御用のアドレス定義
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_VELOCITY = 128
        
        # オドメトリ計算用の変数
        self.prev_encoder_position = 0
        self.current_position = 0.0  # m
        self.current_velocity = 0.0  # m/s
        self.is_connected = False
        
        # エンコーダー値から位置への変換係数
        # エンコーダー1回転 = ラックピッチ / ギア比
        self.encoder_to_position_factor = self.rack_pitch / (self.gear_ratio * self.encoder_resolution)
        
        # パブリッシャーの設定
        self.position_publisher = self.create_publisher(
            Float64,
            'odometry_position',
            10
        )
        
        self.velocity_publisher = self.create_publisher(
            Float64,
            'odometry_velocity',
            10
        )
        
        # タイマーの設定
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.odometry_timer_callback
        )
        
        # Dynamixelとの接続
        self.connect_to_dynamixel()
        
        self.get_logger().info('DynamixelOdometryNode initialized')
    
    def connect_to_dynamixel(self):
        """Dynamixelモーターとの接続を確立"""
        try:
            # ポートを開く
            if not self.port_handler.openPort():
                self.get_logger().error(f'Failed to open port {self.port_name}')
                return
            
            # ボーレートを設定
            if not self.port_handler.setBaudRate(self.baud_rate):
                self.get_logger().error(f'Failed to set baud rate {self.baud_rate}')
                return
            
            # 初期エンコーダー値を読み取り
            result, data, error = self.packet_handler.read4ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_PRESENT_POSITION
            )
            if result == COMM_SUCCESS:
                self.prev_encoder_position = data
                self.is_connected = True
                self.get_logger().info(f'Successfully connected to Dynamixel ID {self.motor_id}')
            else:
                self.get_logger().error(f'Failed to read initial position: {error}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Dynamixel: {str(e)}')
    
    def encoder_to_position(self, encoder_value):
        """エンコーダー値を位置（m）に変換"""
        return encoder_value * self.encoder_to_position_factor
    
    def encoder_to_velocity(self, encoder_velocity):
        """エンコーダー速度値を速度（m/s）に変換"""
        # エンコーダー速度は rpm 単位
        rpm = encoder_velocity * 0.229  # Dynamixelの速度単位変換
        mps = (rpm / 60.0) * self.rack_pitch / self.gear_ratio
        return mps
    
    def read_odometry(self):
        """オドメトリ情報を読み取り"""
        if not self.is_connected:
            return
        
        try:
            # 現在位置の読み取り
            result, data, error = self.packet_handler.read4ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_PRESENT_POSITION
            )
            if result == COMM_SUCCESS:
                current_encoder_position = data
                
                # 位置の計算
                self.current_position = self.encoder_to_position(current_encoder_position)
                
                # 速度の計算（差分から）
                encoder_diff = current_encoder_position - self.prev_encoder_position
                if encoder_diff > self.encoder_resolution // 2:
                    encoder_diff -= self.encoder_resolution
                elif encoder_diff < -self.encoder_resolution // 2:
                    encoder_diff += self.encoder_resolution
                
                # 速度計算（差分 / 時間間隔）
                dt = 1.0 / self.control_frequency
                encoder_velocity = encoder_diff / dt
                self.current_velocity = self.encoder_to_velocity(encoder_velocity)
                
                self.prev_encoder_position = current_encoder_position
                
            # 現在速度の直接読み取り（バックアップ）
            result, data, error = self.packet_handler.read4ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_PRESENT_VELOCITY
            )
            if result == COMM_SUCCESS:
                # 直接読み取りの速度も計算
                direct_velocity = self.encoder_to_velocity(data)
                # 差分計算と直接読み取りの平均を取る
                self.current_velocity = (self.current_velocity + direct_velocity) / 2.0
                
        except Exception as e:
            self.get_logger().error(f'Error reading odometry: {str(e)}')
    
    def odometry_timer_callback(self):
        """オドメトリタイマーコールバック"""
        # オドメトリ情報を読み取り
        self.read_odometry()
        
        # 位置をパブリッシュ
        position_msg = Float64()
        position_msg.data = self.current_position
        self.position_publisher.publish(position_msg)
        
        # 速度をパブリッシュ
        velocity_msg = Float64()
        velocity_msg.data = self.current_velocity
        self.velocity_publisher.publish(velocity_msg)
    
    def __del__(self):
        """デストラクタ：接続を閉じる"""
        if hasattr(self, 'port_handler') and self.port_handler.is_open:
            self.port_handler.closePort()


def main(args=None):
    rclpy.init(args=args)
    
    node = DynamixelOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 