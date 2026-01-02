#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from dynamixel_sdk import *
import math
import time


class DynamixelDriverNode(Node):
    """
    Dynamixelモーターの直接制御を行うノード
    
    機能:
    - 速度指令値の受信
    - モーター制御
    - エンコーダー値の読み取り
    """
    
    def __init__(self):
        super().__init__('dynamixel_driver_node')
        
        # パラメータの取得
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('port_name', '/dev/ttyUSB0')
        self.declare_parameter('control_frequency', 100.0)
        
        self.motor_id = self.get_parameter('motor_id').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.port_name = self.get_parameter('port_name').value
        self.control_frequency = self.get_parameter('control_frequency').value
        
        # Dynamixel SDK の初期化
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(2.0)  # Protocol 2.0
        
        # 制御用のアドレス定義
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_VELOCITY = 104
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_VELOCITY = 128
        self.ADDR_OPERATING_MODE = 11
        
        # 速度制限（rpm）
        self.MAX_VELOCITY = 100.0
        self.MIN_VELOCITY = -100.0
        
        # 現在の状態
        self.current_velocity = 0.0
        self.current_position = 0.0
        self.is_connected = False
        
        # パブリッシャーとサブスクライバーの設定
        self.velocity_subscription = self.create_subscription(
            Float64,
            'velocity_command',
            self.velocity_command_callback,
            10
        )
        
        self.status_publisher = self.create_publisher(
            String,
            'motor_status',
            10
        )
        
        # タイマーの設定
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_timer_callback
        )
        
        # Dynamixelとの接続
        self.connect_to_dynamixel()
        
        self.get_logger().info('DynamixelDriverNode initialized')
    
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
            
            # トルクを有効化
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 1
            )
            if result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to enable torque: {error}')
                return
            
            # 速度制御モードに設定
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_OPERATING_MODE, 1
            )
            if result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to set velocity control mode: {error}')
                return
            
            self.is_connected = True
            self.get_logger().info(f'Successfully connected to Dynamixel ID {self.motor_id}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Dynamixel: {str(e)}')
    
    def velocity_command_callback(self, msg):
        """速度指令値のコールバック"""
        if not self.is_connected:
            self.get_logger().warn('Dynamixel not connected, ignoring velocity command')
            return
        
        # 速度制限の適用
        velocity = max(self.MIN_VELOCITY, min(self.MAX_VELOCITY, msg.data))
        
        # 速度指令を送信
        try:
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_GOAL_VELOCITY, 
                int(velocity)
            )
            if result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to set velocity: {error}')
            else:
                self.current_velocity = velocity
                
        except Exception as e:
            self.get_logger().error(f'Error setting velocity: {str(e)}')
    
    def read_current_state(self):
        """現在の位置と速度を読み取り"""
        if not self.is_connected:
            return
        
        try:
            # 現在位置の読み取り
            result, data, error = self.packet_handler.read4ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_PRESENT_POSITION
            )
            if result == COMM_SUCCESS:
                self.current_position = data
            
            # 現在速度の読み取り
            result, data, error = self.packet_handler.read4ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_PRESENT_VELOCITY
            )
            if result == COMM_SUCCESS:
                self.current_velocity = data
                
        except Exception as e:
            self.get_logger().error(f'Error reading motor state: {str(e)}')
    
    def control_timer_callback(self):
        """制御タイマーコールバック"""
        # 現在の状態を読み取り
        self.read_current_state()
        
        # ステータスをパブリッシュ
        status_msg = String()
        status_msg.data = f'Position: {self.current_position}, Velocity: {self.current_velocity}, Connected: {self.is_connected}'
        self.status_publisher.publish(status_msg)
    
    def __del__(self):
        """デストラクタ：接続を閉じる"""
        if hasattr(self, 'port_handler') and self.port_handler.is_open:
            self.port_handler.closePort()


def main(args=None):
    rclpy.init(args=args)
    
    node = DynamixelDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 