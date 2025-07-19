#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
import sys


class ConvergenceBrakeTestNode(Node):
    """
    収束ブレーキ機能のテストノード
    
    機能:
    - 目標位置の送信
    - 収束状態の監視
    - テスト結果の出力
    """
    
    def __init__(self):
        super().__init__('convergence_brake_test_node')
        
        # パラメータの取得
        self.declare_parameter('test_duration', 30.0)  # テスト時間 (秒)
        self.declare_parameter('position_step', 0.02)  # 位置ステップ (m)
        self.declare_parameter('step_interval', 5.0)   # ステップ間隔 (秒)
        
        self.test_duration = self.get_parameter('test_duration').value
        self.position_step = self.get_parameter('position_step').value
        self.step_interval = self.get_parameter('step_interval').value
        
        # パブリッシャーの設定
        self.target_position_publisher = self.create_publisher(
            Float64,
            'target_position_command',
            10
        )
        
        # サブスクライバーの設定
        self.estimated_position_subscription = self.create_subscription(
            Float64,
            'estimated_position',
            self.estimated_position_callback,
            10
        )
        
        self.velocity_command_subscription = self.create_subscription(
            Float64,
            'velocity_command',
            self.velocity_command_callback,
            10
        )
        
        # テスト状態
        self.test_start_time = time.time()
        self.current_step = 0
        self.estimated_position = 0.0
        self.velocity_command = 0.0
        self.position_valid = False
        self.velocity_valid = False
        
        # タイマーの設定
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('ConvergenceBrakeTestNode initialized')
        self.get_logger().info(f'Test duration: {self.test_duration} s')
        self.get_logger().info(f'Position step: {self.position_step} m')
        self.get_logger().info(f'Step interval: {self.step_interval} s')
    
    def estimated_position_callback(self, msg):
        """推定位置のコールバック"""
        self.estimated_position = msg.data
        self.position_valid = True
    
    def velocity_command_callback(self, msg):
        """速度指令のコールバック"""
        self.velocity_command = msg.data
        self.velocity_valid = True
    
    def timer_callback(self):
        """タイマーコールバック"""
        current_time = time.time() - self.test_start_time
        
        # テスト終了判定
        if current_time >= self.test_duration:
            self.get_logger().info('Test completed')
            rclpy.shutdown()
            return
        
        # ステップ更新
        step_time = current_time % self.step_interval
        new_step = int(current_time / self.step_interval)
        
        if new_step != self.current_step:
            self.current_step = new_step
            target_position = self.position_step * (self.current_step + 1)
            
            # 目標位置を送信
            position_msg = Float64()
            position_msg.data = target_position
            self.target_position_publisher.publish(position_msg)
            
            self.get_logger().info(f'Step {self.current_step + 1}: Target position = {target_position:.4f} m')
        
        # 状態監視
        if self.position_valid and self.velocity_valid:
            position_error = abs(target_position - self.estimated_position)
            self.get_logger().info(
                f'Time: {current_time:.1f}s, '
                f'Target: {target_position:.4f}m, '
                f'Estimated: {self.estimated_position:.4f}m, '
                f'Error: {position_error:.6f}m, '
                f'Velocity: {self.velocity_command:.6f}m/s'
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = ConvergenceBrakeTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 