#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time


class PositionTestPublisherNode(Node):
    """
    位置制御のテスト用パブリッシャーノード
    
    機能:
    - 定期的に目標位置をパブリッシュ
    - 正弦波、ステップ、ランダムパターンの生成
    - 位置制御のテストとデバッグ
    """
    
    def __init__(self):
        super().__init__('position_test_publisher_node')
        
        # パラメータの取得
        self.declare_parameter('publish_frequency', 0.1)  # Hz（10秒ごと）
        self.declare_parameter('position_amplitude', 0.5)  # 位置振幅 (m)
        self.declare_parameter('position_offset', 1.0)  # 位置オフセット (m)
        self.declare_parameter('test_pattern', 'step')  # step, sine, random
        self.declare_parameter('enable_test', True)  # テストの有効/無効
        
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.position_amplitude = self.get_parameter('position_amplitude').value
        self.position_offset = self.get_parameter('position_offset').value
        self.test_pattern = self.get_parameter('test_pattern').value
        self.enable_test = self.get_parameter('enable_test').value
        
        # パブリッシャーの設定
        self.target_position_publisher = self.create_publisher(
            Float64,
            'target_position_deg',
            10
        )
        
        # タイマーの設定
        self.timer = self.create_timer(
            1.0 / self.publish_frequency,
            self.timer_callback
        )
        
        # テスト状態
        self.test_start_time = time.time()
        self.position_counter = 0
        self.current_position = 0.0
        
        # ランダム生成用
        import random
        self.random_generator = random.Random()
        
        self.get_logger().info('PositionTestPublisherNode initialized')
        self.get_logger().info(f'Publish frequency: {self.publish_frequency} Hz')
        self.get_logger().info(f'Position amplitude: {self.position_amplitude} m')
        self.get_logger().info(f'Position offset: {self.position_offset} m')
        self.get_logger().info(f'Test pattern: {self.test_pattern}')
        self.get_logger().info(f'Enable test: {self.enable_test}')
    
    def timer_callback(self):
        """タイマーコールバック"""
        if not self.enable_test:
            return
        
        # テストパターンに応じた目標位置の生成（m単位）
        target_position_m = self.generate_target_position()
        
        # 目標位置をパブリッシュ
        position_msg = Float64()
        position_msg.data = target_position_m
        self.target_position_publisher.publish(position_msg)
        
        self.get_logger().info(f'Published target position: {target_position_m:.4f} m')
    
    def generate_target_position(self):
        """目標位置の生成（m単位）"""
        current_time = time.time() - self.test_start_time
        
        if self.test_pattern == 'step':
            # ステップパターン: 一定間隔で位置を切り替え
            step_interval = 5.0  # 5秒ごとに位置を切り替え
            step_time = current_time % (step_interval * 2)
            
            if step_time < step_interval:
                target_position_m = self.position_offset + self.position_amplitude
            else:
                target_position_m = self.position_offset - self.position_amplitude
        
        elif self.test_pattern == 'sine':
            # 正弦波パターン
            frequency = 0.1  # 0.1 Hz（10秒周期）
            target_position_m = self.position_offset + self.position_amplitude * math.sin(2.0 * math.pi * frequency * current_time)
        
        elif self.test_pattern == 'random':
            # ランダムパターン
            if self.position_counter % 5 == 0:  # 5回ごとに新しい位置を生成
                self.current_position = self.position_offset + self.random_generator.uniform(-self.position_amplitude, self.position_amplitude)
            
            target_position_m = self.current_position
        
        elif self.test_pattern == 'increment':
            # インクリメントパターン: 徐々に位置を増加
            increment_step = 0.1  # 0.1mずつ増加
            target_position_m = self.position_offset + (self.position_counter * increment_step) % (2 * self.position_amplitude) - self.position_amplitude
        
        else:
            # デフォルト: ステップパターン
            target_position_m = self.position_offset + self.position_amplitude if self.position_counter % 2 == 0 else self.position_offset - self.position_amplitude
        
        self.position_counter += 1
        return target_position_m
    
    def stop_test(self):
        """テストの停止"""
        self.enable_test = False
        self.get_logger().info('Position test stopped')


def main(args=None):
    rclpy.init(args=args)
    
    node = PositionTestPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_test()
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 