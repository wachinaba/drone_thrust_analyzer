#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from dynamixel_handler_msgs.msg import DxlCommandsX
import math
import time


class SimulatorTestPublisher(Node):
    """
    Dynamixelシミュレーションノードのテスト用パブリッシャー
    
    機能:
    - 様々な速度指令パターンの生成
    - 正弦波、ステップ、ランプ波形の生成
    """
    
    def __init__(self):
        super().__init__('simulator_test_publisher')
        
        # パラメータの取得
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('test_pattern', 'sine')  # sine, step, ramp
        self.declare_parameter('amplitude', 300.0)  # deg/s
        self.declare_parameter('frequency', 0.5)  # Hz
        self.declare_parameter('publish_frequency', 10.0)  # Hz
        
        self.motor_id = self.get_parameter('motor_id').value
        self.test_pattern = self.get_parameter('test_pattern').value
        self.amplitude = self.get_parameter('amplitude').value
        self.frequency = self.get_parameter('frequency').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        
        # DynamixelHandler形式のコマンドパブリッシャー
        self.command_publisher = self.create_publisher(
            DxlCommandsX,
            '/dynamixel/commands/x',
            10
        )
        
        # タイマーの設定
        self.timer = self.create_timer(
            1.0 / self.publish_frequency,
            self.timer_callback
        )
        
        # 時間管理
        self.start_time = time.time()
        
        self.get_logger().info('SimulatorTestPublisher initialized')
        self.get_logger().info(f'Motor ID: {self.motor_id}')
        self.get_logger().info(f'Test pattern: {self.test_pattern}')
        self.get_logger().info(f'Amplitude: {self.amplitude} deg/s')
        self.get_logger().info(f'Frequency: {self.frequency} Hz')
    
    def timer_callback(self):
        """タイマーコールバック"""
        current_time = time.time() - self.start_time
        
        # テストパターンに応じた速度指令の生成（deg/s）
        if self.test_pattern == 'sine':
            velocity_deg_s = self.amplitude * math.sin(2.0 * math.pi * self.frequency * current_time)
        elif self.test_pattern == 'step':
            # ステップ波形（0.5秒ごとに振幅を切り替え）
            step_time = current_time % 1.0
            if step_time < 0.5:
                velocity_deg_s = self.amplitude
            else:
                velocity_deg_s = -self.amplitude
        elif self.test_pattern == 'ramp':
            # ランプ波形（-amplitude から +amplitude まで）
            ramp_time = current_time % (2.0 / self.frequency)
            velocity_deg_s = -self.amplitude + (2.0 * self.amplitude * ramp_time * self.frequency)
        else:
            velocity_deg_s = 0.0
        
        # DynamixelHandler形式のコマンドメッセージを作成
        command_msg = DxlCommandsX()
        command_msg.velocity_control.id_list = [self.motor_id]
        command_msg.velocity_control.velocity_deg_s = [velocity_deg_s]
        command_msg.velocity_control.profile_acc_deg_ss = []  # 空配列
        
        # コマンドをパブリッシュ
        self.command_publisher.publish(command_msg)
        
        self.get_logger().debug(f'Published velocity command: {velocity_deg_s:.2f} deg/s')


def main(args=None):
    rclpy.init(args=args)
    
    node = SimulatorTestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 