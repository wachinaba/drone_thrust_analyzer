#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from dynamixel_sdk_custom_interfaces.msg import SetPosition
import time
import math

class DynamixelVelocityPublisher(Node):
    """
    Dynamixel linear sliderのvelocity指令値をpublishするノード
    """
    
    def __init__(self):
        super().__init__('dynamixel_velocity_publisher')
        
        # パラメータの宣言
        self.declare_parameter('target_velocity', 0.1)  # m/s
        self.declare_parameter('publish_rate', 10.0)    # Hz
        self.declare_parameter('velocity_topic', '/dynamixel/velocity_command')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('max_velocity', 0.5)     # m/s
        self.declare_parameter('min_velocity', -0.5)    # m/s
        
        # パラメータの取得
        self.target_velocity = self.get_parameter('target_velocity').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.velocity_topic = self.get_parameter('velocity_topic').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.min_velocity = self.get_parameter('min_velocity').value
        
        # パブリッシャーの作成
        self.velocity_pub = self.create_publisher(
            Float64, 
            self.velocity_topic, 
            10
        )
        
        # サブスクライバーの作成（現在位置の監視用）
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10
        )
        
        # タイマーの作成
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # 内部変数
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.start_time = time.time()
        
        self.get_logger().info(f'Dynamixel Velocity Publisher started')
        self.get_logger().info(f'Target velocity: {self.target_velocity} m/s')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        
    def joint_state_callback(self, msg):
        """
        JointStateメッセージを受信して現在位置と速度を更新
        """
        if len(msg.position) > 0:
            self.current_position = msg.position[0]
        if len(msg.velocity) > 0:
            self.current_velocity = msg.velocity[0]
            
    def timer_callback(self):
        """
        定期的にvelocity指令値をpublish
        """
        # 速度制限の適用
        limited_velocity = max(self.min_velocity, min(self.max_velocity, self.target_velocity))
        
        # メッセージの作成
        velocity_msg = Float64()
        velocity_msg.data = limited_velocity
        
        # パブリッシュ
        self.velocity_pub.publish(velocity_msg)
        
        # ログ出力（1秒に1回）
        current_time = time.time()
        if int(current_time) % 5 == 0 and int(current_time) != int(self.start_time):
            self.get_logger().info(
                f'Published velocity: {limited_velocity:.3f} m/s, '
                f'Current position: {self.current_position:.3f} m, '
                f'Current velocity: {self.current_velocity:.3f} m/s'
            )
            
    def set_target_velocity(self, velocity):
        """
        目標速度を設定
        """
        self.target_velocity = max(self.min_velocity, min(self.max_velocity, velocity))
        self.get_logger().info(f'Target velocity set to: {self.target_velocity} m/s')
        
    def stop_motor(self):
        """
        モータを停止
        """
        self.target_velocity = 0.0
        self.get_logger().info('Motor stopped')

def main(args=None):
    rclpy.init(args=args)
    
    node = DynamixelVelocityPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, stopping motor...')
        node.stop_motor()
        # 停止指令を数回送信
        for _ in range(5):
            node.timer_callback()
            time.sleep(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 