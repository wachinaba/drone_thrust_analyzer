#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
import time

class JointPubNode(Node):
    """
    C++コードを完全に移植したPython版
    """
    
    def __init__(self):
        super().__init__('pub_dynamixel_data_node')
        
        # パブリッシャーの作成
        self.publisher_ = self.create_publisher(SetPosition, "/set_position", 10)
        
        # タイマーの作成（500ms間隔）
        self.timer_ = self.create_timer(0.5, self.publishData)
        
        # C++コードと同じ配列データ
        self.velocity_1_ = [1000, 1500, 2000, 2500, 3000, 3500, 4000, 3500, 3000, 2500, 2000, 1500]
        self.current_position_index_ = 0
        
        self.get_logger().info('JointPubNode started')
        
    def publishData(self):
        """
        C++コードと同じpublishData関数
        """
        msg = SetPosition()
        
        msg.position = self.velocity_1_[self.current_position_index_]
        msg.id = 1
        
        self.get_logger().info(f"Publishing ID: {msg.id} Position: {msg.position}")
        
        self.publisher_.publish(msg)
        self.current_position_index_ = (self.current_position_index_ + 1) % len(self.velocity_1_)

def main(args=None):
    rclpy.init(args=args)
    node = JointPubNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 