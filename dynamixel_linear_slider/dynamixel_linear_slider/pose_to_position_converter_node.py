#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


class PoseToPositionConverterNode(Node):
    """
    PoseStampedメッセージからFloat64メッセージへの変換ノード
    
    機能:
    - estimator_nodeからのPoseStampedメッセージを受信
    - スライダ軸方向の位置成分を抽出
    - Float64メッセージとして出力（sensor_fusion_nodeとの互換性のため）
    """
    
    def __init__(self):
        super().__init__('pose_to_position_converter_node')
        
        # パラメータの取得
        self.declare_parameter('position_axis', 'x')  # 位置を抽出する軸 (x, y, z)
        self.declare_parameter('scale_factor', -1.0)  # スケール係数（メートル単位への変換）
        
        self.position_axis = self.get_parameter('position_axis').value
        self.scale_factor = self.get_parameter('scale_factor').value
        
        # サブスクライバーの設定
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            'estimator_node/output/slider_pose',
            self.pose_callback,
            10
        )
        
        # パブリッシャーの設定
        self.position_publisher = self.create_publisher(
            Float64,
            'ar_marker_position',
            10
        )
        
        self.get_logger().info('PoseToPositionConverterNode initialized')
        self.get_logger().info(f'Position axis: {self.position_axis}, Scale factor: {self.scale_factor}')
    
    def pose_callback(self, msg):
        """PoseStampedメッセージのコールバック"""
        try:
            # 指定された軸の位置を抽出
            if self.position_axis == 'x':
                position = msg.pose.position.x
            elif self.position_axis == 'y':
                position = msg.pose.position.y
            elif self.position_axis == 'z':
                position = msg.pose.position.z
            else:
                self.get_logger().error(f'Invalid position axis: {self.position_axis}')
                return
            
            # スケール係数を適用
            scaled_position = position * self.scale_factor
            
            # Float64メッセージを作成してパブリッシュ
            position_msg = Float64()
            position_msg.data = scaled_position
            self.position_publisher.publish(position_msg)
            
            self.get_logger().debug(f'Converted position: {scaled_position:.4f} m')
            
        except Exception as e:
            self.get_logger().error(f'Error in pose callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = PoseToPositionConverterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 