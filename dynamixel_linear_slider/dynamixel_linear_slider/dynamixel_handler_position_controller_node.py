#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Bool
import math
import time
from dynamixel_handler_msgs.msg import DxlStates, DxlCommandsX

class DynamixelHandlerPositionControllerNode(Node):
    def __init__(self):
        super().__init__('dynamixel_handler_position_controller_node')

        # パラメータの取得
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('rack_pitch', 0.106214)
        self.declare_parameter('gear_ratio', 1.0)
        
        # 収束判定パラメータ
        self.declare_parameter('convergence_threshold', 0.002)  # 位置偏差の閾値 (m)
        self.declare_parameter('convergence_duration', 1.0)    # 収束判定に必要な持続時間 (秒)
        self.declare_parameter('velocity_zero_threshold', 0.01) # velocityコマンドのゼロ判定閾値 (m/s)
        self.declare_parameter('enable_convergence_brake', True) # 収束ブレーキの有効/無効
        
        # 移動状態判定パラメータ
        self.declare_parameter('movement_velocity_threshold', 0.005) # 移動判定の速度閾値 (m/s)
        self.declare_parameter('movement_position_threshold', 0.001) # 移動判定の位置偏差閾値 (m)
        
        self.control_frequency = self.get_parameter('control_frequency').value
        self.rack_pitch = self.get_parameter('rack_pitch').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        
        # 収束判定パラメータ
        self.convergence_threshold = self.get_parameter('convergence_threshold').value
        self.convergence_duration = self.get_parameter('convergence_duration').value
        self.velocity_zero_threshold = self.get_parameter('velocity_zero_threshold').value
        self.enable_convergence_brake = self.get_parameter('enable_convergence_brake').value
        
        # 移動状態判定パラメータ
        self.movement_velocity_threshold = self.get_parameter('movement_velocity_threshold').value
        self.movement_position_threshold = self.get_parameter('movement_position_threshold').value
        
        # サブスクライバーの設定
        self.estimated_position_subscription = self.create_subscription(
            Float64,
            'estimated_position',
            self.estimated_position_callback,
            10
        )
        
        self.target_position_subscription = self.create_subscription(
            Float64,
            'target_position',
            self.target_position_callback,
            10
        )
        
        self.feedforward_velocity_subscription = self.create_subscription(
            Float64,
            'feedforward_velocity',
            self.feedforward_velocity_callback,
            10
        )

        self.dynamixel_present_state_subscription = self.create_subscription(
            DxlStates,
            '/dynamixel/states',
            self.dynamixel_present_state_callback,
            10
        )
        
        # velocityコマンドの監視用サブスクライバー
        self.velocity_command_subscription = self.create_subscription(
            Float64,
            'velocity_command',
            self.velocity_command_callback,
            10
        )
        
        # パブリッシャーの設定
        self.dynamixel_command_publisher = self.create_publisher(
            DxlCommandsX,
            '/dynamixel/commands/x',
            10
        )
        
        # 移動状態パブリッシャー（bool型）
        self.movement_status_publisher = self.create_publisher(
            Bool,
            'movement_status',
            10
        )

        # 制御用の変数
        self.estimated_position = 0.0
        self.target_position = 0.0
        self.feedforward_velocity = 0.0
        self.velocity_command = 0.0  # velocityコマンドの監視用

        self.current_dynamixel_position_deg = 0.0

        self.m_to_deg_factor = 360.0 / self.rack_pitch * self.gear_ratio

        # データの有効性フラグ
        self.estimated_position_valid = False
        self.target_position_valid = False
        self.feedforward_velocity_valid = False
        self.dynamixel_present_state_valid = False
        self.velocity_command_valid = False
        
        # 収束判定用の変数
        self.convergence_start_time = None
        self.is_converged = False
        self.brake_applied = False
        
        # 移動状態管理
        self.is_moving = False
        
        # タイマーの設定
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_timer_callback)
        
        self.get_logger().info('DynamixelHandlerPositionControllerNode initialized')
        self.get_logger().info(f'Convergence threshold: {self.convergence_threshold} m')
        self.get_logger().info(f'Convergence duration: {self.convergence_duration} s')
        self.get_logger().info(f'Velocity zero threshold: {self.velocity_zero_threshold} m/s')
        self.get_logger().info(f'Enable convergence brake: {self.enable_convergence_brake}')
        self.get_logger().info(f'Movement velocity threshold: {self.movement_velocity_threshold} m/s')
        self.get_logger().info(f'Movement position threshold: {self.movement_position_threshold} m')
        
        
    def estimated_position_callback(self, msg):
        self.estimated_position = msg.data
        self.estimated_position_valid = True
    
    def target_position_callback(self, msg):
        self.target_position = msg.data
        self.target_position_valid = True
    
    def feedforward_velocity_callback(self, msg):
        self.feedforward_velocity = msg.data
        self.feedforward_velocity_valid = True
    
    def velocity_command_callback(self, msg):
        """velocityコマンドのコールバック"""
        self.velocity_command = msg.data
        self.velocity_command_valid = True
    
    def dynamixel_present_state_callback(self, msg):
        present_state = msg.present
        if len(present_state.id_list) > 0:
            self.current_dynamixel_position_deg = present_state.position_deg[0]
            self.dynamixel_present_state_valid = True

    def calculate_position_error(self):
        if not self.estimated_position_valid or not self.target_position_valid or not self.dynamixel_present_state_valid:
            return 0.0
        
        error = self.target_position - self.estimated_position
        return error
    
    def publish_movement_status(self):
        """移動状態をパブリッシュ"""
        status_msg = Bool()
        status_msg.data = self.is_moving
        
        self.movement_status_publisher.publish(status_msg)
        
        """
        # デバッグ用ログ（状態変化時のみ出力）
        if self.is_moving:
            self.get_logger().info("Movement status: MOVING")
        else:
            self.get_logger().info("Movement status: STOPPED")
        """
    
    def check_convergence(self):
        """収束判定"""
        if not self.enable_convergence_brake:
            return False
        
        # 位置偏差の計算
        position_error = abs(self.calculate_position_error())
        
        # velocityコマンドのゼロ判定
        velocity_is_zero = abs(self.velocity_command) <= self.velocity_zero_threshold
        
        # 収束条件の確認
        if position_error <= self.convergence_threshold and velocity_is_zero:
            # 初回収束検出時
            if self.convergence_start_time is None:
                self.convergence_start_time = time.time()
                self.get_logger().info(f'Convergence detected: position_error={position_error:.6f}m, velocity={self.velocity_command:.6f}m/s')
            
            # 収束持続時間の確認
            elif time.time() - self.convergence_start_time >= self.convergence_duration:
                if not self.is_converged:
                    self.is_converged = True
                    self.get_logger().info(f'Position converged for {self.convergence_duration}s: position_error={position_error:.6f}m, velocity={self.velocity_command:.6f}m/s')
                return True
        else:
            # 収束条件を満たさない場合、リセット
            if self.convergence_start_time is not None:
                self.convergence_start_time = None
                self.is_converged = False
                self.get_logger().debug(f'Convergence reset: position_error={position_error:.6f}m, velocity={self.velocity_command:.6f}m/s')
        
        return False
    
    def control_timer_callback(self):
        if not self.estimated_position_valid or not self.target_position_valid or not self.feedforward_velocity_valid or not self.dynamixel_present_state_valid:
            return
        
        # 収束判定
        self.is_moving = not self.check_convergence()
        self.publish_movement_status()

        if not self.is_moving:
            self.apply_brake()
            return
        
        # 通常の制御処理
        position_error = self.calculate_position_error()
        degree_error = position_error * self.m_to_deg_factor

        feedforward_velocity_deg_s = self.feedforward_velocity * self.m_to_deg_factor

        self.publish_position_command(degree_error, feedforward_velocity_deg_s)
        
    def publish_position_command(self, position_error, feedforward_velocity_deg_s):
        dxl_msg = DxlCommandsX()
        dxl_msg.extended_position_control.id_list = [1]
        dxl_msg.extended_position_control.position_deg = [position_error + self.current_dynamixel_position_deg]

        velocity_control = math.fabs(feedforward_velocity_deg_s)

        # 少し余裕を持たせる
        velocity_control_compensated = velocity_control * 1.3

        if (velocity_control_compensated < 20.0):
            velocity_control_compensated = 20.0
        
        dxl_msg.extended_position_control.profile_vel_deg_s = [velocity_control_compensated]

        self.dynamixel_command_publisher.publish(dxl_msg)
        
    def apply_brake(self):
        """ブレーキ処理の実行"""
        self.publish_brake_command()
        #self.get_logger().info('Brake applied due to position convergence')
        
    def publish_brake_command(self):
        """ブレーキコマンドの送信"""
        dxl_msg = DxlCommandsX()
        dxl_msg.velocity_control.id_list = [1]
        dxl_msg.velocity_control.velocity_deg_s = [0.0]
        dxl_msg.velocity_control.profile_acc_deg_ss = []
        self.dynamixel_command_publisher.publish(dxl_msg)
        #self.get_logger().info('Brake command sent: velocity=0.0 deg/s')

        
def main(args=None):
    rclpy.init(args=args)
    node = DynamixelHandlerPositionControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()