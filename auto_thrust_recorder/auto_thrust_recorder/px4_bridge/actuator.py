# px4_controller.py

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, ActuatorMotors, ActuatorServos
from px4_msgs.srv import VehicleCommand as VehicleCommandSrv
from px4_msgs.msg import VehicleCommandAck, VehicleCommand
from std_srvs.srv import SetBool
import numpy as np

class ActuatorController(Node):
    """Handles PX4-related functionalities such as motor commands and mode changes."""

    def __init__(self):
        super().__init__('px4_controller')

        # Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.actuator_motors_publisher_ = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', 10)
        self.actuator_servos_publisher_ = self.create_publisher(
            ActuatorServos, '/fmu/in/actuator_servos', 10)

        # Service Clients
        self.vehicle_command_client_ = self.create_client(
            VehicleCommandSrv, '/fmu/vehicle_command')
        while not self.vehicle_command_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /fmu/vehicle_command service...')

        # Control Variables
        self.current_control_ = np.zeros(4)
        self.target_control_ = np.zeros(4)

        self.moving_control_ = False

        # Parameters
        self.max_thrust_percent = self.declare_parameter(
            'max_thrust_percent', 50.0).get_parameter_value().double_value
        self.control_smoothing_factor = self.declare_parameter(
            'control_smoothing_factor', 0.1).get_parameter_value().double_value
        
        self.get_logger().info(f"max_thrust_percent: {self.max_thrust_percent}")
        self.get_logger().info(f"control_smoothing_factor: {self.control_smoothing_factor}")

        self.update_timer = self.create_timer(
            1 / self.declare_parameter(
                'update_rate', 100.0).get_parameter_value().double_value,
            self.update_control
            )

    def set_offboard_mode(self):
        """Set the vehicle to offboard control mode."""
        self.request_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0
        )

    def set_arming(self, arm: bool):
        """Arm or disarm the vehicle."""
        self.request_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0 if arm else 0.0
        )

    def request_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Send a vehicle command request."""
        request = VehicleCommandSrv.Request()
        request.request.timestamp = self.get_clock().now().nanoseconds // 1000
        request.request.param1 = param1
        request.request.param2 = param2
        request.request.command = command
        request.request.target_system = 1
        request.request.target_component = 1
        request.request.source_system = 1
        request.request.source_component = 1
        request.request.from_external = True

        future = self.vehicle_command_client_.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """Handle responses from vehicle command requests."""
        try:
            response = future.result()
            if response.reply.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self.get_logger().info('Vehicle command accepted.')
            else:
                self.get_logger().warn(f'Vehicle command result: {response.reply.result}')
        except Exception as e:
            self.get_logger().error(f'Failed to call vehicle command service: {e}')

    def set_target_control(self, thrust_percent: float | np.ndarray):
        """Set the target thrust percentage for the vehicle."""
        if isinstance(thrust_percent, float):
            self.target_control_ = np.array([thrust_percent, thrust_percent, thrust_percent, thrust_percent])
        elif isinstance(thrust_percent, np.ndarray) and thrust_percent.shape == (4,):
            self.target_control_ = thrust_percent
        else:
            raise ValueError("Invalid thrust percentage input. Expected a float or a 4-element numpy array.")

    def update_control(self):
        """Smoothly interpolate current control towards target control."""
        for i in range(4):
            self.current_control_[i] = min(
                max(
                    self.current_control_[i] * (1 - self.control_smoothing_factor) +
                    self.target_control_[i] * self.control_smoothing_factor, 0.0
                ),
                self.max_thrust_percent / 100.0
            )
            if abs(self.current_control_[i] - self.target_control_[i]) < 0.002:
                self.current_control_[i] = self.target_control_[i]
                self.moving_control_ = False
            else:
                self.moving_control_ = True

    def is_moving_control(self) -> bool:
        return self.moving_control_

    def get_current_control(self) -> np.ndarray:
        return self.current_control_
    
    def get_target_control(self) -> np.ndarray:
        return self.target_control_

    def publish_actuator_messages(self):
        """Publish OffboardControlMode, ActuatorMotors, and ActuatorServos."""
        timestamp = self.get_clock().now().nanoseconds // 1000

        # OffboardControlMode message
        ocm_msg = OffboardControlMode()
        ocm_msg.timestamp = timestamp
        ocm_msg.position = False
        ocm_msg.velocity = False
        ocm_msg.acceleration = False
        ocm_msg.attitude = False
        ocm_msg.thrust_and_torque = False
        ocm_msg.direct_actuator = True
        self.offboard_control_mode_publisher_.publish(ocm_msg)

        # ActuatorMotors message
        am_msg = ActuatorMotors()
        am_msg.timestamp = timestamp
        am_msg.reversible_flags = 0
        am_msg.control = self.current_control_.tolist() + [0.0] * 8  # total 12 control signals
        self.actuator_motors_publisher_.publish(am_msg)

        # ActuatorServos message
        as_msg = ActuatorServos()
        as_msg.timestamp = timestamp
        as_msg.control = [0.0] * 8  # Assuming 8 servos
        self.actuator_servos_publisher_.publish(as_msg)
