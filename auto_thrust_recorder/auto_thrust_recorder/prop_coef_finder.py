import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, ActuatorMotors, ActuatorServos
from px4_msgs.srv import VehicleCommand as VehicleCommandSrv
from px4_msgs.msg import VehicleCommand as VehicleCommandMsg
from px4_msgs.msg import VehicleCommandAck
from std_srvs.srv import SetBool, Trigger, Empty
from geometry_msgs.msg import WrenchStamped
import numpy as np
import csv
import datetime

class PropCoefFinderNode(Node):
    """ROS2 Node to incrementally increase thrust and log wrench data."""

    def __init__(self):
        super().__init__('prop_coef_finder')

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
        
        self.command_publish_rate_ = self.declare_parameter('command_publish_rate', 100.0).get_parameter_value().double_value

        # Timers
        self.timer_ = self.create_timer(1.0 / self.command_publish_rate_, self.timer_callback)
        self.thrust_timer = None  # Timer for incremental thrust increase
        self.pause_timer = None   # One-shot timer for pause processing

        # Control Variables
        self.current_control_ = np.zeros(4)
        self.target_control_ = np.zeros(4)

        # Services
        self.set_arming_service_ = self.create_service(
            SetBool, '~/set_arming', self.set_arming_callback)
        self.start_service_ = self.create_service(
            Trigger, '~/start', self.start_callback)
        self.stop_service_ = self.create_service(
            Trigger, '~/stop', self.stop_callback)

        # Subscribers
        self.wrench_subscriber_ = self.create_subscription(
            WrenchStamped, '/force_sensor_node/data', self.wrench_callback, 10)
        
        # Clients
        self.set_offset_client_ = self.create_client(Empty, '/force_sensor_node/set_offset')

        # Logging Variables
        self.logging_active = False
        self.pause_logging = False
        self.csv_writer = None
        self.csv_file = None
        self.log_filename_prefix = self.declare_parameter(
            'log_filename_prefix', 'thrust').get_parameter_value().string_value
        self.max_thrust_percent = self.declare_parameter(
            'max_thrust_percent', 40.0).get_parameter_value().double_value
        self.thrust_step = self.declare_parameter(
            'thrust_step', 0.01).get_parameter_value().double_value
        self.pause_duration = self.declare_parameter(
            'pause_duration', 0.2).get_parameter_value().double_value
        self.recording_time_increment = self.declare_parameter(
            'recording_time_increment', 1.0).get_parameter_value().double_value
        self.control_smoothing_factor = self.declare_parameter(
            'control_smoothing_factor', 0.1).get_parameter_value().double_value

        self.current_thrust = 0.0
        self.logging_start_time = None
        self.thrust_increment_timer = None

    def timer_callback(self):
        """Regular timer callback to update control and publish messages."""
        self.update_control()
        self.publish_actuator_messages()

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
        am_msg.control = self.current_control_.tolist() + [0.0] * 8 # total 12 control signals
        self.actuator_motors_publisher_.publish(am_msg)

        # ActuatorServos message
        as_msg = ActuatorServos()
        as_msg.timestamp = timestamp
        as_msg.control = [0.0] * 8  # Assuming 8 servos
        self.actuator_servos_publisher_.publish(as_msg)

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

    def wrench_callback(self, msg):
        """Callback to handle wrench data and write to CSV if logging is active."""
        if self.logging_active and not self.pause_logging and self.csv_writer:
            row = [
                self.get_clock().now().nanoseconds / 1e9,  # Timestamp
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
                *self.current_control_.tolist()
            ]
            self.csv_writer.writerow(row)

    def set_arming_callback(self, request, response):
        """Service callback to arm or disarm the vehicle."""
        self.set_offboard_mode()
        self.set_arming(request.data)
        response.success = True
        response.message = 'Arming status set.'
        return response

    def set_arming(self, arm):
        """Send arming command to the vehicle."""
        self.request_vehicle_command(
            VehicleCommandMsg.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0 if arm else 0.0
        )

    def set_offboard_mode(self):
        """Set the vehicle to offboard control mode."""
        self.request_vehicle_command(
            VehicleCommandMsg.VEHICLE_CMD_DO_SET_MODE,
            1.0, 6.0
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
                self.get_logger().info('Command accepted')
            else:
                self.get_logger().warn(f'Command result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def set_sensor_offset(self):
        """Service callback to set the force sensor offset."""
        self.get_logger().info('Setting force sensor offset')
        self.set_offset_client_.call_async(Empty.Request())
        return

    def start_callback(self, request, response):
        """Service callback to start the recording and thrust increase."""
        self.get_logger().info('Start recording')

        # Start a non-blocking countdown
        self.countdown = 10
        self.countdown_timer = self.create_timer(1.0, self.countdown_callback)

        response.success = True
        response.message = 'Recording started.'
        return response

    def countdown_callback(self):
        """Countdown before arming and starting the thrust increase."""
        if self.countdown == 10:
            self.current_thrust = 0.0
            self.target_control_ = np.zeros(4)
            self.current_control_ = np.zeros(4)
            self.set_sensor_offset()
            
        elif self.countdown == 5:
            self.get_logger().info('Setting offboard mode')
            self.set_offboard_mode()
            self.get_logger().info('Arming...')
            self.set_arming(True)

        if self.countdown > 0:
            self.get_logger().info(f"Starting in {self.countdown} seconds...")
            self.countdown -= 1
        else:
            self.countdown_timer.cancel()
            self.perform_start_sequence()

    def perform_start_sequence(self):
        """Initialize variables and start the thrust increase sequence."""
        self.logging_active = True
        self.logging_start_time = self.get_clock().now().nanoseconds / 1e9

        # Create CSV file
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.log_filename_prefix}_{timestamp}.csv"
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time', 'force_x', 'force_y', 'force_z',
            'torque_x', 'torque_y', 'torque_z',
            'motor_0', 'motor_1', 'motor_2', 'motor_3'
        ])

        # Start the thrust increment timer
        self.thrust_timer = self.create_timer(
            self.recording_time_increment, self.increase_thrust)
        
        self.get_logger().info('Starting thrust increase sequence, initial thrust: 0.0')

    def increase_thrust(self):
        """Increase the thrust incrementally."""
        max_thrust = self.max_thrust_percent / 100.0
        if self.current_thrust < max_thrust:
            self.current_thrust += self.thrust_step
            self.current_thrust = min(self.current_thrust, max_thrust)
            self.target_control_[:] = self.current_thrust
            self.get_logger().info(f'Thrust increased to {self.current_thrust:.2f}')

            # Pause logging for the specified duration
            self.pause_logging = True
            if self.pause_timer:
                self.pause_timer.cancel()
            self.pause_timer = self.create_timer(self.pause_duration, self.end_pause)
        else:
            self.stop_logging()

    def end_pause(self):
        """Resume logging after pause."""
        self.get_logger().info('Resuming logging after pause')
        self.pause_logging = False
        if self.pause_timer:
            self.pause_timer.cancel()

    def stop_logging(self):
        """Stop logging data and disarm the vehicle."""
        self.get_logger().info('Stop recording and disarm')
        self.logging_active = False
        self.target_control_ = np.zeros(4)
        self.set_arming(False)

        # Close CSV file
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()

        # Cancel timers
        if self.thrust_timer:
            self.thrust_timer.cancel()
        if self.pause_timer:
            self.pause_timer.cancel()

    def stop_callback(self, request, response):
        """Service callback to stop the recording and disarm."""
        self.get_logger().warn('Stop triggered! Stopping recording and disarming.')
        self.stop_logging()
        response.success = True
        response.message = 'Recording stopped.'
        return response

    def disarm_on_shutdown(self):
        """Disarm the vehicle when the node is shutting down."""
        self.set_arming(False)
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()

def main(args=None):
    """Main function to start the node."""
    rclpy.init(args=args)
    node = PropCoefFinderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt, disarming and shutting down...")
        pass
    finally:
        node.disarm_on_shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
