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
import matplotlib.pyplot as plt  # Matplotlibのインポート

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

        # 新しいCSVファイル用の変数
        self.avg_csv_writer = None
        self.avg_csv_file = None
        self.accumulated_data = []
        self.data_count = 0

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
        
        # パラメータのログ出力
        self.get_logger().info(f"max_thrust_percent: {self.max_thrust_percent}")
        self.get_logger().info(f"thrust_step: {self.thrust_step}")
        self.get_logger().info(f"pause_duration: {self.pause_duration}")
        self.get_logger().info(f"recording_time_increment: {self.recording_time_increment}")
        self.get_logger().info(f"control_smoothing_factor: {self.control_smoothing_factor}")

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
            current_time = self.get_clock().now().nanoseconds / 1e9
            row = [
                current_time,  # Timestamp
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
                *self.current_control_.tolist()
            ]
            self.csv_writer.writerow(row)

            # データを累積して平均用に保存
            self.accumulated_data.append([
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
                *self.current_control_.tolist()
            ])
            self.data_count += 1

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
                self.get_logger().warn(f'Command result: {response.reply.result}')
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

        # Create primary CSV file
        self.file_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.log_filename_prefix}_{self.file_timestamp}.csv"
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time', 'force_x', 'force_y', 'force_z',
            'torque_x', 'torque_y', 'torque_z',
            'motor_0', 'motor_1', 'motor_2', 'motor_3'
        ])

        # Create average CSV file
        avg_filename = f"{self.log_filename_prefix}_average_{self.file_timestamp}.csv"
        self.avg_csv_file = open(avg_filename, mode='w', newline='')
        self.avg_csv_writer = csv.writer(self.avg_csv_file)
        self.avg_csv_writer.writerow([
            'thrust_step', 'avg_force_x', 'avg_force_y', 'avg_force_z',
            'avg_torque_x', 'avg_torque_y', 'avg_torque_z',
            'avg_motor_0', 'avg_motor_1', 'avg_motor_2', 'avg_motor_3'
        ])

        # Save average CSV filename for later use
        self.avg_csv_filename = avg_filename

        # Initialize accumulation variables
        self.accumulated_data = []
        self.data_count = 0

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

            # Calculate and write average for the previous thrust step
            if self.data_count > 0:
                avg_data = np.mean(self.accumulated_data, axis=0)
                self.avg_csv_writer.writerow([self.current_thrust - self.thrust_step] + avg_data.tolist())
                self.get_logger().info(f'Average data for thrust step {self.current_thrust - self.thrust_step:.2f} recorded.')
                
                # リセット
                self.accumulated_data = []
                self.data_count = 0

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

        # Write average for the last thrust step if any data is present
        if self.data_count > 0 and self.avg_csv_writer:
            avg_data = np.mean(self.accumulated_data, axis=0)
            self.avg_csv_writer.writerow([self.current_thrust] + avg_data.tolist())
            self.get_logger().info(f'Average data for thrust step {self.current_thrust:.2f} recorded.')

        # Close CSV files
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f'CSV file written: {self.csv_file.name}')
        if self.avg_csv_file and not self.avg_csv_file.closed:
            self.avg_csv_file.close()
            self.get_logger().info(f'CSV file written: {self.avg_csv_file.name}')
        # Plot the collected data
        self.plot_data()

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

    def plot_data(self):
        """Read the average CSV and plot thrust vs force and torque in a single figure with dual y-axes for Force XY and Z."""
        try:
            # Initialize dictionaries to store data
            avg_data = {
                'thrust_step': [],
                'avg_force_x': [],
                'avg_force_y': [],
                'avg_force_z': [],
                'avg_torque_x': [],
                'avg_torque_y': [],
                'avg_torque_z': [],
                'avg_motor_0': [],
                'avg_motor_1': [],
                'avg_motor_2': [],
                'avg_motor_3': []
            }

            # Read the average CSV file
            with open(self.avg_csv_filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    avg_data['thrust_step'].append(float(row['thrust_step']) * 100) # パーセントに変換
                    avg_data['avg_force_x'].append(float(row['avg_force_x']))
                    avg_data['avg_force_y'].append(float(row['avg_force_y']))
                    avg_data['avg_force_z'].append(float(row['avg_force_z']))
                    avg_data['avg_torque_x'].append(float(row['avg_torque_x']))
                    avg_data['avg_torque_y'].append(float(row['avg_torque_y']))
                    avg_data['avg_torque_z'].append(float(row['avg_torque_z']))
                    # Motor data can be processed similarly if needed

            # Create a figure with 2 vertically aligned subplots
            fig, axs = plt.subplots(2, 1, figsize=(12, 14), sharex=True)

            # 1つ目のサブプロット：Thrust vs Force XY and Force Z (Dual y-axes)
            ax1 = axs[0]
            ax2 = ax1.twinx()  # 右側のy軸を作成

            # Force XY
            ax1.plot(avg_data['thrust_step'], avg_data['avg_force_x'], label='Force X', color='r')
            ax1.plot(avg_data['thrust_step'], avg_data['avg_force_y'], label='Force Y', color='g')
            ax1.set_ylabel('Force XY (N)', color='k')
            ax1.tick_params(axis='y', labelcolor='k')
            ax1.set_ylim([-2, 2])

            # Force Z
            ax2.plot(avg_data['thrust_step'], avg_data['avg_force_z'], label='Force Z', color='b')
            ax2.set_ylabel('Force Z (N)', color='b')
            ax2.tick_params(axis='y', labelcolor='b')
            ax2.set_ylim([0, 30])

            # タイトルと凡例
            ax1.set_title('Thrust vs Force XY and Force Z')
            ax1.legend(loc='upper left')
            ax2.legend(loc='upper right')
            ax1.grid(True)

            # 2つ目のサブプロット：Thrust vs Torque X, Y, Z
            axs[1].plot(avg_data['thrust_step'], avg_data['avg_torque_x'], label='Torque X', color='r')
            axs[1].plot(avg_data['thrust_step'], avg_data['avg_torque_y'], label='Torque Y', color='g')
            axs[1].plot(avg_data['thrust_step'], avg_data['avg_torque_z'], label='Torque Z', color='b')
            axs[1].set_xlabel('Thrust (%)')
            axs[1].set_ylabel('Torque (Nm)')
            axs[1].set_title('Thrust vs Torque X, Y, Z')
            axs[1].legend()
            axs[1].grid(True)
            axs[1].set_ylim([-1, 1])

            # レイアウトを調整
            plt.tight_layout()

            # グラフを表示（または保存）
            plt.show()

            fig.savefig(f"{self.log_filename_prefix}_plots_{self.file_timestamp}.png")
            self.get_logger().info(f'Plots saved as {self.log_filename_prefix}_plots_{self.file_timestamp}.png')

        except Exception as e:
            self.get_logger().error(f'Failed to plot data: {e}')

    def disarm_on_shutdown(self):
        """Disarm the vehicle when the node is shutting down."""
        self.set_arming(False)
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
        if self.avg_csv_file and not self.avg_csv_file.closed:
            self.avg_csv_file.close()

    def perform_shutdown(self):
        """Handle node shutdown."""
        self.disarm_on_shutdown()

def main(args=None):
    """Main function to start the node."""
    rclpy.init(args=args)
    node = PropCoefFinderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, disarming and shutting down...")
    finally:
        node.perform_shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
