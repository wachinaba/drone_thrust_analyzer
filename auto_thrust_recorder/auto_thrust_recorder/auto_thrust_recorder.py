import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
import numpy as np
import datetime

from auto_thrust_recorder.scheduler.stepwise_scheduler import StepwiseThrustRatioScheduler, StepwiseThrustScheduler, PolynomialModelThrustController, LinearThrustController, ThrustMultiplier
from auto_thrust_recorder.scheduler.breakpoint_scheduler import BreakpointScheduler
from auto_thrust_recorder.logger.average_logger import AverageLogger
from auto_thrust_recorder.logger.raw_logger import RawLogger
from auto_thrust_recorder.exporter.csv_exporter import CSVExporter
from auto_thrust_recorder.px4_bridge.actuator import ActuatorController
from auto_thrust_recorder.sensor_bridge.force_sensor import ForceSensor
from auto_thrust_recorder.sensor_bridge.seven_segment_sensor import FlowSensor
from auto_thrust_recorder.plotter.average_plotter import AveragePlotter

class AutoThrustRecorder(Node):
    def __init__(self, force_sensor: ForceSensor, actuator_controller: ActuatorController, flow_sensor: FlowSensor = None):
        super().__init__("auto_thrust_recorder")
        self.force_sensor = force_sensor
        self.actuator_controller = actuator_controller
        self.flow_sensor = flow_sensor

        # -15deg_fold15deg: y = 110.97x2 + 18.971x + 0.5445
        # -30deg_fold15deg: y = 96.732x2 + 16.219x + 0.7684

        thrust_coefs = {
            "0deg_long": [117.9, 21.811, 0.5403],
            "15deg_long": [113.8, 22.766, 0.6355],
            "30deg_long": [97.806, 21.468, 0.5682],
            "15deg_short": [117.5, 18.492, 0.5346],
            "30deg_short": [91.475, 21.633, 0.4504],
            "0deg_short": [126.67, 14.089, 0.6691],
            "linear": [0.0, 1.0, 0.0],
            "tilt-15deg_fold15deg": [110.97, 18.971, 0.5445],
            "tilt-30deg_fold15deg": [96.732, 16.219, 0.7684],
            "tilt0deg_fold15deg": [124.45, 17.182, 0.6627],
            "tilt8deg_fold15deg": [127.1, 15.612, 0.6906], #127.1x2 + 15.612x + 0.6906
            "tilt15deg_fold15deg": [107.09, 18.039, 0.5855], #107.09x2 + 18.039x + 0.5855
            "tilt23deg_fold15deg": [104.91, 17.476, 0.5441], #104.91x2 + 17.476x + 0.5441
            "tilt30deg_fold15deg": [92.596, 17.961, 0.5213], #92.596x2 + 17.961x + 0.5213
        }

        self.scheduler_params = {
            "step_size": self.declare_parameter("step_size", 0.01).get_parameter_value().double_value,
            "min_thrust": self.declare_parameter("min_thrust", 0.0).get_parameter_value().double_value,
            "max_thrust": self.declare_parameter("max_thrust", 0.4).get_parameter_value().double_value,
            "step_duration": self.declare_parameter("step_duration", 1.0).get_parameter_value().double_value,
            "thrust_coef": self.declare_parameter("thrust_coef", thrust_coefs["tilt8deg_fold15deg"]).get_parameter_value().double_array_value,
        }

        coef_name = self.declare_parameter("coef_name", "").get_parameter_value().string_value
        if coef_name in thrust_coefs:
            self.scheduler_params["thrust_coef"] = thrust_coefs[coef_name]
        elif coef_name == "":
            pass
        else:
            self.get_logger().error(f"Invalid coef_name: {coef_name}")
            self.get_logger().error("Valid coef_names:")
            for coef_name in thrust_coefs:
                self.get_logger().error(f"  - {coef_name}")
            self.get_logger().error("Using tilt0deg_fold15deg as default.")
            self.scheduler_params["thrust_coef"] = thrust_coefs["tilt0deg_fold15deg"]

        self.enable_breakpoint = self.declare_parameter("enable_breakpoint", False).get_parameter_value().bool_value
        
        self.filename_prefix = self.declare_parameter("filename_prefix", "thrust").get_parameter_value().string_value

        self.thrust_multiplier = self.declare_parameter("thrust_multiplier", [1.0, 1.0, 1.0, 1.0]).get_parameter_value().double_array_value

        self.repeat_count = 0
        self.num_repetitions = self.declare_parameter("num_repetitions", 1).get_parameter_value().integer_value

        self.sensor_reversed = self.declare_parameter("sensor_reversed", False).get_parameter_value().bool_value

        self.autoexit = self.declare_parameter("autoexit", True).get_parameter_value().bool_value

        # 7セグメントディスプレイ関連のパラメータ（後方互換性のため残す）
        self.enable_seven_segment = self.declare_parameter("enable_seven_segment", True).get_parameter_value().bool_value

        self.mode = self.declare_parameter("mode", "linear").get_parameter_value().string_value
        if self.mode not in ["polynomial", "linear"]:
            self.get_logger().error(f"Invalid mode: {self.mode}")
            self.get_logger().error("Valid modes: polynomial, linear")
            self.get_logger().error("Using polynomial mode as default.")
            self.mode = "polynomial"

        self.get_logger().info(f"Filename prefix: {self.filename_prefix}")
        self.get_logger().info(f"Mode: {self.mode}")

        if self.mode == "polynomial":            
            self.get_logger().info(f"Step size: {self.scheduler_params['step_size']}")
            self.get_logger().info(f"Min thrust: {self.scheduler_params['min_thrust']}")
            self.get_logger().info(f"Max thrust: {self.scheduler_params['max_thrust']}")
            self.get_logger().info(f"Step duration: {self.scheduler_params['step_duration']}")
            self.get_logger().info(f"Thrust coef: {self.scheduler_params['thrust_coef']}")
        elif self.mode == "linear":
            pass

        self.get_logger().info(f"Thrust multiplier: {self.thrust_multiplier}")
        self.get_logger().info(f"Num repetitions: {self.num_repetitions}")
        self.get_logger().info(f"Enable breakpoint: {self.enable_breakpoint}")
        self.get_logger().info(f"Sensor reversed: {self.sensor_reversed}")
        self.get_logger().info(f"Auto exit: {self.autoexit}")
        self.get_logger().info(f"Enable seven segment: {self.enable_seven_segment}")
        if self.flow_sensor:
            self.get_logger().info("フローセンサーが有効です")
        else:
            self.get_logger().info("フローセンサーは無効です")

        self.initialize_logger()
        self.start_recording()

    def initialize_logger(self):
        self.raw_logger = RawLogger()
        self.average_logger = AverageLogger()
        self.raw_log_exporter = CSVExporter(self.raw_logger, "raw_log.csv")
        self.average_log_exporter = CSVExporter(self.average_logger, "average_log.csv")

        if self.repeat_count == 0:
            self.repetition_average_logger = AverageLogger()
            self.repetition_average_log_exporter = CSVExporter(self.repetition_average_logger, "repetition_average_log.csv")

    def start_recording(self):
        if self.mode == "polynomial":
            #"""
            self.scheduler = BreakpointScheduler(
            node = self,
            step_size = self.scheduler_params["step_size"],
            min_thrust = self.scheduler_params["min_thrust"],
            max_thrust = self.scheduler_params["max_thrust"],
            step_duration = self.scheduler_params["step_duration"],
            thrust_controller = ThrustMultiplier(
                base_controller = PolynomialModelThrustController(
                    node = self,
                    thrust_coef = self.scheduler_params["thrust_coef"],
                ),
                multiplier = np.array(self.thrust_multiplier),
            ),
            on_breakpoint_callback = self.breakpoint_callback
            )
            #"""
        else:
            # 30deg_short = 91.475x2 + 21.633x + 0.4504
            # 15deg_short = 117.5x2 + 18.492x + 0.5346
            # 0deg_short = 119.92x2 + 18.022x + 0.5599

            # 0deg_short_20250202 = 126.95x2 + 15.215x + 0.6733
            # 0deg_short_20250206 = 126.67x2 + 14.089x + 0.6691
            self.scheduler = BreakpointScheduler(
            node = self,
            step_size = self.scheduler_params["step_size"],
            min_thrust = self.scheduler_params["min_thrust"],
            max_thrust = self.scheduler_params["max_thrust"],
            step_duration = self.scheduler_params["step_duration"],
            thrust_controller = ThrustMultiplier(
                base_controller = LinearThrustController(
                    thrust_coef = [1.0, 0.0],
                ),
                multiplier = np.array(self.thrust_multiplier),
            ),
            on_breakpoint_callback = self.breakpoint_callback
            )
        self.disarming_timer = None
        self.disarming_thrust = 0.0
        self.start_timer = self.create_timer(1.0, self.perform_start_sequence)

    def perform_start_sequence(self):
        self.start_timer.cancel()
        self.start_timer = None

        rate = self.create_rate(30)
        timeout = 300
        timeout_counter = 0

        wait = self.create_rate(0.3)
        wait.sleep()
        wait.destroy()

        self.get_logger().info("Setting sensor offset...")
        future = self.force_sensor.set_sensor_offset()
        while not future.done():
            rate.sleep()
            timeout_counter += 1
            if timeout_counter > timeout:
                self.get_logger().error("sensor offset setting timeout...")
                break
        self.get_logger().info("Sensor offset set.")
        timeout_counter = 0

        self.get_logger().info("Setting offboard mode...")
        future = self.actuator_controller.set_offboard_mode()
        while not future.done():
            rate.sleep()
            timeout_counter += 1
            if timeout_counter > timeout:
                self.get_logger().error("offboard mode setting timeout...")
                break
        self.get_logger().info("Offboard mode set.")
        timeout_counter = 0

        self.get_logger().info("Arming...")
        future = self.actuator_controller.set_arming(True)
        while not future.done():
            rate.sleep()
            timeout_counter += 1
            if timeout_counter > timeout:
                self.get_logger().error("arming timeout...")
                break
        self.get_logger().info("Arming complete.")
        timeout_counter = 0
        rate.destroy()

        # wait for 1 second
        rate = self.create_rate(1)
        rate.sleep()
        rate.destroy()

        self.get_logger().info("Start recording...")

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.raw_log_exporter = CSVExporter(self.raw_logger, f"{self.filename_prefix}_raw_{timestamp}.csv")
        self.average_log_exporter = CSVExporter(self.average_logger, f"{self.filename_prefix}_average_{timestamp}.csv")
        if self.repeat_count == 0:
            self.repetition_average_log_exporter = CSVExporter(self.repetition_average_logger, f"{self.filename_prefix}_repetitionaverage_{timestamp}.csv")

        self.scheduler.set_on_change_thrust(self.change_thrust_callback)
        self.scheduler.set_on_complete_callback(self.complete_callback)

        self.scheduler.initialize()
        self.actuator_controller.set_target_control(self.scheduler.get_current_control())

        self.force_sensor.set_on_sensor_update(self.sensor_update_callback)
        
        # フローセンサーのコールバックを設定
        if self.flow_sensor:
            self.flow_sensor.set_on_sensor_update(self.flow_sensor_update_callback)
    
    def change_thrust_callback(self):
        self.average_logger.next()
        self.repetition_average_logger.next()
        current_control = self.scheduler.get_current_control()

        self.actuator_controller.set_target_control(current_control)
        self.get_logger().info(f"Thrust changed to {current_control}")

    def complete_callback(self):
        self.average_logger.next()
        self.repetition_average_logger.next()
        self.force_sensor.set_on_sensor_update(None)
        
        # フローセンサーのコールバックを停止
        if self.flow_sensor:
            self.flow_sensor.set_on_sensor_update(None)

        self.get_logger().info("Complete recording...")
        try:
            if self.raw_log_exporter:
                self.raw_log_exporter.export()
            if self.average_log_exporter:
                self.average_log_exporter.export()
        except Exception as e:
            self.get_logger().error(f"Error exporting logs: {e}")

        self.get_logger().info("Disarming...")
        self.disarming_thrust = self.scheduler.get_current_control()
        self.disarming_timer = self.create_timer(0.05, self.disarming_callback)

    def plot(self):
        plotter = AveragePlotter(self.average_logger.get_data())
        plotter.plot()
        plotter.save(f"{self.average_log_exporter.file_path}.png")
        plotter.show()

    def disarming_callback(self):
        self.get_logger().info(f"Disarming thrust: {self.disarming_thrust.max()}")
        if self.disarming_thrust.max() < 0.01:
            self.disarming_timer.cancel()
            rate = self.create_rate(30)
            timeout = 300
            timeout_counter = 0

            future = self.actuator_controller.set_arming(False)
            while not future.done():
                rate.sleep()
                timeout_counter += 1
                if timeout_counter > timeout:
                    self.get_logger().error("disarming timeout...")
                    break
            rate.destroy()

            self.get_logger().info("Disarming complete.")
            self.disarming_timer = None
            #self.plot()
            self.raw_log_exporter = None
            self.average_log_exporter = None
            self.repeat_count += 1
            self.get_logger().info(f"Finish recording {self.repeat_count} / {self.num_repetitions} ...")

            if self.repeat_count < self.num_repetitions:
                self.get_logger().info(f"Repeating {self.repeat_count} / {self.num_repetitions} ...")
                self.initialize_logger()
                self.start_recording()
            else:
                self.get_logger().info("Finished all repetitions.")
                if self.repetition_average_log_exporter:
                    self.repetition_average_log_exporter.export() 
                    self.repetition_average_log_exporter = None
                self.perform_shutdown()
            return
        
        self.actuator_controller.set_target_control(self.disarming_thrust)
        self.actuator_controller.update_control()

        self.disarming_thrust -= 0.01
        self.disarming_thrust = np.clip(self.disarming_thrust, 0.0, 0.5)

    def breakpoint_callback(self):
        if not self.enable_breakpoint:
            return

        self.get_logger().info("Breakpoint reached...")
        current_control = self.scheduler.get_current_control()
        self.get_logger().info(f"Current control: {current_control}")

        rate = self.create_rate(30)
        timeout = 300
        timeout_counter = 0

        while current_control.mean() > 0.0:
            rate.sleep()
            current_control -= 0.01
            self.actuator_controller.set_target_control(current_control)
            self.actuator_controller.update_control()

        self.get_logger().info("Breakpoint complete.")

        future = self.actuator_controller.set_arming(False)
        while not future.done():
            rate.sleep()
            timeout_counter += 1
            if timeout_counter > timeout:
                self.get_logger().error("disarming timeout...")
                break
        self.get_logger().info("Disarming complete.")
        timeout_counter = 0

        # プロペラが止まるまで待つ
        wait = self.create_rate(1.0)
        wait.sleep()
        wait.destroy()

        future = self.force_sensor.set_sensor_offset()
        while not future.done():
            rate.sleep()
            timeout_counter += 1
            if timeout_counter > timeout:
                self.get_logger().error("sensor offset setting timeout...")
                break
        self.get_logger().info("Sensor offset set.")
        timeout_counter = 0

        self.get_logger().info("Resuming...")
        future = self.actuator_controller.set_arming(True)
        while not future.done():
            rate.sleep()
            timeout_counter += 1
            if timeout_counter > timeout:
                self.get_logger().error("arming timeout...")
                break
        self.get_logger().info("Arming complete.")
        timeout_counter = 0

        rate.destroy()
        rate = self.create_rate(60.0)

        while current_control.mean() < self.scheduler.get_current_control().mean():
            rate.sleep()
            current_control += 0.01
            self.actuator_controller.set_target_control(current_control)
            self.actuator_controller.update_control()

        # プロペラが定常になるまで待つ
        wait = self.create_rate(1.0)
        wait.sleep()
        wait.destroy()

        rate.destroy()
        self.get_logger().info("Resuming complete.")
        return
    
    def flow_sensor_update_callback(self, data, timestamp, valid_count):
        """フローセンサーのデータ更新コールバック"""
        self.get_logger().debug(f"フローセンサーデータ更新: {valid_count}個の有効値")
    
    def sensor_update_callback(self, msg: WrenchStamped):
        if not self.scheduler.ready_to_record() or self.actuator_controller.is_moving_control():
            return
        
        current_control = self.actuator_controller.get_current_control().tolist()
        average_control = self.actuator_controller.get_current_control().mean()

        row = {
            "time": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "control_0": current_control[0],
            "control_1": current_control[1],
            "control_2": current_control[2],
            "control_3": current_control[3],
            "control": average_control,
            "target_thrust": self.scheduler.get_current_thrust(),
            "force_x": msg.wrench.force.x,
            "force_y": msg.wrench.force.y,
            "force_z": msg.wrench.force.z,
            "torque_x": msg.wrench.torque.x,
            "torque_y": msg.wrench.torque.y,
            "torque_z": msg.wrench.torque.z,
        }
        
        # フローセンサーのデータを追加
        if self.flow_sensor:
            wind_speed_data = self.flow_sensor.get_wind_speed_data()
            row.update(wind_speed_data)
        else:
            # フローセンサーが無効またはデータがない場合はNaNで埋める
            max_elements = self.declare_parameter("max_elements", 4).get_parameter_value().integer_value
            for i in range(max_elements):
                row[f"seven_segment_value_{i}"] = float('nan')
            row["seven_segment_count"] = 0
            row["seven_segment_timestamp"] = float('nan')
        if self.sensor_reversed:
            row["force_x"] = -row["force_x"]
            row["force_y"] = -row["force_y"]
            row["torque_x"] = -row["torque_x"]
            row["torque_y"] = -row["torque_y"]
            
        self.raw_logger.log(row)
        self.repetition_average_logger.log(row)
        self.average_logger.log(row)

    def perform_shutdown(self):
        self.get_logger().info("Disarming...")
        self.actuator_controller.set_arming(False)
        
        if self.autoexit:
            self.get_logger().info("All measurements completed. Auto exit enabled. Shutting down...")
            rclpy.try_shutdown()
        else:
            self.get_logger().info("All measurements completed. Auto exit disabled. Node will continue running.")
        

def main():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=3)

    actuator_controller = ActuatorController()
    force_sensor = ForceSensor()
    flow_sensor = FlowSensor()
    
    auto_thrust_recorder = AutoThrustRecorder(force_sensor, actuator_controller, flow_sensor)

    executor.add_node(auto_thrust_recorder)
    executor.add_node(actuator_controller)
    executor.add_node(force_sensor)
    executor.add_node(flow_sensor)

    try:
        executor.spin()
    except KeyboardInterrupt:
        auto_thrust_recorder.get_logger().info("KeyboardInterrupt, disarming and shutting down...")
    finally:
        executor.shutdown()
        
        auto_thrust_recorder.perform_shutdown()

        actuator_controller.destroy_node()
        force_sensor.destroy_node()
        flow_sensor.destroy_node()
        auto_thrust_recorder.destroy_node()

        rclpy.try_shutdown()

if __name__ == "__main__":
    main()