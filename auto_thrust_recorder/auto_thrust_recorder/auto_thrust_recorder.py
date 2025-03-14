import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger

import datetime

from auto_thrust_recorder.scheduler.stepwise_scheduler import StepwiseThrustRatioScheduler, StepwiseThrustScheduler, PolynomialModelThrustController, LinearThrustController
from auto_thrust_recorder.scheduler.breakpoint_scheduler import BreakpointScheduler
from auto_thrust_recorder.logger.average_logger import AverageLogger
from auto_thrust_recorder.logger.raw_logger import RawLogger
from auto_thrust_recorder.exporter.csv_exporter import CSVExporter
from auto_thrust_recorder.px4_bridge.actuator import ActuatorController
from auto_thrust_recorder.sensor_bridge.force_sensor import ForceSensor
from auto_thrust_recorder.plotter.average_plotter import AveragePlotter

class AutoThrustRecorder(Node):
    def __init__(self, force_sensor: ForceSensor, actuator_controller: ActuatorController):
        super().__init__("auto_thrust_recorder")
        self.force_sensor = force_sensor
        self.actuator_controller = actuator_controller

        thrust_coefs = {
            "0deg_long": [117.9, 21.811, 0.5403],
            "15deg_long": [113.8, 22.766, 0.6355],
            "30deg_long": [97.806, 21.468, 0.5682],
            "15deg_short": [117.5, 18.492, 0.5346],
            "30deg_short": [91.475, 21.633, 0.4504],
            "0deg_short": [126.67, 14.089, 0.6691],
            "linear": [0.0, 1.0, 0.0],
            "tilt0deg_fold15deg": [124.45, 17.182, 0.6627],
            "tilt8deg_fold15deg": [127.1, 15.612, 0.6906] #127.1x2 + 15.612x + 0.6906
        }

        self.scheduler_params = {
            "step_size": self.declare_parameter("step_size", 0.5).get_parameter_value().double_value,
            "min_thrust": self.declare_parameter("min_thrust", 16.0).get_parameter_value().double_value,
            "max_thrust": self.declare_parameter("max_thrust", 20.0).get_parameter_value().double_value,
            "step_duration": self.declare_parameter("step_duration", 3.0).get_parameter_value().double_value,
            "thrust_coef": self.declare_parameter("thrust_coef", thrust_coefs["tilt8deg_fold15deg"]).get_parameter_value().double_array_value,
        }

        self.enable_breakpoint = self.declare_parameter("enable_breakpoint", True).get_parameter_value().bool_value
        
        self.filename_prefix = self.declare_parameter("filename_prefix", "thrust").get_parameter_value().string_value

        self.repeat_count = 0
        self.num_repetitions = self.declare_parameter("num_repetitions", 1).get_parameter_value().integer_value

        self.get_logger().info(f"Filename prefix: {self.filename_prefix}")
        self.get_logger().info(f"Step size: {self.scheduler_params['step_size']}")
        self.get_logger().info(f"Min thrust: {self.scheduler_params['min_thrust']}")
        self.get_logger().info(f"Max thrust: {self.scheduler_params['max_thrust']}")
        self.get_logger().info(f"Step duration: {self.scheduler_params['step_duration']}")
        self.get_logger().info(f"Thrust coef: {self.scheduler_params['thrust_coef']}")
        self.get_logger().info(f"Num repetitions: {self.num_repetitions}")

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
        if True: 
            #"""
            self.scheduler = BreakpointScheduler(
            node = self,
            step_size = self.scheduler_params["step_size"],
            min_thrust = self.scheduler_params["min_thrust"],
            max_thrust = self.scheduler_params["max_thrust"],
            step_duration = self.scheduler_params["step_duration"],
            thrust_controller = PolynomialModelThrustController(
                node = self,
                thrust_coef = self.scheduler_params["thrust_coef"],
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
            thrust_controller = LinearThrustController(
                node = self,
                thrust_coef = [1.0, 0.0],
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
        self.get_logger().info(f"Disarming thrust: {self.disarming_thrust.mean()}")
        if self.disarming_thrust.mean() < 0.0:
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
    
    def sensor_update_callback(self, msg: WrenchStamped):
        if not self.scheduler.ready_to_record() or not self.actuator_controller.is_moving_control():
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
            "force_x": msg.wrench.force.x,
            "force_y": msg.wrench.force.y,
            "force_z": msg.wrench.force.z,
            "torque_x": msg.wrench.torque.x,
            "torque_y": msg.wrench.torque.y,
            "torque_z": msg.wrench.torque.z,
        }
        self.raw_logger.log(row)
        self.repetition_average_logger.log(row)
        self.average_logger.log(row)

    def perform_shutdown(self):
        self.get_logger().info("Disarming...")
        self.actuator_controller.set_arming(False)
        

def main():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=3)

    actuator_controller = ActuatorController()
    force_sensor = ForceSensor() 
    
    auto_thrust_recorder = AutoThrustRecorder(force_sensor, actuator_controller)

    executor.add_node(auto_thrust_recorder)
    executor.add_node(actuator_controller)
    executor.add_node(force_sensor)

    try:
        executor.spin()
    except KeyboardInterrupt:
        auto_thrust_recorder.get_logger().info("KeyboardInterrupt, disarming and shutting down...")
    finally:
        executor.shutdown()
        
        auto_thrust_recorder.perform_shutdown()

        actuator_controller.destroy_node()
        force_sensor.destroy_node()
        auto_thrust_recorder.destroy_node()

        rclpy.try_shutdown()

if __name__ == "__main__":
    main()