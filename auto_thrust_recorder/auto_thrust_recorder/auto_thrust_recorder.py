import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger

import datetime

from auto_thrust_recorder.scheduler.stepwise_scheduler import StepwiseThrustRatioScheduler, StepwiseThrustScheduler, PolynomialModelThrustController
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
            "0deg_short": [119.92, 18.022, 0.5599],
        }

        if True: 
            #"""
            self.scheduler = StepwiseThrustScheduler(
            node = self,
            step_size = self.declare_parameter("step_size", 0.5).get_parameter_value().double_value,
            min_thrust = self.declare_parameter("min_thrust", 16.0).get_parameter_value().double_value,
            max_thrust = self.declare_parameter("max_thrust", 20.0).get_parameter_value().double_value,
            step_duration = self.declare_parameter("step_duration", 3.0).get_parameter_value().double_value,
            thrust_controller = PolynomialModelThrustController(
                node = self,
                thrust_coef = self.declare_parameter("thrust_coef", thrust_coefs["0deg_short"]).get_parameter_value().double_array_value,
            )
            )
            #"""
        else:
            # 30deg_short = 91.475x2 + 21.633x + 0.4504
            # 15deg_short = 117.5x2 + 18.492x + 0.5346
            # 0deg_short = 119.92x2 + 18.022x + 0.5599
            self.scheduler = StepwiseThrustRatioScheduler(
            node = self,
            step_size = self.declare_parameter("step_size", 0.01).get_parameter_value().double_value,
            min_thrust = self.declare_parameter("min_thrust", 0.0).get_parameter_value().double_value,
            max_thrust = self.declare_parameter("max_thrust", 0.4).get_parameter_value().double_value,
            step_duration = self.declare_parameter("step_duration", 1.0).get_parameter_value().double_value,
            )
        

        self.filename_prefix = self.declare_parameter("filename_prefix", "thrust").get_parameter_value().string_value

        self.raw_logger = RawLogger()
        self.average_logger = AverageLogger()
        self.raw_log_exporter = CSVExporter(self.raw_logger, "raw_log.csv")
        self.average_log_exporter = CSVExporter(self.average_logger, "average_log.csv")

        self.countdown = 10
        self.countdown_timer = self.create_timer(1.0, self.countdown_callback)
        self.disarming_timer = None
        self.disarming_thrust = 0.0

    def countdown_callback(self):
        if self.countdown == 10:
            self.get_logger().info("Starting recording...")

            self.get_logger().info(f"Filename prefix: {self.filename_prefix}")
            self.get_logger().info(f"Step size: {self.scheduler.step_size}")
            self.get_logger().info(f"Min thrust: {self.scheduler.min_thrust}")
            self.get_logger().info(f"Max thrust: {self.scheduler.max_thrust}")
            self.get_logger().info(f"Step duration: {self.scheduler.step_duration}")

            self.get_logger().info("Setting sensor offset...")
            self.force_sensor.set_sensor_offset()
            self.actuator_controller.set_target_control(self.scheduler.get_current_control())
        if self.countdown == 5:
            self.get_logger().info("Setting offboard mode...")
            self.actuator_controller.set_offboard_mode()
            self.get_logger().warning("Arming...")
            self.actuator_controller.set_arming(True)
        if self.countdown > 0:
            self.get_logger().info(f"Starting in {self.countdown} seconds...")
            self.countdown -= 1
        else:
            self.countdown_timer.cancel()
            self.countdown = 10
            self.perform_start_sequence()

    def perform_start_sequence(self):
        self.get_logger().info("Start recording...")

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.raw_log_exporter = CSVExporter(self.raw_logger, f"{self.filename_prefix}_raw_{timestamp}.csv")
        self.average_log_exporter = CSVExporter(self.average_logger, f"{self.filename_prefix}_average_{timestamp}.csv")

        self.scheduler.set_on_change_thrust(self.change_thrust_callback)
        self.scheduler.set_on_complete_callback(self.complete_callback)

        self.scheduler.initialize()
        self.actuator_controller.set_target_control(self.scheduler.get_current_control())

        self.force_sensor.set_on_sensor_update(self.sensor_update_callback)
    
    def change_thrust_callback(self):
        self.average_logger.next()
        current_control = self.scheduler.get_current_control()
        self.actuator_controller.set_target_control(current_control)
        self.get_logger().info(f"Thrust changed to {current_control}")

    def complete_callback(self):
        self.average_logger.next()

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
        
        self.raw_log_exporter = None
        self.average_log_exporter = None

        self.get_logger().info("Finish recording...")

    def disarming_callback(self):
        self.get_logger().info(f"Disarming thrust: {self.disarming_thrust.mean()}")
        if self.disarming_thrust.mean() < 0.0:
            self.disarming_timer.cancel()
            self.actuator_controller.set_arming(False)
            self.get_logger().info("Disarming complete.")
            self.disarming_timer = None
            self.plot()
            return
        
        self.actuator_controller.set_target_control(self.disarming_thrust)
        self.actuator_controller.update_control()

        self.disarming_thrust -= 0.01


    def sensor_update_callback(self, msg: WrenchStamped):
        if self.actuator_controller.is_moving_control():
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
        self.average_logger.log(row)

    def perform_shutdown(self):
        self.get_logger().info("Disarming...")
        self.actuator_controller.set_arming(False)
        

def main():
    rclpy.init()
    executor = SingleThreadedExecutor()      

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