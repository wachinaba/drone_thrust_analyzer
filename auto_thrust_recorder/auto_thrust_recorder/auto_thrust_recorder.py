import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger

import datetime

from auto_thrust_recorder.scheduler.stepwise_scheduler import StepwiseThrustRatioScheduler
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
        self.scheduler = StepwiseThrustRatioScheduler(
            node = self,
            step_size = self.declare_parameter("step_size", 0.02).get_parameter_value().double_value,
            min_thrust = self.declare_parameter("min_thrust", 0.1).get_parameter_value().double_value,
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
        self.force_sensor.set_on_sensor_update(None)

        self.get_logger().info("Complete recording...")
        self.get_logger().info("Disarming...")
        self.actuator_controller.set_arming(False)

        plotter = AveragePlotter(self.average_logger.get_data())
        plotter.plot()
        plotter.show()
        plotter.save(f"{self.average_log_exporter.file_path}.png")

        if self.raw_log_exporter:
            self.raw_log_exporter.export()
        if self.average_log_exporter:
            self.average_log_exporter.export()
        self.raw_log_exporter = None
        self.average_log_exporter = None

        self.get_logger().info("Finish recording...")


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