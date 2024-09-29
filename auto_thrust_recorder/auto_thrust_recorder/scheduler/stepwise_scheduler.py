# stepwise_scheduler.py

from rclpy.node import Node
from auto_thrust_recorder.scheduler.scheduler import Scheduler
from rclpy.duration import Duration
import numpy as np
from typing import Callable

class StepwiseThrustRatioScheduler(Scheduler):
    """Scheduler that incrementally increases motor commands in steps."""

    def __init__(self, node: Node, step_size: float, min_thrust: float, max_thrust: float, step_duration: float):
        """
        Args:
            node (Node): ROS2ノード。タイマーの作成に使用。
            step_size (float): 各ステップでスラストを増加させる量（0.0〜1.0）。
            max_thrust (float): 最大スラスト値（0.0〜1.0）。
            step_duration (float): 各ステップの持続時間（秒）。
        """
        self.node = node
        self.step_size = step_size
        self.min_thrust = min_thrust
        self.max_thrust = max_thrust
        self.step_duration = step_duration
        self.current_thrust = 0.0
        self.change_callback = None
        self.complete_callback = None
        self.timer = None

    def initialize(self): # override
        """Initialize the scheduler and start the timer."""
        self.current_thrust = self.min_thrust
        if self.timer:
            self.timer.cancel()
        self.timer = self.node.create_timer(self.step_duration, self.step_callback)
        self.node.get_logger().info("StepwiseScheduler initialized and timer started.")
        self.node.get_logger().info(f"StepwiseScheduler: Thrust increased to {self.current_thrust:.2f}")

    def step_callback(self):
        """Timer callback to increment thrust and invoke the control callback."""
        if self.current_thrust < self.max_thrust:
            self.current_thrust += self.step_size
            self.current_thrust = min(self.current_thrust, self.max_thrust)
            self.node.get_logger().info(f"StepwiseScheduler: Thrust increased to {self.current_thrust:.2f}")
            if self.change_callback:
                self.change_callback()
        else:
            self.node.get_logger().info("StepwiseScheduler: Max thrust reached. Finalizing scheduler.")
            self.finalize()

    def finalize(self): # override
        """Finalize the scheduler by stopping the timer."""
        if self.timer:
            self.timer.cancel()
            self.timer = None
        self.current_thrust = 0.0
        if self.change_callback:
            self.change_callback()  # 全てのモータを停止
        if self.complete_callback:
            self.complete_callback()
        self.node.get_logger().info("StepwiseScheduler finalized.")

    def set_on_change_thrust(self, callback: Callable[[], None]):
        self.change_callback = callback

    def set_on_complete_callback(self, callback: Callable[[], None]):
        self.complete_callback = callback

    def get_current_control(self) -> np.ndarray: # override
        return np.array([self.current_thrust] * 4)


class StepwiseThrustScheduler(Scheduler):
    """Scheduler that incrementally increases motor commands in steps."""

    def __init__(self, node: Node, step_size: float, min_thrust: float, max_thrust: float, step_duration: float, thrust_controller: Callable[[float], np.ndarray]):
        """
        Args:
            node (Node): ROS2ノード。タイマーの作成に使用。
            step_size (float): 各ステップでスラストを増加させる量（0.0〜1.0）。
            max_thrust (float): 最大スラスト値（0.0〜1.0）。
            step_duration (float): 各ステップの持続時間（秒）。
            thrust_controller (Callable[[float], np.ndarray]): [N]単位の目標力を受け取り、その力を生成するための制御信号を返す関数。
        """
        self.node = node
        self.step_size = step_size
        self.min_thrust = min_thrust
        self.max_thrust = max_thrust
        self.step_duration = step_duration
        self.thrust_controller = thrust_controller
        self.current_thrust = 0.0
        self.change_callback = None
        self.complete_callback = None
        self.timer = None

    def initialize(self): # override
        """Initialize the scheduler and start the timer."""
        self.current_thrust = self.min_thrust
        if self.timer:
            self.timer.cancel()
        
        self.timer = self.node.create_timer(self.step_duration, self.step_callback)
        self.node.get_logger().info("StepwiseScheduler initialized and timer started.")
        self.node.get_logger().info(f"StepwiseScheduler: Thrust increased to {self.current_thrust:.2f}")

    def step_callback(self):
        """Timer callback to increment thrust and invoke the control callback."""
        if self.current_thrust < self.max_thrust:
            self.current_thrust += self.step_size
            self.current_thrust = min(self.current_thrust, self.max_thrust)
            control = self.thrust_controller(self.current_thrust)
            if self.change_callback:
                self.change_callback(control)
        else:
            self.node.get_logger().info("StepwiseScheduler: Max thrust reached. Finalizing scheduler.")
            self.finalize()

    def finalize(self): # override
        """Finalize the scheduler by stopping the timer."""
        if self.timer:
            self.timer.cancel()
            self.timer = None
        if self.change_callback:
            self.change_callback()  # 全てのモータを停止
        if self.complete_callback:
            self.complete_callback()
        self.node.get_logger().info("StepwiseScheduler finalized.")

    def set_on_change_thrust(self, callback: Callable[[np.ndarray], None]):
        self.change_callback = callback

    def set_on_complete_callback(self, callback: Callable[[], None]):
        self.complete_callback = callback

    def get_current_control(self) -> np.ndarray: # override
        return self.thrust_controller(self.current_thrust)


class PolynomialModelThrustController:
    def __init__(self, node: Node, thrust_coef: list[float]):
        self.node = node
        self.thrust_coef = thrust_coef

    def calculate_thrust(self, thrust: float) -> float:
        return (-self.thrust_coef[1] + np.sqrt(self.thrust_coef[1]**2 - 4 * self.thrust_coef[0] * (self.thrust_coef[2] - thrust))) / (2 * self.thrust_coef[0])

    def __call__(self, target: np.ndarray) -> np.ndarray:
        return np.array([self.calculate_thrust(f) for f in target])

