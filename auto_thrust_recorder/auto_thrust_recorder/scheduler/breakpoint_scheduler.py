# breakpoint_scheduler.py

from rclpy.node import Node
from auto_thrust_recorder.scheduler.scheduler import Scheduler
from rclpy.duration import Duration
import numpy as np
from typing import Callable

class BreakpointScheduler(Scheduler):
    """
    スラストを変更する際にブレークを設けるスケジューラ。
    ブレークポイントに達すると、コールバック関数を呼び出す。
    コールバック関数が戻ると、次のスラストに進む。
    """

    def __init__(self, node: Node, step_size: float, min_thrust: float, max_thrust: float, step_duration: float, thrust_controller: Callable[[float], np.ndarray], on_breakpoint_callback: Callable[[], None]):
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

        self.ready = False
        self.on_breakpoint_callback = on_breakpoint_callback

    def initialize(self):
        self.current_thrust = self.min_thrust
        
        if self.timer:
            self.timer.cancel()
        
        self.timer = self.node.create_timer(self.step_duration, self.step_callback)
        self.node.get_logger().info("BreakpointScheduler initialized and timer started.")
        self.node.get_logger().info(f"BreakpointScheduler: Thrust increased to {self.current_thrust:.2f}")
        self.ready = True

    def step_callback(self):
        if self.timer:
            self.timer.cancel()
            self.timer = None
        self.ready = False

        if self.current_thrust < self.max_thrust:
            if (self.on_breakpoint_callback):
                self.on_breakpoint_callback()

            self.current_thrust += self.step_size
            self.current_thrust = min(self.current_thrust, self.max_thrust)

            if self.change_callback:
                self.change_callback()

            self.timer = self.node.create_timer(self.step_duration, self.step_callback)
            self.ready = True
        else:
            self.node.get_logger().info(f"BreakpointScheduler: Max thrust reached. Finalizing scheduler. {self.timer}")
            self.finalize()

    def finalize(self):
        self.ready = False
        if self.timer:
            self.timer.cancel()
            self.timer = None
        if self.change_callback:
            self.change_callback()
        if self.complete_callback:
            self.complete_callback()
        self.node.get_logger().info("BreakpointScheduler finalized.")
        
    def set_on_change_thrust(self, callback: Callable[[np.ndarray], None]):
        self.change_callback = callback

    def set_on_complete_callback(self, callback: Callable[[], None]):
        self.complete_callback = callback

    def get_current_control(self) -> np.ndarray:
        return self.thrust_controller(self.current_thrust)

    def ready_to_record(self) -> bool:
        return self.ready
