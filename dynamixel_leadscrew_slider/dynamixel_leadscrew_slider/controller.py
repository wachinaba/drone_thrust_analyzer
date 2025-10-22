#!/usr/bin/env python3

import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_srvs.srv import Trigger

from dynamixel_handler_msgs.msg import DxlCommandsX, DxlStates

from dynamixel_leadscrew_slider_msgs.msg import SliderState, MovePositionMm
from dynamixel_leadscrew_slider_msgs.srv import SetPositionMm


def mm_to_counts(mm: float, pitch: float, cpr: int) -> int:
    rev = mm / pitch
    return int(round(rev * cpr))


def counts_to_mm(counts: int, pitch: float, cpr: int) -> float:
    return (counts / cpr) * pitch


class LeadscrewSliderController(Node):
    def __init__(self) -> None:
        super().__init__('leadscrew_slider_controller')

        # Parameters
        self.declare_parameter('motor.id', 1)
        self.declare_parameter('mechanics.pitch_mm_per_rev', 10.0)
        self.declare_parameter('mechanics.counts_per_rev', 4096)
        self.declare_parameter('limits.soft_min_mm', float('nan'))
        self.declare_parameter('limits.soft_max_mm', float('nan'))
        self.declare_parameter('homing.origin_reference', 'center')
        self.declare_parameter('homing.origin_offset_mm', 0.0)
        self.declare_parameter('homing.return_to_position_mm', 0.0)
        self.declare_parameter('homing.seek_current_ma', 150)
        self.declare_parameter('homing.hold_time_ms', 150)
        self.declare_parameter('homing.backoff_mm', 1.0)
        self.declare_parameter('homing.timeout_s', 30.0)
        # Backoff enhancement params
        self.declare_parameter('homing.backoff_current_ma', 200)
        self.declare_parameter('homing.backoff_profile_vel_mm_s', 20.0)
        self.declare_parameter('homing.backoff_profile_acc_mm_s2', 200.0)
        # Seek ramp-up params
        self.declare_parameter('homing.seek_current_ma_max', 400)
        self.declare_parameter('homing.seek_current_ma_step', 50)
        self.declare_parameter('homing.seek_ramp_stage_ms', 0.5 * 1000.0)
        self.declare_parameter('homing.move_min_mm', 0.2)
        self.declare_parameter('profile.max_vel_mm_s', 50.0)
        self.declare_parameter('profile.max_acc_mm_s2', 500.0)
        # Stall detection / travel guard
        self.declare_parameter('homing.stall_vel_threshold_mm_s', 0.05)
        self.declare_parameter('homing.stall_hold_time_s', 0.25)
        self.declare_parameter('homing.max_travel_mm', 300.0)
        self.declare_parameter('homing.max_seek_time_s', 15.0)
        self.declare_parameter('homing.move_vel_threshold_mm_s', 0.2)
        self.declare_parameter('homing.backoff_timeout_s', 3.0)
        self.declare_parameter('homing.return_to_zero_timeout_s', 5.0)
        self.declare_parameter('homing.return_to_zero_tolerance_mm', 0.05)
        self.declare_parameter('homing.return_to_zero_monitor', False)
        # Debug
        self.declare_parameter('debug.log_states', True)
        self.declare_parameter('debug.log_states_period_s', 0.5)
        self.declare_parameter('debug.states_watchdog_s', 1.0)

        # Resolve
        self.motor_id: int = int(self.get_parameter('motor.id').get_parameter_value().integer_value)
        self.pitch: float = self.get_parameter('mechanics.pitch_mm_per_rev').get_parameter_value().double_value
        self.cpr: int = int(self.get_parameter('mechanics.counts_per_rev').get_parameter_value().integer_value)
        self.origin_reference: str = self.get_parameter('homing.origin_reference').get_parameter_value().string_value
        self.origin_offset_mm: float = self.get_parameter('homing.origin_offset_mm').get_parameter_value().double_value
        self.return_to_position_mm: float = self.get_parameter('homing.return_to_position_mm').get_parameter_value().double_value
        self.seek_current_mA: int = int(self.get_parameter('homing.seek_current_ma').get_parameter_value().integer_value)
        self.hold_time_ms: int = int(self.get_parameter('homing.hold_time_ms').get_parameter_value().integer_value)
        self.backoff_mm: float = self.get_parameter('homing.backoff_mm').get_parameter_value().double_value
        self.timeout_s: float = self.get_parameter('homing.timeout_s').get_parameter_value().double_value
        self.max_vel_mm_s: float = self.get_parameter('profile.max_vel_mm_s').get_parameter_value().double_value
        self.max_acc_mm_s2: float = self.get_parameter('profile.max_acc_mm_s2').get_parameter_value().double_value
        self.backoff_current_ma: int = int(self.get_parameter('homing.backoff_current_ma').get_parameter_value().integer_value)
        self.backoff_profile_vel_mm_s: float = self.get_parameter('homing.backoff_profile_vel_mm_s').get_parameter_value().double_value
        self.backoff_profile_acc_mm_s2: float = self.get_parameter('homing.backoff_profile_acc_mm_s2').get_parameter_value().double_value
        self.seek_current_ma_max: int = int(self.get_parameter('homing.seek_current_ma_max').get_parameter_value().integer_value)
        self.seek_current_ma_step: int = int(self.get_parameter('homing.seek_current_ma_step').get_parameter_value().integer_value)
        self.seek_ramp_stage_ms: float = self.get_parameter('homing.seek_ramp_stage_ms').get_parameter_value().double_value
        self.move_min_mm: float = self.get_parameter('homing.move_min_mm').get_parameter_value().double_value
        self.stall_vel_threshold_mm_s: float = self.get_parameter('homing.stall_vel_threshold_mm_s').get_parameter_value().double_value
        self.stall_hold_time_s: float = self.get_parameter('homing.stall_hold_time_s').get_parameter_value().double_value
        self.max_travel_mm: float = self.get_parameter('homing.max_travel_mm').get_parameter_value().double_value
        self.max_seek_time_s: float = self.get_parameter('homing.max_seek_time_s').get_parameter_value().double_value
        self.move_vel_threshold_mm_s: float = self.get_parameter('homing.move_vel_threshold_mm_s').get_parameter_value().double_value
        self.backoff_timeout_s: float = self.get_parameter('homing.backoff_timeout_s').get_parameter_value().double_value
        self.return_to_zero_timeout_s: float = self.get_parameter('homing.return_to_zero_timeout_s').get_parameter_value().double_value
        self.return_to_zero_tolerance_mm: float = self.get_parameter('homing.return_to_zero_tolerance_mm').get_parameter_value().double_value
        self.return_to_zero_monitor: bool = self.get_parameter('homing.return_to_zero_monitor').get_parameter_value().bool_value
        self.log_states: bool = self.get_parameter('debug.log_states').get_parameter_value().bool_value
        self.log_states_period_s: float = self.get_parameter('debug.log_states_period_s').get_parameter_value().double_value
        self.states_watchdog_s: float = self.get_parameter('debug.states_watchdog_s').get_parameter_value().double_value

        # Optional limits may be None
        self.soft_min_mm: Optional[float] = None
        self.soft_max_mm: Optional[float] = None
        vmin = self.get_parameter('limits.soft_min_mm').value
        vmax = self.get_parameter('limits.soft_max_mm').value
        if isinstance(vmin, float) and math.isfinite(vmin):
            self.soft_min_mm = vmin
        if isinstance(vmax, float) and math.isfinite(vmax):
            self.soft_max_mm = vmax

        # State
        self.homed: bool = False
        self.zero_offset_counts: int = 0
        self.min_end_counts: Optional[int] = None
        self.max_end_counts: Optional[int] = None
        self.present_counts: Optional[int] = None
        self.present_velocity_deg_s: Optional[float] = None
        self.present_current_ma: Optional[float] = None
        self._last_states_log_time: Optional[Time] = None
        self._last_states_update_time: Optional[Time] = None

        # Non-blocking move command state (pub/sub move)
        self.mv_active: bool = False
        self.mv_target_counts: Optional[int] = None
        self.mv_vel_mm_s: float = 0.0
        self.mv_acc_mm_s2: float = 0.0
        self.mv_last_send: Optional[Time] = None
        self.mv_last_torque: Optional[Time] = None

        # Pub/Sub (use absolute topic names to avoid namespace mismatch)
        self.pub_cmd = self.create_publisher(DxlCommandsX, '/dynamixel/commands/x', 10)
        self.sub_states = self.create_subscription(DxlStates, '/dynamixel/states', self._on_states, 10)
        self.pub_state = self.create_publisher(SliderState, 'state', 10)
        self.pub_position = self.create_publisher(SliderState, '/current_position', 10)
        self.sub_move = self.create_subscription(MovePositionMm, '/move_mm', self._on_move_mm, 10)

        # Services
        self.srv_home = self.create_service(Trigger, 'home', self._srv_home)
        self.srv_move = self.create_service(SetPositionMm, 'move_mm', self._srv_move_mm)
        self.srv_stop = self.create_service(Trigger, 'stop', self._srv_stop)

        # Periodic state publisher
        self.create_timer(0.05, self._publish_state)
        # Periodic move resend/monitor
        self.create_timer(0.1, self._move_tick)
        self.get_logger().info(
            f"LeadscrewSliderController ready: id={self.motor_id}, pitch={self.pitch} mm/rev, cpr={self.cpr}, "
            f"origin={self.origin_reference}, seek_current_ma={self.seek_current_mA}, backoff_mm={self.backoff_mm}, "
            f"timeout_s={self.timeout_s}, profile(v={self.max_vel_mm_s} mm/s, a={self.max_acc_mm_s2} mm/s^2), "
            f"backoff(current={self.backoff_current_ma} mA, v={self.backoff_profile_vel_mm_s} mm/s, a={self.backoff_profile_acc_mm_s2} mm/s^2), "
            f"seek_ramp(max={self.seek_current_ma_max} mA, step={self.seek_current_ma_step} mA, stage_ms={self.seek_ramp_stage_ms}, move_min_mm={self.move_min_mm}), "
            f"stall(v<= {self.stall_vel_threshold_mm_s} mm/s for {self.stall_hold_time_s}s), guards(max_travel={self.max_travel_mm} mm, max_time={self.max_seek_time_s}s), "
            f"backoff_timeout={self.backoff_timeout_s}s, rtz_timeout={self.return_to_zero_timeout_s}s, rtz_tol={self.return_to_zero_tolerance_mm}mm, rtz_monitor={self.return_to_zero_monitor}, "
            f"origin_offset_mm={self.origin_offset_mm}, return_to_position_mm={self.return_to_position_mm}"
        )

        # ---- State Machine (non-blocking) ----
        self.sm_state: str = 'idle'
        self.sm_state_prev: Optional[str] = None
        self.sm_dir: int = 0
        self.sm_current_level: int = self.seek_current_mA
        self.sm_start_time: Optional[Time] = None
        self.sm_last_cmd_time: Optional[Time] = None
        self.sm_last_move_time: Optional[Time] = None
        self.sm_stage_start: Optional[Time] = None
        self.sm_stage_baseline_counts: Optional[int] = None
        self.sm_moved_once: bool = False
        self.sm_stall_start: Optional[Time] = None
        self.sm_backoff_target: Optional[int] = None
        self.sm_backoff_start: Optional[Time] = None
        self.sm_end_left: Optional[int] = None
        self.sm_end_right: Optional[int] = None
        self.sm_rtz_target_counts: Optional[int] = None
        self.sm_rtz_start: Optional[Time] = None

        # control loop
        self.create_timer(0.01, self._tick)

    # ---- ROS Callbacks ----
    def _on_states(self, msg: DxlStates) -> None:
        try:
            now = self.get_clock().now()
            found = False
            if msg.present.id_list:
                for i, sid in enumerate(msg.present.id_list):
                    if sid == self.motor_id:
                        pos_deg = msg.present.position_deg[i]
                        vel_deg_s = msg.present.velocity_deg_s[i]
                        cur_mA = msg.present.current_ma[i]
                        # Handler側が多回転位置を提供する前提で、そのままcountsへ変換
                        counts = int(round((pos_deg / 360.0) * self.cpr))
                        self.present_counts = counts
                        self.present_velocity_deg_s = vel_deg_s
                        self.present_current_ma = cur_mA
                        found = True
                        break
            # Throttled debug log
            if self.log_states:
                should_log = False
                if self._last_states_log_time is None:
                    should_log = True
                else:
                    dt = (now - self._last_states_log_time).nanoseconds * 1e-9
                    if dt >= self.log_states_period_s:
                        should_log = True
                if should_log:
                    ids = list(msg.present.id_list)
                    pd = list(msg.present.position_deg[:min(3, len(msg.present.position_deg))])
                    vd = list(msg.present.velocity_deg_s[:min(3, len(msg.present.velocity_deg_s))])
                    cm = list(msg.present.current_ma[:min(3, len(msg.present.current_ma))])
                    if found:
                        self.get_logger().info(
                            f"states: ids={ids} sample_deg={pd} sample_vel={vd} sample_cur={cm} | target_id={self.motor_id} pos_deg={pos_deg:.3f} counts={self.present_counts} vel_deg_s={self.present_velocity_deg_s:.3f} cur_mA={self.present_current_ma:.1f}"
                        )
                    else:
                        self.get_logger().warn(
                            f"states: ids={ids} sample_deg={pd} | target_id={self.motor_id} not found"
                        )
                    self._last_states_log_time = now
            # update last states timestamp
            self._last_states_update_time = now
        except Exception as e:
            self.get_logger().warn(f'states parse error: {e}')

    def _publish_state(self) -> None:
        msg = SliderState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.homed = self.homed
        if self.present_counts is not None:
            rel_counts = self.present_counts - self.zero_offset_counts
            msg.position_counts = rel_counts
            msg.position_mm = counts_to_mm(rel_counts, self.pitch, self.cpr)
        else:
            msg.position_counts = 0
            msg.position_mm = 0.0
        msg.velocity_mm_s = 0.0
        if self.present_velocity_deg_s is not None:
            # deg/s -> rev/s -> mm/s
            rev_s = self.present_velocity_deg_s / 360.0
            msg.velocity_mm_s = rev_s * self.pitch
        msg.current_ma = float(self.present_current_ma) if self.present_current_ma is not None else 0.0
        self.pub_state.publish(msg)
        # 現在位置も別トピックでパブリッシュ
        self.pub_position.publish(msg)

    # ---- Services ----
    def _srv_home(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.get_logger().info("Homing requested")
        self._start_homing()
        response.success = True
        response.message = 'Homing started'
        return response

    def _srv_move_mm(self, request: SetPositionMm.Request, response: SetPositionMm.Response) -> SetPositionMm.Response:
        if not self.homed:
            response.accepted = False
            response.message = 'Not homed'
            return response
        target_mm = request.target_mm
        self.get_logger().info(
            f"move_mm request: target_mm={target_mm}, profile_vel_mm_s={request.profile_vel_mm_s}, "
            f"profile_acc_mm_s2={request.profile_acc_mm_s2}"
        )
        # Apply soft limit
        if self.soft_min_mm is not None and target_mm < self.soft_min_mm:
            response.accepted = False
            response.message = 'Target below soft_min_mm'
            return response
        if self.soft_max_mm is not None and target_mm > self.soft_max_mm:
            response.accepted = False
            response.message = 'Target above soft_max_mm'
            return response
        # Send extended position goal (deg + rotation)
        self._command_extended_position_mm(target_mm, request.profile_vel_mm_s, request.profile_acc_mm_s2)
        response.accepted = True
        response.message = 'ok'
        return response

    def _srv_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        # Zero velocity is not enough if current control is active; torque off for safety
        self.get_logger().info("Stop requested: torque off")
        cmd = DxlCommandsX()
        cmd.status.id_list = [self.motor_id]
        cmd.status.torque = [False]
        self.pub_cmd.publish(cmd)
        # reset state machine to idle
        self.sm_state = 'idle'
        # cancel pub/sub move resend so we don't re-enable torque
        self.mv_active = False
        self.mv_target_counts = None
        response.success = True
        response.message = 'Stopped (torque off)'
        return response

    def _on_move_mm(self, msg: MovePositionMm) -> None:
        """Pub/sub形式の移動指示コールバック"""
        if not self.homed:
            self.get_logger().warn("Move command ignored: not homed")
            return
        
        target_mm = msg.target_mm
        profile_vel_mm_s = msg.profile_vel_mm_s
        profile_acc_mm_s2 = msg.profile_acc_mm_s2
        
        self.get_logger().info(
            f"move_mm topic: target_mm={target_mm}, profile_vel_mm_s={profile_vel_mm_s}, "
            f"profile_acc_mm_s2={profile_acc_mm_s2}"
        )
        
        # Apply soft limit
        if self.soft_min_mm is not None and target_mm < self.soft_min_mm:
            self.get_logger().warn(f"Move command ignored: target {target_mm} below soft_min_mm {self.soft_min_mm}")
            return
        if self.soft_max_mm is not None and target_mm > self.soft_max_mm:
            self.get_logger().warn(f"Move command ignored: target {target_mm} above soft_max_mm {self.soft_max_mm}")
            return
        # Prepare non-blocking resend/monitor
        self._torque(True)
        target_counts = mm_to_counts(target_mm, self.pitch, self.cpr) + self.zero_offset_counts
        self.mv_target_counts = int(target_counts)
        self.mv_vel_mm_s = float(profile_vel_mm_s if profile_vel_mm_s > 0 else self.max_vel_mm_s)
        self.mv_acc_mm_s2 = float(profile_acc_mm_s2 if profile_acc_mm_s2 > 0 else self.max_acc_mm_s2)
        self.mv_active = True
        self.mv_last_send = None
        # send immediately once
        self._send_extended_counts_nonblocking(self.mv_target_counts, self.mv_vel_mm_s, self.mv_acc_mm_s2)
        self.get_logger().info(f"Move command accepted: target_mm={target_mm} (target_counts={self.mv_target_counts})")

    def _move_tick(self) -> None:
        if not self.mv_active or self.mv_target_counts is None:
            return
        # Check reach
        if self.present_counts is not None:
            tol_counts = max(5, mm_to_counts(self.return_to_zero_tolerance_mm, self.pitch, self.cpr))
            if abs(self.present_counts - self.mv_target_counts) <= tol_counts:
                self.get_logger().info("Move target reached (pub/sub)")
                self.mv_active = False
                self.mv_target_counts = None
                return
        # Resend at ~5 Hz
        now = self.get_clock().now()
        # Keep torque ON at ~1 Hz while moving to avoid accidental torque-off states
        if self.mv_last_torque is None or (now - self.mv_last_torque).nanoseconds * 1e-9 > 1.0:
            self._torque(True)
            self.mv_last_torque = now
        if self.mv_last_send is None or (now - self.mv_last_send).nanoseconds * 1e-9 > 0.2:
            self._send_extended_counts_nonblocking(self.mv_target_counts, self.mv_vel_mm_s, self.mv_acc_mm_s2)
            self.mv_last_send = now

    def _send_extended_counts_nonblocking(self, target_counts: int, vel_mm_s: float, acc_mm_s2: float) -> None:
        # Switch to extended position mode and send once (non-blocking)
        # Ensure torque ON prior to mode/goal
        self._torque(True)
        cmd_mode = DxlCommandsX()
        cmd_mode.status.id_list = [self.motor_id]
        cmd_mode.status.mode = [cmd_mode.status.CONTROL_EXTENDED_POSITION]
        self.pub_cmd.publish(cmd_mode)

        deg_total = (target_counts / self.cpr) * 360.0
        rotation = float(math.floor(deg_total / 360.0))
        pos_deg = deg_total - rotation * 360.0
        vel_deg_s = (vel_mm_s if vel_mm_s > 0 else self.max_vel_mm_s) / self.pitch * 360.0
        acc_deg_ss = (acc_mm_s2 if acc_mm_s2 > 0 else self.max_acc_mm_s2) / self.pitch * 360.0

        cmd = DxlCommandsX()
        cmd.extended_position_control.id_list = [self.motor_id]
        cmd.extended_position_control.position_deg = [float(pos_deg)]
        cmd.extended_position_control.rotation = [float(rotation)]
        cmd.extended_position_control.profile_vel_deg_s = [float(vel_deg_s)]
        cmd.extended_position_control.profile_acc_deg_ss = [float(acc_deg_ss)]
        self.pub_cmd.publish(cmd)

    # ---- Core logic ----
    # ========== Non-blocking Homing State Machine ==========
    def _start_homing(self) -> None:
        self._torque(True)
        self.sm_state = 'homing_left'
        self.sm_dir = -1
        self.sm_current_level = max(0, abs(self.seek_current_mA))
        self.sm_start_time = self.get_clock().now()
        self.sm_last_cmd_time = None
        self.sm_last_move_time = self.get_clock().now()
        self.sm_stage_start = self.get_clock().now()
        self.sm_stage_baseline_counts = self.present_counts
        self.sm_moved_once = False
        self.sm_stall_start = None
        self.sm_backoff_target = None
        self.sm_backoff_start = None
        self.sm_end_left = None
        self.sm_end_right = None
        self.get_logger().info(
            f"Start homing: origin={self.origin_reference}, seek_current_ma={self.seek_current_mA}, backoff_mm={self.backoff_mm}"
        )

    def _tick(self) -> None:
        state = self.sm_state
        if state == 'idle' or state == 'homed' or state == 'error':
            return
        now = self.get_clock().now()
        vel_mm_s = 0.0
        if self.present_velocity_deg_s is not None:
            vel_mm_s = (self.present_velocity_deg_s / 360.0) * self.pitch

        # on state entry: set operating mode once
        if self.sm_state_prev != state:
            if state in ('homing_left', 'homing_right'):
                cmd_mode = DxlCommandsX()
                cmd_mode.status.id_list = [self.motor_id]
                cmd_mode.status.mode = [cmd_mode.status.CONTROL_CURRENT]
                self.pub_cmd.publish(cmd_mode)
                self._torque(True)
                self.sm_last_cmd_time = None
            elif state in ('backoff_left', 'backoff_right'):
                cmd_mode = DxlCommandsX()
                cmd_mode.status.id_list = [self.motor_id]
                cmd_mode.status.mode = [cmd_mode.status.CONTROL_CURRENT_BASE_POSITION]
                self.pub_cmd.publish(cmd_mode)
                self._torque(True)
                self.sm_last_cmd_time = None
            self.sm_state_prev = state

        # helper to (re)send current command periodically
        def maybe_send_current():
            nonlocal now
            if self.sm_last_cmd_time is None or (now - self.sm_last_cmd_time).nanoseconds * 1e-9 > 0.2:
                c = DxlCommandsX()
                c.current_control.id_list = [self.motor_id]
                c.current_control.current_ma = [float(self.sm_dir * self.sm_current_level)]
                self.pub_cmd.publish(c)
                self.sm_last_cmd_time = now

        # helper to send current-base position backoff
        def maybe_send_backoff(target_counts: int):
            nonlocal now
            if self.sm_last_cmd_time is None or (now - self.sm_last_cmd_time).nanoseconds * 1e-9 > 0.2:
                deg_total = (target_counts / self.cpr) * 360.0
                rotation = float(math.floor(deg_total / 360.0))
                pos_deg = deg_total - rotation * 360.0
                vel_deg_s = (self.backoff_profile_vel_mm_s if self.backoff_profile_vel_mm_s > 0 else self.max_vel_mm_s) / self.pitch * 360.0
                acc_deg_ss = (self.backoff_profile_acc_mm_s2 if self.backoff_profile_acc_mm_s2 > 0 else self.max_acc_mm_s2) / self.pitch * 360.0
                cmd = DxlCommandsX()
                cmd.current_base_position_control.id_list = [self.motor_id]
                cmd.current_base_position_control.current_ma = [float(abs(self.backoff_current_ma))]
                cmd.current_base_position_control.position_deg = [float(pos_deg)]
                cmd.current_base_position_control.rotation = [float(rotation)]
                cmd.current_base_position_control.profile_vel_deg_s = [float(vel_deg_s)]
                cmd.current_base_position_control.profile_acc_deg_ss = [float(acc_deg_ss)]
                self.pub_cmd.publish(cmd)
                self.sm_last_cmd_time = now

        def maybe_send_extended_to(target_counts: int, vel_mm_s: float, acc_mm_s2: float):
            nonlocal now
            if self.sm_last_cmd_time is None or (now - self.sm_last_cmd_time).nanoseconds * 1e-9 > 0.2:
                cmd_mode = DxlCommandsX()
                cmd_mode.status.id_list = [self.motor_id]
                cmd_mode.status.mode = [cmd_mode.status.CONTROL_EXTENDED_POSITION]
                self.pub_cmd.publish(cmd_mode)
                deg_total = (target_counts / self.cpr) * 360.0
                rotation = float(math.floor(deg_total / 360.0))
                pos_deg = deg_total - rotation * 360.0
                vel_deg_s = (vel_mm_s if vel_mm_s > 0 else self.max_vel_mm_s) / self.pitch * 360.0
                acc_deg_ss = (acc_mm_s2 if acc_mm_s2 > 0 else self.max_acc_mm_s2) / self.pitch * 360.0
                cmd = DxlCommandsX()
                cmd.extended_position_control.id_list = [self.motor_id]
                cmd.extended_position_control.position_deg = [float(pos_deg)]
                cmd.extended_position_control.rotation = [float(rotation)]
                cmd.extended_position_control.profile_vel_deg_s = [float(vel_deg_s)]
                cmd.extended_position_control.profile_acc_deg_ss = [float(acc_deg_ss)]
                self.pub_cmd.publish(cmd)
                self.sm_last_cmd_time = now

        # movement & stall tracking
        if self.present_counts is not None and self.sm_stage_baseline_counts is not None:
            if self.present_counts != self.sm_stage_baseline_counts:
                self.sm_last_move_time = now
        if not self.sm_moved_once and self.present_counts is not None and self.sm_stage_baseline_counts is not None:
            move_min_counts = max(5, mm_to_counts(self.move_min_mm, self.pitch, self.cpr))
            if abs(self.present_counts - self.sm_stage_baseline_counts) >= move_min_counts or abs(vel_mm_s) > self.move_vel_threshold_mm_s:
                self.sm_moved_once = True
                self.get_logger().info("Seek movement detected, waiting stall")

        # states
        if state == 'homing_left' or state == 'homing_right':
            # ramp-up if not moved yet
            if not self.sm_moved_once and self.sm_stage_start is not None:
                if (now - self.sm_stage_start).nanoseconds * 1e-9 > (self.seek_ramp_stage_ms / 1000.0):
                    if self.sm_current_level < self.seek_current_ma_max:
                        self.sm_current_level = min(self.seek_current_ma_max, self.sm_current_level + self.seek_current_ma_step)
                        self.get_logger().info(f"Seek ramp-up: current -> {self.sm_current_level} mA")
                        self.sm_stage_start = now
                        self.sm_stage_baseline_counts = self.present_counts
            # send current
            maybe_send_current()
            # stall detection after movement
            if self.sm_moved_once and abs(vel_mm_s) <= self.stall_vel_threshold_mm_s:
                if self.sm_stall_start is None:
                    self.sm_stall_start = now
                if (now - self.sm_stall_start).nanoseconds * 1e-9 >= self.stall_hold_time_s:
                    self.get_logger().info("Seek end detected: stall condition")
                    self._enforce_stop()
                    # compute backoff
                    backoff_counts = abs(mm_to_counts(self.backoff_mm, self.pitch, self.cpr))
                    target = (self.present_counts or 0) - self.sm_dir * backoff_counts
                    self.sm_backoff_target = target
                    self.sm_backoff_start = now
                    # re-enable torque for backoff motion
                    self._torque(True)
                    self.sm_last_cmd_time = None
                    self.sm_state = 'backoff_left' if self.sm_dir < 0 else 'backoff_right'
                    self.get_logger().info(f"Backoff start: target_counts={target}")
            else:
                self.sm_stall_start = None
            # guards
            if self.sm_start_time and (now - self.sm_start_time).nanoseconds * 1e-9 > self.max_seek_time_s:
                self._zero_current()
                self.get_logger().warn("Seek timeout")
                self.sm_state = 'error'
                return

        elif state == 'backoff_left' or state == 'backoff_right':
            # drive to backoff target
            if self.sm_backoff_target is not None:
                maybe_send_backoff(self.sm_backoff_target)
                # log progress periodically
                if self.present_counts is not None and self.sm_last_move_time is not None:
                    if (now - self.sm_last_move_time).nanoseconds * 1e-9 > 0.3:
                        err = abs(self.present_counts - self.sm_backoff_target)
                        self.get_logger().info(f"Backoff progress: counts={self.present_counts}, err={err}")
                        self.sm_last_move_time = now
            # check reached or timeout
            if self.present_counts is not None and self.sm_backoff_target is not None:
                if abs(self.present_counts - self.sm_backoff_target) < 8:
                    # backoff done
                    end_counts = self.present_counts
                    if state == 'backoff_left':
                        self.sm_end_left = end_counts
                        # now seek right
                        self.sm_state = 'homing_right'
                        self.sm_dir = +1
                    else:
                        self.sm_end_right = end_counts
                        self.sm_state = 'finalize_homing'
                    # re-init stage tracking
                    self.sm_current_level = max(0, abs(self.seek_current_mA))
                    self.sm_last_cmd_time = None
                    self.sm_moved_once = False
                    self.sm_stage_start = now
                    self.sm_stage_baseline_counts = self.present_counts
                    self.sm_stall_start = None
                    self.get_logger().info(f"Backoff done: reached {end_counts}")
                    return
            if self.sm_backoff_start and (now - self.sm_backoff_start).nanoseconds * 1e-9 > self.backoff_timeout_s:
                self.get_logger().warn("Backoff timeout, proceeding")
                if state == 'backoff_left':
                    self.sm_end_left = self.present_counts
                    self.sm_state = 'homing_right'
                    self.sm_dir = +1
                else:
                    self.sm_end_right = self.present_counts
                    self.sm_state = 'finalize_homing'
                self.sm_current_level = max(0, abs(self.seek_current_mA))
                self.sm_last_cmd_time = None
                self.sm_moved_once = False
                self.sm_stage_start = now
                self.sm_stage_baseline_counts = self.present_counts
                self.sm_stall_start = None

        elif state == 'finalize_homing':
            # compute min/max from ends
            if self.sm_end_left is None or self.sm_end_right is None:
                self.get_logger().warn("Finalize called without both ends, abort")
                self.sm_state = 'error'
                return
            self.min_end_counts = min(self.sm_end_left, self.sm_end_right)
            self.max_end_counts = max(self.sm_end_left, self.sm_end_right)
            # zero offset by preference
            if self.origin_reference == 'min':
                self.zero_offset_counts = self.min_end_counts
            elif self.origin_reference == 'max':
                self.zero_offset_counts = self.max_end_counts
            else:
                self.zero_offset_counts = int(round((self.min_end_counts + self.max_end_counts) / 2))
            # apply configurable origin offset (mm -> counts)
            offset_counts = mm_to_counts(self.origin_offset_mm, self.pitch, self.cpr)
            self.zero_offset_counts += offset_counts
            if self.soft_min_mm is None or self.soft_max_mm is None:
                min_mm = counts_to_mm(self.min_end_counts - self.zero_offset_counts, self.pitch, self.cpr)
                max_mm = counts_to_mm(self.max_end_counts - self.zero_offset_counts, self.pitch, self.cpr)
                self.soft_min_mm = min(min_mm, max_mm)
                self.soft_max_mm = max(min_mm, max_mm)
            self.get_logger().info(
                f"Homing finalized: min={self.min_end_counts}, max={self.max_end_counts}, zero={self.zero_offset_counts}, origin_offset_mm={self.origin_offset_mm}, soft=[{self.soft_min_mm}, {self.soft_max_mm}] mm"
            )
            # return-to-position (relative to zero after offset)
            rtz_counts = self.zero_offset_counts + mm_to_counts(self.return_to_position_mm, self.pitch, self.cpr)
            if not self.return_to_zero_monitor:
                # One-shot command only, no monitoring
                self.sm_last_cmd_time = None
                maybe_send_extended_to(rtz_counts, self.max_vel_mm_s, self.max_acc_mm_s2)
                self.get_logger().info("Return-to-position command sent (one-shot), homed")
                self.homed = True
                self.sm_state = 'homed'
                return
            else:
                # Monitor until reached/timed out
                self.sm_rtz_target_counts = rtz_counts
                self.sm_rtz_start = now
                self.sm_last_cmd_time = None
                self.sm_state = 'return_to_zero'
                self.get_logger().info(f"Return-to-position start: target_counts={self.sm_rtz_target_counts}")

        elif state == 'return_to_zero':
            if self.sm_rtz_target_counts is None:
                self.get_logger().warn("RTZ without target, homing done")
                self.homed = True
                self.sm_state = 'homed'
                return
            # send extended position command periodically
            maybe_send_extended_to(self.sm_rtz_target_counts, self.max_vel_mm_s, self.max_acc_mm_s2)
            # check completion or timeout
            if self.present_counts is not None:
                tol_counts = max(5, mm_to_counts(self.return_to_zero_tolerance_mm, self.pitch, self.cpr))
                if abs(self.present_counts - self.sm_rtz_target_counts) <= tol_counts:
                    self.get_logger().info("Return-to-zero reached")
                    self.homed = True
                    self.sm_state = 'homed'
                    return
            if self.sm_rtz_start and (now - self.sm_rtz_start).nanoseconds * 1e-9 > self.return_to_zero_timeout_s:
                self.get_logger().warn("Return-to-zero timeout, proceeding to homed")
                self.homed = True
                self.sm_state = 'homed'

    def _torque(self, enable: bool) -> bool:
        self.get_logger().info(f"Torque {'ON' if enable else 'OFF'}")
        cmd = DxlCommandsX()
        cmd.status.id_list = [self.motor_id]
        cmd.status.torque = [enable]
        self.pub_cmd.publish(cmd)
        return True

    def _seek_end(self, direction_sign: int) -> Tuple[bool, int]:
        # Enter current control and apply signed current
        # Change operating mode by status.mode
        self.get_logger().info(
            f"Seek end: direction_sign={direction_sign}, apply_current_ma={direction_sign * self.seek_current_mA}"
        )
        cmd_mode = DxlCommandsX()
        cmd_mode.status.id_list = [self.motor_id]
        cmd_mode.status.mode = [cmd_mode.status.CONTROL_CURRENT]
        self.pub_cmd.publish(cmd_mode)

        # Apply current with ramp-up
        current_level = max(0, abs(self.seek_current_mA))
        move_min_counts = max(5, mm_to_counts(self.move_min_mm, self.pitch, self.cpr))
        stage_start = self.get_clock().now()
        stage_baseline = self.present_counts
        def send_current(ma: int):
            c = DxlCommandsX()
            c.current_control.id_list = [self.motor_id]
            c.current_control.current_ma = [float(direction_sign * ma)]
            self.pub_cmd.publish(c)
        send_current(current_level)

        # Wait until stall detected or timeout
        start = self.get_clock().now()
        hold = Duration(seconds=self.hold_time_ms / 1000.0)
        timeout = Duration(seconds=self.max_seek_time_s)
        last_move_time = self.get_clock().now()
        last_counts = self.present_counts
        last_log = self.get_clock().now()
        last_watchdog_log = self.get_clock().now()
        # velocity-based stall detection
        stall_start: Optional[Time] = None
        # travel guard baseline
        travel_baseline = self.present_counts
        max_travel_counts = mm_to_counts(self.max_travel_mm, self.pitch, self.cpr)
        # periodic refresh
        last_current_refresh = self.get_clock().now()
        moved_once = False
        while True:
            now = self.get_clock().now()
            if now - start > timeout:
                # stop current
                self._zero_current()
                self.get_logger().warn("Seek end timeout")
                return False, 0
            if self.present_counts is None or last_counts is None:
                rclpy.spin_once(self, timeout_sec=0.01)
                continue
            # travel guard
            if travel_baseline is not None and max_travel_counts > 0:
                if abs(self.present_counts - travel_baseline) > max_travel_counts:
                    self._zero_current()
                    self.get_logger().warn("Seek travel exceeded max_travel guard")
                    return False, 0
            if self.present_counts != last_counts:
                last_move_time = now
                last_counts = self.present_counts
            # ramp-up stage: if not moved enough since stage start, increase current
            if stage_baseline is not None and not moved_once:
                if abs(self.present_counts - stage_baseline) >= move_min_counts:
                    moved_once = True
                    self.get_logger().info(f"Seek movement detected (>= {move_min_counts} counts), hold until stall")
                else:
                    if (now - stage_start).nanoseconds * 1e-9 > (self.seek_ramp_stage_ms / 1000.0):
                        if current_level < self.seek_current_ma_max:
                            current_level = min(self.seek_current_ma_max, current_level + self.seek_current_ma_step)
                            self.get_logger().info(f"Seek ramp-up: increase current to {current_level} mA")
                            send_current(current_level)
                            stage_start = now
                            stage_baseline = self.present_counts
            # velocity-based movement detection fallback
            if self.present_velocity_deg_s is not None:
                vel_mm_s = (self.present_velocity_deg_s / 360.0) * self.pitch
                if abs(vel_mm_s) > self.move_vel_threshold_mm_s:
                    moved_once = True
                    last_move_time = now
            # velocity-based stall detection (after any movement has occurred)
            if self.present_velocity_deg_s is not None and moved_once:
                vel_mm_s = (self.present_velocity_deg_s / 360.0) * self.pitch
                if abs(vel_mm_s) <= self.stall_vel_threshold_mm_s:
                    if stall_start is None:
                        stall_start = now
                    if (now - stall_start).nanoseconds * 1e-9 >= self.stall_hold_time_s:
                        self.get_logger().info("Seek end detected: velocity-based stall condition")
                        # ensure stop before leaving seek loop
                        self._enforce_stop()
                        break
                else:
                    stall_start = None
            if (now - last_log).nanoseconds * 1e-9 > 0.2:
                last_log = now
                self.get_logger().info(
                    f"Seek progress: counts={self.present_counts}, since_move={float((now - last_move_time).nanoseconds)/1e9:.3f}s"
                )
            # states watchdog
            if self._last_states_update_time is not None and self.states_watchdog_s > 0:
                dt_upd = (now - self._last_states_update_time).nanoseconds * 1e-9
                if dt_upd > self.states_watchdog_s and (now - last_watchdog_log).nanoseconds * 1e-9 > 0.5:
                    self.get_logger().warn(f"No /dynamixel/states update for {dt_upd:.2f}s (check handler pub_ratio / fast_read)")
                    last_watchdog_log = now
            # periodic current refresh (to be robust)
            if (now - last_current_refresh).nanoseconds * 1e-9 > 0.5:
                send_current(current_level)
                last_current_refresh = now
            rclpy.spin_once(self, timeout_sec=0.01)

        # Stop current and backoff slightly
        self._zero_current()
        self.get_logger().info("Zero current applied before backoff")
        # Backoff by sending opposite current for a short time based on backoff_mm
        backoff_counts = abs(mm_to_counts(self.backoff_mm, self.pitch, self.cpr))
        target_counts = (self.present_counts or 0) - direction_sign * backoff_counts
        self.get_logger().info(f"Backoff: backoff_counts={backoff_counts}, target_counts={target_counts}")
        self._nudge_to_counts(target_counts)
        # Final position counts
        self.get_logger().info(f"End found at counts={int(self.present_counts or 0)}")
        return True, int(self.present_counts or 0)

    def _zero_current(self) -> None:
        self.get_logger().info("Zero current command publish")
        cmd = DxlCommandsX()
        cmd.current_control.id_list = [self.motor_id]
        cmd.current_control.current_ma = [0.0]
        self.pub_cmd.publish(cmd)

    def _enforce_stop(self) -> None:
        # Non-blocking, torque remains ON: send zero current once
        self.get_logger().info("Stop: zero current (non-blocking)")
        cmd = DxlCommandsX()
        cmd.current_control.id_list = [self.motor_id]
        cmd.current_control.current_ma = [0.0]
        self.pub_cmd.publish(cmd)
        self.sm_last_cmd_time = None

    def _nudge_to_counts(self, target_counts: int) -> None:
        # Use current-base position to gently move a small distance with current limit
        cmd_mode = DxlCommandsX()
        cmd_mode.status.id_list = [self.motor_id]
        cmd_mode.status.mode = [cmd_mode.status.CONTROL_CURRENT_BASE_POSITION]
        self.pub_cmd.publish(cmd_mode)

        # Convert counts to deg and rotation
        rel_counts = target_counts
        deg_total = (rel_counts / self.cpr) * 360.0
        rotation = float(math.floor(deg_total / 360.0))
        pos_deg = deg_total - rotation * 360.0
        self.get_logger().info(
            f"Nudge: target_counts={target_counts}, deg_total={deg_total:.3f}, rotation={rotation}, pos_deg={pos_deg:.3f}"
        )

        cmd = DxlCommandsX()
        cmd.current_base_position_control.id_list = [self.motor_id]
        cmd.current_base_position_control.current_ma = [float(abs(self.backoff_current_ma))]
        cmd.current_base_position_control.position_deg = [float(pos_deg)]
        cmd.current_base_position_control.rotation = [float(rotation)]
        # add profile for backoff
        vel_deg_s = (self.backoff_profile_vel_mm_s if self.backoff_profile_vel_mm_s > 0 else self.max_vel_mm_s) / self.pitch * 360.0
        acc_deg_ss = (self.backoff_profile_acc_mm_s2 if self.backoff_profile_acc_mm_s2 > 0 else self.max_acc_mm_s2) / self.pitch * 360.0
        cmd.current_base_position_control.profile_vel_deg_s = [float(vel_deg_s)]
        cmd.current_base_position_control.profile_acc_deg_ss = [float(acc_deg_ss)]
        self.pub_cmd.publish(cmd)

        # Wait briefly until position reaches (best-effort)
        t0 = time.time()
        reached = False
        while time.time() - t0 < 1.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.present_counts is not None and abs(self.present_counts - target_counts) < 5:
                reached = True
                break
        if self.present_counts is not None:
            self.get_logger().info(
                f"Nudge result: present_counts={self.present_counts}, error={abs(self.present_counts - target_counts)}"
            )
        if not reached:
            self.get_logger().warn("Backoff nudge not effective, fallback to extended-position small move")
            self._extended_move_to_counts(target_counts, self.backoff_profile_vel_mm_s, self.backoff_profile_acc_mm_s2)

    def _extended_move_to_counts(self, target_counts: int, vel_mm_s: float, acc_mm_s2: float) -> None:
        cmd_mode = DxlCommandsX()
        cmd_mode.status.id_list = [self.motor_id]
        cmd_mode.status.mode = [cmd_mode.status.CONTROL_EXTENDED_POSITION]
        self.pub_cmd.publish(cmd_mode)

        deg_total = (target_counts / self.cpr) * 360.0
        rotation = float(math.floor(deg_total / 360.0))
        pos_deg = deg_total - rotation * 360.0
        vel_deg_s = (vel_mm_s if vel_mm_s > 0 else self.max_vel_mm_s) / self.pitch * 360.0
        acc_deg_ss = (acc_mm_s2 if acc_mm_s2 > 0 else self.max_acc_mm_s2) / self.pitch * 360.0
        self.get_logger().info(
            f"Fallback ex-pos: target_counts={target_counts}, rotation={rotation}, pos_deg={pos_deg:.3f}, "
            f"vel_deg_s={vel_deg_s:.3f}, acc_deg_ss={acc_deg_ss:.3f}"
        )
        cmd = DxlCommandsX()
        cmd.extended_position_control.id_list = [self.motor_id]
        cmd.extended_position_control.position_deg = [float(pos_deg)]
        cmd.extended_position_control.rotation = [float(rotation)]
        cmd.extended_position_control.profile_vel_deg_s = [float(vel_deg_s)]
        cmd.extended_position_control.profile_acc_deg_ss = [float(acc_deg_ss)]
        self.pub_cmd.publish(cmd)
        t0 = time.time()
        while time.time() - t0 < 1.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.present_counts is not None and abs(self.present_counts - target_counts) < 10:
                break

    def _command_extended_position_mm(self, target_mm: float, vel_mm_s: float, acc_mm_s2: float) -> None:
        # Switch to extended position mode and send degrees with rotation
        cmd_mode = DxlCommandsX()
        cmd_mode.status.id_list = [self.motor_id]
        cmd_mode.status.mode = [cmd_mode.status.CONTROL_EXTENDED_POSITION]
        self.pub_cmd.publish(cmd_mode)

        target_counts = mm_to_counts(target_mm, self.pitch, self.cpr) + self.zero_offset_counts
        deg_total = (target_counts / self.cpr) * 360.0
        rotation = float(math.floor(deg_total / 360.0))
        pos_deg = deg_total - rotation * 360.0

        # Convert profile from mm/s to deg/s
        # mm/s -> rev/s -> deg/s
        vel_deg_s = (vel_mm_s if vel_mm_s > 0 else self.max_vel_mm_s) / self.pitch * 360.0
        acc_deg_ss = (acc_mm_s2 if acc_mm_s2 > 0 else self.max_acc_mm_s2) / self.pitch * 360.0
        self.get_logger().info(
            f"Cmd ex-pos: target_mm={target_mm}, target_counts={target_counts}, rotation={rotation}, pos_deg={pos_deg:.3f}, "
            f"vel_deg_s={vel_deg_s:.3f}, acc_deg_ss={acc_deg_ss:.3f}"
        )

        cmd = DxlCommandsX()
        cmd.extended_position_control.id_list = [self.motor_id]
        cmd.extended_position_control.position_deg = [float(pos_deg)]
        cmd.extended_position_control.rotation = [float(rotation)]
        cmd.extended_position_control.profile_vel_deg_s = [float(vel_deg_s)]
        cmd.extended_position_control.profile_acc_deg_ss = [float(acc_deg_ss)]
        self.pub_cmd.publish(cmd)


def main() -> None:
    rclpy.init()
    node = LeadscrewSliderController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


