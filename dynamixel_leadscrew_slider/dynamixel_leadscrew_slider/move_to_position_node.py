#!/usr/bin/env python3

import argparse
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time

from dynamixel_leadscrew_slider_msgs.msg import SliderState, MovePositionMm


class MoveToPositionNode(Node):
    def __init__(self, namespace: str, target_mm: float, tolerance_mm: float = 0.5, timeout_s: float = 30.0, vel_mm_s: float = 20.0, acc_mm_s2: float = 100.0) -> None:
        super().__init__('move_to_position_node')
        
        self.namespace = namespace
        self.target_mm = target_mm
        self.tolerance_mm = tolerance_mm
        self.timeout_s = timeout_s
        self.vel_mm_s = vel_mm_s
        self.acc_mm_s2 = acc_mm_s2
        self.start_time: Optional[Time] = None
        self.movement_started = False
        self._done_future: Future = Future()
        self._last_progress_log_sec: int = -1
        
        # Publishers
        self.pub_move = self.create_publisher(MovePositionMm, f'/{namespace}/move_mm', 10)
        
        # Subscribers
        self.sub_position = self.create_subscription(
            SliderState, f'/{namespace}/current_position', self._on_position, 10
        )
        
        self.get_logger().info(
            f"MoveToPositionNode initialized: namespace={namespace}, target_mm={target_mm}, "
            f"tolerance_mm={tolerance_mm}, timeout_s={timeout_s}, vel_mm_s={vel_mm_s}, acc_mm_s2={acc_mm_s2}"
        )
        self.get_logger().info("Waiting for current_position topic...")
        
        # 移動コマンドを送信（到達まで定期的に再送）
        self._send_move_command()
        self._resend_timer = self.create_timer(0.5, self._maybe_resend_move)
        
        # タイマーでタイムアウトをチェック
        self.timeout_timer = self.create_timer(0.1, self._check_timeout)
    
    def _send_move_command(self) -> None:
        """移動コマンドを送信"""
        msg = MovePositionMm()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.target_mm = self.target_mm
        msg.profile_vel_mm_s = float(self.vel_mm_s)
        msg.profile_acc_mm_s2 = float(self.acc_mm_s2)
        
        self.pub_move.publish(msg)
        if not self.movement_started:
            self.start_time = self.get_clock().now()
            self.movement_started = True
        
        self.get_logger().info(f"Move command sent: namespace={self.namespace}, target_mm={self.target_mm}")
    
    def _maybe_resend_move(self) -> None:
        if self._done_future.done():
            return
        # 再送（コントローラが取りこぼした場合の保険）
        self._send_move_command()
    
    def _on_position(self, msg: SliderState) -> None:
        """現在位置のコールバック"""
        if not self.movement_started:
            return
            
        current_mm = msg.position_mm
        error_mm = abs(current_mm - self.target_mm)
        
        # ホーミング状態もチェック
        if not msg.homed:
            self.get_logger().warn("Slider is not homed! Movement may not work properly.")
        
        self.get_logger().info(
            f"Position update: namespace={self.namespace}, current_mm={current_mm:.3f}, target_mm={self.target_mm:.3f}, "
            f"error_mm={error_mm:.3f}, tolerance_mm={self.tolerance_mm:.3f}, "
            f"homed={msg.homed}, velocity_mm_s={msg.velocity_mm_s:.3f}"
        )
        
        # 目標到達をチェック
        if error_mm <= self.tolerance_mm:
            self.get_logger().info(
                f"Target reached! namespace={self.namespace}, current_mm={current_mm:.3f}, target_mm={self.target_mm:.3f}, "
                f"error_mm={error_mm:.3f}"
            )
            if not self._done_future.done():
                self._done_future.set_result(True)
            return
    
    def _check_timeout(self) -> None:
        """タイムアウトをチェック"""
        if not self.movement_started or self.start_time is None:
            return
            
        elapsed_s = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        
        # 定期的に経過時間をログ出力（1秒ごと、同じ秒は一度のみ）
        sec = int(elapsed_s)
        if sec != self._last_progress_log_sec:
            self._last_progress_log_sec = sec
            if sec % 10 == 0 and sec > 0:
                self.get_logger().info(f"Movement in progress: namespace={self.namespace}, {elapsed_s:.1f}s elapsed, timeout at {self.timeout_s}s")
        
        if elapsed_s > self.timeout_s:
            self.get_logger().error(
                f"Movement timeout after {elapsed_s:.1f}s. namespace={self.namespace}. Target may not be reached. "
                f"Check if slider is homed and move_mm topic is working."
            )
            if not self._done_future.done():
                self._done_future.set_result(False)
            return
    
    @property
    def done_future(self) -> Future:
        return self._done_future


class MultiMoveToPositionNode(Node):
    def __init__(self, namespaces: list, targets_mm: list, tolerance_mm: float = 0.5, timeout_s: float = 100.0, vel_mm_s: float = 20.0, acc_mm_s2: float = 100.0) -> None:
        super().__init__('move_to_position_node')
        
        self.namespaces = list(namespaces)
        self.targets = {ns: float(t) for ns, t in zip(self.namespaces, targets_mm)}
        self.tolerance_mm = tolerance_mm
        self.timeout_s = timeout_s
        self.vel_mm_s = vel_mm_s
        self.acc_mm_s2 = acc_mm_s2
        self.start_time: Optional[Time] = None
        self._done_future: Future = Future()
        self._last_progress_log_sec: int = -1
        
        # per-namespace state
        self._movement_started = False
        self._reached = {ns: False for ns in self.namespaces}
        self._pub_move = {}
        self._subs = {}
        
        for ns in self.namespaces:
            self._pub_move[ns] = self.create_publisher(MovePositionMm, f'/{ns}/move_mm', 10)
            # capture namespace in callback
            self._subs[ns] = self.create_subscription(
                SliderState, f'/{ns}/current_position', lambda msg, ns=ns: self._on_position(ns, msg), 10
            )
        
        self.get_logger().info(
            f"MultiMove initialized: namespaces={self.namespaces}, targets_mm={[self.targets[ns] for ns in self.namespaces]}, "
            f"tolerance_mm={tolerance_mm}, timeout_s={timeout_s}, vel_mm_s={vel_mm_s}, acc_mm_s2={acc_mm_s2}"
        )
        
        # send once immediately and start timers
        self._send_all()
        self._resend_timer = self.create_timer(0.5, self._maybe_resend_all)
        self.timeout_timer = self.create_timer(0.1, self._check_timeout)
    
    def _send_all(self) -> None:
        now = self.get_clock().now()
        for ns in self.namespaces:
            if self._reached[ns]:
                continue
            msg = MovePositionMm()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'base_link'
            msg.target_mm = float(self.targets[ns])
            msg.profile_vel_mm_s = float(self.vel_mm_s)
            msg.profile_acc_mm_s2 = float(self.acc_mm_s2)
            self._pub_move[ns].publish(msg)
            self.get_logger().info(f"Move command sent: namespace={ns}, target_mm={self.targets[ns]}")
        if not self._movement_started:
            self.start_time = now
            self._movement_started = True
    
    def _maybe_resend_all(self) -> None:
        if self._done_future.done():
            return
        self._send_all()
    
    def _on_position(self, ns: str, msg: SliderState) -> None:
        if not self._movement_started:
            return
        if self._reached.get(ns, False):
            return
        current_mm = msg.position_mm
        target_mm = self.targets[ns]
        error_mm = abs(current_mm - target_mm)
        if not msg.homed:
            self.get_logger().warn(f"[{ns}] Slider is not homed! Movement may not work properly.")
        self.get_logger().info(
            f"Position update: namespace={ns}, current_mm={current_mm:.3f}, target_mm={target_mm:.3f}, error_mm={error_mm:.3f}, "
            f"tolerance_mm={self.tolerance_mm:.3f}, homed={msg.homed}, velocity_mm_s={msg.velocity_mm_s:.3f}"
        )
        if error_mm <= self.tolerance_mm:
            self._reached[ns] = True
            self.get_logger().info(f"Target reached: namespace={ns}, current_mm={current_mm:.3f}, target_mm={target_mm:.3f}")
            # if all reached -> done
            if all(self._reached.values()) and not self._done_future.done():
                self._done_future.set_result(True)
    
    def _check_timeout(self) -> None:
        if not self._movement_started or self.start_time is None:
            return
        elapsed_s = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        sec = int(elapsed_s)
        if sec != self._last_progress_log_sec:
            self._last_progress_log_sec = sec
            if sec % 10 == 0 and sec > 0:
                pending = [ns for ns, r in self._reached.items() if not r]
                self.get_logger().info(f"Movement in progress: {elapsed_s:.1f}s elapsed, timeout at {self.timeout_s}s, pending={pending}")
        if elapsed_s > self.timeout_s:
            pending = [ns for ns, r in self._reached.items() if not r]
            self.get_logger().error(f"Movement timeout after {elapsed_s:.1f}s. pending={pending}")
            if not self._done_future.done():
                self._done_future.set_result(False)
    
    @property
    def done_future(self) -> Future:
        return self._done_future


def main() -> None:
    parser = argparse.ArgumentParser(description='Move slider to target position and exit')
    # 単体指定（後方互換）
    parser.add_argument('namespace', type=str, nargs='?', help='Namespace of the slider')
    parser.add_argument('target_mm', type=float, nargs='?', help='Target position in mm')
    # 複数指定
    parser.add_argument('--namespaces', type=str, nargs='+', help='Namespaces of the sliders (space-separated)')
    parser.add_argument('--targets', type=float, nargs='+', help='Target positions in mm (space-separated, same length as namespaces)')
    parser.add_argument('--tolerance', type=float, default=0.6, 
                       help='Position tolerance in mm (default: 0.6)')
    parser.add_argument('--timeout', type=float, default=30.0,
                       help='Timeout in seconds (default: 30.0)')
    parser.add_argument('--vel', type=float, default=20.0, help='Profile velocity in mm/s (default: 20.0)')
    parser.add_argument('--acc', type=float, default=100.0, help='Profile acceleration in mm/s^2 (default: 100.0)')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = None
    
    try:
        # 複数指定がある場合
        if args.namespaces is not None or args.targets is not None:
            if not args.namespaces or not args.targets:
                raise ValueError('--namespaces と --targets は両方指定してください')
            if len(args.namespaces) != len(args.targets):
                raise ValueError('namespaces と targets の個数が一致しません')
            node = MultiMoveToPositionNode(
                namespaces=args.namespaces,
                targets_mm=args.targets,
                tolerance_mm=args.tolerance,
                timeout_s=args.timeout,
                vel_mm_s=args.vel,
                acc_mm_s2=args.acc
            )
        else:
            if args.namespace is None or args.target_mm is None:
                raise ValueError('単体指定の場合は namespace と target_mm を指定してください')
            node = MoveToPositionNode(
                namespace=args.namespace,
                target_mm=args.target_mm,
                tolerance_mm=args.tolerance,
                timeout_s=args.timeout,
                vel_mm_s=args.vel,
                acc_mm_s2=args.acc
            )
        
        rclpy.spin_until_future_complete(node, node.done_future)
        # キャンセル/クリーンアップ
        if hasattr(node, 'timeout_timer'):
            node.timeout_timer.cancel()
        if hasattr(node, '_resend_timer'):
            node._resend_timer.cancel()
        
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
