#!/usr/bin/env python3

import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import matplotlib as mpl
import matplotlib.pyplot as plt


def _deg2rad(deg: float) -> float:
    return float(deg) * np.pi / 180.0


def _normalize(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return v
    return v / n


def _rot_z(angle_rad: float) -> np.ndarray:
    c = float(np.cos(angle_rad))
    s = float(np.sin(angle_rad))
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _rot_axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    # Rodrigues' rotation formula
    a = _normalize(axis)
    x, y, z = float(a[0]), float(a[1]), float(a[2])
    c = float(np.cos(angle_rad))
    s = float(np.sin(angle_rad))
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ],
        dtype=float,
    )


def _circle_points(center: np.ndarray, normal: np.ndarray, radius: float, n: int = 120) -> np.ndarray:
    # Returns (n,3) points for a circle in 3D.
    nrm = _normalize(normal)
    ref = np.array([0.0, 0.0, 1.0], dtype=float)
    u = np.cross(nrm, ref)
    if float(np.linalg.norm(u)) < 1e-10:
        ref = np.array([1.0, 0.0, 0.0], dtype=float)
        u = np.cross(nrm, ref)
    u = _normalize(u)
    v = np.cross(nrm, u)
    t = np.linspace(0.0, 2.0 * np.pi, int(n), endpoint=True)
    return np.asarray(center, dtype=float).reshape(3)[None, :] + float(radius) * (
        np.cos(t)[:, None] * u[None, :] + np.sin(t)[:, None] * v[None, :]
    )


@dataclass(frozen=True)
class ArmPose:
    hinge: np.ndarray  # (3,)
    arm_dir: np.ndarray  # (3,) unit vector (after fold+slant)
    rotor_normal: np.ndarray  # (3,) unit vector (after fold+slant+tilt)


def _make_arm_pose(
    hinge: np.ndarray,
    arm_dir0: np.ndarray,
    phi_deg: float,
    psi_deg: float,
    theta_deg: float,
) -> ArmPose:
    z = np.array([0.0, 0.0, 1.0], dtype=float)

    # Keep sign convention consistent with visualize_morph_drone.py:
    # internal uses negative phi/psi.
    phi = _deg2rad(-float(phi_deg))
    psi = _deg2rad(-float(psi_deg))
    theta = _deg2rad(float(theta_deg))

    R_fold = _rot_z(phi)
    arm_dir1 = _normalize(R_fold @ _normalize(arm_dir0))

    slant_axis = np.cross(z, arm_dir1)
    if float(np.linalg.norm(slant_axis)) < 1e-10:
        R_slant = np.eye(3, dtype=float)
    else:
        R_slant = _rot_axis_angle(slant_axis, psi)

    arm_dir2 = _normalize(R_slant @ arm_dir1)
    rotor_n2 = _normalize(R_slant @ (R_fold @ z))

    R_tilt = _rot_axis_angle(arm_dir2, theta)
    rotor_n3 = _normalize(R_tilt @ rotor_n2)

    return ArmPose(hinge=np.asarray(hinge, dtype=float).reshape(3), arm_dir=arm_dir2, rotor_normal=rotor_n3)


def _compute_poses(phi_deg: float, psi_deg: float, theta_deg: float, *, cx: float, cy: float, symmetry: str) -> list[ArmPose]:
    if symmetry not in {"mirror_xy", "none"}:
        raise ValueError(f"Unknown symmetry: {symmetry}")

    base_hinge = np.array([+float(cx), +float(cy), 0.0], dtype=float)
    base_arm_dir0 = _normalize(np.array([+1.0, +1.0, 0.0], dtype=float))
    base_pose = _make_arm_pose(
        hinge=base_hinge,
        arm_dir0=base_arm_dir0,
        phi_deg=float(phi_deg),
        psi_deg=float(psi_deg),
        theta_deg=float(theta_deg),
    )

    if symmetry == "mirror_xy":
        M_id = np.diag([1.0, 1.0, 1.0])
        M_x = np.diag([-1.0, 1.0, 1.0])
        M_y = np.diag([1.0, -1.0, 1.0])
        M_xy = np.diag([-1.0, -1.0, 1.0])
        Ms = [M_id, M_x, M_xy, M_y]
        out: list[ArmPose] = []
        for M in Ms:
            out.append(
                ArmPose(
                    hinge=(M @ base_pose.hinge),
                    arm_dir=_normalize(M @ base_pose.arm_dir),
                    rotor_normal=_normalize(M @ base_pose.rotor_normal),
                )
            )
        return out

    hinges = [
        np.array([+float(cx), +float(cy), 0.0], dtype=float),
        np.array([-float(cx), +float(cy), 0.0], dtype=float),
        np.array([-float(cx), -float(cy), 0.0], dtype=float),
        np.array([+float(cx), -float(cy), 0.0], dtype=float),
    ]
    arm_dirs0 = []
    for h in hinges:
        sx = 1.0 if float(h[0]) >= 0.0 else -1.0
        sy = 1.0 if float(h[1]) >= 0.0 else -1.0
        arm_dirs0.append(_normalize(np.array([sx, sy, 0.0], dtype=float)))
    return [
        _make_arm_pose(hinge=h, arm_dir0=d0, phi_deg=float(phi_deg), psi_deg=float(psi_deg), theta_deg=float(theta_deg))
        for (h, d0) in zip(hinges, arm_dirs0, strict=True)
    ]


class MorphDronePreviewNode(Node):
    """
    morph_drone_control のスライダ操作に追従する、matplotlib 3Dプレビュー表示ノード。
    - /morph_drone/angles_deg (Float64MultiArray): [phi, psi, theta] in deg
    """

    def __init__(self):
        super().__init__("morph_drone_preview_node")

        # Geometry parameters (match visualize defaults)
        self.declare_parameter("cx", 0.035)
        self.declare_parameter("cy", 0.035)
        self.declare_parameter("arm_length_m", 0.12)
        self.declare_parameter("rotor_radius_in", 3.5)
        self.declare_parameter("symmetry", "mirror_xy")
        self.declare_parameter("rotor_inflow_offset_m", 0.02)

        # IO
        self.declare_parameter("angles_topic", "/morph_drone/angles_deg")
        self.declare_parameter("update_hz", 20.0)

        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.arm_length_m = float(self.get_parameter("arm_length_m").value)
        self.rotor_radius_m = float(self.get_parameter("rotor_radius_in").value) * 0.0254
        self.symmetry = str(self.get_parameter("symmetry").value)
        self.rotor_inflow_offset_m = float(self.get_parameter("rotor_inflow_offset_m").value)

        self.angles_topic = str(self.get_parameter("angles_topic").value)
        self.update_hz = float(self.get_parameter("update_hz").value)

        self._lock = threading.Lock()
        self._latest_angles_deg: Optional[tuple[float, float, float]] = None
        self._dirty = False

        self._sub = self.create_subscription(Float64MultiArray, self.angles_topic, self._angles_cb, 10)

        # matplotlib objects (initialized in init_gui)
        self.fig = None
        self.ax = None
        self._body_line = None
        self._arm_lines = []
        self._rotor_lines = []
        self._normal_lines = []
        self._title = None

        self.get_logger().info(f"Preview subscribing: {self.angles_topic}")

    def _angles_cb(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) < 3:
            return
        with self._lock:
            self._latest_angles_deg = (float(data[0]), float(data[1]), float(data[2]))
            self._dirty = True

    def init_gui(self):
        # Ensure 3D projection is registered.
        # NOTE: If user-site packages override venv/site-packages, Axes3D import may fail.
        try:
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        except Exception as e:
            self.get_logger().error(
                "Failed to import mpl_toolkits.mplot3d.Axes3D (3D projection unavailable). "
                "This is often caused by mixed matplotlib installs (e.g., ~/.local vs venv). "
                "Try running with environment: PYTHONNOUSERSITE=1. "
                "matplotlib=%s (%s) error=%r",
                getattr(mpl, "__version__", "unknown"),
                getattr(mpl, "__file__", "unknown"),
                e,
            )
            raise

        plt.ion()
        self.fig = plt.figure(figsize=(9, 7))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.set_zlabel("z [m]")
        self._title = self.ax.set_title("Morph drone 3D preview")

        # create initial artists
        self._init_artists(phi_deg=0.0, psi_deg=0.0, theta_deg=0.0)

        # periodic update using matplotlib timer (keeps GUI responsive)
        interval_ms = int(max(10.0, 1000.0 / max(self.update_hz, 1e-3)))
        t = self.fig.canvas.new_timer(interval=interval_ms)
        t.add_callback(self._on_gui_timer)
        t.start()

    def _init_artists(self, *, phi_deg: float, psi_deg: float, theta_deg: float):
        poses = _compute_poses(phi_deg, psi_deg, theta_deg, cx=self.cx, cy=self.cy, symmetry=self.symmetry)
        hinges = [p.hinge for p in poses]
        hs = np.array(hinges + [hinges[0]], dtype=float)

        # body outline
        (self._body_line,) = self.ax.plot(hs[:, 0], hs[:, 1], hs[:, 2], color="k", linewidth=1.4)

        colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
        for i, p in enumerate(poses):
            c = colors[i % len(colors)]
            p0 = p.hinge
            arm_tip = p0 + float(self.arm_length_m) * p.arm_dir
            rotor_center = arm_tip + float(self.rotor_inflow_offset_m) * p.rotor_normal

            (arm_ln,) = self.ax.plot([p0[0], arm_tip[0]], [p0[1], arm_tip[1]], [p0[2], arm_tip[2]], color=c, linewidth=3.0)
            self._arm_lines.append(arm_ln)

            circ = _circle_points(center=rotor_center, normal=p.rotor_normal, radius=float(self.rotor_radius_m), n=140)
            (rot_ln,) = self.ax.plot(circ[:, 0], circ[:, 1], circ[:, 2], color=c, linewidth=1.2)
            self._rotor_lines.append(rot_ln)

            n_scale = float(self.rotor_radius_m) * 0.8
            p2 = rotor_center + n_scale * p.rotor_normal
            (n_ln,) = self.ax.plot([rotor_center[0], p2[0]], [rotor_center[1], p2[1]], [rotor_center[2], p2[2]], color=c, linewidth=1.0)
            self._normal_lines.append(n_ln)

        # limits
        self._set_limits_from_poses(poses)

    def _set_limits_from_poses(self, poses: list[ArmPose]):
        pts = []
        for p in poses:
            p0 = p.hinge
            arm_tip = p0 + float(self.arm_length_m) * p.arm_dir
            rotor_center = arm_tip + float(self.rotor_inflow_offset_m) * p.rotor_normal
            pts.append(p0)
            pts.append(arm_tip)
            pts.append(rotor_center)
            pts.append(rotor_center + float(self.rotor_radius_m) * p.rotor_normal)
            pts.append(rotor_center - float(self.rotor_radius_m) * p.rotor_normal)
        P = np.vstack(pts)
        pad = float(self.rotor_radius_m) * 0.8
        x0, x1 = float(P[:, 0].min() - pad), float(P[:, 0].max() + pad)
        y0, y1 = float(P[:, 1].min() - pad), float(P[:, 1].max() + pad)
        z0, z1 = float(P[:, 2].min() - pad), float(P[:, 2].max() + pad)

        # equal aspect (best effort)
        xc, yc, zc = 0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1)
        half = max((x1 - x0) * 0.5, (y1 - y0) * 0.5, (z1 - z0) * 0.5)
        self.ax.set_xlim(xc - half, xc + half)
        self.ax.set_ylim(yc - half, yc + half)
        self.ax.set_zlim(zc - half, zc + half)

    def _update_artists(self, *, phi_deg: float, psi_deg: float, theta_deg: float):
        poses = _compute_poses(phi_deg, psi_deg, theta_deg, cx=self.cx, cy=self.cy, symmetry=self.symmetry)
        hinges = [p.hinge for p in poses]
        hs = np.array(hinges + [hinges[0]], dtype=float)
        self._body_line.set_data(hs[:, 0], hs[:, 1])
        self._body_line.set_3d_properties(hs[:, 2])

        for i, p in enumerate(poses):
            p0 = p.hinge
            arm_tip = p0 + float(self.arm_length_m) * p.arm_dir
            rotor_center = arm_tip + float(self.rotor_inflow_offset_m) * p.rotor_normal

            self._arm_lines[i].set_data([p0[0], arm_tip[0]], [p0[1], arm_tip[1]])
            self._arm_lines[i].set_3d_properties([p0[2], arm_tip[2]])

            circ = _circle_points(center=rotor_center, normal=p.rotor_normal, radius=float(self.rotor_radius_m), n=140)
            self._rotor_lines[i].set_data(circ[:, 0], circ[:, 1])
            self._rotor_lines[i].set_3d_properties(circ[:, 2])

            n_scale = float(self.rotor_radius_m) * 0.8
            p2 = rotor_center + n_scale * p.rotor_normal
            self._normal_lines[i].set_data([rotor_center[0], p2[0]], [rotor_center[1], p2[1]])
            self._normal_lines[i].set_3d_properties([rotor_center[2], p2[2]])

        if self._title is not None:
            self._title.set_text(f"Morph drone 3D preview  (phi={phi_deg:+.1f}°, psi={psi_deg:+.1f}°, theta={theta_deg:+.1f}°)")
        self._set_limits_from_poses(poses)

    def _on_gui_timer(self):
        with self._lock:
            if not self._dirty or self._latest_angles_deg is None:
                return
            phi, psi, theta = self._latest_angles_deg
            self._dirty = False
        try:
            self._update_artists(phi_deg=phi, psi_deg=psi, theta_deg=theta)
            self.fig.canvas.draw_idle()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MorphDronePreviewNode()

    # GUIをメインスレッドで実行
    node.init_gui()

    # ROSスピンは別スレッド
    def _spin():
        rclpy.spin(node)

    th = threading.Thread(target=_spin, daemon=True)
    th.start()

    try:
        plt.show(block=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            plt.close("all")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


