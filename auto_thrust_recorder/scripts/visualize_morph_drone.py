import argparse
import logging
import os
import sys
from dataclasses import dataclass
import io

import numpy as np
import matplotlib as mpl

_HAS_3D = True
_AXES3D_IMPORT_ERROR = None
try:
    # 3D projection registration (matplotlib + mpl_toolkits must be consistent)
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
except Exception as e:
    _HAS_3D = False
    _AXES3D_IMPORT_ERROR = e


def _setup_logging(level: str):
    numeric = getattr(logging, str(level).upper(), None)
    if not isinstance(numeric, int):
        numeric = logging.INFO
    logging.basicConfig(
        level=numeric,
        format="%(asctime)s.%(msecs)03d %(levelname)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def _set_library_log_levels(mpl_level: str):
    lvl = getattr(logging, str(mpl_level).upper(), None)
    if not isinstance(lvl, int):
        lvl = logging.WARNING
    # Avoid matplotlib's internal debug spam unless explicitly requested.
    logging.getLogger("matplotlib").setLevel(lvl)
    logging.getLogger("PIL").setLevel(lvl)


def _log_runtime_env():
    logging.info("python=%s", sys.version.replace("\n", " "))
    logging.info("executable=%s", sys.executable)
    logging.info("cwd=%s", os.getcwd())
    logging.info("matplotlib=%s (%s)", mpl.__version__, mpl.__file__)
    logging.info("matplotlib backend=%s", mpl.get_backend())
    logging.info("env DISPLAY=%s", os.environ.get("DISPLAY"))
    logging.info("env WAYLAND_DISPLAY=%s", os.environ.get("WAYLAND_DISPLAY"))
    logging.info("env XDG_RUNTIME_DIR=%s", os.environ.get("XDG_RUNTIME_DIR"))
    logging.info("env MPLBACKEND=%s", os.environ.get("MPLBACKEND"))
    logging.info("3D available=%s", _HAS_3D)
    if not _HAS_3D and _AXES3D_IMPORT_ERROR is not None:
        logging.info("3D import error=%r", _AXES3D_IMPORT_ERROR)


def _deg2rad(deg: float) -> float:
    return float(deg) * np.pi / 180.0


def _normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError("zero-length vector")
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
    """
    Rodrigues' rotation formula.
    axis: (3,) unit vector
    """
    a = _normalize(axis.astype(float))
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


def _set_axes_equal(ax):
    # Matplotlib 3D: make x/y/z scales equal.
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


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
    """
    arm_dir0: initial arm direction (unit), defined at hinge.

    Transform order (as requested):
      1) fold: rotate around z-axis by phi (about hinge, so direction/orientation only)
      2) slant: rotate around axis orthogonal to both (arm after fold) and z-axis by psi
      3) tilt: rotate rotor around arm axis by theta
    """
    z = np.array([0.0, 0.0, 1.0], dtype=float)

    # Sign convention:
    # Treat input phi/psi with opposite sign compared to the original implementation.
    # (+phi, +psi) now rotate in the opposite direction.
    phi = _deg2rad(-phi_deg)
    psi = _deg2rad(-psi_deg)
    theta = _deg2rad(theta_deg)

    R_fold = _rot_z(phi)

    arm_dir1 = _normalize(R_fold @ arm_dir0)
    slant_axis = np.cross(z, arm_dir1)
    if np.linalg.norm(slant_axis) < 1e-10:
        # Arm parallel to z: slant axis is undefined; keep as-is.
        R_slant = np.eye(3, dtype=float)
    else:
        R_slant = _rot_axis_angle(slant_axis, psi)

    arm_dir2 = _normalize(R_slant @ arm_dir1)

    rotor_n0 = z
    rotor_n2 = _normalize(R_slant @ (R_fold @ rotor_n0))

    R_tilt = _rot_axis_angle(arm_dir2, theta)
    rotor_n3 = _normalize(R_tilt @ rotor_n2)

    return ArmPose(hinge=hinge.astype(float), arm_dir=arm_dir2, rotor_normal=rotor_n3)


def _circle_points(center: np.ndarray, normal: np.ndarray, radius: float, n: int = 200) -> np.ndarray:
    """
    Returns (n,3) points for a circle in 3D.
    """
    nrm = _normalize(normal.astype(float))
    ref = np.array([0.0, 0.0, 1.0], dtype=float)
    u = np.cross(nrm, ref)
    if np.linalg.norm(u) < 1e-10:
        ref = np.array([1.0, 0.0, 0.0], dtype=float)
        u = np.cross(nrm, ref)
    u = _normalize(u)
    v = np.cross(nrm, u)
    t = np.linspace(0.0, 2.0 * np.pi, n, endpoint=True)
    pts = center[None, :] + radius * (np.cos(t)[:, None] * u[None, :] + np.sin(t)[:, None] * v[None, :])
    return pts


def plot_morphing_drone(
    cx: float,
    cy: float,
    rotor_radius_m: float,
    arm_length_m: float,
    phi_deg: float,
    psi_deg: float,
    theta_deg: float,
    symmetry: str = "none",
    force_2d: bool = False,
    y_clearance: float = 0.0,
    draw_y0_plane: bool = True,
    save_path: str | None = None,
    dpi: int = 200,
    show: bool = True,
    sliders_enabled: bool = True,
    hide_decorations: bool = False,
):
    # Import pyplot lazily so that main() can set backend beforehand.
    import matplotlib.pyplot as plt

    def thrust_angles_alpha_beta_deg(thrust_vec: np.ndarray) -> tuple[float, float]:
        """
        thrust_vec: (3,) 推力ベクトル（ここではロータ法線ベクトル）を想定。

        定義:
          alpha: thrustをyz平面に射影したベクトルが +z 軸となす角
          beta : thrustをzx平面に射影したベクトルが +z 軸となす角

        角度は符号付きで返す（+y側/+x側に倒れると負、-y/-xで正）。  # 出力のみ符号反転
        """
        v = _normalize(np.asarray(thrust_vec, dtype=float).reshape(3))
        vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
        # NOTE: alpha/beta の符号規約変更（出力のみ反転）
        alpha = -float(np.degrees(np.arctan2(vy, vz)))  # yz-plane
        beta = -float(np.degrees(np.arctan2(vx, vz)))   # zx-plane
        return alpha, beta

    def format_alpha_beta(poses_list: list[ArmPose]) -> str:
        lines = ["alpha/beta (deg) from thrust vec (rotor_normal):"]
        for i, p in enumerate(poses_list):
            a, b = thrust_angles_alpha_beta_deg(p.rotor_normal)
            lines.append(f"  rotor{i}: alpha={a:+6.1f}, beta={b:+6.1f}")
        return "\n".join(lines)

    """
    Assumption (documented):
      - Quadrotor with 4 arms.
      - Hinge centers are at (±cx, ±cy, 0).
      - Each arm initially points to the outward diagonal direction of its quadrant:
          (sign(x), sign(y), 0), normalized.
    """
    if symmetry not in {"none", "mirror_xy"}:
        raise ValueError(f"Unknown symmetry mode: {symmetry}")

    def compute_y_offset(_poses: list[ArmPose], _L: float, _clearance: float) -> float:
        """
        y+方向に平行移動する量 dy を返す。
        定義: 全ロータ円周のうち y=0 平面に最も近い点（最小y）を y_min として、
          dy = clearance - y_min
        とする（移動後は最小yが clearance になる）。

        解析的に:
          y_min(rotor) = y_center - R * sqrt(1 - n_y^2)
        """
        ymins = []
        for _pose in _poses:
            y_center = float(_pose.hinge[1] + _L * _pose.arm_dir[1])
            ny = float(_pose.rotor_normal[1])
            extent = rotor_radius_m * float(np.sqrt(max(0.0, 1.0 - ny * ny)))
            ymins.append(y_center - extent)
        if not ymins:
            return 0.0
        return float(_clearance) - float(min(ymins))

    def compute_poses(_phi: float, _psi: float, _theta: float) -> list[ArmPose]:
        # Base definition: one arm in +x,+y quadrant
        base_hinge = np.array([+cx, +cy, 0.0], dtype=float)
        base_arm_dir0 = _normalize(np.array([+1.0, +1.0, 0.0], dtype=float))

        base_pose = _make_arm_pose(
            hinge=base_hinge,
            arm_dir0=base_arm_dir0,
            phi_deg=float(_phi),
            psi_deg=float(_psi),
            theta_deg=float(_theta),
        )

        if symmetry == "mirror_xy":
            # Enforce symmetry about X and Y axes by mirroring the *result* of a single arm.
            M_id = np.diag([1.0, 1.0, 1.0])
            M_x = np.diag([-1.0, 1.0, 1.0])  # x -> -x
            M_y = np.diag([1.0, -1.0, 1.0])  # y -> -y
            M_xy = np.diag([-1.0, -1.0, 1.0])

            Ms = [M_id, M_x, M_xy, M_y]
            poses_local = []
            for M in Ms:
                poses_local.append(
                    ArmPose(
                        hinge=(M @ base_pose.hinge),
                        arm_dir=_normalize(M @ base_pose.arm_dir),
                        rotor_normal=_normalize(M @ base_pose.rotor_normal),
                    )
                )
            return poses_local

        hinges = [
            np.array([+cx, +cy, 0.0]),
            np.array([-cx, +cy, 0.0]),
            np.array([-cx, -cy, 0.0]),
            np.array([+cx, -cy, 0.0]),
        ]

        arm_dirs0 = []
        for h in hinges:
            sx = 1.0 if h[0] >= 0.0 else -1.0
            sy = 1.0 if h[1] >= 0.0 else -1.0
            arm_dirs0.append(_normalize(np.array([sx, sy, 0.0], dtype=float)))

        return [
            _make_arm_pose(hinge=h, arm_dir0=d0, phi_deg=float(_phi), psi_deg=float(_psi), theta_deg=float(_theta))
            for h, d0 in zip(hinges, arm_dirs0, strict=True)
        ]

    poses = compute_poses(phi_deg, psi_deg, theta_deg)
    dy = compute_y_offset(poses, arm_length_m, y_clearance)

    title = (
        f"Morphing drone visualization\n"
        f"cx={cx:.3f} m, cy={cy:.3f} m, R={rotor_radius_m:.4f} m, L={arm_length_m:.3f} m\n"
        f"fold(phi)={phi_deg:.1f} deg, slant(psi)={psi_deg:.1f} deg, tilt(theta)={theta_deg:.1f} deg\n"
        f"y_clearance={y_clearance:.3f} m"
    )

    # --- 3D only ---
    # 要求: 必ず3Dプロットする。2Dフォールバックは禁止。
    if bool(force_2d):
        raise RuntimeError("force_2d=True は禁止されています（フォールバック禁止のため）。")
    if not _HAS_3D:
        raise RuntimeError(
            "3D projection is unavailable in this Python environment (fallback disabled). "
            f"Reason: {_AXES3D_IMPORT_ERROR!r}"
        )
    use_3d = True

    hinges_for_outline = [pose.hinge for pose in poses]
    hs = np.array(hinges_for_outline + [hinges_for_outline[0]])
    colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]

    def _format_title(_L: float, _phi: float, _psi: float, _theta: float, _y_clear: float) -> str:
        return (
            f"Morphing drone visualization\n"
            f"cx={cx:.3f} m, cy={cy:.3f} m, R={rotor_radius_m:.4f} m, L={_L:.3f} m\n"
            f"fold(phi)={_phi:.1f} deg, slant(psi)={_psi:.1f} deg, tilt(theta)={_theta:.1f} deg\n"
            f"y_clearance={_y_clear:.3f} m"
        )

    def _set_3d_line(line, xs, ys, zs):
        line.set_data(xs, ys)
        line.set_3d_properties(zs)

    def _set_3d_point(scatter, p: np.ndarray):
        scatter._offsets3d = ([float(p[0])], [float(p[1])], [float(p[2])])

    def _compute_limits(_poses: list[ArmPose], _L: float):
        all_points = []
        for _pose in _poses:
            _p0 = _pose.hinge
            _p1 = _pose.hinge + _L * _pose.arm_dir
            all_points.append(_p0)
            all_points.append(_p1)
            all_points.append(_p1 + rotor_radius_m * _pose.rotor_normal)
            all_points.append(_p1 - rotor_radius_m * _pose.rotor_normal)
        P = np.vstack(all_points)
        pad = rotor_radius_m * 1.2
        ax.set_xlim(P[:, 0].min() - pad, P[:, 0].max() + pad)
        ax.set_ylim(P[:, 1].min() - pad, P[:, 1].max() + pad)
        ax.set_zlim(P[:, 2].min() - pad, P[:, 2].max() + pad)
        _set_axes_equal(ax)

    fig = None
    if use_3d:
        logging.info("Creating 3D figure/axes...")
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")
        if not bool(hide_decorations):
            ax.set_title(_format_title(arm_length_m, phi_deg, psi_deg, theta_deg, y_clearance))

        angle_text = ax.text2D(
            0.02,
            0.98,
            format_alpha_beta(poses),
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=9,
            family="monospace",
        )
        if bool(hide_decorations):
            try:
                angle_text.set_visible(False)
            except Exception:
                pass

        # Body outline (hinge square)
        hs_shift = hs.copy()
        hs_shift[:, 1] += dy
        body_line = ax.plot(hs_shift[:, 0], hs_shift[:, 1], hs_shift[:, 2], color="k", linewidth=1.5, label="hinge square")[0]

        arm_lines = []
        hinge_pts = []
        tip_pts = []
        rotor_lines = []
        normal_lines = []

        for i, pose in enumerate(poses):
            c = colors[i % len(colors)]
            p0 = pose.hinge + np.array([0.0, dy, 0.0])
            p1 = p0 + arm_length_m * pose.arm_dir

            arm_lines.append(
                ax.plot(
                    [p0[0], p1[0]],
                    [p0[1], p1[1]],
                    [p0[2], p1[2]],
                    color=c,
                    linewidth=3.0,
                    label=f"arm {i}" if i == 0 else None,
                )[0]
            )
            hinge_pts.append(ax.scatter([p0[0]], [p0[1]], [p0[2]], color=c, s=30))
            tip_pts.append(ax.scatter([p1[0]], [p1[1]], [p1[2]], color=c, s=40))

            circ = _circle_points(center=p1, normal=pose.rotor_normal, radius=rotor_radius_m, n=200)
            rotor_lines.append(ax.plot(circ[:, 0], circ[:, 1], circ[:, 2], color=c, linewidth=1.5)[0])

            # Rotor normal (simple line; easier to update than quiver)
            n_scale = rotor_radius_m * 0.8
            p2 = p1 + n_scale * pose.rotor_normal
            normal_lines.append(ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color=c, linewidth=1.2)[0])

        origin = np.array([0.0, 0.0, 0.0])
        axis_len = max(arm_length_m + rotor_radius_m, 0.15)
        if not bool(hide_decorations):
            ax.quiver(origin[0], origin[1], origin[2], 1, 0, 0, length=axis_len, color="r")
            ax.quiver(origin[0], origin[1], origin[2], 0, 1, 0, length=axis_len, color="g")
            ax.quiver(origin[0], origin[1], origin[2], 0, 0, 1, length=axis_len, color="b")
            ax.text(axis_len, 0, 0, "x", color="r")
            ax.text(0, axis_len, 0, "y", color="g")
            ax.text(0, 0, axis_len, "z", color="b")

            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            ax.set_zlabel("z [m]")

        _compute_limits([ArmPose(hinge=p.hinge + np.array([0.0, dy, 0.0]), arm_dir=p.arm_dir, rotor_normal=p.rotor_normal) for p in poses], arm_length_m)

        if bool(hide_decorations):
            try:
                ax.set_axis_off()
            except Exception:
                # fallback: hide labels/ticks
                ax.set_xlabel("")
                ax.set_ylabel("")
                ax.set_zlabel("")
                ax.set_xticks([])
                ax.set_yticks([])
                ax.set_zticks([])

        y0_plane_artist = None

        def _draw_plane():
            nonlocal y0_plane_artist
            if not bool(draw_y0_plane):
                return
            if y0_plane_artist is not None:
                try:
                    y0_plane_artist.remove()
                except Exception:
                    pass
                y0_plane_artist = None
            xlim = ax.get_xlim3d()
            zlim = ax.get_zlim3d()
            xs = np.linspace(float(xlim[0]), float(xlim[1]), 2)
            zs = np.linspace(float(zlim[0]), float(zlim[1]), 2)
            X, Z = np.meshgrid(xs, zs)
            Y = np.zeros_like(X)
            y0_plane_artist = ax.plot_surface(X, Y, Z, color="gray", alpha=0.12, shade=False)

        _draw_plane()

        sliders_enabled = bool(sliders_enabled) and bool(show) and (not bool(hide_decorations))
        if sliders_enabled:
            from matplotlib.widgets import Slider, Button

            fig.subplots_adjust(bottom=0.32)
            ax_phi = fig.add_axes([0.12, 0.23, 0.76, 0.03])
            ax_psi = fig.add_axes([0.12, 0.19, 0.76, 0.03])
            ax_theta = fig.add_axes([0.12, 0.15, 0.76, 0.03])
            ax_L = fig.add_axes([0.12, 0.11, 0.76, 0.03])
            ax_clear = fig.add_axes([0.12, 0.07, 0.76, 0.03])
            ax_reset = fig.add_axes([0.82, 0.01, 0.12, 0.04])

            s_phi = Slider(ax_phi, "phi [deg]", -180.0, 180.0, valinit=float(phi_deg), valstep=1.0)
            s_psi = Slider(ax_psi, "psi [deg]", -90.0, 90.0, valinit=float(psi_deg), valstep=1.0)
            s_theta = Slider(ax_theta, "theta [deg]", -180.0, 180.0, valinit=float(theta_deg), valstep=1.0)
            s_L = Slider(ax_L, "arm L [m]", 0.05, 0.50, valinit=float(arm_length_m), valstep=0.005)
            s_clear = Slider(ax_clear, "y_clear [m]", 0.0, 0.20, valinit=float(y_clearance), valstep=0.001)
            b_reset = Button(ax_reset, "Reset")

            def _update(_val=None):
                new_phi = float(s_phi.val)
                new_psi = float(s_psi.val)
                new_theta = float(s_theta.val)
                new_L = float(s_L.val)
                new_clear = float(s_clear.val)

                new_poses = compute_poses(new_phi, new_psi, new_theta)
                new_dy = compute_y_offset(new_poses, new_L, new_clear)
                angle_text.set_text(format_alpha_beta(new_poses))
                new_hs = np.array([p.hinge for p in new_poses] + [new_poses[0].hinge])
                new_hs[:, 1] += new_dy
                _set_3d_line(body_line, new_hs[:, 0], new_hs[:, 1], new_hs[:, 2])

                for i, pose in enumerate(new_poses):
                    p0 = pose.hinge + np.array([0.0, new_dy, 0.0])
                    p1 = p0 + new_L * pose.arm_dir
                    _set_3d_line(arm_lines[i], [p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]])
                    _set_3d_point(hinge_pts[i], p0)
                    _set_3d_point(tip_pts[i], p1)

                    circ = _circle_points(center=p1, normal=pose.rotor_normal, radius=rotor_radius_m, n=200)
                    _set_3d_line(rotor_lines[i], circ[:, 0], circ[:, 1], circ[:, 2])

                    n_scale = rotor_radius_m * 0.8
                    p2 = p1 + n_scale * pose.rotor_normal
                    _set_3d_line(normal_lines[i], [p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]])

                ax.set_title(_format_title(new_L, new_phi, new_psi, new_theta, new_clear))
                _compute_limits([ArmPose(hinge=p.hinge + np.array([0.0, new_dy, 0.0]), arm_dir=p.arm_dir, rotor_normal=p.rotor_normal) for p in new_poses], new_L)
                _draw_plane()
                fig.canvas.draw_idle()

            def _reset(_event=None):
                s_phi.reset()
                s_psi.reset()
                s_theta.reset()
                s_L.reset()
                s_clear.reset()

            s_phi.on_changed(_update)
            s_psi.on_changed(_update)
            s_theta.on_changed(_update)
            s_L.on_changed(_update)
            s_clear.on_changed(_update)
            b_reset.on_clicked(_reset)
        else:
            plt.tight_layout()
    else:
        # ここには到達しない（3D only）
        raise RuntimeError("internal error: reached 2D fallback path but fallback is disabled")

    if fig is None:
        raise RuntimeError("internal error: figure was not created")

    if save_path:
        logging.info("Saving figure to %s (dpi=%d)...", save_path, int(dpi))
        if bool(hide_decorations):
            fig.savefig(save_path, dpi=int(dpi), bbox_inches="tight", pad_inches=0.0)
        else:
            fig.savefig(save_path, dpi=int(dpi), bbox_inches="tight")
        logging.info("Saved: %s", save_path)

    if show:
        logging.info("Calling plt.show()...")
        plt.show()
    else:
        logging.info("Skipping plt.show() because show=False")

    plt.close(fig)
    logging.info("Figure closed; done.")


def render_morphing_drone_rgba(
    phi_deg: float,
    psi_deg: float,
    theta_deg: float,
    *,
    cx: float = 0.035,
    cy: float = 0.035,
    rotor_radius_in: float = 3.5,
    arm_length_m: float = 0.18,
    symmetry: str = "mirror_xy",
    force_2d: bool = False,
    y_clearance: float = 0.0,
    draw_y0_plane: bool = False,
    dpi: int = 160,
    figsize: tuple[float, float] = (3.2, 2.6),
) -> np.ndarray:
    """
    Morphing drone の図を RGBA (H,W,4) uint8 として返す。
    - ファイル保存なし
    - window表示なし
    - GPR ファセット背景貼り付け用途
    """
    # Ensure headless rendering.
    try:
        mpl.use("Agg", force=True)
    except Exception:
        pass

    import matplotlib.pyplot as plt

    # plot_morphing_drone は内部で figure を作るので、ここでは save_path を BytesIO にする。
    buf = io.BytesIO()
    # plot_morphing_drone は fig を close するので、bufにpngを書き込んだ後に読み取る。
    plot_morphing_drone(
        cx=float(cx),
        cy=float(cy),
        rotor_radius_m=float(rotor_radius_in) * 0.0254,
        arm_length_m=float(arm_length_m),
        phi_deg=float(phi_deg),
        psi_deg=float(psi_deg),
        theta_deg=float(theta_deg),
        symmetry=str(symmetry),
        force_2d=bool(force_2d),
        y_clearance=float(y_clearance),
        draw_y0_plane=bool(draw_y0_plane),
        save_path=buf,
        dpi=int(dpi),
        show=False,
        sliders_enabled=False,
        hide_decorations=True,
    )
    buf.seek(0)

    # Read PNG bytes -> RGBA numpy
    try:
        from PIL import Image

        img = Image.open(buf).convert("RGBA")
        arr = np.asarray(img, dtype=np.uint8)
        return arr
    except Exception:
        # fallback: matplotlib imread (may accept file-like)
        try:
            import matplotlib.image as mpimg

            arr = mpimg.imread(buf)
            # mpimg.imread may return float [0,1] with shape (H,W,4)
            if arr.dtype != np.uint8:
                arr = (np.clip(arr, 0.0, 1.0) * 255.0).astype(np.uint8)
            if arr.shape[-1] == 3:
                alpha = np.full((arr.shape[0], arr.shape[1], 1), 255, dtype=np.uint8)
                arr = np.concatenate([arr, alpha], axis=-1)
            return arr
        except Exception as e:
            raise RuntimeError(f"Failed to decode rendered image bytes: {e}") from e


def plot_three_view_drone(
    cx: float,
    cy: float,
    rotor_radius_m: float,
    arm_length_m: float,
    phi_deg: float,
    psi_deg: float,
    theta_deg: float,
    symmetry: str = "mirror_xy",
    y_clearance: float = 0.0,
    draw_y0_plane: bool = True,
    save_path: str | None = None,
    dpi: int = 200,
    show: bool = True,
    include_3d_view: bool = True,
    drone_center: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotor_inflow_offset: float = 0.02,
    hide_decorations: bool = False,
    drone_color: str = "multi",
    body_color: str = "gray",
):
    """
    3面図 (正面図・側面図・平面図) を描画する。
    
    - Front view (正面図): Y軸方向から見た XZ平面への投影
    - Side view (側面図): X軸方向から見た YZ平面への投影
    - Top view (平面図): Z軸方向から見た XY平面への投影 (Y軸横、X軸縦)
    
    include_3d_view=True の場合、4つ目のサブプロットに3Dビューを追加。
    drone_center: ドローン中心のオフセット (x, y, z) [m]
    rotor_inflow_offset: ロータ中心を流入側（推力の逆方向）にオフセットする量 [m]
    hide_decorations: True の場合、軸ラベル、グリッド、タイトルなどを非表示
    drone_color: "multi" で4色、それ以外は指定した1色でロータを描画
    body_color: ボディ（ヒンジスクエア、アーム）の色
    """
    drone_offset = np.array(drone_center, dtype=float)
    import matplotlib.pyplot as plt

    def compute_poses(_phi: float, _psi: float, _theta: float) -> list[ArmPose]:
        base_hinge = np.array([+cx, +cy, 0.0], dtype=float)
        base_arm_dir0 = _normalize(np.array([+1.0, +1.0, 0.0], dtype=float))

        base_pose = _make_arm_pose(
            hinge=base_hinge,
            arm_dir0=base_arm_dir0,
            phi_deg=float(_phi),
            psi_deg=float(_psi),
            theta_deg=float(_theta),
        )

        if symmetry == "mirror_xy":
            M_id = np.diag([1.0, 1.0, 1.0])
            M_x = np.diag([-1.0, 1.0, 1.0])
            M_y = np.diag([1.0, -1.0, 1.0])
            M_xy = np.diag([-1.0, -1.0, 1.0])

            Ms = [M_id, M_x, M_xy, M_y]
            poses_local = []
            for M in Ms:
                poses_local.append(
                    ArmPose(
                        hinge=(M @ base_pose.hinge),
                        arm_dir=_normalize(M @ base_pose.arm_dir),
                        rotor_normal=_normalize(M @ base_pose.rotor_normal),
                    )
                )
            return poses_local

        hinges = [
            np.array([+cx, +cy, 0.0]),
            np.array([-cx, +cy, 0.0]),
            np.array([-cx, -cy, 0.0]),
            np.array([+cx, -cy, 0.0]),
        ]

        arm_dirs0 = []
        for h in hinges:
            sx = 1.0 if h[0] >= 0.0 else -1.0
            sy = 1.0 if h[1] >= 0.0 else -1.0
            arm_dirs0.append(_normalize(np.array([sx, sy, 0.0], dtype=float)))

        return [
            _make_arm_pose(hinge=h, arm_dir0=d0, phi_deg=float(_phi), psi_deg=float(_psi), theta_deg=float(_theta))
            for h, d0 in zip(hinges, arm_dirs0, strict=True)
        ]

    def compute_y_offset(_poses: list[ArmPose], _L: float, _clearance: float) -> float:
        ymins = []
        for _pose in _poses:
            y_center = float(_pose.hinge[1] + _L * _pose.arm_dir[1])
            ny = float(_pose.rotor_normal[1])
            extent = rotor_radius_m * float(np.sqrt(max(0.0, 1.0 - ny * ny)))
            ymins.append(y_center - extent)
        if not ymins:
            return 0.0
        return float(_clearance) - float(min(ymins))

    poses = compute_poses(phi_deg, psi_deg, theta_deg)

    # Color configuration
    if drone_color == "multi":
        colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
    else:
        colors = [drone_color] * 4  # 全ロータを同じ色で描画

    # Collect all geometry (with drone_offset applied)
    # drone_offset はドローンボディ中心（ヒンジ中心）の絶対位置を指定
    all_points = []
    arm_segments = []  # list of (p0, arm_tip, color)
    rotor_circles = []  # list of (rotor_center, normal, arm_tip, color)

    for i, pose in enumerate(poses):
        c = colors[i % len(colors)]
        p0 = pose.hinge + drone_offset  # ドローン中心 = drone_center
        arm_tip = p0 + arm_length_m * pose.arm_dir  # アーム先端
        # ロータ中心を流入側（推力方向）にオフセット
        rotor_center = arm_tip + rotor_inflow_offset * pose.rotor_normal
        arm_segments.append((p0, arm_tip, c))
        rotor_circles.append((rotor_center, pose.rotor_normal, arm_tip, c))
        
        all_points.append(p0)
        all_points.append(arm_tip)
        circ = _circle_points(center=rotor_center, normal=pose.rotor_normal, radius=rotor_radius_m, n=100)
        all_points.extend(circ)

    all_points = np.array(all_points)
    pad = rotor_radius_m * 0.3

    # Compute minimum distance from rotor edge to wall (y=0)
    min_rotor_to_wall_distance = float("inf")
    for (rotor_center, rotor_normal, _, _) in rotor_circles:
        # ロータ円周上の点を計算
        circ = _circle_points(center=rotor_center, normal=rotor_normal, radius=rotor_radius_m, n=360)
        # Y座標の最小値（壁に最も近い点）
        min_y = circ[:, 1].min()
        # 壁（y=0）からの距離
        distance_to_wall = min_y  # y=0 からの距離（負なら壁に食い込んでいる）
        min_rotor_to_wall_distance = min(min_rotor_to_wall_distance, distance_to_wall)

    # Compute limits for each projection
    x_min, x_max = all_points[:, 0].min() - pad, all_points[:, 0].max() + pad
    y_min, y_max = all_points[:, 1].min() - pad, all_points[:, 1].max() + pad
    z_min, z_max = all_points[:, 2].min() - pad, all_points[:, 2].max() + pad

    # Ensure y=0 is always included in the range (for wall visibility)
    if draw_y0_plane:
        y_min = min(y_min, -pad)
        y_max = max(y_max, pad)

    # Make aspect equal
    def equalize_range(lo, hi):
        center = (lo + hi) / 2
        half = (hi - lo) / 2
        return center, half

    max_half = max(
        (x_max - x_min) / 2,
        (y_max - y_min) / 2,
        (z_max - z_min) / 2,
    )
    x_center, _ = equalize_range(x_min, x_max)
    y_center, _ = equalize_range(y_min, y_max)
    z_center, _ = equalize_range(z_min, z_max)

    # Create figure
    # Layout: 3D(左上), Top(右上), Side(左下), Front(右下)
    if include_3d_view and _HAS_3D:
        fig = plt.figure(figsize=(14, 10))
        ax_3d = fig.add_subplot(2, 2, 1, projection="3d")
        ax_top = fig.add_subplot(2, 2, 2)
        ax_side = fig.add_subplot(2, 2, 3)
        ax_front = fig.add_subplot(2, 2, 4)
    else:
        fig = plt.figure(figsize=(14, 5))
        ax_top = fig.add_subplot(1, 3, 1)
        ax_side = fig.add_subplot(1, 3, 2)
        ax_front = fig.add_subplot(1, 3, 3)
        ax_3d = None

    def draw_2d_view(ax, proj_func, depth_func, xlabel, ylabel, title):
        """
        proj_func: 3Dポイントを2D座標に変換する関数 (x, y, z) -> (u, v)
        depth_func: 奥行きを計算する関数 (x, y, z) -> depth (小さいほど手前)
        """
        # Body outline (hinge square)
        hinges_for_outline = [pose.hinge + drone_offset for pose in poses]
        hs = np.array(hinges_for_outline + [hinges_for_outline[0]])
        hs_2d = np.array([proj_func(p) for p in hs])
        ax.plot(hs_2d[:, 0], hs_2d[:, 1], color=body_color, linewidth=1.5, label="body", zorder=1)

        # Sort arms and rotors by depth (draw back to front)
        arm_rotor_data = list(zip(arm_segments, rotor_circles))
        # Sort by depth of rotor center (larger depth = farther back = draw first)
        arm_rotor_data_sorted = sorted(
            arm_rotor_data,
            key=lambda x: depth_func(x[1][0]),  # x[1][0] is rotor center
            reverse=True,  # 奥から描画（奥のものは小さいzorder）
        )

        # Arms and rotors (sorted by depth, back to front)
        for idx, ((p0, arm_tip, rotor_c), (rotor_center, normal, _, _)) in enumerate(arm_rotor_data_sorted):
            base_zorder = 10 + idx * 10  # 手前のものほど大きいzorder

            # Arm (hinge to arm tip) - use body_color
            p0_2d = proj_func(p0)
            arm_tip_2d = proj_func(arm_tip)
            ax.plot([p0_2d[0], arm_tip_2d[0]], [p0_2d[1], arm_tip_2d[1]], color=body_color, linewidth=2.5, zorder=base_zorder)
            ax.scatter([p0_2d[0]], [p0_2d[1]], color=body_color, s=25, zorder=base_zorder + 1)
            ax.scatter([arm_tip_2d[0]], [arm_tip_2d[1]], color=body_color, s=35, zorder=base_zorder + 2)

            # Line from arm tip to rotor center - use body_color
            rotor_center_2d = proj_func(rotor_center)
            ax.plot([arm_tip_2d[0], rotor_center_2d[0]], [arm_tip_2d[1], rotor_center_2d[1]], 
                    color=body_color, linewidth=1.5, zorder=base_zorder + 2)

            # Rotor circle - use rotor_c (drone_color)
            circ = _circle_points(center=rotor_center, normal=normal, radius=rotor_radius_m, n=100)
            circ_2d = np.array([proj_func(pt) for pt in circ])
            ax.plot(circ_2d[:, 0], circ_2d[:, 1], color=rotor_c, linewidth=1.2, zorder=base_zorder + 3)

            # Rotor normal arrow - use rotor_c (drone_color)
            n_scale = rotor_radius_m * 0.6
            p2 = rotor_center + n_scale * normal
            p2_2d = proj_func(p2)
            ax.annotate(
                "",
                xy=(p2_2d[0], p2_2d[1]),
                xytext=(rotor_center_2d[0], rotor_center_2d[1]),
                arrowprops=dict(arrowstyle="->", color=rotor_c, lw=1.0, zorder=base_zorder + 4),
                zorder=base_zorder + 4,
            )

        ax.set_aspect("equal", adjustable="box")
        if hide_decorations:
            ax.set_axis_off()
        else:
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.set_title(title)
            ax.grid(True, alpha=0.3, zorder=0)

    # Front view: +X方向から見る -> YZ平面 (壁効果の説明用: Y軸で壁との距離を表示)
    def proj_front(p):
        return np.array([p[1], p[2]])

    def depth_front(p):
        return -p[0]  # +X から見る: X が大きいほど手前（depth小）

    draw_2d_view(ax_front, proj_front, depth_front, "Y [m]", "Z [m]", "Front View (from +X)")
    ax_front.set_xlim(y_center - max_half, y_center + max_half)
    ax_front.set_ylim(z_center - max_half, z_center + max_half)
    if draw_y0_plane and not hide_decorations:
        ax_front.axvline(x=0, color="gray", linestyle="--", alpha=0.5, label="y=0 (wall)")

    # Side view: -Y方向から見る -> XZ平面
    def proj_side(p):
        return np.array([p[0], p[2]])

    def depth_side(p):
        return p[1]  # -Y から見る: Y が小さいほど手前（depth小）

    draw_2d_view(ax_side, proj_side, depth_side, "X [m]", "Z [m]", "Side View (from -Y)")
    ax_side.set_xlim(x_center - max_half, x_center + max_half)
    ax_side.set_ylim(z_center - max_half, z_center + max_half)
    if draw_y0_plane:
        # Y=0 plane は側面図では見えない（視線方向）
        pass

    # Top view: +Z方向から見る -> XY平面 (Y横軸、X縦軸反転 - Front viewの横軸Yと整合)
    def proj_top(p):
        return np.array([p[1], p[0]])  # (Y, X) -> Y横軸, X縦軸

    def depth_top(p):
        return -p[2]  # +Z から見る: Z が大きいほど手前（depth小）

    draw_2d_view(ax_top, proj_top, depth_top, "Y [m]", "X [m]", "Top View (from +Z)")
    ax_top.set_xlim(y_center - max_half, y_center + max_half)
    ax_top.set_ylim(x_center + max_half, x_center - max_half)  # X軸反転
    if draw_y0_plane and not hide_decorations:
        ax_top.axvline(x=0, color="gray", linestyle="--", alpha=0.5, label="y=0")

    # 3D view (if enabled)
    if ax_3d is not None:
        # Body outline
        hinges_for_outline = [pose.hinge + drone_offset for pose in poses]
        hs = np.array(hinges_for_outline + [hinges_for_outline[0]])
        ax_3d.plot(hs[:, 0], hs[:, 1], hs[:, 2], color=body_color, linewidth=1.5)

        for (p0, arm_tip, rotor_c), (rotor_center, normal, _, _) in zip(arm_segments, rotor_circles):
            # Arm (hinge to arm tip) - use body_color
            ax_3d.plot([p0[0], arm_tip[0]], [p0[1], arm_tip[1]], [p0[2], arm_tip[2]], color=body_color, linewidth=2.5)
            ax_3d.scatter([p0[0]], [p0[1]], [p0[2]], color=body_color, s=25)
            ax_3d.scatter([arm_tip[0]], [arm_tip[1]], [arm_tip[2]], color=body_color, s=35)

            # Line from arm tip to rotor center - use body_color
            ax_3d.plot([arm_tip[0], rotor_center[0]], [arm_tip[1], rotor_center[1]], [arm_tip[2], rotor_center[2]], 
                       color=body_color, linewidth=1.5)

            # Rotor circle - use rotor_c (drone_color)
            circ = _circle_points(center=rotor_center, normal=normal, radius=rotor_radius_m, n=100)
            ax_3d.plot(circ[:, 0], circ[:, 1], circ[:, 2], color=rotor_c, linewidth=1.2)

            # Rotor normal arrow - use rotor_c (drone_color)
            n_scale = rotor_radius_m * 0.6
            p2 = rotor_center + n_scale * normal
            ax_3d.plot([rotor_center[0], p2[0]], [rotor_center[1], p2[1]], [rotor_center[2], p2[2]], color=rotor_c, linewidth=1.0)

        # Set initial camera angle (X軸寄りに傾ける)
        # elev: 仰角 (Z軸からの角度), azim: 方位角 (XY平面上の回転)
        ax_3d.view_init(elev=25, azim=-30)

        # Equal aspect
        ax_3d.set_xlim(x_center - max_half, x_center + max_half)
        ax_3d.set_ylim(y_center - max_half, y_center + max_half)
        ax_3d.set_zlim(z_center - max_half, z_center + max_half)

        if hide_decorations:
            ax_3d.set_axis_off()
        else:
            # Axes arrows
            axis_len = max(arm_length_m + rotor_radius_m, 0.15) * 0.5
            origin = np.array([0.0, 0.0, 0.0])
            ax_3d.quiver(origin[0], origin[1], origin[2], 1, 0, 0, length=axis_len, color="r")
            ax_3d.quiver(origin[0], origin[1], origin[2], 0, 1, 0, length=axis_len, color="g")
            ax_3d.quiver(origin[0], origin[1], origin[2], 0, 0, 1, length=axis_len, color="b")

            ax_3d.set_xlabel("X [m]")
            ax_3d.set_ylabel("Y [m]")
            ax_3d.set_zlabel("Z [m]")
            ax_3d.set_title("3D View")

            if draw_y0_plane:
                xs = np.linspace(x_center - max_half, x_center + max_half, 2)
                zs = np.linspace(z_center - max_half, z_center + max_half, 2)
                X, Z = np.meshgrid(xs, zs)
                Y = np.zeros_like(X)
                ax_3d.plot_surface(X, Y, Z, color="gray", alpha=0.12, shade=False)

    # Title with rotor-to-wall distance
    if not hide_decorations:
        wall_dist_str = f"{min_rotor_to_wall_distance * 1000:.1f}mm"
        if min_rotor_to_wall_distance < 0:
            wall_dist_str += " (collision!)"
        fig.suptitle(
            f"Morphing Drone - Three View\n"
            f"phi={phi_deg:.1f}°, psi={psi_deg:.1f}°, theta={theta_deg:.1f}°, "
            f"L={arm_length_m:.3f}m, R={rotor_radius_m:.4f}m\n"
            f"Rotor inflow offset={rotor_inflow_offset * 1000:.1f}mm, "
            f"Min rotor-to-wall distance={wall_dist_str}",
            fontsize=11,
        )

    plt.tight_layout()

    if save_path:
        logging.info("Saving three-view figure to %s (dpi=%d)...", save_path, int(dpi))
        fig.savefig(save_path, dpi=int(dpi), bbox_inches="tight")
        logging.info("Saved: %s", save_path)

    if show:
        logging.info("Calling plt.show()...")
        plt.show()
    else:
        logging.info("Skipping plt.show() because show=False")

    plt.close(fig)
    logging.info("Three-view figure closed; done.")


def main():
    parser = argparse.ArgumentParser(description="Visualize morphing drone arms/rotors with fold/slant/tilt angles.")
    parser.add_argument(
        "--backend",
        type=str,
        default="auto",
        choices=["auto", "QtAgg", "TkAgg", "Agg"],
        help="Matplotlib backend. 'auto' prefers QtAgg if PySide6 is available. Default: auto",
    )
    parser.add_argument("--cx", type=float, default=0.035, help="Hinge x-offset [m]. Default: 0.035")
    parser.add_argument("--cy", type=float, default=0.035, help="Hinge y-offset [m]. Default: 0.035")
    parser.add_argument("--arm-length", type=float, default=0.18, help="Arm length from hinge to rotor center [m]. Default: 0.18")
    parser.add_argument("--rotor-radius-in", type=float, default=3.5, help="Rotor radius [inch]. Default: 3.5")
    parser.add_argument("--phi", type=float, default=0.0, help="Fold angle Phi [deg]. Default: 0")
    parser.add_argument("--psi", type=float, default=0.0, help="Slant angle Psi [deg]. Default: 0")
    parser.add_argument("--theta", type=float, default=0.0, help="Tilt angle Theta [deg]. Default: 0")
    parser.add_argument(
        "--symmetry",
        type=str,
        default="mirror_xy",
        choices=["mirror_xy", "none"],
        help="How to build 4 arms. 'mirror_xy' enforces symmetry about X/Y axes by mirroring one computed arm.",
    )
    parser.add_argument("--force-2d", action="store_true", help="Force 2D projections even if 3D is available.")
    parser.add_argument(
        "--y-clearance",
        type=float,
        default=0.0,
        help="Offset drone in +y so that the closest rotor rim point has distance y_clearance to the y=0 plane. Default: 0.0",
    )
    parser.add_argument("--no-y0-plane", action="store_true", help="Do not draw y=0 plane (or y=0 line in 2D projections).")
    parser.add_argument("--save", type=str, default=None, help="Save figure to a file (e.g. out.png).")
    parser.add_argument("--dpi", type=int, default=200, help="DPI for --save. Default: 200")
    parser.add_argument("--no-show", action="store_true", help="Do not open a window (useful with --save on WSL/headless).")
    parser.add_argument("--no-sliders", action="store_true", help="Disable slider UI (enabled by default when showing).")
    parser.add_argument(
        "--three-view",
        action="store_true",
        help="Output three-view diagram (Front/Side/Top) with optional 3D view.",
    )
    parser.add_argument(
        "--three-view-only",
        action="store_true",
        help="Output three-view diagram without 3D view subplot.",
    )
    parser.add_argument(
        "--drone-center-x",
        type=float,
        default=0.0,
        help="Drone center X offset [m]. Default: 0.0",
    )
    parser.add_argument(
        "--drone-center-y",
        type=float,
        default=0.0,
        help="Drone center Y offset [m]. Default: 0.0",
    )
    parser.add_argument(
        "--drone-center-z",
        type=float,
        default=0.0,
        help="Drone center Z offset [m]. Default: 0.0",
    )
    parser.add_argument(
        "--rotor-inflow-offset",
        type=float,
        default=0.02,
        help="Rotor center offset toward inflow side (opposite of thrust direction) [m]. Default: 0.02 (20mm)",
    )
    parser.add_argument(
        "--hide-decorations",
        action="store_true",
        help="Hide axis labels, grid, titles, and wall lines (show only the drone).",
    )
    parser.add_argument(
        "--drone-color",
        type=str,
        default="multi",
        help="Rotor color. 'multi' for 4 different colors, or specify a single color (e.g. 'black', 'blue', '#FF0000'). Default: multi",
    )
    parser.add_argument(
        "--body-color",
        type=str,
        default="gray",
        help="Body color (hinge square, arms). Default: gray",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level. Default: INFO",
    )
    parser.add_argument(
        "--mpl-log-level",
        type=str,
        default="WARNING",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level for matplotlib/PIL internals. Default: WARNING",
    )
    parser.add_argument(
        "--log-env",
        action="store_true",
        help="Log matplotlib backend and GUI-related environment variables.",
    )
    args = parser.parse_args()

    _setup_logging(str(args.log_level))
    _set_library_log_levels(str(args.mpl_log_level))
    logging.info("Args: %s", vars(args))

    # Apply backend selection BEFORE importing pyplot anywhere.
    backend_arg = str(args.backend)
    if backend_arg != "auto":
        mpl.use(backend_arg, force=True)
        logging.info("Requested backend=%s", backend_arg)
    else:
        # Auto mode: respect explicit MPLBACKEND, otherwise prefer QtAgg if PySide6 is present.
        if os.environ.get("MPLBACKEND"):
            logging.info("Auto backend: respecting env MPLBACKEND=%s", os.environ.get("MPLBACKEND"))
        else:
            try:
                import PySide6  # noqa: F401

                mpl.use("QtAgg", force=True)
                logging.info("Auto backend: selected QtAgg (PySide6 available)")
            except Exception as e:
                logging.debug("Auto backend: QtAgg not selected (%r); using matplotlib default", e)

    if bool(args.log_env):
        _log_runtime_env()

    rotor_radius_m = float(args.rotor_radius_in) * 0.0254

    # Check if three-view mode is requested
    use_three_view = bool(args.three_view) or bool(args.three_view_only)

    if use_three_view:
        plot_three_view_drone(
            cx=float(args.cx),
            cy=float(args.cy),
            rotor_radius_m=rotor_radius_m,
            arm_length_m=float(args.arm_length),
            phi_deg=float(args.phi),
            psi_deg=float(args.psi),
            theta_deg=float(args.theta),
            symmetry=str(args.symmetry),
            y_clearance=float(args.y_clearance),
            draw_y0_plane=(not bool(args.no_y0_plane)),
            save_path=(str(args.save) if args.save else None),
            dpi=int(args.dpi),
            show=(not bool(args.no_show)),
            include_3d_view=(not bool(args.three_view_only)),
            drone_center=(
                float(args.drone_center_x),
                float(args.drone_center_y),
                float(args.drone_center_z),
            ),
            rotor_inflow_offset=float(args.rotor_inflow_offset),
            hide_decorations=bool(args.hide_decorations),
            drone_color=str(args.drone_color),
            body_color=str(args.body_color),
        )
    else:
        plot_morphing_drone(
            cx=float(args.cx),
            cy=float(args.cy),
            rotor_radius_m=rotor_radius_m,
            arm_length_m=float(args.arm_length),
            phi_deg=float(args.phi),
            psi_deg=float(args.psi),
            theta_deg=float(args.theta),
            symmetry=str(args.symmetry),
            force_2d=bool(args.force_2d),
            y_clearance=float(args.y_clearance),
            draw_y0_plane=(not bool(args.no_y0_plane)),
            save_path=(str(args.save) if args.save else None),
            dpi=int(args.dpi),
            show=(not bool(args.no_show)),
            sliders_enabled=(not bool(args.no_sliders)),
        )


if __name__ == "__main__":
    main()

