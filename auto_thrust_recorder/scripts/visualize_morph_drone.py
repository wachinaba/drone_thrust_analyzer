import argparse
import logging
import os
import sys
from dataclasses import dataclass

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

    phi = _deg2rad(phi_deg)
    psi = _deg2rad(psi_deg)
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
    save_path: str | None = None,
    dpi: int = 200,
    show: bool = True,
    sliders_enabled: bool = True,
):
    # Import pyplot lazily so that main() can set backend beforehand.
    import matplotlib.pyplot as plt

    """
    Assumption (documented):
      - Quadrotor with 4 arms.
      - Hinge centers are at (±cx, ±cy, 0).
      - Each arm initially points to the outward diagonal direction of its quadrant:
          (sign(x), sign(y), 0), normalized.
    """
    if symmetry not in {"none", "mirror_xy"}:
        raise ValueError(f"Unknown symmetry mode: {symmetry}")

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

    title = (
        f"Morphing drone visualization\n"
        f"cx={cx:.3f} m, cy={cy:.3f} m, R={rotor_radius_m:.4f} m, L={arm_length_m:.3f} m\n"
        f"fold(phi)={phi_deg:.1f} deg, slant(psi)={psi_deg:.1f} deg, tilt(theta)={theta_deg:.1f} deg"
    )

    use_3d = _HAS_3D and (not force_2d)
    if not use_3d and _AXES3D_IMPORT_ERROR is not None:
        print(
            "Warning: 3D projection is unavailable in this Python environment.\n"
            f"  Reason: {_AXES3D_IMPORT_ERROR}\n"
            "  Falling back to 2D projections (xy/xz/yz).\n"
            "  If you want 3D, ensure matplotlib and mpl_toolkits are from the same installation."
        )

    hinges_for_outline = [pose.hinge for pose in poses]
    hs = np.array(hinges_for_outline + [hinges_for_outline[0]])
    colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]

    def _format_title(_L: float, _phi: float, _psi: float, _theta: float) -> str:
        return (
            f"Morphing drone visualization\n"
            f"cx={cx:.3f} m, cy={cy:.3f} m, R={rotor_radius_m:.4f} m, L={_L:.3f} m\n"
            f"fold(phi)={_phi:.1f} deg, slant(psi)={_psi:.1f} deg, tilt(theta)={_theta:.1f} deg"
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
        ax.set_title(_format_title(arm_length_m, phi_deg, psi_deg, theta_deg))

        # Body outline (hinge square)
        body_line = ax.plot(hs[:, 0], hs[:, 1], hs[:, 2], color="k", linewidth=1.5, label="hinge square")[0]

        arm_lines = []
        hinge_pts = []
        tip_pts = []
        rotor_lines = []
        normal_lines = []

        for i, pose in enumerate(poses):
            c = colors[i % len(colors)]
            p0 = pose.hinge
            p1 = pose.hinge + arm_length_m * pose.arm_dir

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
        ax.quiver(origin[0], origin[1], origin[2], 1, 0, 0, length=axis_len, color="r")
        ax.quiver(origin[0], origin[1], origin[2], 0, 1, 0, length=axis_len, color="g")
        ax.quiver(origin[0], origin[1], origin[2], 0, 0, 1, length=axis_len, color="b")
        ax.text(axis_len, 0, 0, "x", color="r")
        ax.text(0, axis_len, 0, "y", color="g")
        ax.text(0, 0, axis_len, "z", color="b")

        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")

        _compute_limits(poses, arm_length_m)

        sliders_enabled = bool(sliders_enabled) and bool(show)
        if sliders_enabled:
            from matplotlib.widgets import Slider, Button

            fig.subplots_adjust(bottom=0.27)
            ax_phi = fig.add_axes([0.12, 0.18, 0.76, 0.03])
            ax_psi = fig.add_axes([0.12, 0.14, 0.76, 0.03])
            ax_theta = fig.add_axes([0.12, 0.10, 0.76, 0.03])
            ax_L = fig.add_axes([0.12, 0.06, 0.76, 0.03])
            ax_reset = fig.add_axes([0.82, 0.01, 0.12, 0.04])

            s_phi = Slider(ax_phi, "phi [deg]", -180.0, 180.0, valinit=float(phi_deg), valstep=1.0)
            s_psi = Slider(ax_psi, "psi [deg]", -90.0, 90.0, valinit=float(psi_deg), valstep=1.0)
            s_theta = Slider(ax_theta, "theta [deg]", -180.0, 180.0, valinit=float(theta_deg), valstep=1.0)
            s_L = Slider(ax_L, "arm L [m]", 0.05, 0.50, valinit=float(arm_length_m), valstep=0.005)
            b_reset = Button(ax_reset, "Reset")

            def _update(_val=None):
                new_phi = float(s_phi.val)
                new_psi = float(s_psi.val)
                new_theta = float(s_theta.val)
                new_L = float(s_L.val)

                new_poses = compute_poses(new_phi, new_psi, new_theta)
                new_hs = np.array([p.hinge for p in new_poses] + [new_poses[0].hinge])
                _set_3d_line(body_line, new_hs[:, 0], new_hs[:, 1], new_hs[:, 2])

                for i, pose in enumerate(new_poses):
                    p0 = pose.hinge
                    p1 = pose.hinge + new_L * pose.arm_dir
                    _set_3d_line(arm_lines[i], [p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]])
                    _set_3d_point(hinge_pts[i], p0)
                    _set_3d_point(tip_pts[i], p1)

                    circ = _circle_points(center=p1, normal=pose.rotor_normal, radius=rotor_radius_m, n=200)
                    _set_3d_line(rotor_lines[i], circ[:, 0], circ[:, 1], circ[:, 2])

                    n_scale = rotor_radius_m * 0.8
                    p2 = p1 + n_scale * pose.rotor_normal
                    _set_3d_line(normal_lines[i], [p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]])

                ax.set_title(_format_title(new_L, new_phi, new_psi, new_theta))
                _compute_limits(new_poses, new_L)
                fig.canvas.draw_idle()

            def _reset(_event=None):
                s_phi.reset()
                s_psi.reset()
                s_theta.reset()
                s_L.reset()

            s_phi.on_changed(_update)
            s_psi.on_changed(_update)
            s_theta.on_changed(_update)
            s_L.on_changed(_update)
            b_reset.on_clicked(_reset)
        else:
            plt.tight_layout()
    else:
        # 2D fallback: xy / xz / yz projections
        if sliders_enabled and show:
            logging.warning("Sliders are currently implemented only for 3D mode; continuing without sliders (2D fallback).")
        fig, axs = plt.subplots(1, 3, figsize=(13, 4))
        fig.suptitle(title)

        views = [
            ("xy", (0, 1), ("x [m]", "y [m]")),
            ("xz", (0, 2), ("x [m]", "z [m]")),
            ("yz", (1, 2), ("y [m]", "z [m]")),
        ]

        for ax, (name, (i0, i1), (xl, yl)) in zip(axs, views, strict=True):
            ax.set_title(name)
            ax.set_xlabel(xl)
            ax.set_ylabel(yl)
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True, alpha=0.3)
            ax.plot(hs[:, i0], hs[:, i1], color="k", linewidth=1.5)

        all_proj = []
        for i, pose in enumerate(poses):
            c = colors[i % len(colors)]
            p0 = pose.hinge
            p1 = pose.hinge + arm_length_m * pose.arm_dir
            circ = _circle_points(center=p1, normal=pose.rotor_normal, radius=rotor_radius_m, n=200)
            all_proj.append(p0)
            all_proj.append(p1)
            all_proj.append(circ)

            for ax, (_, (i0, i1), _) in zip(axs, views, strict=True):
                ax.plot([p0[i0], p1[i0]], [p0[i1], p1[i1]], color=c, linewidth=2.5)
                ax.scatter([p0[i0]], [p0[i1]], color=c, s=20)
                ax.scatter([p1[i0]], [p1[i1]], color=c, s=25)
                ax.plot(circ[:, i0], circ[:, i1], color=c, linewidth=1.2)

        # Set common limits with padding
        pts = []
        for item in all_proj:
            if isinstance(item, np.ndarray) and item.ndim == 2:
                pts.append(item)
            else:
                pts.append(np.array(item, dtype=float).reshape(1, 3))
        P = np.vstack(pts)
        pad = rotor_radius_m * 1.2

        # xy
        axs[0].set_xlim(P[:, 0].min() - pad, P[:, 0].max() + pad)
        axs[0].set_ylim(P[:, 1].min() - pad, P[:, 1].max() + pad)
        # xz
        axs[1].set_xlim(P[:, 0].min() - pad, P[:, 0].max() + pad)
        axs[1].set_ylim(P[:, 2].min() - pad, P[:, 2].max() + pad)
        # yz
        axs[2].set_xlim(P[:, 1].min() - pad, P[:, 1].max() + pad)
        axs[2].set_ylim(P[:, 2].min() - pad, P[:, 2].max() + pad)

        plt.tight_layout()

    if fig is None:
        raise RuntimeError("internal error: figure was not created")

    if save_path:
        logging.info("Saving figure to %s (dpi=%d)...", save_path, int(dpi))
        fig.savefig(save_path, dpi=int(dpi), bbox_inches="tight")
        logging.info("Saved: %s", save_path)

    if show:
        logging.info("Calling plt.show()...")
        plt.show()
    else:
        logging.info("Skipping plt.show() because show=False")

    plt.close(fig)
    logging.info("Figure closed; done.")


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
    parser.add_argument("--save", type=str, default=None, help="Save figure to a file (e.g. out.png).")
    parser.add_argument("--dpi", type=int, default=200, help="DPI for --save. Default: 200")
    parser.add_argument("--no-show", action="store_true", help="Do not open a window (useful with --save on WSL/headless).")
    parser.add_argument("--no-sliders", action="store_true", help="Disable slider UI (enabled by default when showing).")
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
        save_path=(str(args.save) if args.save else None),
        dpi=int(args.dpi),
        show=(not bool(args.no_show)),
        sliders_enabled=(not bool(args.no_sliders)),
    )


if __name__ == "__main__":
    main()

