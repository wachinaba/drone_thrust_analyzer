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


@dataclass(frozen=True)
class DotGridSpec:
    """
    ドットグリッドの定義（複数可）。

    生成点（ローカル）:
      p(i,j) = origin + i*u + j*v,  i=0..nu-1, j=0..nv-1

    frame（平行移動のみ、回転なし）:
      - world: 変換なし
      - drone: ドローン原点へ平行移動
      - rotor: 指定ロータ中心へ平行移動
      - rotor_between: 指定2ロータ中心の中点へ平行移動
    """

    frame: str = "world"  # "world" | "drone" | "rotor" | "rotor_between"
    origin: np.ndarray = np.zeros(3, dtype=float)
    u: np.ndarray = np.array([1.0, 0.0, 0.0], dtype=float)
    v: np.ndarray = np.array([0.0, 1.0, 0.0], dtype=float)
    nu: int = 1
    nv: int = 1
    # frame-specific
    rotor: int | None = None
    rotors: tuple[int, int] | None = None
    # style (matplotlib scatter)
    color: str = "gray"
    alpha: float = 0.25
    size: float = 6.0
    marker: str = "."
    edgecolor: str | None = None
    lw: float = 0.0


def _parse_vec3_csv(s: str) -> np.ndarray:
    parts = [p.strip() for p in str(s).split(",") if p.strip() != ""]
    if len(parts) != 3:
        raise ValueError(f"Expected 3 comma-separated numbers, got: {s!r}")
    return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=float)


def _parse_kv_semicolon(s: str) -> dict[str, str]:
    out: dict[str, str] = {}
    raw = str(s).strip()
    if not raw:
        return out
    for item in raw.split(";"):
        item = item.strip()
        if not item:
            continue
        if "=" not in item:
            raise ValueError(f"Invalid token (expected key=value): {item!r}")
        k, v = item.split("=", 1)
        k = k.strip().lower()
        v = v.strip()
        if not k:
            raise ValueError(f"Empty key in token: {item!r}")
        out[k] = v
    return out


def parse_dot_grid_spec(spec_str: str) -> DotGridSpec:
    """
    CLI string -> DotGridSpec
    Format: "key=value;key=value;..."
    Required keys: origin, u, v, nu, nv
    Optional keys:
      frame=world|drone|rotor|rotor_between
      rotor=<int> (frame=rotor)
      rotors=a,b (frame=rotor_between)
      color, alpha, size, marker, edgecolor, lw
    """
    kv = _parse_kv_semicolon(spec_str)
    frame = str(kv.get("frame", "world")).strip().lower()

    def req(name: str) -> str:
        if name not in kv:
            raise ValueError(f"Missing required key: {name}")
        return str(kv[name])

    origin = _parse_vec3_csv(req("origin"))
    u = _parse_vec3_csv(req("u"))
    v = _parse_vec3_csv(req("v"))
    nu = int(float(req("nu")))
    nv = int(float(req("nv")))
    if nu <= 0 or nv <= 0:
        raise ValueError(f"nu/nv must be positive integers, got nu={nu}, nv={nv}")

    rotor: int | None = None
    rotors: tuple[int, int] | None = None
    if frame == "rotor":
        if "rotor" not in kv:
            raise ValueError("frame=rotor requires key 'rotor=<index>'")
        rotor = int(float(str(kv["rotor"])))
    elif frame == "rotor_between":
        if "rotors" not in kv:
            raise ValueError("frame=rotor_between requires key 'rotors=a,b'")
        parts = [p.strip() for p in str(kv["rotors"]).split(",") if p.strip() != ""]
        if len(parts) != 2:
            raise ValueError(f"rotors must be 'a,b', got: {kv['rotors']!r}")
        rotors = (int(float(parts[0])), int(float(parts[1])))
    elif frame in {"world", "drone"}:
        pass
    else:
        raise ValueError(f"Unknown frame: {frame!r}")

    color = str(kv.get("color", "gray"))
    alpha = float(kv.get("alpha", 0.25))
    size = float(kv.get("size", 6.0))
    marker = str(kv.get("marker", "."))
    edgecolor = (str(kv["edgecolor"]) if "edgecolor" in kv else None)
    lw = float(kv.get("lw", 0.0))

    return DotGridSpec(
        frame=frame,
        origin=origin,
        u=u,
        v=v,
        nu=nu,
        nv=nv,
        rotor=rotor,
        rotors=rotors,
        color=color,
        alpha=alpha,
        size=size,
        marker=marker,
        edgecolor=edgecolor,
        lw=lw,
    )


def _dot_grid_points_local(spec: DotGridSpec) -> np.ndarray:
    i = np.arange(int(spec.nu), dtype=float)[:, None]  # (nu,1)
    j = np.arange(int(spec.nv), dtype=float)[None, :]  # (1,nv)
    pts = (
        spec.origin[None, None, :]
        + i[:, :, None] * spec.u[None, None, :]
        + j[:, :, None] * spec.v[None, None, :]
    )
    return pts.reshape(-1, 3)


def _dot_grid_translation(spec: DotGridSpec, *, drone_origin: np.ndarray, rotor_centers: list[np.ndarray]) -> np.ndarray:
    frame = str(spec.frame).lower()
    if frame == "world":
        return np.zeros(3, dtype=float)
    if frame == "drone":
        return np.asarray(drone_origin, dtype=float).reshape(3)
    if frame == "rotor":
        if spec.rotor is None:
            raise ValueError("DotGridSpec.frame='rotor' requires spec.rotor")
        idx = int(spec.rotor)
        if idx < 0 or idx >= len(rotor_centers):
            raise ValueError(f"rotor index out of range: {idx} (0..{len(rotor_centers)-1})")
        return np.asarray(rotor_centers[idx], dtype=float).reshape(3)
    if frame == "rotor_between":
        if spec.rotors is None:
            raise ValueError("DotGridSpec.frame='rotor_between' requires spec.rotors=(a,b)")
        a, b = int(spec.rotors[0]), int(spec.rotors[1])
        if a < 0 or a >= len(rotor_centers) or b < 0 or b >= len(rotor_centers):
            raise ValueError(f"rotors indices out of range: ({a},{b}) (0..{len(rotor_centers)-1})")
        return 0.5 * (np.asarray(rotor_centers[a], dtype=float).reshape(3) + np.asarray(rotor_centers[b], dtype=float).reshape(3))
    raise ValueError(f"Unknown frame: {frame!r}")


def _dot_grid_points_world(spec: DotGridSpec, *, drone_origin: np.ndarray, rotor_centers: list[np.ndarray]) -> np.ndarray:
    pts = _dot_grid_points_local(spec)
    t = _dot_grid_translation(spec, drone_origin=drone_origin, rotor_centers=rotor_centers)
    return pts + t[None, :]


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


def _thrust_vec_from_alpha_beta_deg(alpha_deg: float, beta_deg: float) -> np.ndarray:
    """
    plot_morphing_drone() 内の thrust_angles_alpha_beta_deg() の逆変換（同じ符号規約）。

    定義（既存）:
      alpha = -atan2(vy, vz) [deg]
      beta  = -atan2(vx, vz) [deg]
    ここから unit vector v を復元する。
    """
    a = _deg2rad(float(alpha_deg))
    b = _deg2rad(float(beta_deg))
    kx = -float(np.tan(b))  # vx = kx * vz
    ky = -float(np.tan(a))  # vy = ky * vz
    vz = 1.0 / float(np.sqrt(1.0 + kx * kx + ky * ky))
    vx = kx * vz
    vy = ky * vz
    return _normalize(np.array([vx, vy, vz], dtype=float))


def _arm_dir2_and_rotor_n2_from_phi_psi(arm_dir0: np.ndarray, phi_deg: float, psi_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """
    _make_arm_pose() の fold+slant までを切り出したもの。
    Returns:
      arm_dir2: fold+slant 後のアーム方向(unit)
      rotor_n2: fold+slant 後のロータ法線(unit)（tilt前）
    """
    z = np.array([0.0, 0.0, 1.0], dtype=float)

    # _make_arm_pose() と同じ符号規約（phi/psi は内部で反転）
    phi = _deg2rad(-float(phi_deg))
    psi = _deg2rad(-float(psi_deg))

    R_fold = _rot_z(phi)
    arm_dir1 = _normalize(R_fold @ _normalize(np.asarray(arm_dir0, dtype=float).reshape(3)))

    slant_axis = np.cross(z, arm_dir1)
    if np.linalg.norm(slant_axis) < 1e-10:
        R_slant = np.eye(3, dtype=float)
    else:
        R_slant = _rot_axis_angle(slant_axis, psi)

    arm_dir2 = _normalize(R_slant @ arm_dir1)
    rotor_n2 = _normalize(R_slant @ (R_fold @ z))
    return arm_dir2, rotor_n2


def solve_psi_theta_from_alpha_beta_deg(
    *,
    phi_deg: float,
    alpha_deg: float,
    beta_deg: float,
    psi_search_range: tuple[float, float] = (-90.0, 90.0),
) -> tuple[float, float, float]:
    """
    alpha/beta (deg) から psi/theta (deg) を逆算する（phi は固定）。
    - rotor0（+x,+y象限のアーム）基準
    - psi は 1次元探索で dot(arm_axis, rotor_n2) と dot(arm_axis, target) を一致させる
    - theta はその psi で軸回り回転角を解析的に求める

    Returns:
      (psi_deg, theta_deg, residual_deg)
      residual_deg は最終的なロータ法線と target の角度誤差（deg）
    """
    v_target = _thrust_vec_from_alpha_beta_deg(alpha_deg=float(alpha_deg), beta_deg=float(beta_deg))
    arm_dir0 = _normalize(np.array([+1.0, +1.0, 0.0], dtype=float))  # rotor0

    psi_lo, psi_hi = float(psi_search_range[0]), float(psi_search_range[1])
    psi_lo = max(-90.0, psi_lo)
    psi_hi = min(90.0, psi_hi)

    def f(psi: float) -> float:
        a, u = _arm_dir2_and_rotor_n2_from_phi_psi(arm_dir0, phi_deg=float(phi_deg), psi_deg=float(psi))
        return float(np.dot(a, u) - np.dot(a, v_target))

    def theta_for_psi(psi: float) -> float:
        a, u = _arm_dir2_and_rotor_n2_from_phi_psi(arm_dir0, phi_deg=float(phi_deg), psi_deg=float(psi))
        a = _normalize(a)
        u = _normalize(u)
        v = v_target
        u_perp = u - a * float(np.dot(a, u))
        v_perp = v - a * float(np.dot(a, v))
        nu = float(np.linalg.norm(u_perp))
        nv = float(np.linalg.norm(v_perp))
        if nu < 1e-12 or nv < 1e-12:
            return 0.0
        u_perp /= nu
        v_perp /= nv
        # oriented angle around axis a from u_perp to v_perp
        ang = float(np.arctan2(np.dot(a, np.cross(u_perp, v_perp)), np.dot(u_perp, v_perp)))
        th = float(np.degrees(ang))
        # wrap to [-180, 180]
        if th > 180.0:
            th -= 360.0
        if th < -180.0:
            th += 360.0
        return th

    def residual_deg(psi: float, theta: float) -> float:
        a, u = _arm_dir2_and_rotor_n2_from_phi_psi(arm_dir0, phi_deg=float(phi_deg), psi_deg=float(psi))
        R = _rot_axis_angle(_normalize(a), _deg2rad(float(theta)))
        v = _normalize(R @ _normalize(u))
        dotv = float(np.clip(np.dot(v, v_target), -1.0, 1.0))
        return float(np.degrees(np.arccos(dotv)))

    # Find candidate roots by scanning for sign changes.
    n_scan = 181  # 1 deg step across [-90, 90]
    psis = np.linspace(psi_lo, psi_hi, n_scan)
    fs = np.array([f(float(p)) for p in psis], dtype=float)

    brackets: list[tuple[float, float]] = []
    for i in range(len(psis) - 1):
        f0, f1 = float(fs[i]), float(fs[i + 1])
        if f0 == 0.0:
            brackets.append((float(psis[i]), float(psis[i])))
        elif f0 * f1 < 0.0:
            brackets.append((float(psis[i]), float(psis[i + 1])))

    def bisect(a0: float, a1: float) -> float:
        if a0 == a1:
            return a0
        lo, hi = a0, a1
        flo, fhi = f(lo), f(hi)
        # If not a valid bracket (numerical), just return midpoint.
        if flo == 0.0:
            return lo
        if fhi == 0.0:
            return hi
        if flo * fhi > 0.0:
            return 0.5 * (lo + hi)
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            fm = f(mid)
            if abs(fm) < 1e-10 or abs(hi - lo) < 1e-6:
                return mid
            if flo * fm <= 0.0:
                hi, fhi = mid, fm
            else:
                lo, flo = mid, fm
        return 0.5 * (lo + hi)

    candidates: list[tuple[float, float, float]] = []
    if brackets:
        for (b0, b1) in brackets[:12]:  # cap
            psi_star = bisect(b0, b1)
            th = theta_for_psi(psi_star)
            res = residual_deg(psi_star, th)
            candidates.append((psi_star, th, res))
    else:
        # No sign change found -> pick psi that minimizes |f|.
        idx = int(np.argmin(np.abs(fs)))
        psi_star = float(psis[idx])
        th = theta_for_psi(psi_star)
        res = residual_deg(psi_star, th)
        candidates.append((psi_star, th, res))

    # Select best candidate: smallest residual, then smallest |theta|.
    candidates.sort(key=lambda t: (float(t[2]), abs(float(t[1]))))
    psi_best, th_best, res_best = candidates[0]
    return float(psi_best), float(th_best), float(res_best)


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


def get_drone_yz_polylines(
    *,
    phi_deg: float,
    psi_deg: float = 0.0,
    theta_deg: float = 0.0,
    cx: float = 0.035,
    cy: float = 0.035,
    arm_length_m: float = 0.18,
    rotor_radius_in: float = 3.5,
    rotor_inflow_offset_m: float = 0.0,
    symmetry: str = "mirror_xy",
    circle_n: int = 100,
) -> dict[str, list[np.ndarray]]:
    """Morphing drone の YZ 投影(= front view)用の2Dポリラインを返す。

    Returns dict of polylines in meters:
      - body:  (N,2) arrays of (y,z)
      - arms:  list of (M,2) arrays of (y,z) line segments
      - rotors:list of (K,2) arrays of (y,z) circles
      - rotor_centers: list of (2,) arrays of (y,z) rotor centers
    """
    rotor_radius_m = float(rotor_radius_in) * 0.0254

    if symmetry not in {"mirror_xy", "none"}:
        raise ValueError(f"Unknown symmetry: {symmetry}")

    def _compute_poses(_phi: float, _psi: float, _theta: float):
        base_hinge = np.array([+float(cx), +float(cy), 0.0], dtype=float)
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
            _make_arm_pose(hinge=h, arm_dir0=d0, phi_deg=float(_phi), psi_deg=float(_psi), theta_deg=float(_theta))
            for h, d0 in zip(hinges, arm_dirs0, strict=True)
        ]

    poses = _compute_poses(float(phi_deg), float(psi_deg), float(theta_deg))

    # body outline: hinge square (YZ projection -> (y,z))
    hinges_for_outline = [p.hinge for p in poses]
    hs = np.array(hinges_for_outline + [hinges_for_outline[0]], dtype=float)
    body = np.stack([hs[:, 1], hs[:, 2]], axis=1)

    arms: list[np.ndarray] = []
    rotors: list[np.ndarray] = []
    rotor_centers_yz: list[np.ndarray] = []
    for p in poses:
        hinge = p.hinge
        arm_tip = hinge + float(arm_length_m) * p.arm_dir
        rotor_center = arm_tip + float(rotor_inflow_offset_m) * p.rotor_normal
        rotor_centers_yz.append(np.array([rotor_center[1], rotor_center[2]], dtype=float))
        # hinge -> arm_tip
        arms.append(np.array([[hinge[1], hinge[2]], [arm_tip[1], arm_tip[2]]], dtype=float))
        # arm_tip -> rotor_center
        arms.append(np.array([[arm_tip[1], arm_tip[2]], [rotor_center[1], rotor_center[2]]], dtype=float))
        circ = _circle_points(center=rotor_center, normal=p.rotor_normal, radius=float(rotor_radius_m), n=int(circle_n))
        rotors.append(np.stack([circ[:, 1], circ[:, 2]], axis=1))

    return {"body": [body], "arms": arms, "rotors": rotors, "rotor_centers": rotor_centers_yz}


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
    no_drone: bool = False,
    view_elev: float | None = None,
    view_azim: float | None = None,
    save_path: str | None = None,
    transparent: bool = True,
    drone_lw: float = 1.0,
    dpi: int = 200,
    show: bool = True,
    sliders_enabled: bool = True,
    hide_decorations: bool = False,
    dot_grids: list[DotGridSpec] | None = None,
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

    def format_status_text(poses_list: list[ArmPose], *, arm_length_m: float, dy: float) -> str:
        """
        図中に表示するステータス文字列:
        - 各ロータの alpha/beta
        - 各ロータ中心位置 (x,y,z) [m]（小数3桁）
        """
        lines = ["alpha/beta (deg) from thrust vec (rotor_normal):"]
        for i, p in enumerate(poses_list):
            a, b = thrust_angles_alpha_beta_deg(p.rotor_normal)
            lines.append(f"  rotor{i}: alpha={a:+6.1f}, beta={b:+6.1f}")

        lines.append("rotor center position (x,y,z) [m]:")
        for i, p in enumerate(poses_list):
            # rotor center is at arm tip (hinge + y-shift + L * arm_dir)
            rotor_center = p.hinge + np.array([0.0, float(dy), 0.0], dtype=float) + float(arm_length_m) * p.arm_dir
            lines.append(
                f"  rotor{i}: ({rotor_center[0]:+.3f}, {rotor_center[1]:+.3f}, {rotor_center[2]:+.3f})"
            )
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

    # NOTE: title text is generated by _format_title() (includes alpha/beta).

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
    lw_scale = max(0.0, float(drone_lw))

    def _format_title(_poses: list[ArmPose], _L: float, _phi: float, _psi: float, _theta: float, _y_clear: float) -> str:
        # Show alpha/beta derived from rotor0 thrust vector (rotor_normal).
        # rotor0 is defined as the +x,+y quadrant arm (base_pose), which is poses[0] in both symmetry modes.
        try:
            a0, b0 = thrust_angles_alpha_beta_deg(_poses[0].rotor_normal)
            ab_str = f"alpha={a0:+.1f} deg, beta={b0:+.1f} deg"
        except Exception:
            ab_str = "alpha=nan deg, beta=nan deg"
        return (
            f"Morphing drone visualization\n"
            f"cx={cx:.3f} m, cy={cy:.3f} m, R={rotor_radius_m:.4f} m, L={_L:.3f} m\n"
            f"fold(phi)={_phi:.1f} deg, slant(psi)={_psi:.1f} deg, tilt(theta)={_theta:.1f} deg\n"
            f"{ab_str}\n"
            f"y_clearance={_y_clear:.3f} m"
        )

    def _set_3d_line(line, xs, ys, zs):
        line.set_data(xs, ys)
        line.set_3d_properties(zs)

    def _set_3d_point(scatter, p: np.ndarray):
        scatter._offsets3d = ([float(p[0])], [float(p[1])], [float(p[2])])

    def _set_3d_scatter(scatter, pts: np.ndarray):
        pts = np.asarray(pts, dtype=float)
        scatter._offsets3d = (pts[:, 0].tolist(), pts[:, 1].tolist(), pts[:, 2].tolist())

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
        # Camera pose (optional). Keep matplotlib default if not specified.
        if (view_elev is not None) or (view_azim is not None):
            ax.view_init(elev=view_elev, azim=view_azim)
        if not bool(hide_decorations):
            ax.set_title(_format_title(poses, arm_length_m, phi_deg, psi_deg, theta_deg, y_clearance))

        angle_text = ax.text2D(
            0.02,
            0.98,
            format_status_text(poses, arm_length_m=arm_length_m, dy=dy),
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

        # Dot grids (background reference). Points are in WORLD coords.
        dot_grid_artists = []
        dot_grid_specs = list(dot_grids) if dot_grids else []
        if dot_grid_specs:
            # Current drone origin includes y_clearance shift (A).
            drone_origin = np.array([0.0, float(dy), 0.0], dtype=float)
            rotor_centers = [
                (pose.hinge + np.array([0.0, float(dy), 0.0], dtype=float) + float(arm_length_m) * pose.arm_dir)
                for pose in poses
            ]
            for spec in dot_grid_specs:
                try:
                    pts = _dot_grid_points_world(spec, drone_origin=drone_origin, rotor_centers=rotor_centers)
                except Exception as e:
                    logging.warning("Failed to build dot grid %r: %r", spec, e)
                    continue
                kwargs = dict(
                    s=float(spec.size),
                    c=str(spec.color),
                    alpha=float(spec.alpha),
                    marker=str(spec.marker),
                    depthshade=False,
                    zorder=0.1,
                )
                if spec.edgecolor is not None:
                    kwargs["edgecolors"] = str(spec.edgecolor)
                if float(spec.lw) > 0.0:
                    kwargs["linewidths"] = float(spec.lw)
                sc = ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], **kwargs)
                dot_grid_artists.append((spec, sc))

        # Axis limits are derived from the drone geometry even when no_drone=True,
        # so that the wall/dot-grids have a stable frame of reference.
        _compute_limits([ArmPose(hinge=p.hinge + np.array([0.0, dy, 0.0]), arm_dir=p.arm_dir, rotor_normal=p.rotor_normal) for p in poses], arm_length_m)

        # Drone geometry (optional)
        body_line = None
        arm_lines = []
        hinge_pts = []
        tip_pts = []
        rotor_lines = []
        normal_lines = []
        if not bool(no_drone):
            # Body outline (hinge square)
            hs_shift = hs.copy()
            hs_shift[:, 1] += dy
            body_line = ax.plot(
                hs_shift[:, 0],
                hs_shift[:, 1],
                hs_shift[:, 2],
                color="k",
                linewidth=1.5 * lw_scale,
                label="hinge square",
            )[0]

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
                        linewidth=3.0 * lw_scale,
                        label=f"arm {i}" if i == 0 else None,
                    )[0]
                )
                hinge_pts.append(ax.scatter([p0[0]], [p0[1]], [p0[2]], color=c, s=30))
                tip_pts.append(ax.scatter([p1[0]], [p1[1]], [p1[2]], color=c, s=40))

                circ = _circle_points(center=p1, normal=pose.rotor_normal, radius=rotor_radius_m, n=200)
                rotor_lines.append(ax.plot(circ[:, 0], circ[:, 1], circ[:, 2], color=c, linewidth=1.5 * lw_scale)[0])

                # Rotor normal (simple line; easier to update than quiver)
                n_scale = rotor_radius_m * 0.8
                p2 = p1 + n_scale * pose.rotor_normal
                normal_lines.append(
                    ax.plot(
                        [p1[0], p2[0]],
                        [p1[1], p2[1]],
                        [p1[2], p2[2]],
                        color=c,
                        linewidth=1.2 * lw_scale,
                    )[0]
                )

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
        y0_plane_border_lines = []
        y0_plane_hatch_lines = []

        def _draw_plane():
            nonlocal y0_plane_artist, y0_plane_border_lines, y0_plane_hatch_lines
            if not bool(draw_y0_plane):
                return
            if y0_plane_artist is not None:
                try:
                    y0_plane_artist.remove()
                except Exception:
                    pass
                y0_plane_artist = None
            if y0_plane_border_lines:
                for _ln in y0_plane_border_lines:
                    try:
                        _ln.remove()
                    except Exception:
                        pass
                y0_plane_border_lines = []
            if y0_plane_hatch_lines:
                for _ln in y0_plane_hatch_lines:
                    try:
                        _ln.remove()
                    except Exception:
                        pass
                y0_plane_hatch_lines = []
            xlim = ax.get_xlim3d()
            zlim = ax.get_zlim3d()
            xs = np.linspace(float(xlim[0]), float(xlim[1]), 2)
            zs = np.linspace(float(zlim[0]), float(zlim[1]), 2)
            X, Z = np.meshgrid(xs, zs)
            Y = np.zeros_like(X)
            y0_plane_artist = ax.plot_surface(X, Y, Z, color="gray", alpha=0.12, shade=False)
            # 3D "wall" border (outline rectangle) to remain visible even when decorations are hidden.
            x0, x1 = float(xlim[0]), float(xlim[1])
            z0, z1 = float(zlim[0]), float(zlim[1])
            xr = [x0, x1, x1, x0, x0]
            yr = [0.0, 0.0, 0.0, 0.0, 0.0]
            zr = [z0, z0, z1, z1, z0]
            try:
                (border_line,) = ax.plot(xr, yr, zr, color="gray", alpha=0.55, linewidth=1.0)
                y0_plane_border_lines = [border_line]
            except Exception:
                y0_plane_border_lines = []
            # Pseudo-hatching: draw many diagonal line segments on the wall plane (y=0).
            # This is more robust than true hatch support in 3D.
            hatch_pitch_m = 0.03  # spacing between lines
            hatch_alpha = 0.18
            hatch_lw = 0.8

            def _clip_line_x_minus_z_eq_s(_s: float):
                # Line in x-z plane: x - z = s (slope +1, like ////)
                pts = []
                # Intersections with x = x0/x1
                z_at_x0 = x0 - _s
                if z0 <= z_at_x0 <= z1:
                    pts.append((x0, z_at_x0))
                z_at_x1 = x1 - _s
                if z0 <= z_at_x1 <= z1:
                    pts.append((x1, z_at_x1))
                # Intersections with z = z0/z1
                x_at_z0 = _s + z0
                if x0 <= x_at_z0 <= x1:
                    pts.append((x_at_z0, z0))
                x_at_z1 = _s + z1
                if x0 <= x_at_z1 <= x1:
                    pts.append((x_at_z1, z1))
                # Deduplicate (floating comparisons: keep simple)
                uniq = []
                for p in pts:
                    if all((abs(p[0] - q[0]) > 1e-9) or (abs(p[1] - q[1]) > 1e-9) for q in uniq):
                        uniq.append(p)
                if len(uniq) < 2:
                    return None
                return uniq[0], uniq[1]

            # Sweep s over the rectangle extent.
            s_min = x0 - z1
            s_max = x1 - z0
            if hatch_pitch_m > 1e-9:
                n_lines = int(np.ceil((s_max - s_min) / hatch_pitch_m)) + 1
                n_lines = max(0, min(n_lines, 400))  # safety cap
                ss = np.linspace(s_min, s_max, n_lines)
                hatch_lines = []
                for s in ss:
                    seg = _clip_line_x_minus_z_eq_s(float(s))
                    if seg is None:
                        continue
                    (xa, za), (xb, zb) = seg
                    try:
                        (ln,) = ax.plot([xa, xb], [0.0, 0.0], [za, zb], color="gray", alpha=hatch_alpha, linewidth=hatch_lw)
                        hatch_lines.append(ln)
                    except Exception:
                        break
                y0_plane_hatch_lines = hatch_lines

        _draw_plane()

        sliders_enabled = bool(sliders_enabled) and bool(show) and (not bool(hide_decorations)) and (not bool(no_drone))
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
                angle_text.set_text(format_status_text(new_poses, arm_length_m=new_L, dy=new_dy))
                new_hs = np.array([p.hinge for p in new_poses] + [new_poses[0].hinge])
                new_hs[:, 1] += new_dy
                if body_line is not None:
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

                # Update dot grids (if any). They depend on dy/poses (drone/rotor frames).
                if dot_grid_artists:
                    drone_origin_u = np.array([0.0, float(new_dy), 0.0], dtype=float)
                    rotor_centers_u = [
                        (pose.hinge + np.array([0.0, float(new_dy), 0.0], dtype=float) + float(new_L) * pose.arm_dir)
                        for pose in new_poses
                    ]
                    for (spec, sc) in dot_grid_artists:
                        try:
                            pts_u = _dot_grid_points_world(spec, drone_origin=drone_origin_u, rotor_centers=rotor_centers_u)
                            _set_3d_scatter(sc, pts_u)
                        except Exception as e:
                            logging.debug("Dot grid update failed for %r: %r", spec, e)

                ax.set_title(_format_title(new_poses, new_L, new_phi, new_psi, new_theta, new_clear))
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
        if bool(transparent):
            try:
                fig.patch.set_alpha(0.0)
            except Exception:
                pass
            try:
                ax.set_facecolor((0, 0, 0, 0))
            except Exception:
                pass
            # 3D panes (optional; depends on backend/matplotlib version)
            try:
                for a in (ax.xaxis, ax.yaxis, ax.zaxis):
                    try:
                        a.pane.set_alpha(0.0)
                    except Exception:
                        pass
            except Exception:
                pass
        if bool(hide_decorations):
            fig.savefig(save_path, dpi=int(dpi), bbox_inches="tight", pad_inches=0.0, transparent=bool(transparent))
        else:
            fig.savefig(save_path, dpi=int(dpi), bbox_inches="tight", transparent=bool(transparent))
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


def render_scene3d_rgba(
    phi_deg: float,
    psi_deg: float,
    theta_deg: float,
    *,
    cx: float = 0.035,
    cy: float = 0.035,
    rotor_radius_in: float = 3.5,
    arm_length_m: float = 0.18,
    symmetry: str = "mirror_xy",
    drone_center: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotor_inflow_offset: float = 0.02,
    view_elev: float | None = 12.0,
    view_azim: float | None = 20.0,
    draw_y0_plane: bool = True,
    drone_lw: float = 2.5,
    dpi: int = 180,
    figsize: tuple[float, float] = (4.0, 4.0),
) -> np.ndarray:
    """
    plot_three_view_drone() の3Dビュー相当を、RGBA画像として返す（プレゼン/動画用途）。
    - three-view（2D投影）は作らない
    - drone_center / rotor_inflow_offset / view_elev/view_azim / 壁(y=0)に対応
    """
    # Ensure headless rendering.
    try:
        mpl.use("Agg", force=True)
    except Exception:
        pass

    import matplotlib.pyplot as plt

    if not _HAS_3D:
        raise RuntimeError(
            "3D projection is unavailable in this Python environment. "
            f"Reason: {_AXES3D_IMPORT_ERROR!r}"
        )

    rotor_radius_m = float(rotor_radius_in) * 0.0254
    lw_scale = max(0.0, float(drone_lw))
    drone_offset = np.array(drone_center, dtype=float).reshape(3)

    def _normalize_local(v: np.ndarray) -> np.ndarray:
        n = float(np.linalg.norm(v))
        if n < 1e-12:
            return v
        return v / n

    def compute_poses(_phi: float, _psi: float, _theta: float) -> list[ArmPose]:
        base_hinge = np.array([+cx, +cy, 0.0], dtype=float)
        base_arm_dir0 = _normalize_local(np.array([+1.0, +1.0, 0.0], dtype=float))
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
                        arm_dir=_normalize_local(M @ base_pose.arm_dir),
                        rotor_normal=_normalize_local(M @ base_pose.rotor_normal),
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
            arm_dirs0.append(_normalize_local(np.array([sx, sy, 0.0], dtype=float)))
        return [
            _make_arm_pose(hinge=h, arm_dir0=d0, phi_deg=float(_phi), psi_deg=float(_psi), theta_deg=float(_theta))
            for h, d0 in zip(hinges, arm_dirs0, strict=True)
        ]

    poses = compute_poses(float(phi_deg), float(psi_deg), float(theta_deg))

    # Collect geometry with drone_offset + rotor_inflow_offset
    hinges_for_outline = [pose.hinge + drone_offset for pose in poses]
    hs = np.array(hinges_for_outline + [hinges_for_outline[0]], dtype=float)

    arm_segments = []
    rotor_circles = []
    all_points = [hs]
    for pose in poses:
        p0 = pose.hinge + drone_offset
        arm_tip = p0 + float(arm_length_m) * pose.arm_dir
        rotor_center = arm_tip + float(rotor_inflow_offset) * pose.rotor_normal
        arm_segments.append((p0, arm_tip))
        rotor_circles.append((rotor_center, pose.rotor_normal))
        all_points.append(np.array([p0, arm_tip, rotor_center], dtype=float))
        circ = _circle_points(center=rotor_center, normal=pose.rotor_normal, radius=float(rotor_radius_m), n=80)
        all_points.append(circ)

    P = np.vstack(all_points)
    pad = float(rotor_radius_m) * 0.35
    x_min, x_max = float(P[:, 0].min() - pad), float(P[:, 0].max() + pad)
    y_min, y_max = float(P[:, 1].min() - pad), float(P[:, 1].max() + pad)
    z_min, z_max = float(P[:, 2].min() - pad), float(P[:, 2].max() + pad)
    if bool(draw_y0_plane):
        y_min = min(y_min, -pad)
        y_max = max(y_max, +pad)

    # Equal aspect bounds
    x_center = 0.5 * (x_min + x_max)
    y_center = 0.5 * (y_min + y_max)
    z_center = 0.5 * (z_min + z_max)
    max_half = max((x_max - x_min) * 0.5, (y_max - y_min) * 0.5, (z_max - z_min) * 0.5)

    fig = plt.figure(figsize=tuple(figsize))
    ax = fig.add_subplot(111, projection="3d")
    if (view_elev is not None) or (view_azim is not None):
        ax.view_init(elev=view_elev, azim=view_azim)

    # Hide decorations for clean composition
    ax.set_axis_off()

    # Draw wall (y=0) if requested
    if bool(draw_y0_plane):
        xs = np.linspace(x_center - max_half, x_center + max_half, 2)
        zs = np.linspace(z_center - max_half, z_center + max_half, 2)
        X, Z = np.meshgrid(xs, zs)
        Y = np.zeros_like(X)
        ax.plot_surface(X, Y, Z, color="gray", alpha=0.12, shade=False)
        # Outline border
        xr = [x_center - max_half, x_center + max_half, x_center + max_half, x_center - max_half, x_center - max_half]
        yr = [0.0, 0.0, 0.0, 0.0, 0.0]
        zr = [z_center - max_half, z_center - max_half, z_center + max_half, z_center + max_half, z_center - max_half]
        try:
            ax.plot(xr, yr, zr, color="gray", alpha=0.55, linewidth=1.0)
        except Exception:
            pass

    # Draw body outline (hinge square)
    ax.plot(hs[:, 0], hs[:, 1], hs[:, 2], color="gray", linewidth=1.5 * lw_scale)

    # Per-rotor color (consistent with other views)
    colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
    for i, pose in enumerate(poses):
        c = colors[i % len(colors)]
        p0 = pose.hinge + drone_offset
        arm_tip = p0 + float(arm_length_m) * pose.arm_dir
        rotor_center = arm_tip + float(rotor_inflow_offset) * pose.rotor_normal
        ax.plot([p0[0], arm_tip[0]], [p0[1], arm_tip[1]], [p0[2], arm_tip[2]], color="gray", linewidth=2.5 * lw_scale)
        ax.scatter([p0[0]], [p0[1]], [p0[2]], color="gray", s=25)
        ax.scatter([arm_tip[0]], [arm_tip[1]], [arm_tip[2]], color="gray", s=35)
        # tip -> rotor center
        ax.plot(
            [arm_tip[0], rotor_center[0]],
            [arm_tip[1], rotor_center[1]],
            [arm_tip[2], rotor_center[2]],
            color="gray",
            linewidth=1.5 * lw_scale,
        )
        circ = _circle_points(center=rotor_center, normal=pose.rotor_normal, radius=float(rotor_radius_m), n=120)
        ax.plot(circ[:, 0], circ[:, 1], circ[:, 2], color=c, linewidth=1.2 * lw_scale)
        # rotor normal (short line)
        n_scale = float(rotor_radius_m) * 0.6
        p2 = rotor_center + n_scale * pose.rotor_normal
        ax.plot([rotor_center[0], p2[0]], [rotor_center[1], p2[1]], [rotor_center[2], p2[2]], color=c, linewidth=1.0 * lw_scale)

    ax.set_xlim(x_center - max_half, x_center + max_half)
    ax.set_ylim(y_center - max_half, y_center + max_half)
    ax.set_zlim(z_center - max_half, z_center + max_half)
    try:
        _set_axes_equal(ax)
    except Exception:
        pass

    buf = io.BytesIO()
    fig.savefig(buf, dpi=int(dpi), bbox_inches="tight", pad_inches=0.0, transparent=True)
    plt.close(fig)
    buf.seek(0)

    try:
        from PIL import Image as _Image

        img = _Image.open(buf).convert("RGBA")
        return np.asarray(img, dtype=np.uint8)
    except Exception:
        import matplotlib.image as mpimg

        arr = mpimg.imread(buf)
        if arr.dtype != np.uint8:
            arr = (np.clip(arr, 0.0, 1.0) * 255.0).astype(np.uint8)
        if arr.shape[-1] == 3:
            alpha = np.full((arr.shape[0], arr.shape[1], 1), 255, dtype=np.uint8)
            arr = np.concatenate([arr, alpha], axis=-1)
        return arr


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
    no_drone: bool = False,
    view_elev: float | None = 25.0,
    view_azim: float | None = -30.0,
    save_path: str | None = None,
    save_split: bool = False,
    transparent: bool = True,
    drone_lw: float = 1.0,
    dpi: int = 200,
    show: bool = True,
    include_3d_view: bool = True,
    drone_center: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotor_inflow_offset: float = 0.02,
    hide_decorations: bool = False,
    drone_color: str = "multi",
    body_color: str = "gray",
    dot_grids: list[DotGridSpec] | None = None,
):
    """
    3面図 (正面図・側面図・平面図) を描画する。
    
    - Front view (正面図): Y軸方向から見た XZ平面への投影
    - Side view (側面図): X軸方向から見た YZ平面への投影
    - Top view (平面図): Z軸方向から見た XY平面への投影 (Y軸横、X軸縦)
    
    include_3d_view=True の場合、4つ目のサブプロットに3Dビューを追加。
    view_elev/view_azim: 3Dビューのカメラ姿勢（度）。None の場合は matplotlib のデフォルトに従う。
    drone_center: ドローン中心のオフセット (x, y, z) [m]
    rotor_inflow_offset: ロータ中心を流入側（推力の逆方向）にオフセットする量 [m]
    hide_decorations: True の場合、軸ラベル、グリッド、タイトルなどを非表示
    drone_color: "multi" で4色、それ以外は指定した1色でロータを描画
    body_color: ボディ（ヒンジスクエア、アーム）の色
    """
    lw_scale = max(0.0, float(drone_lw))
    drone_offset = np.array(drone_center, dtype=float)
    import matplotlib.pyplot as plt

    def thrust_angles_alpha_beta_deg(thrust_vec: np.ndarray) -> tuple[float, float]:
        """
        thrust_vec: (3,) 推力ベクトル（ここではロータ法線ベクトル）を想定。

        定義（plot_morphing_drone() と同じ）:
          alpha = -atan2(vy, vz) [deg]
          beta  = -atan2(vx, vz) [deg]
        """
        v = _normalize(np.asarray(thrust_vec, dtype=float).reshape(3))
        vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
        alpha = -float(np.degrees(np.arctan2(vy, vz)))
        beta = -float(np.degrees(np.arctan2(vx, vz)))
        return alpha, beta

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
    # rotor0 is the +x,+y quadrant arm (base_pose), which is poses[0] in both symmetry modes.
    try:
        alpha0_deg, beta0_deg = thrust_angles_alpha_beta_deg(poses[0].rotor_normal)
    except Exception:
        alpha0_deg, beta0_deg = float("nan"), float("nan")

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

    # Dot grids: precompute points in WORLD coords once for all views.
    dot_grid_specs = list(dot_grids) if dot_grids else []
    dot_grid_points_world_list: list[tuple[DotGridSpec, np.ndarray]] = []
    if dot_grid_specs:
        # Drone origin in three-view: use drone_center offset (and any other applied translation).
        # NOTE: three-view currently does not apply y_clearance shift to the drone geometry.
        drone_origin = np.asarray(drone_offset, dtype=float).reshape(3)
        rotor_centers = [np.asarray(rc[0], dtype=float).reshape(3) for rc in rotor_circles]
        for spec in dot_grid_specs:
            try:
                pts = _dot_grid_points_world(spec, drone_origin=drone_origin, rotor_centers=rotor_centers)
                dot_grid_points_world_list.append((spec, pts))
            except Exception as e:
                logging.warning("Failed to build dot grid %r: %r", spec, e)

    def _format_status_text_three_view() -> str:
        """
        三面図用の図中テキスト:
        - rotor0 の alpha/beta
        - 各ロータ中心位置 (x,y,z) [m]（小数3桁）
        """
        lines = [
            "alpha/beta (deg) from thrust vec (rotor_normal):",
            f"  rotor0: alpha={alpha0_deg:+6.1f}, beta={beta0_deg:+6.1f}",
            "rotor center position (x,y,z) [m]:",
        ]
        for i, (rotor_center, _normal, _arm_tip, _c) in enumerate(rotor_circles):
            lines.append(f"  rotor{i}: ({rotor_center[0]:+.3f}, {rotor_center[1]:+.3f}, {rotor_center[2]:+.3f})")
        return "\n".join(lines)

    status_text_artist = None
    if not bool(hide_decorations):
        # Place at top-left of the figure (avoid putting long text into the title).
        status_text_artist = fig.text(
            0.01,
            0.99,
            _format_status_text_three_view(),
            ha="left",
            va="top",
            fontsize=9,
            family="monospace",
        )

    def draw_2d_view(ax, proj_func, depth_func, xlabel, ylabel, title):
        """
        proj_func: 3Dポイントを2D座標に変換する関数 (x, y, z) -> (u, v)
        depth_func: 奥行きを計算する関数 (x, y, z) -> depth (小さいほど手前)
        """
        if not bool(no_drone):
            # Body outline (hinge square)
            hinges_for_outline = [pose.hinge + drone_offset for pose in poses]
            hs = np.array(hinges_for_outline + [hinges_for_outline[0]])
            hs_2d = np.array([proj_func(p) for p in hs])
            ax.plot(hs_2d[:, 0], hs_2d[:, 1], color=body_color, linewidth=1.5 * lw_scale, label="body", zorder=1)

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
                ax.plot(
                    [p0_2d[0], arm_tip_2d[0]],
                    [p0_2d[1], arm_tip_2d[1]],
                    color=body_color,
                    linewidth=2.5 * lw_scale,
                    zorder=base_zorder,
                )
                ax.scatter([p0_2d[0]], [p0_2d[1]], color=body_color, s=25, zorder=base_zorder + 1)
                ax.scatter([arm_tip_2d[0]], [arm_tip_2d[1]], color=body_color, s=35, zorder=base_zorder + 2)

                # Line from arm tip to rotor center - use body_color
                rotor_center_2d = proj_func(rotor_center)
                ax.plot([arm_tip_2d[0], rotor_center_2d[0]], [arm_tip_2d[1], rotor_center_2d[1]], 
                        color=body_color, linewidth=1.5 * lw_scale, zorder=base_zorder + 2)

                # Rotor circle - use rotor_c (drone_color)
                circ = _circle_points(center=rotor_center, normal=normal, radius=rotor_radius_m, n=100)
                circ_2d = np.array([proj_func(pt) for pt in circ])
                ax.plot(circ_2d[:, 0], circ_2d[:, 1], color=rotor_c, linewidth=1.2 * lw_scale, zorder=base_zorder + 3)

                # Rotor normal arrow - use rotor_c (drone_color)
                n_scale = rotor_radius_m * 0.6
                p2 = rotor_center + n_scale * normal
                p2_2d = proj_func(p2)
                ax.annotate(
                    "",
                    xy=(p2_2d[0], p2_2d[1]),
                    xytext=(rotor_center_2d[0], rotor_center_2d[1]),
                    arrowprops=dict(arrowstyle="->", color=rotor_c, lw=1.0 * lw_scale, zorder=base_zorder + 4),
                    zorder=base_zorder + 4,
                )

        # Dot grids (background)
        if dot_grid_points_world_list:
            for (spec, pts) in dot_grid_points_world_list:
                pts2 = np.asarray([proj_func(p) for p in pts], dtype=float)
                kwargs = dict(
                    s=float(spec.size),
                    c=str(spec.color),
                    alpha=float(spec.alpha),
                    marker=str(spec.marker),
                    zorder=0.15,
                )
                if spec.edgecolor is not None:
                    kwargs["edgecolors"] = str(spec.edgecolor)
                if float(spec.lw) > 0.0:
                    kwargs["linewidths"] = float(spec.lw)
                ax.scatter(pts2[:, 0], pts2[:, 1], **kwargs)

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
    # 壁(y=0)は decorations を隠しても表示したい（--no-y0-plane 指定時のみ非表示）
    if draw_y0_plane:
        # Hatching for y<0 region (left side in this projection)
        try:
            x_left = float(min(ax_front.get_xlim()))
            ax_front.axvspan(
                x_left,
                0.0,
                facecolor=(0, 0, 0, 0),
                edgecolor="gray",
                hatch="////",
                linewidth=0.0,
                alpha=0.35,
                zorder=0.2,
            )
        except Exception:
            pass
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
    if draw_y0_plane:
        # Hatching for y<0 region (left side in this projection)
        try:
            x_left = float(min(ax_top.get_xlim()))
            ax_top.axvspan(
                x_left,
                0.0,
                facecolor=(0, 0, 0, 0),
                edgecolor="gray",
                hatch="////",
                linewidth=0.0,
                alpha=0.35,
                zorder=0.2,
            )
        except Exception:
            pass
        ax_top.axvline(x=0, color="gray", linestyle="--", alpha=0.5, label="y=0")

    # 3D view (if enabled)
    if ax_3d is not None:
        # Dot grids in 3D (background)
        if dot_grid_points_world_list:
            for (spec, pts) in dot_grid_points_world_list:
                kwargs = dict(
                    s=float(spec.size),
                    c=str(spec.color),
                    alpha=float(spec.alpha),
                    marker=str(spec.marker),
                    depthshade=False,
                )
                if spec.edgecolor is not None:
                    kwargs["edgecolors"] = str(spec.edgecolor)
                if float(spec.lw) > 0.0:
                    kwargs["linewidths"] = float(spec.lw)
                ax_3d.scatter(pts[:, 0], pts[:, 1], pts[:, 2], **kwargs)

        if not bool(no_drone):
            # Body outline
            hinges_for_outline = [pose.hinge + drone_offset for pose in poses]
            hs = np.array(hinges_for_outline + [hinges_for_outline[0]])
            ax_3d.plot(hs[:, 0], hs[:, 1], hs[:, 2], color=body_color, linewidth=1.5 * lw_scale)

            for (p0, arm_tip, rotor_c), (rotor_center, normal, _, _) in zip(arm_segments, rotor_circles):
                # Arm (hinge to arm tip) - use body_color
                ax_3d.plot(
                    [p0[0], arm_tip[0]],
                    [p0[1], arm_tip[1]],
                    [p0[2], arm_tip[2]],
                    color=body_color,
                    linewidth=2.5 * lw_scale,
                )
                ax_3d.scatter([p0[0]], [p0[1]], [p0[2]], color=body_color, s=25)
                ax_3d.scatter([arm_tip[0]], [arm_tip[1]], [arm_tip[2]], color=body_color, s=35)

                # Line from arm tip to rotor center - use body_color
                ax_3d.plot([arm_tip[0], rotor_center[0]], [arm_tip[1], rotor_center[1]], [arm_tip[2], rotor_center[2]], 
                           color=body_color, linewidth=1.5 * lw_scale)

                # Rotor circle - use rotor_c (drone_color)
                circ = _circle_points(center=rotor_center, normal=normal, radius=rotor_radius_m, n=100)
                ax_3d.plot(circ[:, 0], circ[:, 1], circ[:, 2], color=rotor_c, linewidth=1.2 * lw_scale)

                # Rotor normal arrow - use rotor_c (drone_color)
                n_scale = rotor_radius_m * 0.6
                p2 = rotor_center + n_scale * normal
                ax_3d.plot(
                    [rotor_center[0], p2[0]],
                    [rotor_center[1], p2[1]],
                    [rotor_center[2], p2[2]],
                    color=rotor_c,
                    linewidth=1.0 * lw_scale,
                )

        # Set initial camera angle (X軸寄りに傾ける)
        # elev: 仰角 (Z軸からの角度), azim: 方位角 (XY平面上の回転)
        if (view_elev is not None) or (view_azim is not None):
            ax_3d.view_init(elev=view_elev, azim=view_azim)

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
        # 壁(y=0)は decorations を隠しても表示したい（--no-y0-plane 指定時のみ非表示）
        if draw_y0_plane:
            xs = np.linspace(x_center - max_half, x_center + max_half, 2)
            zs = np.linspace(z_center - max_half, z_center + max_half, 2)
            X, Z = np.meshgrid(xs, zs)
            Y = np.zeros_like(X)
            ax_3d.plot_surface(X, Y, Z, color="gray", alpha=0.12, shade=False)
            # Outline border (rectangle) for the wall plane
            xr = [x_center - max_half, x_center + max_half, x_center + max_half, x_center - max_half, x_center - max_half]
            yr = [0.0, 0.0, 0.0, 0.0, 0.0]
            zr = [z_center - max_half, z_center - max_half, z_center + max_half, z_center + max_half, z_center - max_half]
            try:
                ax_3d.plot(xr, yr, zr, color="gray", alpha=0.55, linewidth=1.0)
            except Exception:
                pass
            # Pseudo-hatching (diagonal lines) on the 3D wall plane.
            try:
                x0, x1 = float(x_center - max_half), float(x_center + max_half)
                z0, z1 = float(z_center - max_half), float(z_center + max_half)
                hatch_pitch_m = 0.03
                hatch_alpha = 0.18
                hatch_lw = 0.8

                def _clip_line_x_minus_z_eq_s(_s: float):
                    pts = []
                    z_at_x0 = x0 - _s
                    if z0 <= z_at_x0 <= z1:
                        pts.append((x0, z_at_x0))
                    z_at_x1 = x1 - _s
                    if z0 <= z_at_x1 <= z1:
                        pts.append((x1, z_at_x1))
                    x_at_z0 = _s + z0
                    if x0 <= x_at_z0 <= x1:
                        pts.append((x_at_z0, z0))
                    x_at_z1 = _s + z1
                    if x0 <= x_at_z1 <= x1:
                        pts.append((x_at_z1, z1))
                    uniq = []
                    for p in pts:
                        if all((abs(p[0] - q[0]) > 1e-9) or (abs(p[1] - q[1]) > 1e-9) for q in uniq):
                            uniq.append(p)
                    if len(uniq) < 2:
                        return None
                    return uniq[0], uniq[1]

                s_min = x0 - z1
                s_max = x1 - z0
                if hatch_pitch_m > 1e-9:
                    n_lines = int(np.ceil((s_max - s_min) / hatch_pitch_m)) + 1
                    n_lines = max(0, min(n_lines, 400))
                    ss = np.linspace(s_min, s_max, n_lines)
                    for s in ss:
                        seg = _clip_line_x_minus_z_eq_s(float(s))
                        if seg is None:
                            continue
                        (xa, za), (xb, zb) = seg
                        ax_3d.plot([xa, xb], [0.0, 0.0], [za, zb], color="gray", alpha=hatch_alpha, linewidth=hatch_lw)
            except Exception:
                pass

    # Title with rotor-to-wall distance
    if not hide_decorations:
        wall_dist_str = f"{min_rotor_to_wall_distance * 1000:.1f}mm"
        if min_rotor_to_wall_distance < 0:
            wall_dist_str += " (collision!)"
        fig.suptitle(
            f"Morphing Drone - Three View\n"
            f"phi={phi_deg:.1f}°, psi={psi_deg:.1f}°, theta={theta_deg:.1f}°, "
            f"L={arm_length_m:.3f}m, R={rotor_radius_m:.4f}m\n"
            f"alpha={alpha0_deg:+.1f}°, beta={beta0_deg:+.1f}° (from rotor0)\n"
            f"Rotor inflow offset={rotor_inflow_offset * 1000:.1f}mm, "
            f"Min rotor-to-wall distance={wall_dist_str}",
            fontsize=11,
        )

    plt.tight_layout()

    # Prepare transparent output (default) when saving.
    if bool(transparent):
        try:
            fig.patch.set_alpha(0.0)
        except Exception:
            pass
        for _ax in [ax_top, ax_side, ax_front]:
            try:
                _ax.set_facecolor((0, 0, 0, 0))
            except Exception:
                pass
        if ax_3d is not None:
            try:
                ax_3d.set_facecolor((0, 0, 0, 0))
            except Exception:
                pass
            try:
                for a in (ax_3d.xaxis, ax_3d.yaxis, ax_3d.zaxis):
                    try:
                        a.pane.set_alpha(0.0)
                    except Exception:
                        pass
            except Exception:
                pass

    if save_path:
        logging.info("Saving three-view figure to %s (dpi=%d)...", save_path, int(dpi))
        fig.savefig(save_path, dpi=int(dpi), bbox_inches="tight", transparent=bool(transparent))
        logging.info("Saved: %s", save_path)

        # Optional: save each subplot as a separate image with derived filenames.
        if bool(save_split):
            try:
                # Ensure artists are laid out; needed for tightbbox.
                fig.canvas.draw()
                renderer = fig.canvas.get_renderer()

                def _split_path(_suffix: str) -> str:
                    d = os.path.dirname(str(save_path))
                    base = os.path.basename(str(save_path))
                    stem, ext = os.path.splitext(base)
                    if not ext:
                        ext = ".png"
                    return os.path.join(d, f"{stem}_{_suffix}{ext}")

                def _save_ax(_ax, _suffix: str):
                    # bbox in display coords -> inches
                    bb = _ax.get_tightbbox(renderer).transformed(fig.dpi_scale_trans.inverted())
                    out = _split_path(_suffix)
                    fig.savefig(out, dpi=int(dpi), bbox_inches=bb, transparent=bool(transparent))
                    logging.info("Saved split: %s", out)

                # NOTE: axes exist regardless of hide_decorations; they may be axis_off but bbox still works.
                _save_ax(ax_front, "front")
                _save_ax(ax_side, "side")
                _save_ax(ax_top, "top")
                if ax_3d is not None:
                    _save_ax(ax_3d, "3d")
                else:
                    logging.info("Split save: 3D axis not available; skipping *_3d.*")
            except Exception as e:
                logging.warning("Failed to save split views: %r", e)

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
    parser.add_argument("--alpha", type=float, default=None, help="Target alpha [deg] (derived from rotor0 thrust vec).")
    parser.add_argument("--beta", type=float, default=None, help="Target beta [deg] (derived from rotor0 thrust vec).")
    parser.add_argument(
        "--solve-psi-theta",
        action="store_true",
        help="Solve psi/theta from given --alpha/--beta while keeping --phi fixed. rotor0 (+x,+y arm) is used.",
    )
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
    parser.add_argument(
        "--save-split",
        action="store_true",
        help="(three-view only) Also save each view as a separate image with derived filenames, e.g. out_front.png, out_side.png, out_top.png, out_3d.png.",
    )
    parser.add_argument(
        "--opaque",
        action="store_true",
        help="Save images with opaque background (disable transparency). Default is transparent output.",
    )
    parser.add_argument("--dpi", type=int, default=200, help="DPI for --save. Default: 200")
    parser.add_argument("--no-show", action="store_true", help="Do not open a window (useful with --save on WSL/headless).")
    parser.add_argument("--no-sliders", action="store_true", help="Disable slider UI (enabled by default when showing).")
    parser.add_argument(
        "--no-drone",
        action="store_true",
        help="Do not draw drone geometry (body/arms/rotors/normals). Wall (y=0) and dot-grids can still be drawn.",
    )
    parser.add_argument(
        "--dot-grid",
        action="append",
        default=[],
        help=(
            "Draw dot grid(s). Repeatable. Format: \"key=value;key=value;...\". "
            "Required: origin=x,y,z; u=dx,dy,dz; v=dx,dy,dz; nu=N; nv=M. "
            "Optional: frame=world|drone|rotor|rotor_between; rotor=i; rotors=a,b; "
            "color=...; alpha=...; size=...; marker=...; edgecolor=...; lw=..."
        ),
    )
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
        help="Hide axis labels, grid, titles, etc. Note: wall (y=0) is still shown unless --no-y0-plane is specified.",
    )
    parser.add_argument(
        "--view-elev",
        type=float,
        default=None,
        help="3D view camera elevation angle [deg]. If omitted, keep matplotlib default (or function default in three-view).",
    )
    parser.add_argument(
        "--view-azim",
        type=float,
        default=None,
        help="3D view camera azimuth angle [deg]. If omitted, keep matplotlib default (or function default in three-view).",
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
        "--drone-lw",
        type=float,
        default=1.0,
        help="Line width scale for drone geometry (body/arms/rotors/normals). Default: 1.0",
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

    # Parse dot grids (if any)
    dot_grids: list[DotGridSpec] = []
    try:
        for s in (args.dot_grid or []):
            dot_grids.append(parse_dot_grid_spec(str(s)))
    except Exception as e:
        raise SystemExit(f"Failed to parse --dot-grid: {e}") from e

    # Optional inverse: alpha/beta -> psi/theta (phi fixed)
    if bool(args.solve_psi_theta):
        if args.alpha is None or args.beta is None:
            raise SystemExit("--solve-psi-theta requires both --alpha and --beta.")
        psi_s, th_s, res = solve_psi_theta_from_alpha_beta_deg(
            phi_deg=float(args.phi),
            alpha_deg=float(args.alpha),
            beta_deg=float(args.beta),
        )
        logging.info(
            "Solved from alpha/beta: target alpha=%.3f beta=%.3f => psi=%.3f theta=%.3f (residual=%.3f deg)",
            float(args.alpha),
            float(args.beta),
            float(psi_s),
            float(th_s),
            float(res),
        )
        args.psi = float(psi_s)
        args.theta = float(th_s)

    # Check if three-view mode is requested
    use_three_view = bool(args.three_view) or bool(args.three_view_only)

    if use_three_view:
        # three-view has its own defaults; only override when user explicitly specifies.
        tv_elev = 25.0 if args.view_elev is None else float(args.view_elev)
        tv_azim = -30.0 if args.view_azim is None else float(args.view_azim)
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
            no_drone=bool(args.no_drone),
            view_elev=tv_elev,
            view_azim=tv_azim,
            save_path=(str(args.save) if args.save else None),
            save_split=bool(args.save_split),
            transparent=(not bool(args.opaque)),
            drone_lw=float(args.drone_lw),
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
            dot_grids=dot_grids,
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
            no_drone=bool(args.no_drone),
            view_elev=(None if args.view_elev is None else float(args.view_elev)),
            view_azim=(None if args.view_azim is None else float(args.view_azim)),
            save_path=(str(args.save) if args.save else None),
            transparent=(not bool(args.opaque)),
            drone_lw=float(args.drone_lw),
            dpi=int(args.dpi),
            show=(not bool(args.no_show)),
            sliders_enabled=(not bool(args.no_sliders)),
            dot_grids=dot_grids,
        )


if __name__ == "__main__":
    main()

