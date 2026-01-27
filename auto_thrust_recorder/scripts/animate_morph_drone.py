#!/usr/bin/env python3
"""
Morphing drone animation (MP4) generator using visualize_morph_drone.render_morphing_drone_rgba().

Design goals:
  - No extra Python dependencies (uses stdlib + numpy + existing repo deps).
  - MP4 encoding is delegated to external `ffmpeg` via stdin rawvideo pipe.
  - Animation is described by a JSON "spec" with segments:
      - angles_keyframes: phi/psi/theta (deg) and L (m) as keyframes
      - alpha_beta_circle: alpha/beta circle motion + phi/L keyframes; psi/theta solved per-frame

Usage:
  python3 auto_thrust_recorder/scripts/animate_morph_drone.py --spec path/to/spec.json
  python3 auto_thrust_recorder/scripts/animate_morph_drone.py --spec spec.json --out out.mp4
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from typing import Any

import numpy as np

try:
    from tqdm import tqdm as _tqdm  # type: ignore

    _HAS_TQDM = True
except Exception:
    _tqdm = None
    _HAS_TQDM = False

# Optional: PIL for overlay text rendering
try:
    from PIL import Image, ImageDraw, ImageFont  # type: ignore

    _HAS_PIL = True
except Exception:
    Image = None
    ImageDraw = None
    ImageFont = None
    _HAS_PIL = False

# Optional: matplotlib color parsing (for flexible background colors)
try:
    import matplotlib.colors as _mpl_colors  # type: ignore

    _HAS_MPL_COLORS = True
except Exception:
    _mpl_colors = None
    _HAS_MPL_COLORS = False


def _load_visualize_module():
    """
    Import visualize_morph_drone from the same directory as this script.
    This works whether invoked as a file (`python path/to/script.py`) or via module.
    """
    try:
        # If the working directory / pythonpath already includes scripts/
        import visualize_morph_drone as vm  # type: ignore

        return vm
    except Exception:
        pass

    import importlib.util

    here = os.path.dirname(os.path.abspath(__file__))
    target = os.path.join(here, "visualize_morph_drone.py")
    if not os.path.exists(target):
        raise RuntimeError(f"visualize_morph_drone.py not found next to this script: {target}")

    spec = importlib.util.spec_from_file_location("visualize_morph_drone", target)
    if spec is None or spec.loader is None:
        raise RuntimeError("Failed to create import spec for visualize_morph_drone.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)  # type: ignore[attr-defined]
    return mod


VM = _load_visualize_module()
render_morphing_drone_rgba = VM.render_morphing_drone_rgba
render_scene3d_rgba = getattr(VM, "render_scene3d_rgba", None)
solve_psi_theta_from_alpha_beta_deg = VM.solve_psi_theta_from_alpha_beta_deg
_make_arm_pose = getattr(VM, "_make_arm_pose", None)


def _require(cond: bool, msg: str):
    if not cond:
        raise ValueError(msg)


def _get(d: dict[str, Any], key: str, default: Any) -> Any:
    if key not in d:
        return default
    return d[key]


def _as_float(x: Any, *, name: str) -> float:
    try:
        return float(x)
    except Exception as e:
        raise ValueError(f"{name} must be a number, got {x!r}") from e


def _as_int(x: Any, *, name: str) -> int:
    try:
        return int(x)
    except Exception as e:
        raise ValueError(f"{name} must be an integer, got {x!r}") from e


def _parse_keyframes(kfs: Any, *, name: str) -> list[tuple[float, float]]:
    """
    kfs: [[t_s, value], ...]
    """
    if kfs is None:
        return []
    if not isinstance(kfs, list):
        raise ValueError(f"{name} must be a list like [[t,value],...], got {type(kfs)}")
    out: list[tuple[float, float]] = []
    for i, item in enumerate(kfs):
        if not (isinstance(item, list) or isinstance(item, tuple)) or len(item) != 2:
            raise ValueError(f"{name}[{i}] must be [t,value], got {item!r}")
        t = _as_float(item[0], name=f"{name}[{i}][0] (t)")
        v = _as_float(item[1], name=f"{name}[{i}][1] (value)")
        out.append((t, v))
    out.sort(key=lambda tv: tv[0])
    return out


def _interp_linear(kfs: list[tuple[float, float]], t: float, *, default: float) -> float:
    if not kfs:
        return float(default)
    if t <= kfs[0][0]:
        return float(kfs[0][1])
    if t >= kfs[-1][0]:
        return float(kfs[-1][1])
    # find segment
    for i in range(len(kfs) - 1):
        t0, v0 = kfs[i]
        t1, v1 = kfs[i + 1]
        if t0 <= t <= t1:
            if abs(t1 - t0) < 1e-12:
                return float(v1)
            u = (t - t0) / (t1 - t0)
            return float(v0 + u * (v1 - v0))
    return float(kfs[-1][1])


def _clamp01(u: float) -> float:
    if u < 0.0:
        return 0.0
    if u > 1.0:
        return 1.0
    return float(u)


def _easing_fn(name: str):
    n = str(name).strip().lower()
    if n in {"", "linear"}:
        return lambda u: _clamp01(u)
    if n in {"smoothstep"}:
        return lambda u: (lambda x: x * x * (3.0 - 2.0 * x))(_clamp01(u))
    if n in {"cosine", "sin", "sine"}:
        return lambda u: 0.5 - 0.5 * math.cos(math.pi * _clamp01(u))
    if n in {"quad_in", "quadratic_in"}:
        return lambda u: (lambda x: x * x)(_clamp01(u))
    if n in {"quad_out", "quadratic_out"}:
        return lambda u: (lambda x: 1.0 - (1.0 - x) * (1.0 - x))(_clamp01(u))
    if n in {"quad_in_out", "quadratic_in_out"}:
        def _f(u: float) -> float:
            x = _clamp01(u)
            if x < 0.5:
                return 2.0 * x * x
            y = 1.0 - x
            return 1.0 - 2.0 * y * y
        return _f
    raise ValueError(f"Unknown easing: {name!r}")


def _interp_keyframes(kfs: list[tuple[float, float]], t: float, *, default: float, easing: str = "linear") -> float:
    if not kfs:
        return float(default)
    if t <= kfs[0][0]:
        return float(kfs[0][1])
    if t >= kfs[-1][0]:
        return float(kfs[-1][1])
    ef = _easing_fn(str(easing))
    for i in range(len(kfs) - 1):
        t0, v0 = kfs[i]
        t1, v1 = kfs[i + 1]
        if t0 <= t <= t1:
            if abs(t1 - t0) < 1e-12:
                return float(v1)
            u = (t - t0) / (t1 - t0)
            uu = float(ef(float(u)))
            return float(v0 + uu * (v1 - v0))
    return float(kfs[-1][1])


def _blend_rgba_to_rgb(rgba: np.ndarray, bg_rgb: tuple[int, int, int]) -> np.ndarray:
    rgba_u8 = np.asarray(rgba, dtype=np.uint8)
    if rgba_u8.ndim != 3 or rgba_u8.shape[2] != 4:
        raise ValueError(f"Expected RGBA uint8 (H,W,4), got shape={rgba_u8.shape}, dtype={rgba_u8.dtype}")
    rgb = rgba_u8[:, :, :3].astype(np.float32)
    a = rgba_u8[:, :, 3:4].astype(np.float32) / 255.0
    bg = np.array(bg_rgb, dtype=np.float32).reshape(1, 1, 3)
    out = rgb * a + bg * (1.0 - a)
    return np.clip(out, 0.0, 255.0).astype(np.uint8)


def _parse_bg_rgb(bg: Any) -> tuple[int, int, int]:
    """
    Background color specification -> (R,G,B) uint8.

    Supported:
      - matplotlib color strings: "white", "black", "tab:blue", ...
      - hex: "#RRGGBB" or "#RGB"
      - CSV: "255,255,255" or "0.2,0.3,0.4" (0..1 floats)
      - list/tuple: [r,g,b] (0..255 ints or 0..1 floats)
    """
    if isinstance(bg, (list, tuple)) and len(bg) == 3:
        vals = [float(bg[0]), float(bg[1]), float(bg[2])]
        # Heuristic: if any > 1 => treat as 0..255
        if any(v > 1.0 for v in vals):
            vals = [max(0.0, min(255.0, v)) / 255.0 for v in vals]
        else:
            vals = [max(0.0, min(1.0, v)) for v in vals]
        return (int(round(vals[0] * 255.0)), int(round(vals[1] * 255.0)), int(round(vals[2] * 255.0)))

    s = str(bg).strip()
    if not s:
        s = "white"

    # CSV "r,g,b"
    if "," in s:
        parts = [p.strip() for p in s.split(",") if p.strip() != ""]
        if len(parts) == 3:
            vals = [float(parts[0]), float(parts[1]), float(parts[2])]
            if any(v > 1.0 for v in vals):
                vals = [max(0.0, min(255.0, v)) / 255.0 for v in vals]
            else:
                vals = [max(0.0, min(1.0, v)) for v in vals]
            return (int(round(vals[0] * 255.0)), int(round(vals[1] * 255.0)), int(round(vals[2] * 255.0)))

    # Hex handling (manual, to avoid depending on mpl.colors)
    if s.startswith("#"):
        hx = s[1:]
        if len(hx) == 3:
            r = int(hx[0] * 2, 16)
            g = int(hx[1] * 2, 16)
            b = int(hx[2] * 2, 16)
            return (r, g, b)
        if len(hx) == 6:
            r = int(hx[0:2], 16)
            g = int(hx[2:4], 16)
            b = int(hx[4:6], 16)
            return (r, g, b)

    # Fallback: matplotlib color names if available
    if _HAS_MPL_COLORS:
        try:
            r, g, b = _mpl_colors.to_rgb(s)  # 0..1 floats
            return (int(round(r * 255.0)), int(round(g * 255.0)), int(round(b * 255.0)))
        except Exception:
            pass

    raise ValueError(
        f"render.bg could not be parsed: {bg!r}. "
        "Use e.g. '#RRGGBB', 'r,g,b', [r,g,b], or a matplotlib color name."
    )


@dataclass
class RenderCfg:
    dpi: int = 160
    figsize: tuple[float, float] = (3.2, 2.6)
    bg: Any = "white"
    overlay: dict[str, Any] | None = None
    mode: str = "morph_3d"  # "morph_3d" | "scene3d"
    scene3d: dict[str, Any] | None = None

    def bg_rgb(self) -> tuple[int, int, int]:
        return _parse_bg_rgb(self.bg)

    def overlay_enabled(self) -> bool:
        ov = self.overlay or {}
        try:
            return bool(ov.get("enabled", False))
        except Exception:
            return False

    def overlay_layout(self) -> str:
        ov = self.overlay or {}
        try:
            return str(ov.get("layout", "top_left")).strip().lower()
        except Exception:
            return "top_left"

    def overlay_y(self) -> int:
        ov = self.overlay or {}
        try:
            return int(ov.get("y", 8))
        except Exception:
            return 8

    def overlay_pos(self) -> tuple[int, int]:
        ov = self.overlay or {}
        x = int(ov.get("x", 8))
        y = int(ov.get("y", 8))
        return (x, y)

    def overlay_font_size(self) -> int:
        ov = self.overlay or {}
        return int(ov.get("font_size", 14))

    def overlay_color_rgb(self) -> tuple[int, int, int]:
        ov = self.overlay or {}
        c = ov.get("color", None)
        if c is None:
            # auto: pick high contrast vs background
            r, g, b = self.bg_rgb()
            lum = 0.2126 * r + 0.7152 * g + 0.0722 * b
            return (0, 0, 0) if lum > 128 else (255, 255, 255)
        try:
            return _parse_bg_rgb(c)
        except Exception:
            return (0, 0, 0)

    def overlay_color_rgb_at(self, t_sec: float) -> tuple[int, int, int]:
        """
        render.overlay.color_keyframes による滑らかな色変化をサポートする。
          - color_keyframes: [[t_sec, "#RRGGBB"], ...]
          - 各区間でRGBを線形補間する（tは動画開始からの秒）
        未指定なら overlay_color_rgb() を返す。
        """
        ov = self.overlay or {}
        kfs = ov.get("color_keyframes", None)
        if not kfs:
            return self.overlay_color_rgb()
        if not isinstance(kfs, list):
            return self.overlay_color_rgb()

        pts: list[tuple[float, tuple[int, int, int]]] = []
        for it in kfs:
            if not isinstance(it, (list, tuple)) or len(it) != 2:
                continue
            try:
                tt = float(it[0])
                cc = _parse_bg_rgb(it[1])
                pts.append((tt, cc))
            except Exception:
                continue
        if not pts:
            return self.overlay_color_rgb()
        pts.sort(key=lambda x: x[0])

        t = float(t_sec)
        if t <= pts[0][0]:
            return pts[0][1]
        if t >= pts[-1][0]:
            return pts[-1][1]
        for i in range(len(pts) - 1):
            t0, c0 = pts[i]
            t1, c1 = pts[i + 1]
            if t0 <= t <= t1:
                if abs(t1 - t0) < 1e-12:
                    return c1
                u = _clamp01((t - t0) / (t1 - t0))
                r = int(round((1.0 - u) * c0[0] + u * c1[0]))
                g = int(round((1.0 - u) * c0[1] + u * c1[1]))
                b = int(round((1.0 - u) * c0[2] + u * c1[2]))
                return (max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b)))
        return pts[-1][1]

    def overlay_shadow(self) -> bool:
        ov = self.overlay or {}
        return bool(ov.get("shadow", True))


@dataclass
class DefaultsCfg:
    # animation channels
    phi_deg: float = 0.0
    psi_deg: float = 0.0
    theta_deg: float = 0.0
    L_m: float = 0.18
    alpha_deg: float = 0.0
    beta_deg: float = 0.0
    # geometry
    cx: float = 0.035
    cy: float = 0.035
    rotor_radius_in: float = 3.5
    symmetry: str = "mirror_xy"
    y_clearance: float = 0.0


def _parse_spec(path: str) -> tuple[int, RenderCfg, DefaultsCfg, list[dict[str, Any]], str]:
    with open(path, "r", encoding="utf-8") as f:
        spec = json.load(f)
    if not isinstance(spec, dict):
        raise ValueError("spec root must be a JSON object")

    fps = _as_int(_get(spec, "fps", 30), name="fps")
    _require(fps > 0, "fps must be positive")

    out_path = str(_get(_get(spec, "output", {}), "path", "out.mp4"))
    _require(bool(out_path), "output.path must be a non-empty string")

    render = _get(spec, "render", {})
    if render is None:
        render = {}
    _require(isinstance(render, dict), "render must be an object")
    dpi = _as_int(_get(render, "dpi", 160), name="render.dpi")
    figsize = _get(render, "figsize", [3.2, 2.6])
    if isinstance(figsize, (list, tuple)) and len(figsize) == 2:
        figw = _as_float(figsize[0], name="render.figsize[0]")
        figh = _as_float(figsize[1], name="render.figsize[1]")
        figsize_t = (float(figw), float(figh))
    else:
        raise ValueError("render.figsize must be [w,h]")
    bg = _get(render, "bg", "white")
    overlay = _get(render, "overlay", None)
    if overlay is not None:
        _require(isinstance(overlay, dict), "render.overlay must be an object when provided")
    mode = str(_get(render, "mode", "morph_3d")).strip().lower()
    _require(mode in {"morph_3d", "scene3d"}, "render.mode must be 'morph_3d' or 'scene3d'")
    scene3d = _get(render, "scene3d", None)
    if scene3d is not None:
        _require(isinstance(scene3d, dict), "render.scene3d must be an object when provided")
    render_cfg = RenderCfg(dpi=int(dpi), figsize=figsize_t, bg=bg, overlay=overlay, mode=mode, scene3d=scene3d)

    defaults = _get(spec, "defaults", {})
    if defaults is None:
        defaults = {}
    _require(isinstance(defaults, dict), "defaults must be an object")

    def _def(name: str, dv: Any) -> Any:
        return _get(defaults, name, dv)

    defaults_cfg = DefaultsCfg(
        phi_deg=_as_float(_def("phi_deg", 0.0), name="defaults.phi_deg"),
        psi_deg=_as_float(_def("psi_deg", 0.0), name="defaults.psi_deg"),
        theta_deg=_as_float(_def("theta_deg", 0.0), name="defaults.theta_deg"),
        L_m=_as_float(_def("L_m", 0.18), name="defaults.L_m"),
        alpha_deg=_as_float(_def("alpha_deg", 0.0), name="defaults.alpha_deg"),
        beta_deg=_as_float(_def("beta_deg", 0.0), name="defaults.beta_deg"),
        cx=_as_float(_def("cx", 0.035), name="defaults.cx"),
        cy=_as_float(_def("cy", 0.035), name="defaults.cy"),
        rotor_radius_in=_as_float(_def("rotor_radius_in", 3.5), name="defaults.rotor_radius_in"),
        symmetry=str(_def("symmetry", "mirror_xy")),
        y_clearance=_as_float(_def("y_clearance", 0.0), name="defaults.y_clearance"),
    )

    segments = _get(spec, "segments", None)
    _require(isinstance(segments, list) and len(segments) > 0, "segments must be a non-empty list")
    for i, s in enumerate(segments):
        _require(isinstance(s, dict), f"segments[{i}] must be an object")
        _require("duration_s" in s, f"segments[{i}] missing duration_s")
        dur = _as_float(s["duration_s"], name=f"segments[{i}].duration_s")
        _require(dur > 0.0, f"segments[{i}].duration_s must be > 0")
        mode = str(_get(s, "mode", "")).strip()
        _require(
            mode in {"angles_keyframes", "alpha_beta_circle", "alpha_beta_keyframes"},
            f"segments[{i}].mode must be one of angles_keyframes|alpha_beta_circle|alpha_beta_keyframes",
        )

    return fps, render_cfg, defaults_cfg, segments, out_path


def _segment_times(duration_s: float, fps: int, *, include_endpoint: bool) -> np.ndarray:
    dt = 1.0 / float(fps)
    if include_endpoint:
        n = int(round(duration_s * fps)) + 1
        n = max(2, n)
        return np.linspace(0.0, float(duration_s), n, endpoint=True)
    # [0, duration) with step 1/fps
    n = int(round(duration_s * fps))
    n = max(1, n)
    return np.arange(n, dtype=float) * dt


def _total_frames(segments: list[dict[str, Any]], fps: int) -> int:
    """
    Must match exactly the number of frames yielded by _iter_frames_from_segments().
    """
    total = 0
    for i, seg in enumerate(segments):
        duration_s = float(seg["duration_s"])
        include_endpoint = bool(seg.get("include_endpoint", (i == len(segments) - 1)))
        if include_endpoint:
            n = int(round(duration_s * fps)) + 1
            n = max(2, n)
        else:
            n = int(round(duration_s * fps))
            n = max(1, n)
        total += int(n)
    return int(total)


def _iter_frames_from_segments(
    fps: int,
    render_cfg: RenderCfg,
    defaults_cfg: DefaultsCfg,
    segments: list[dict[str, Any]],
    stats: dict[str, Any] | None = None,
):
    """
    Yields RGB uint8 frames.
    """
    state = dict(
        phi_deg=float(defaults_cfg.phi_deg),
        psi_deg=float(defaults_cfg.psi_deg),
        theta_deg=float(defaults_cfg.theta_deg),
        L_m=float(defaults_cfg.L_m),
        alpha_deg=float(defaults_cfg.alpha_deg),
        beta_deg=float(defaults_cfg.beta_deg),
    )

    def _normalize(v: np.ndarray) -> np.ndarray:
        v = np.asarray(v, dtype=float).reshape(3)
        n = float(np.linalg.norm(v))
        if n < 1e-12:
            return v
        return v / n

    def _alpha_beta_from_vec(v: np.ndarray) -> tuple[float, float]:
        v = _normalize(v)
        vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
        alpha = -float(np.degrees(np.arctan2(vy, vz)))
        beta = -float(np.degrees(np.arctan2(vx, vz)))
        return alpha, beta

    def _rotor0_normal(phi_deg: float, psi_deg: float, theta_deg: float, cx: float, cy: float) -> np.ndarray:
        if _make_arm_pose is None:
            raise RuntimeError("visualize_morph_drone._make_arm_pose is unavailable; cannot compute actual alpha/beta.")
        hinge = np.array([float(cx), float(cy), 0.0], dtype=float)
        arm_dir0 = _normalize(np.array([1.0, 1.0, 0.0], dtype=float))
        pose0 = _make_arm_pose(hinge=hinge, arm_dir0=arm_dir0, phi_deg=float(phi_deg), psi_deg=float(psi_deg), theta_deg=float(theta_deg))
        return np.asarray(pose0.rotor_normal, dtype=float).reshape(3)

    def _apply_overlay(rgb: np.ndarray, lines: list[str], *, t_global_sec: float) -> np.ndarray:
        if not render_cfg.overlay_enabled():
            return rgb
        if not _HAS_PIL:
            return rgb
        try:
            img = Image.fromarray(np.asarray(rgb, dtype=np.uint8), mode="RGB")
            draw = ImageDraw.Draw(img)
            fs = int(render_cfg.overlay_font_size())
            try:
                # DejaVuSans supports Greek letters reasonably well.
                font = ImageFont.truetype("DejaVuSans.ttf", fs)
            except Exception:
                font = ImageFont.load_default()
            x0, y0 = render_cfg.overlay_pos()
            layout = render_cfg.overlay_layout()
            if layout == "top_center":
                y0 = int(render_cfg.overlay_y())
            col = render_cfg.overlay_color_rgb_at(float(t_global_sec))
            shadow = render_cfg.overlay_shadow()
            y = int(y0)
            for ln in lines:
                xx = int(x0)
                if layout == "top_center":
                    try:
                        bb = draw.textbbox((0, 0), ln, font=font)
                        tw = int(bb[2] - bb[0])
                        xx = int((img.size[0] - tw) / 2)
                    except Exception:
                        xx = int(x0)
                if shadow:
                    draw.text((xx + 2, y + 2), ln, fill=(0, 0, 0), font=font)
                draw.text((xx, y), ln, fill=col, font=font)
                y += int(fs * 1.2)
            return np.asarray(img, dtype=np.uint8)
        except Exception:
            return rgb

    t_base = 0.0
    for seg_i, seg in enumerate(segments):
        mode = str(seg["mode"])
        duration_s = float(seg["duration_s"])
        include_endpoint = bool(seg.get("include_endpoint", (seg_i == len(segments) - 1)))
        ts = _segment_times(duration_s, fps, include_endpoint=include_endpoint)

        # keyframes are always allowed for phi/theta/psi/L (even if not used by a mode)
        keys_deg = seg.get("keys_deg", {}) or {}
        keys_m = seg.get("keys_m", {}) or {}
        _require(isinstance(keys_deg, dict), f"segments[{seg_i}].keys_deg must be an object")
        _require(isinstance(keys_m, dict), f"segments[{seg_i}].keys_m must be an object")

        easing_deg = seg.get("easing_deg", {}) or {}
        easing_m = seg.get("easing_m", {}) or {}
        _require(isinstance(easing_deg, dict), f"segments[{seg_i}].easing_deg must be an object")
        _require(isinstance(easing_m, dict), f"segments[{seg_i}].easing_m must be an object")

        def _ease_deg(ch: str) -> str:
            return str(easing_deg.get(ch, "linear"))

        def _ease_m(ch: str) -> str:
            return str(easing_m.get(ch, "linear"))

        k_phi = _parse_keyframes(keys_deg.get("phi", None), name=f"segments[{seg_i}].keys_deg.phi")
        k_psi = _parse_keyframes(keys_deg.get("psi", None), name=f"segments[{seg_i}].keys_deg.psi")
        k_theta = _parse_keyframes(keys_deg.get("theta", None), name=f"segments[{seg_i}].keys_deg.theta")
        k_alpha = _parse_keyframes(keys_deg.get("alpha", None), name=f"segments[{seg_i}].keys_deg.alpha")
        k_beta = _parse_keyframes(keys_deg.get("beta", None), name=f"segments[{seg_i}].keys_deg.beta")
        k_L = _parse_keyframes(keys_m.get("L", None), name=f"segments[{seg_i}].keys_m.L")

        # segment-local overrides for geometry (optional)
        geo = seg.get("geometry", {}) or {}
        _require(isinstance(geo, dict), f"segments[{seg_i}].geometry must be an object")
        cx = float(geo.get("cx", defaults_cfg.cx))
        cy = float(geo.get("cy", defaults_cfg.cy))
        rotor_radius_in = float(geo.get("rotor_radius_in", defaults_cfg.rotor_radius_in))
        symmetry = str(geo.get("symmetry", defaults_cfg.symmetry))
        y_clearance = float(geo.get("y_clearance", defaults_cfg.y_clearance))

        # alpha-beta circle params (if used)
        if mode == "alpha_beta_circle":
            alpha_center = _as_float(seg.get("alpha_center_deg", 0.0), name=f"segments[{seg_i}].alpha_center_deg")
            beta_center = _as_float(seg.get("beta_center_deg", 0.0), name=f"segments[{seg_i}].beta_center_deg")
            radius = _as_float(seg.get("radius_deg", 10.0), name=f"segments[{seg_i}].radius_deg")
            cycles = _as_float(seg.get("cycles", 1.0), name=f"segments[{seg_i}].cycles")
            phase_deg = _as_float(seg.get("phase_deg", 0.0), name=f"segments[{seg_i}].phase_deg")
            # Optional easing for circle phase progression. Prefer explicit `easing_circle`,
            # otherwise fall back to easing_deg.alpha if provided.
            easing_circle = str(seg.get("easing_circle", easing_deg.get("alpha", "linear")))
            # avoid division by zero
            cycles = float(cycles)
            phase = math.radians(float(phase_deg))
        else:
            alpha_center = beta_center = radius = cycles = phase = 0.0
            easing_circle = "linear"

        for t in ts:
            # Update state from keyframes if provided; otherwise keep previous value.
            if k_phi:
                state["phi_deg"] = _interp_keyframes(k_phi, float(t), default=state["phi_deg"], easing=_ease_deg("phi"))
            if k_psi:
                state["psi_deg"] = _interp_keyframes(k_psi, float(t), default=state["psi_deg"], easing=_ease_deg("psi"))
            if k_theta:
                state["theta_deg"] = _interp_keyframes(k_theta, float(t), default=state["theta_deg"], easing=_ease_deg("theta"))
            if k_alpha:
                state["alpha_deg"] = _interp_keyframes(k_alpha, float(t), default=state["alpha_deg"], easing=_ease_deg("alpha"))
            if k_beta:
                state["beta_deg"] = _interp_keyframes(k_beta, float(t), default=state["beta_deg"], easing=_ease_deg("beta"))
            if k_L:
                state["L_m"] = _interp_keyframes(k_L, float(t), default=state["L_m"], easing=_ease_m("L"))

            phi = float(state["phi_deg"])
            psi = float(state["psi_deg"])
            theta = float(state["theta_deg"])
            L = float(state["L_m"])
            alpha_kf = float(state["alpha_deg"])
            beta_kf = float(state["beta_deg"])

            if mode == "alpha_beta_circle":
                # alpha/beta circle: parameterize by normalized time within this segment.
                # Use t_seg in [0, duration] (endpoint included only on last segment).
                tau = 0.0 if duration_s <= 1e-12 else float(t) / float(duration_s)
                try:
                    tau = float(_easing_fn(str(easing_circle))(float(tau)))
                except Exception:
                    tau = float(tau)
                ang = 2.0 * math.pi * float(cycles) * float(tau) + float(phase)
                alpha = float(alpha_center) + float(radius) * float(math.cos(ang))
                beta = float(beta_center) + float(radius) * float(math.sin(ang))
                psi, theta, _res = solve_psi_theta_from_alpha_beta_deg(
                    phi_deg=float(phi),
                    alpha_deg=float(alpha),
                    beta_deg=float(beta),
                )
                alpha_tgt, beta_tgt = float(alpha), float(beta)
            elif mode == "alpha_beta_keyframes":
                # alpha/beta are provided by keyframes (or defaults), phi may also be keyframed.
                psi, theta, _res = solve_psi_theta_from_alpha_beta_deg(
                    phi_deg=float(phi),
                    alpha_deg=float(alpha_kf),
                    beta_deg=float(beta_kf),
                )
                alpha_tgt, beta_tgt = float(alpha_kf), float(beta_kf)
            else:
                alpha_tgt, beta_tgt = None, None

            # --- render ---
            if str(render_cfg.mode).lower() == "scene3d":
                if render_scene3d_rgba is None:
                    raise RuntimeError("render.mode='scene3d' requested but visualize_morph_drone.render_scene3d_rgba is not available.")
                sc = render_cfg.scene3d or {}
                drone_center = sc.get("drone_center", [0.0, 0.0, 0.0])
                if isinstance(drone_center, (list, tuple)) and len(drone_center) == 3:
                    drone_center_t = (float(drone_center[0]), float(drone_center[1]), float(drone_center[2]))
                else:
                    raise ValueError("render.scene3d.drone_center must be [x,y,z]")
                rgba = render_scene3d_rgba(
                    float(phi),
                    float(psi),
                    float(theta),
                    cx=float(cx),
                    cy=float(cy),
                    rotor_radius_in=float(rotor_radius_in),
                    arm_length_m=float(sc.get("arm_length_m", L)),
                    symmetry=str(symmetry),
                    drone_center=drone_center_t,
                    rotor_inflow_offset=float(sc.get("rotor_inflow_offset", 0.02)),
                    view_elev=(None if sc.get("view_elev", None) is None else float(sc.get("view_elev"))),
                    view_azim=(None if sc.get("view_azim", None) is None else float(sc.get("view_azim"))),
                    draw_y0_plane=bool(sc.get("draw_y0_plane", True)),
                    drone_lw=float(sc.get("drone_lw", 2.5)),
                    dpi=int(render_cfg.dpi),
                    figsize=tuple(render_cfg.figsize),
                )
            else:
                rgba = render_morphing_drone_rgba(
                    float(phi),
                    float(psi),
                    float(theta),
                    cx=float(cx),
                    cy=float(cy),
                    rotor_radius_in=float(rotor_radius_in),
                    arm_length_m=float(L),
                    symmetry=str(symmetry),
                    y_clearance=float(y_clearance),
                    dpi=int(render_cfg.dpi),
                    figsize=tuple(render_cfg.figsize),
                    draw_y0_plane=False,
                )
            rgb = _blend_rgba_to_rgb(rgba, render_cfg.bg_rgb())

            # Compute actual alpha/beta from rotor0 normal (pose math), for validation and overlay.
            try:
                n0 = _rotor0_normal(phi, psi, theta, cx=float(cx), cy=float(cy))
                alpha_act, beta_act = _alpha_beta_from_vec(n0)
            except Exception:
                alpha_act, beta_act = float("nan"), float("nan")

            if stats is not None and alpha_tgt is not None and beta_tgt is not None:
                try:
                    stats["n_ab"] = int(stats.get("n_ab", 0)) + 1
                    da = abs(float(alpha_act) - float(alpha_tgt))
                    db = abs(float(beta_act) - float(beta_tgt))
                    stats["alpha_max_abs_err_deg"] = max(float(stats.get("alpha_max_abs_err_deg", 0.0)), float(da))
                    stats["beta_max_abs_err_deg"] = max(float(stats.get("beta_max_abs_err_deg", 0.0)), float(db))
                except Exception:
                    pass

            # Overlay numbers (target/actual)
            if render_cfg.overlay_enabled():
                lines = []
                if alpha_tgt is not None and beta_tgt is not None:
                    # Presentation-friendly: show target only.
                    lines.append(f"(α, β) = ({alpha_tgt:+.1f}°, {beta_tgt:+.1f}°)")
                else:
                    lines.append(f"(α, β) = ({alpha_act:+.1f}°, {beta_act:+.1f}°)")
                rgb = _apply_overlay(rgb, lines, t_global_sec=(float(t_base) + float(t)))

            yield rgb

        # Advance global timeline by segment duration (deterministic for spec timestamps).
        t_base += float(duration_s)


def _write_mp4_with_ffmpeg(
    *,
    out_path: str,
    fps: int,
    frames_rgb: Any,
    total_frames: int | None = None,
    show_progress: bool = True,
):
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("ffmpeg が見つかりません。mp4生成には ffmpeg が必要です。 (例: sudo apt-get install -y ffmpeg)")

    out_dir = os.path.dirname(os.path.abspath(out_path))
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    first = next(frames_rgb, None)
    if first is None:
        raise RuntimeError("No frames generated (spec produced empty timeline).")

    first = np.asarray(first, dtype=np.uint8)
    if first.ndim != 3 or first.shape[2] != 3:
        raise ValueError(f"Expected RGB uint8 (H,W,3), got shape={first.shape}, dtype={first.dtype}")

    def _pad_even(fr: np.ndarray) -> np.ndarray:
        fr = np.asarray(fr, dtype=np.uint8)
        hh, ww = int(fr.shape[0]), int(fr.shape[1])
        pad_h = (hh % 2)
        pad_w = (ww % 2)
        if pad_h == 0 and pad_w == 0:
            return fr
        # pad bottom/right by repeating edge pixels to preserve background appearance
        return np.pad(fr, ((0, pad_h), (0, pad_w), (0, 0)), mode="edge")

    first = _pad_even(first)
    h, w = int(first.shape[0]), int(first.shape[1])

    cmd = [
        ffmpeg,
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-vcodec",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{w}x{h}",
        "-r",
        str(int(fps)),
        "-i",
        "-",
        "-an",
        "-vcodec",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(out_path),
    ]

    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    pbar = None
    if bool(show_progress) and _HAS_TQDM:
        try:
            pbar = _tqdm(
                total=(int(total_frames) if total_frames is not None else None),
                desc="Rendering",
                unit="frame",
            )
        except Exception:
            pbar = None
    try:
        assert proc.stdin is not None
        proc.stdin.write(first.tobytes(order="C"))
        n = 1
        if pbar is not None:
            try:
                pbar.update(1)
            except Exception:
                pass
        for fr in frames_rgb:
            fr = np.asarray(fr, dtype=np.uint8)
            if fr.shape != (h, w, 3):
                fr2 = _pad_even(fr)
                if fr2.shape != (h, w, 3):
                    raise ValueError(f"Frame size mismatch: expected {(h,w,3)}, got {fr.shape} (padded -> {fr2.shape})")
                fr = fr2
            try:
                proc.stdin.write(fr.tobytes(order="C"))
            except BrokenPipeError:
                # ffmpeg exited early; capture stderr for diagnosis
                err_b = b""
                try:
                    if proc.stderr is not None:
                        err_b = proc.stderr.read()
                except Exception:
                    pass
                rc = proc.poll()
                raise RuntimeError(
                    "ffmpeg terminated early while writing frames. "
                    f"returncode={rc}. stderr:\n{err_b.decode(errors='replace')}"
                )
            n += 1
            if pbar is not None:
                try:
                    pbar.update(1)
                except Exception:
                    pass
        proc.stdin.close()
        # Avoid subprocess.communicate() here: it may try to flush a closed stdin on some Python versions.
        err_b = b""
        if proc.stderr is not None:
            err_b = proc.stderr.read()
        rc = proc.wait()
        if rc != 0:
            raise RuntimeError(f"ffmpeg failed (code={rc}). stderr:\n{err_b.decode(errors='replace')}")
        return n
    finally:
        if pbar is not None:
            try:
                pbar.close()
            except Exception:
                pass
        try:
            if proc.stdin and not proc.stdin.closed:
                proc.stdin.close()
        except Exception:
            pass
        try:
            if proc.poll() is None:
                proc.terminate()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description="Generate MP4 animation of morphing drone (spec-driven).")
    ap.add_argument("--spec", required=True, type=str, help="Path to JSON spec file.")
    ap.add_argument("--out", default=None, type=str, help="Override output.path in spec.")
    ap.add_argument("--fps", default=None, type=int, help="Override fps in spec.")
    ap.add_argument(
        "--use-venv-mpl",
        action="store_true",
        help=(
            "WSL向け: drone_thrust_analyzer/.venv-mpl を使って再実行する（matplotlib 3D衝突回避用）。 "
            "このプロセスはすぐ終了し、代わりに .venv-mpl/bin/python で同じコマンドを起動します。"
        ),
    )
    ap.add_argument(
        "--no-progress",
        action="store_true",
        help="Disable tqdm progress display.",
    )
    args = ap.parse_args()

    if bool(args.use_venv_mpl):
        # Re-exec with the project's venv to avoid matplotlib/3D import issues.
        # NOTE: this script is located at auto_thrust_recorder/scripts/...
        this_py = os.path.abspath(__file__)
        # repo root = .../drone_thrust_analyzer
        repo_root = os.path.abspath(os.path.join(os.path.dirname(this_py), "..", ".."))
        vpy = os.path.join(repo_root, ".venv-mpl", "bin", "python")
        if not os.path.exists(vpy):
            raise SystemExit(f"--use-venv-mpl: not found: {vpy}")
        # rebuild argv without this flag
        argv2 = [vpy] + [a for a in sys.argv if a != "--use-venv-mpl"]
        os.execv(vpy, argv2)

    fps, render_cfg, defaults_cfg, segments, out_path = _parse_spec(str(args.spec))
    if args.out is not None:
        out_path = str(args.out)
    if args.fps is not None:
        fps = int(args.fps)

    stats: dict[str, Any] = {}
    frames = _iter_frames_from_segments(int(fps), render_cfg, defaults_cfg, segments, stats=stats)
    n_total = _total_frames(segments, int(fps))
    n = _write_mp4_with_ffmpeg(
        out_path=str(out_path),
        fps=int(fps),
        frames_rgb=frames,
        total_frames=int(n_total),
        show_progress=(not bool(args.no_progress)),
    )
    print(f"Saved MP4: {out_path} (frames={n}, fps={fps})")
    if int(stats.get("n_ab", 0)) > 0:
        print(
            "alpha/beta validation (rotor0): "
            f"max|alpha_act-alpha_tgt|={float(stats.get('alpha_max_abs_err_deg', 0.0)):.3f} deg, "
            f"max|beta_act-beta_tgt|={float(stats.get('beta_max_abs_err_deg', 0.0)):.3f} deg "
            f"(frames={int(stats.get('n_ab', 0))})"
        )


if __name__ == "__main__":
    main()


