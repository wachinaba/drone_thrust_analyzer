#!/usr/bin/env python3
"""
CSV に morphing drone の座標変換由来パラメータ列 + 計算列を追加する。

座標変換の定義は auto_thrust_recorder/scripts/visualize_morph_drone.py に準拠:
  fold(phi) -> slant(psi) -> tilt(theta)
  ただし符号規約も同じ:
    phi, psi は内部で -deg を使う（回転方向を反転）
    theta は +deg

追加列:
  - rotor_radius [m] (列が無ければ追加。引数は inch 指定、列は m で保持)
  - fold_center_x, fold_center_y [m] (列が無ければ追加)
  - alpha, beta [deg]
  - prop_spacing_x, prop_spacing_y, aspect_ratio
  - arm_length [m] (arm_length 列が無い/NaN の場合 wheelbase から推定)
  - distance_center, distance_rotortip [m]
  - thrust_coefficient: thrust_coefficient = force_z / control^2
  - normalized_moment: normalized_moment = torque_x_bias_corrected / (prop_spacing_y * force_z / 2) * 100
  - normalized_thrust:
      tilt=fold=slant=0deg における推力係数 [a,b,c] を用いて control -> thrust を推定し、
      thrust を thrust vector の大きさとみなして alpha,beta から鉛直成分 base_thrust_z を作り、
      normalized_thrust = force_z_bias_corrected / base_thrust_z として正規化する。
  - base_thrust: 上記モデルで推定した推力ベクトルの大きさ T (= a*control^2 + b*control + c)
  - base_thrust_z: 上記 base_thrust の鉛直成分

距離補正の考え方:
  入力の distance は [R]（ロータ半径単位）の「設定値」とみなし、m へ変換する。
  実験での位置決めは tilt=0 を前提に rotor tip clearance(distance) を満たすように行われたと仮定し、
  そのときの drone center の y オフセット dy を決める。
  その dy を固定したまま、実際の tilt を考慮した rotor rim の最近点を計算し、
  wall(y=0) までの距離を distance_rotortip とする。
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd


def _deg2rad(deg: float) -> float:
    return float(deg) * np.pi / 180.0


def _normalize(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError("zero-length vector")
    return v / n


def _rot_z(angle_rad: float) -> np.ndarray:
    c = float(np.cos(angle_rad))
    s = float(np.sin(angle_rad))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def _rot_axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    # Rodrigues' rotation formula
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
    visualize_morph_drone.py と同一の変換順/符号規約。
    """
    z = np.array([0.0, 0.0, 1.0], dtype=float)

    phi = _deg2rad(-phi_deg)
    psi = _deg2rad(-psi_deg)
    theta = _deg2rad(theta_deg)

    R_fold = _rot_z(phi)
    arm_dir1 = _normalize(R_fold @ arm_dir0)

    slant_axis = np.cross(z, arm_dir1)
    if np.linalg.norm(slant_axis) < 1e-10:
        R_slant = np.eye(3, dtype=float)
    else:
        R_slant = _rot_axis_angle(slant_axis, psi)

    arm_dir2 = _normalize(R_slant @ arm_dir1)

    rotor_n0 = z
    rotor_n2 = _normalize(R_slant @ (R_fold @ rotor_n0))

    R_tilt = _rot_axis_angle(arm_dir2, theta)
    rotor_n3 = _normalize(R_tilt @ rotor_n2)

    return ArmPose(hinge=hinge.astype(float), arm_dir=arm_dir2, rotor_normal=rotor_n3)


def _compute_poses_mirror_xy(cx: float, cy: float, phi: float, psi: float, theta: float) -> list[ArmPose]:
    base_hinge = np.array([+cx, +cy, 0.0], dtype=float)
    base_arm_dir0 = _normalize(np.array([+1.0, +1.0, 0.0], dtype=float))
    base_pose = _make_arm_pose(base_hinge, base_arm_dir0, phi, psi, theta)

    M_id = np.diag([1.0, 1.0, 1.0])
    M_x = np.diag([-1.0, 1.0, 1.0])  # x -> -x
    M_y = np.diag([1.0, -1.0, 1.0])  # y -> -y
    M_xy = np.diag([-1.0, -1.0, 1.0])
    Ms = [M_id, M_x, M_xy, M_y]

    poses = []
    for M in Ms:
        poses.append(
            ArmPose(
                hinge=(M @ base_pose.hinge),
                arm_dir=_normalize(M @ base_pose.arm_dir),
                rotor_normal=_normalize(M @ base_pose.rotor_normal),
            )
        )
    return poses


def _thrust_angles_alpha_beta_deg(thrust_vec: np.ndarray) -> tuple[float, float]:
    v = _normalize(np.asarray(thrust_vec, dtype=float).reshape(3))
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    # NOTE: alpha/beta の符号規約は visualize_morph_drone.py に合わせる（出力のみ反転）。
    alpha = -float(np.degrees(np.arctan2(vy, vz)))
    beta = -float(np.degrees(np.arctan2(vx, vz)))
    return alpha, beta


def _ymin_rotor_rim(poses: list[ArmPose], arm_length_m: float, rotor_radius_m: float) -> float:
    """
    visualize_morph_drone.py と同じ解析式:
      y_min(rotor) = y_center - R * sqrt(1 - n_y^2)
    """
    ymins = []
    for p in poses:
        y_center = float(p.hinge[1] + arm_length_m * p.arm_dir[1])
        ny = float(p.rotor_normal[1])
        extent = rotor_radius_m * float(np.sqrt(max(0.0, 1.0 - ny * ny)))
        ymins.append(y_center - extent)
    if not ymins:
        return 0.0
    return float(min(ymins))


def _rotor_centers_xyz(poses: list[ArmPose], arm_length_m: float) -> np.ndarray:
    centers = []
    for p in poses:
        centers.append(p.hinge + arm_length_m * p.arm_dir)
    return np.vstack(centers)  # (4,3)


def _build_output_path(input_path: str, output_path: str | None) -> Path:
    if output_path:
        return Path(output_path)
    in_path = Path(input_path)
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    return in_path.with_name(f"{in_path.stem}_calccols_{ts}{in_path.suffix}")


def _ensure_constant_column(df: pd.DataFrame, col: str, value) -> pd.DataFrame:
    if col not in df.columns:
        df[col] = value
    return df


def _fill_arm_length_from_wheelbase(df: pd.DataFrame, *, rotor_radius_default_m: float) -> pd.DataFrame:
    """
    wheelbase から arm_length [m] を当てはめる。

    指定式:
      arm_length = 0.128 + (wheelbase - 2.7) * rotor_radius

    - wheelbase は [R]（ロータ半径単位）を想定
    - rotor_radius は [m]
    arm_length 列がある場合は NaN のみ補完する（既存値は尊重）。
    """
    # 後方互換: csv_concat_4.py の過去の命名で wheelbase が prop_spacing に入っている場合がある
    if "wheelbase" not in df.columns and "prop_spacing" in df.columns:
        df["wheelbase"] = df["prop_spacing"]
    if "wheelbase" not in df.columns:
        raise ValueError("MissingColumns: wheelbase (or prop_spacing)")
    if "rotor_radius" not in df.columns:
        raise ValueError("MissingColumns: rotor_radius")
    wb = pd.to_numeric(df["wheelbase"], errors="coerce")
    rr = pd.to_numeric(df["rotor_radius"], errors="coerce").fillna(float(rotor_radius_default_m))
    estimated = 0.128 + (wb - 2.7) * rr
    if "arm_length" in df.columns:
        al = pd.to_numeric(df["arm_length"], errors="coerce")
        df["arm_length"] = al.where(al.notna(), estimated)
    else:
        df["arm_length"] = estimated
    return df


def _warn_missing(cols: list[str], *, context: str) -> None:
    if cols:
        print(f"[add_calculated_columns_to_csv] WARN: missing columns for {context}: {cols}", file=sys.stderr)


def _compute_thrust_coefficient(df: pd.DataFrame) -> pd.Series:
    required = ["force_z", "control"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        _warn_missing(missing, context="thrust_coefficient")
        return pd.Series(np.nan, index=df.index, dtype=float)
    force_z = pd.to_numeric(df["force_z"], errors="coerce")
    control = pd.to_numeric(df["control"], errors="coerce")
    denom = control * control
    denom = denom.where(denom != 0.0, np.nan)
    return force_z / denom


def _compute_normalized_moment(df: pd.DataFrame) -> pd.Series:
    required = ["torque_x_bias_corrected", "prop_spacing_y", "force_z"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        _warn_missing(missing, context="normalized_moment")
        return pd.Series(np.nan, index=df.index, dtype=float)
    torque = pd.to_numeric(df["torque_x_bias_corrected"], errors="coerce")
    prop_spacing_y = pd.to_numeric(df["prop_spacing_y"], errors="coerce")
    force_z = pd.to_numeric(df["force_z"], errors="coerce")
    denom = prop_spacing_y * force_z / 2.0
    denom = denom.where(denom != 0.0, np.nan)
    return torque / denom * 100.0


def _compute_base_thrust_z_from_control(
    df: pd.DataFrame,
    *,
    a: float,
    b: float,
    c: float,
) -> pd.Series:
    """
    base_thrust_z:
      control -> thrust magnitude: T = a*x^2 + b*x + c
      alpha,beta -> unit vector z component: v_z = 1/sqrt(1+tan(alpha)^2+tan(beta)^2)
      base_thrust_z = T * v_z
    """
    required = ["control", "alpha", "beta"]
    missing = [col for col in required if col not in df.columns]
    if missing:
        _warn_missing(missing, context="base_thrust_z(normalized_thrust)")
        return pd.Series(np.nan, index=df.index, dtype=float)

    x = pd.to_numeric(df["control"], errors="coerce")
    alpha_deg = pd.to_numeric(df["alpha"], errors="coerce")
    beta_deg = pd.to_numeric(df["beta"], errors="coerce")

    # thrust magnitude model (scalar)
    T = float(a) * (x * x) + float(b) * x + float(c)

    # v_z from alpha,beta definition:
    # alpha = atan2(vy, vz), beta = atan2(vx, vz)
    # => tan(alpha)=vy/vz, tan(beta)=vx/vz, and ||v||=1
    alpha = np.deg2rad(alpha_deg.astype(float))
    beta = np.deg2rad(beta_deg.astype(float))
    ta = np.tan(alpha)
    tb = np.tan(beta)
    vz = 1.0 / np.sqrt(1.0 + ta * ta + tb * tb)

    # if alpha/beta are NaN, vz becomes NaN; keep it
    return T * vz


def _compute_base_thrust_from_control(df: pd.DataFrame, *, a: float, b: float, c: float) -> pd.Series:
    required = ["control"]
    missing = [col for col in required if col not in df.columns]
    if missing:
        _warn_missing(missing, context="base_thrust")
        return pd.Series(np.nan, index=df.index, dtype=float)
    x = pd.to_numeric(df["control"], errors="coerce")
    return float(a) * (x * x) + float(b) * x + float(c)


def _compute_normalized_thrust(df: pd.DataFrame, *, a: float, b: float, c: float) -> pd.Series:
    required = ["force_z_bias_corrected"]
    missing = [col for col in required if col not in df.columns]
    if missing:
        _warn_missing(missing, context="normalized_thrust")
        return pd.Series(np.nan, index=df.index, dtype=float)

    base_thrust_z = _compute_base_thrust_z_from_control(df, a=float(a), b=float(b), c=float(c))
    denom = base_thrust_z.where(base_thrust_z != 0.0, np.nan)
    fz = pd.to_numeric(df["force_z_bias_corrected"], errors="coerce")
    return fz / denom


def process_csv(input_csv: str, *, cx: float, cy: float, rotor_radius_in: float) -> pd.DataFrame:
    df = pd.read_csv(input_csv)

    # 追加（列が無い場合のみ）
    rotor_radius_m = float(rotor_radius_in) * 0.0254
    df = _ensure_constant_column(df, "rotor_radius", rotor_radius_m)
    df = _ensure_constant_column(df, "fold_center_x", float(cx))
    df = _ensure_constant_column(df, "fold_center_y", float(cy))

    # 必須列（追加計算に必要）
    # 後方互換: wheelbase が無ければ prop_spacing を wheelbase として扱う
    if "wheelbase" not in df.columns and "prop_spacing" in df.columns:
        df["wheelbase"] = df["prop_spacing"]

    required = ["tilt_angle", "fold_angle", "slant_angle", "distance", "wheelbase"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"MissingColumns: {','.join(missing)}")

    # arm_length を準備
    df = _fill_arm_length_from_wheelbase(df, rotor_radius_default_m=rotor_radius_m)

    # 数値化（計算に必要なもの）
    for c in [
        "tilt_angle",
        "fold_angle",
        "slant_angle",
        "distance",
        "rotor_radius",
        "fold_center_x",
        "fold_center_y",
        "arm_length",
        "force_z",
        "control",
        "torque_x_bias_corrected",
        "target_thrust",
    ]:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")

    # config ごとに計算してマージ（行数が多いCSVでも高速化）
    key_cols = [
        "tilt_angle",
        "fold_angle",
        "slant_angle",
        "distance",
        "wheelbase",
        "rotor_radius",
        "fold_center_x",
        "fold_center_y",
        "arm_length",
    ]
    uniq = df[key_cols].drop_duplicates().reset_index(drop=True)

    out_rows = []
    for row in uniq.itertuples(index=False):
        tilt = float(row.tilt_angle) if pd.notna(row.tilt_angle) else 0.0
        fold = float(row.fold_angle) if pd.notna(row.fold_angle) else 0.0
        slant = float(row.slant_angle) if pd.notna(row.slant_angle) else 0.0
        dist_R = float(row.distance) if pd.notna(row.distance) else np.nan
        rr_m = float(row.rotor_radius) if pd.notna(row.rotor_radius) else rotor_radius_m
        cx_m = float(row.fold_center_x) if pd.notna(row.fold_center_x) else float(cx)
        cy_m = float(row.fold_center_y) if pd.notna(row.fold_center_y) else float(cy)
        arm_L = float(row.arm_length) if pd.notna(row.arm_length) else np.nan

        if not np.isfinite(arm_L):
            out_rows.append({**{k: getattr(row, k) for k in key_cols}})
            continue

        poses = _compute_poses_mirror_xy(cx_m, cy_m, fold, slant, tilt)
        poses_ref = _compute_poses_mirror_xy(cx_m, cy_m, fold, slant, 0.0)

        # alpha/beta: rotor0 (= base pose)
        alpha, beta = _thrust_angles_alpha_beta_deg(poses[0].rotor_normal)

        centers = _rotor_centers_xyz(poses, arm_L)
        prop_spacing_x = float(centers[:, 0].max() - centers[:, 0].min())
        prop_spacing_y = float(centers[:, 1].max() - centers[:, 1].min())
        aspect_ratio = float(prop_spacing_x / prop_spacing_y) if abs(prop_spacing_y) > 1e-12 else np.nan

        # distances
        clearance_m = float(dist_R) * rr_m if np.isfinite(dist_R) else np.nan

        y_min_ref = _ymin_rotor_rim(poses_ref, arm_L, rr_m)
        dy = float(clearance_m - y_min_ref) if np.isfinite(clearance_m) else np.nan

        y_min_actual = _ymin_rotor_rim(poses, arm_L, rr_m)
        distance_rotortip = float(dy + y_min_actual) if np.isfinite(dy) else np.nan
        distance_center = float(dy) if np.isfinite(dy) else np.nan

        out_rows.append(
            {
                **{k: getattr(row, k) for k in key_cols},
                "alpha": alpha,
                "beta": beta,
                "prop_spacing_x": prop_spacing_x,
                "prop_spacing_y": prop_spacing_y,
                "aspect_ratio": aspect_ratio,
                "distance_rotortip": distance_rotortip,
                "distance_center": distance_center,
            }
        )

    computed = pd.DataFrame(out_rows)
    # 既に派生列が存在するCSVに対しても冪等に動くようにする:
    # - まず計算で生成する列（key_cols 以外）を既存 df から除去してからマージする
    #   （列名衝突や suffix による重複列エラーを防ぐ）
    computed_cols = [c for c in computed.columns if c not in key_cols]
    if computed_cols:
        df = df.drop(columns=[c for c in computed_cols if c in df.columns], errors="ignore")
    # uniq は key_cols の drop_duplicates から作っているので many_to_one を期待
    df = df.merge(computed, on=key_cols, how="left", validate="many_to_one")

    # 追加の計算列（既存があっても上書きして常に最新式に合わせる）
    df["thrust_coefficient"] = _compute_thrust_coefficient(df)
    df["normalized_moment"] = _compute_normalized_moment(df)
    # normalized_thrust (defaults from tilt=fold=slant=0deg calibration)
    df["base_thrust"] = _compute_base_thrust_from_control(df, a=116.47, b=20.482, c=0.6069)
    df["base_thrust_z"] = _compute_base_thrust_z_from_control(df, a=116.47, b=20.482, c=0.6069)
    df["normalized_thrust"] = _compute_normalized_thrust(df, a=116.47, b=20.482, c=0.6069)
    return df


def main() -> None:
    parser = argparse.ArgumentParser(
        description="visualize_morph_drone.py 準拠の派生パラメータ列 + 計算列をCSVに追加する"
    )
    parser.add_argument("--input", required=True, help="入力CSVパス")
    parser.add_argument("--output", default=None, help="出力CSVパス（省略時は input と同階層に自動生成）")
    parser.add_argument("--cx", type=float, default=0.035, help="ヒンジ中心 x [m]（fold_center_x）デフォルト: 0.035")
    parser.add_argument("--cy", type=float, default=0.035, help="ヒンジ中心 y [m]（fold_center_y）デフォルト: 0.035")
    parser.add_argument(
        "--rotor-radius-in",
        type=float,
        default=3.5,
        help="ロータ半径 [inch]（列 rotor_radius が無い場合に使用。列には m で保存）デフォルト: 3.5",
    )
    parser.add_argument("--thrust-coef-a", type=float, default=116.47, help="thrust model coef a (default: 116.47)")
    parser.add_argument("--thrust-coef-b", type=float, default=20.482, help="thrust model coef b (default: 20.482)")
    parser.add_argument("--thrust-coef-c", type=float, default=0.6069, help="thrust model coef c (default: 0.6069)")
    args = parser.parse_args()

    in_path = str(args.input)
    if not os.path.exists(in_path):
        raise SystemExit(f"InputNotFound: {in_path}")

    df_out = process_csv(in_path, cx=float(args.cx), cy=float(args.cy), rotor_radius_in=float(args.rotor_radius_in))
    # overwrite baseline columns with user-specified coefficients if provided
    # (kept here to avoid changing process_csv signature)
    a = float(args.thrust_coef_a)
    b = float(args.thrust_coef_b)
    c = float(args.thrust_coef_c)
    df_out["base_thrust"] = _compute_base_thrust_from_control(df_out, a=a, b=b, c=c)
    df_out["base_thrust_z"] = _compute_base_thrust_z_from_control(df_out, a=a, b=b, c=c)
    df_out["normalized_thrust"] = _compute_normalized_thrust(df_out, a=a, b=b, c=c)
    out_path = _build_output_path(in_path, args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    df_out.to_csv(out_path, index=False)
    print(f"出力: {out_path}")


if __name__ == "__main__":
    main()


