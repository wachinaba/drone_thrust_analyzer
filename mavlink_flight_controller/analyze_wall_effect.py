#!/usr/bin/env python3
"""
Wall-effect analysis for ULog flight logs.

Reads .ulg files produced by forward_and_stop.py, auto-detects four flight
phases from MANUAL_CONTROL_SETPOINT pitch transitions, and produces six
publication-quality figures plus a statistical summary.

Usage (from project root):
    .venv/Scripts/python.exe drone_thrust_analyzer/mavlink_flight_controller/analyze_wall_effect.py

See --help for options.
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import pandas as pd
from pyulog import ULog
from scipy import signal, stats

# ---------------------------------------------------------------------------
# Constants / Style
# ---------------------------------------------------------------------------
PHASE_NAMES = ["Hover1\n(no wall)", "Forward", "Brake", "Hover2\n(near wall)"]
PHASE_KEYS = ["hover1", "forward", "brake", "hover2"]
PHASE_COLORS = ["#4e79a7", "#f28e2b", "#e15759", "#76b7b2"]
MOTOR_COLORS = ["#e15759", "#f28e2b", "#4e79a7", "#59a14f"]

_RC_PARAMS = {
    "font.family": "sans-serif",
    "font.size": 9,
    "axes.titlesize": 10,
    "axes.labelsize": 9,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "legend.fontsize": 7.5,
    "figure.dpi": 150,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
}

GROUP_B_PREFIXES = ("log_24_", "log_25_", "log_26_", "log_27_", "log_28_",
                    "log_29_", "log_30_", "log_31_", "log_32_", "log_33_",
                    "log_34_")
GROUP_A_PREFIXES = ("log_0_", "log_0002_", "log_0003_", "log_0010_")

HOVER_TRIM_S = 1.5  # seconds to trim from the start of each hover phase


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------
@dataclass
class PhaseSpan:
    name: str
    key: str
    t_start: float      # start of phase (raw, for plot shading)
    t_end: float
    color: str
    t_eval_start: float = 0.0  # start of evaluation window (after transient trim)


@dataclass
class FlightData:
    filename: str
    t0_abs: float  # first timestamp (µs)

    # manual_control_setpoint
    mc_ts: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    mc_pitch: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    mc_roll: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    mc_throttle: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))

    # vehicle_attitude  (Euler – degrees)
    att_ts: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    att_roll: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    att_pitch: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    att_yaw: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))

    # vehicle_attitude_setpoint  (Euler – degrees)
    att_sp_ts: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    att_sp_roll: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    att_sp_pitch: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))

    # vehicle_angular_velocity
    angvel_ts: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    angvel_roll: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    angvel_pitch: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    angvel_yaw: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))

    # vehicle_rates_setpoint
    rates_sp_ts: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    rates_sp_roll: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))

    # vehicle_torque_setpoint
    torque_ts: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    torque_roll: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    torque_pitch: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    torque_yaw: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))

    # vehicle_thrust_setpoint
    thrust_ts: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    thrust_x: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    thrust_y: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    thrust_z: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))

    # actuator_motors  (control[0..3])
    motor_ts: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    motors: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))  # shape (N,4)

    # actuator_outputs  (output[0..3])
    output_ts: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    outputs: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))

    # sensor_combined  (high-rate gyro & accel)
    sc_ts: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    sc_gyro: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))  # (N,3) rad/s
    sc_accel: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))  # (N,3) m/s²

    # vehicle_acceleration
    vacc_ts: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    vacc: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))  # (N,3)

    # rate_ctrl_status
    rcs_ts: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    rcs_roll_integ: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    rcs_pitch_integ: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    rcs_yaw_integ: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))

    # phases
    phases: list[PhaseSpan] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Quaternion → Euler
# ---------------------------------------------------------------------------
def _quat_to_euler_deg(q0, q1, q2, q3):
    roll = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3),
                                 1 - 2 * (q1**2 + q2**2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)))
    yaw = np.degrees(np.arctan2(2 * (q0 * q3 + q1 * q2),
                                1 - 2 * (q2**2 + q3**2)))
    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Phase detection
# ---------------------------------------------------------------------------
def detect_phases(mc_ts: np.ndarray, mc_pitch: np.ndarray,
                  mc_throttle: np.ndarray) -> list[PhaseSpan]:
    """Detect four flight phases from manual_control_setpoint pitch changes.

    The flight sequence is:
        hover1 (pitch~0.06) → forward (pitch<0) → brake (pitch>0.15) → hover2 (pitch~0.06)
    We ignore the disarm tail where throttle drops to -1.
    """
    fly_mask = mc_throttle > -0.5
    if fly_mask.sum() < 4:
        return []

    fly_ts = mc_ts[fly_mask]
    fly_pitch = mc_pitch[fly_mask]

    pitch_rounded = np.round(fly_pitch, 2)
    unique_pitches = np.unique(pitch_rounded)

    hover_pitch = unique_pitches[np.argmin(np.abs(unique_pitches))]
    fwd_pitch = unique_pitches[unique_pitches < -0.01]
    brake_pitch = unique_pitches[unique_pitches > 0.15]

    if len(fwd_pitch) == 0 or len(brake_pitch) == 0:
        return []

    fwd_pitch = fwd_pitch[0]
    brake_pitch = brake_pitch[0]

    fwd_start = fly_ts[np.argmax(pitch_rounded == fwd_pitch)]
    brake_idxs = np.where(pitch_rounded == brake_pitch)[0]
    brake_start = fly_ts[brake_idxs[0]]
    brake_end = fly_ts[brake_idxs[-1]]

    # Find the sample right after brake ends (next hover pitch after brake)
    post_brake = np.where((fly_ts > brake_end) & (pitch_rounded != brake_pitch))[0]
    if len(post_brake) > 0:
        hover2_start = fly_ts[post_brake[0]]
    else:
        hover2_start = brake_end

    hover2_end = fly_ts[-1]
    hover1_start = fly_ts[0]
    hover1_end = fwd_start

    h1_eval = min(hover1_start + HOVER_TRIM_S, hover1_end)
    h2_eval = min(hover2_start + HOVER_TRIM_S, hover2_end)

    phases = [
        PhaseSpan(PHASE_NAMES[0], PHASE_KEYS[0], hover1_start, hover1_end, PHASE_COLORS[0], h1_eval),
        PhaseSpan(PHASE_NAMES[1], PHASE_KEYS[1], fwd_start, brake_start, PHASE_COLORS[1], fwd_start),
        PhaseSpan(PHASE_NAMES[2], PHASE_KEYS[2], brake_start, hover2_start, PHASE_COLORS[2], brake_start),
        PhaseSpan(PHASE_NAMES[3], PHASE_KEYS[3], hover2_start, hover2_end, PHASE_COLORS[3], h2_eval),
    ]
    return phases


# ---------------------------------------------------------------------------
# Data extraction
# ---------------------------------------------------------------------------
def _get_topic(ulog: ULog, name: str, instance: int = 0):
    """Return the data dict for a topic, or None."""
    matches = [d for d in ulog.data_list if d.name == name]
    if instance < len(matches):
        return matches[instance]
    return None


def load_flight(path: str) -> FlightData:
    """Parse a .ulg file and return a FlightData object with relative time."""
    ulog = ULog(path)
    fd = FlightData(filename=os.path.basename(path), t0_abs=0.0)

    # --- manual_control_setpoint ---
    d = _get_topic(ulog, "manual_control_setpoint")
    if d is not None:
        fd.t0_abs = float(d.data["timestamp"][0])
        fd.mc_ts = (d.data["timestamp"] - fd.t0_abs) / 1e6
        fd.mc_pitch = d.data["pitch"].astype(np.float64)
        fd.mc_roll = d.data["roll"].astype(np.float64)
        fd.mc_throttle = d.data["throttle"].astype(np.float64)

    def _rel(ts):
        return (ts - fd.t0_abs) / 1e6

    # --- vehicle_attitude ---
    d = _get_topic(ulog, "vehicle_attitude")
    if d is not None:
        fd.att_ts = _rel(d.data["timestamp"])
        r, p, y = _quat_to_euler_deg(
            d.data["q[0]"], d.data["q[1]"], d.data["q[2]"], d.data["q[3]"])
        fd.att_roll, fd.att_pitch, fd.att_yaw = r, p, y

    # --- vehicle_attitude_setpoint ---
    d = _get_topic(ulog, "vehicle_attitude_setpoint")
    if d is not None:
        fd.att_sp_ts = _rel(d.data["timestamp"])
        r, p, _ = _quat_to_euler_deg(
            d.data["q_d[0]"], d.data["q_d[1]"], d.data["q_d[2]"], d.data["q_d[3]"])
        fd.att_sp_roll, fd.att_sp_pitch = r, p

    # --- vehicle_angular_velocity ---
    d = _get_topic(ulog, "vehicle_angular_velocity")
    if d is not None:
        fd.angvel_ts = _rel(d.data["timestamp"])
        fd.angvel_roll = np.degrees(d.data["xyz[0]"].astype(np.float64))
        fd.angvel_pitch = np.degrees(d.data["xyz[1]"].astype(np.float64))
        fd.angvel_yaw = np.degrees(d.data["xyz[2]"].astype(np.float64))

    # --- vehicle_rates_setpoint ---
    d = _get_topic(ulog, "vehicle_rates_setpoint")
    if d is not None:
        fd.rates_sp_ts = _rel(d.data["timestamp"])
        fd.rates_sp_roll = np.degrees(d.data["roll"].astype(np.float64))

    # --- vehicle_torque_setpoint ---
    d = _get_topic(ulog, "vehicle_torque_setpoint")
    if d is not None:
        fd.torque_ts = _rel(d.data["timestamp"])
        fd.torque_roll = d.data["xyz[0]"].astype(np.float64)
        fd.torque_pitch = d.data["xyz[1]"].astype(np.float64)
        fd.torque_yaw = d.data["xyz[2]"].astype(np.float64)

    # --- vehicle_thrust_setpoint ---
    d = _get_topic(ulog, "vehicle_thrust_setpoint")
    if d is not None:
        fd.thrust_ts = _rel(d.data["timestamp"])
        fd.thrust_x = d.data["xyz[0]"].astype(np.float64)
        fd.thrust_y = d.data["xyz[1]"].astype(np.float64)
        fd.thrust_z = d.data["xyz[2]"].astype(np.float64)

    # --- actuator_motors ---
    d = _get_topic(ulog, "actuator_motors")
    if d is not None:
        fd.motor_ts = _rel(d.data["timestamp"])
        fd.motors = np.column_stack([
            d.data[f"control[{i}]"].astype(np.float64) for i in range(4)])

    # --- actuator_outputs ---
    d = _get_topic(ulog, "actuator_outputs")
    if d is not None:
        fd.output_ts = _rel(d.data["timestamp"])
        fd.outputs = np.column_stack([
            d.data[f"output[{i}]"].astype(np.float64) for i in range(4)])

    # --- sensor_combined ---
    d = _get_topic(ulog, "sensor_combined")
    if d is not None:
        fd.sc_ts = _rel(d.data["timestamp"])
        fd.sc_gyro = np.column_stack([
            d.data[f"gyro_rad[{i}]"].astype(np.float64) for i in range(3)])
        fd.sc_accel = np.column_stack([
            d.data[f"accelerometer_m_s2[{i}]"].astype(np.float64) for i in range(3)])

    # --- vehicle_acceleration ---
    d = _get_topic(ulog, "vehicle_acceleration")
    if d is not None:
        fd.vacc_ts = _rel(d.data["timestamp"])
        fd.vacc = np.column_stack([
            d.data[f"xyz[{i}]"].astype(np.float64) for i in range(3)])

    # --- rate_ctrl_status ---
    d = _get_topic(ulog, "rate_ctrl_status")
    if d is not None:
        fd.rcs_ts = _rel(d.data["timestamp"])
        fd.rcs_roll_integ = d.data["rollspeed_integ"].astype(np.float64)
        fd.rcs_pitch_integ = d.data["pitchspeed_integ"].astype(np.float64)
        fd.rcs_yaw_integ = d.data["yawspeed_integ"].astype(np.float64)

    # --- phase detection ---
    if len(fd.mc_ts) > 0:
        fd.phases = detect_phases(fd.mc_ts, fd.mc_pitch, fd.mc_throttle)

    return fd


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _phase_mask(ts: np.ndarray, phase: PhaseSpan, eval_only: bool = True) -> np.ndarray:
    """Return boolean mask for samples within a phase.

    If eval_only=True (default), uses t_eval_start to exclude the initial
    transient of hover phases.  Set eval_only=False for plot shading ranges.
    """
    start = phase.t_eval_start if eval_only else phase.t_start
    return (ts >= start) & (ts < phase.t_end)


def _add_phase_spans(ax, phases: list[PhaseSpan], alpha: float = 0.10):
    for p in phases:
        ax.axvspan(p.t_start, p.t_end, color=p.color, alpha=alpha, zorder=0)


def _motor_asymmetry(motors: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (asym_02, asym_13) normalised asymmetry per sample."""
    s02 = motors[:, 0] + motors[:, 2]
    s13 = motors[:, 1] + motors[:, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        a02 = np.where(s02 > 0.01, (motors[:, 0] - motors[:, 2]) / s02, 0.0)
        a13 = np.where(s13 > 0.01, (motors[:, 1] - motors[:, 3]) / s13, 0.0)
    return a02, a13


# ---------------------------------------------------------------------------
# Plot 1 – Timeline overview
# ---------------------------------------------------------------------------
def plot_timeline(fd: FlightData, out_dir: str):
    fig, axes = plt.subplots(6, 1, figsize=(12, 14), sharex=True)

    # 1) Pitch setpoint vs actual
    ax = axes[0]
    ax.plot(fd.mc_ts, fd.mc_pitch * 35, "k--", lw=1, label="Pitch cmd (scaled)")
    ax.plot(fd.att_ts, fd.att_pitch, lw=0.8, label="Pitch actual [deg]")
    ax.plot(fd.att_sp_ts, fd.att_sp_pitch, ":", lw=0.8, label="Pitch setpoint [deg]")
    _add_phase_spans(ax, fd.phases)
    ax.set_ylabel("Pitch [deg]")
    ax.legend(loc="upper right", ncol=3)
    ax.set_title(f"Flight Timeline – {fd.filename}")

    # 2) Roll setpoint vs actual
    ax = axes[1]
    ax.plot(fd.att_ts, fd.att_roll, lw=0.8, label="Roll actual [deg]")
    ax.plot(fd.att_sp_ts, fd.att_sp_roll, ":", lw=0.8, label="Roll setpoint [deg]")
    _add_phase_spans(ax, fd.phases)
    ax.set_ylabel("Roll [deg]")
    ax.legend(loc="upper right")

    # 3) Roll rate
    ax = axes[2]
    ax.plot(fd.angvel_ts, fd.angvel_roll, lw=0.5, alpha=0.7, label="Roll rate [deg/s]")
    if len(fd.rates_sp_ts):
        ax.plot(fd.rates_sp_ts, fd.rates_sp_roll, ":", lw=0.6,
                label="Roll rate SP [deg/s]")
    _add_phase_spans(ax, fd.phases)
    ax.set_ylabel("Roll rate [deg/s]")
    ax.legend(loc="upper right")

    # 4) Motor outputs
    ax = axes[3]
    for i in range(4):
        ax.plot(fd.motor_ts, fd.motors[:, i], lw=0.7,
                color=MOTOR_COLORS[i], label=f"M{i}")
    _add_phase_spans(ax, fd.phases)
    ax.set_ylabel("Motor cmd")
    ax.legend(loc="upper right", ncol=4)

    # 5) Thrust setpoint
    ax = axes[4]
    ax.plot(fd.thrust_ts, fd.thrust_x, lw=0.7, label="Thrust X")
    ax.plot(fd.thrust_ts, fd.thrust_y, lw=0.7, label="Thrust Y")
    ax.plot(fd.thrust_ts, fd.thrust_z, lw=0.7, label="Thrust Z")
    _add_phase_spans(ax, fd.phases)
    ax.set_ylabel("Thrust SP")
    ax.legend(loc="upper right", ncol=3)

    # 6) Torque setpoint
    ax = axes[5]
    ax.plot(fd.torque_ts, fd.torque_roll, lw=0.7, label="Torque Roll")
    ax.plot(fd.torque_ts, fd.torque_pitch, lw=0.7, label="Torque Pitch")
    ax.plot(fd.torque_ts, fd.torque_yaw, lw=0.7, label="Torque Yaw")
    _add_phase_spans(ax, fd.phases)
    ax.set_ylabel("Torque SP")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right", ncol=3)

    # Phase labels at top
    for p in fd.phases:
        axes[0].text((p.t_start + p.t_end) / 2, axes[0].get_ylim()[1],
                      p.name, ha="center", va="bottom", fontsize=7,
                      color=p.color, fontweight="bold")

    fig.align_ylabels(axes)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "plot1_timeline.png"))
    plt.close(fig)
    print("  Plot 1 saved.")


# ---------------------------------------------------------------------------
# Plot 2 – Roll disturbance detail
# ---------------------------------------------------------------------------
def plot_roll_detail(fd: FlightData, out_dir: str):
    h1 = fd.phases[0]
    h2 = fd.phases[3]

    fig = plt.figure(figsize=(14, 10))
    gs = gridspec.GridSpec(3, 2, width_ratios=[3, 1], hspace=0.35, wspace=0.25)

    # --- Row 0: Roll angle ---
    ax_ts = fig.add_subplot(gs[0, 0])
    ax_box = fig.add_subplot(gs[0, 1])

    m1 = _phase_mask(fd.att_ts, h1)
    m2 = _phase_mask(fd.att_ts, h2)
    t1 = fd.att_ts[m1] - h1.t_start
    t2 = fd.att_ts[m2] - h2.t_start

    ax_ts.plot(t1, fd.att_roll[m1], lw=0.8, color=h1.color, label="Hover1")
    ax_ts.plot(t2, fd.att_roll[m2], lw=0.8, color=h2.color, label="Hover2")
    ax_ts.set_ylabel("Roll angle [deg]")
    ax_ts.legend()
    ax_ts.set_title("Roll Angle – Hover1 vs Hover2")

    bp = ax_box.boxplot([fd.att_roll[m1], fd.att_roll[m2]],
                        tick_labels=["Hover1", "Hover2"], patch_artist=True)
    for patch, c in zip(bp["boxes"], [h1.color, h2.color]):
        patch.set_facecolor(c)
        patch.set_alpha(0.5)
    ax_box.set_ylabel("Roll angle [deg]")

    # --- Row 1: Roll rate ---
    ax_ts = fig.add_subplot(gs[1, 0])
    ax_box = fig.add_subplot(gs[1, 1])

    m1 = _phase_mask(fd.angvel_ts, h1)
    m2 = _phase_mask(fd.angvel_ts, h2)
    t1 = fd.angvel_ts[m1] - h1.t_start
    t2 = fd.angvel_ts[m2] - h2.t_start

    ax_ts.plot(t1, fd.angvel_roll[m1], lw=0.5, alpha=0.8, color=h1.color, label="Hover1")
    ax_ts.plot(t2, fd.angvel_roll[m2], lw=0.5, alpha=0.8, color=h2.color, label="Hover2")
    ax_ts.set_ylabel("Roll rate [deg/s]")
    ax_ts.legend()
    ax_ts.set_title("Roll Rate – Hover1 vs Hover2")

    bp = ax_box.boxplot([fd.angvel_roll[m1], fd.angvel_roll[m2]],
                        tick_labels=["Hover1", "Hover2"], patch_artist=True)
    for patch, c in zip(bp["boxes"], [h1.color, h2.color]):
        patch.set_facecolor(c)
        patch.set_alpha(0.5)
    ax_box.set_ylabel("Roll rate [deg/s]")

    # --- Row 2: Roll torque ---
    ax_ts = fig.add_subplot(gs[2, 0])
    ax_box = fig.add_subplot(gs[2, 1])

    m1 = _phase_mask(fd.torque_ts, h1)
    m2 = _phase_mask(fd.torque_ts, h2)
    t1 = fd.torque_ts[m1] - h1.t_start
    t2 = fd.torque_ts[m2] - h2.t_start

    ax_ts.plot(t1, fd.torque_roll[m1], lw=0.7, color=h1.color, label="Hover1")
    ax_ts.plot(t2, fd.torque_roll[m2], lw=0.7, color=h2.color, label="Hover2")
    ax_ts.set_ylabel("Roll torque SP")
    ax_ts.set_xlabel("Phase-relative time [s]")
    ax_ts.legend()
    ax_ts.set_title("Roll Torque Setpoint – Hover1 vs Hover2")

    bp = ax_box.boxplot([fd.torque_roll[m1], fd.torque_roll[m2]],
                        tick_labels=["Hover1", "Hover2"], patch_artist=True)
    for patch, c in zip(bp["boxes"], [h1.color, h2.color]):
        patch.set_facecolor(c)
        patch.set_alpha(0.5)
    ax_box.set_ylabel("Roll torque SP")

    fig.suptitle(f"Roll Disturbance Analysis – {fd.filename}", fontsize=11, y=1.01)
    fig.savefig(os.path.join(out_dir, "plot2_roll_detail.png"))
    plt.close(fig)
    print("  Plot 2 saved.")


# ---------------------------------------------------------------------------
# Plot 3 – Motor asymmetry
# ---------------------------------------------------------------------------
def plot_motors(fd: FlightData, out_dir: str):
    fig = plt.figure(figsize=(14, 12))
    gs = gridspec.GridSpec(3, 2, hspace=0.35, wspace=0.3)

    # (0,0): Motor time series
    ax = fig.add_subplot(gs[0, :])
    for i in range(4):
        ax.plot(fd.motor_ts, fd.motors[:, i], lw=0.6,
                color=MOTOR_COLORS[i], label=f"Motor {i}")
    _add_phase_spans(ax, fd.phases)
    ax.set_ylabel("Motor command")
    ax.set_xlabel("Time [s]")
    ax.legend(ncol=4, loc="upper right")
    ax.set_title(f"Motor Outputs – {fd.filename}")

    # (1,0): Asymmetry time series
    a02, a13 = _motor_asymmetry(fd.motors)
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(fd.motor_ts, a02, lw=0.7, label="Asym M0–M2")
    ax.plot(fd.motor_ts, a13, lw=0.7, label="Asym M1–M3")
    ax.axhline(0, color="gray", lw=0.5)
    _add_phase_spans(ax, fd.phases)
    ax.set_ylabel("Normalised asymmetry")
    ax.set_xlabel("Time [s]")
    ax.legend()
    ax.set_title("Motor Pair Asymmetry")

    # (1,1): Asymmetry box plot by phase
    ax = fig.add_subplot(gs[1, 1])
    data_a13 = []
    for p in fd.phases:
        mask = _phase_mask(fd.motor_ts, p)
        data_a13.append(a13[mask])
    bp = ax.boxplot(data_a13, tick_labels=[p.key for p in fd.phases], patch_artist=True)
    for patch, p in zip(bp["boxes"], fd.phases):
        patch.set_facecolor(p.color)
        patch.set_alpha(0.5)
    ax.set_ylabel("Asym M1–M3")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_title("M1–M3 Asymmetry by Phase")

    # (2,0-1): Radar chart – mean motor output per phase
    ax = fig.add_subplot(gs[2, 0], projection="polar")
    angles = np.linspace(0, 2 * np.pi, 4, endpoint=False).tolist()
    angles += angles[:1]
    for p in fd.phases:
        mask = _phase_mask(fd.motor_ts, p)
        means = [fd.motors[mask, i].mean() for i in range(4)]
        means += means[:1]
        ax.plot(angles, means, "o-", lw=1.2, label=p.key, color=p.color)
        ax.fill(angles, means, alpha=0.08, color=p.color)
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(["M0", "M1", "M2", "M3"])
    ax.legend(loc="upper right", bbox_to_anchor=(1.35, 1.1), fontsize=7)
    ax.set_title("Mean Motor Balance", pad=15)

    # (2,1): Std per motor by phase
    ax = fig.add_subplot(gs[2, 1])
    x_idx = np.arange(4)
    bar_w = 0.18
    for j, p in enumerate(fd.phases):
        mask = _phase_mask(fd.motor_ts, p)
        stds = [fd.motors[mask, i].std() for i in range(4)]
        ax.bar(x_idx + j * bar_w, stds, bar_w, label=p.key,
               color=p.color, alpha=0.7)
    ax.set_xticks(x_idx + 1.5 * bar_w)
    ax.set_xticklabels(["M0", "M1", "M2", "M3"])
    ax.set_ylabel("Std of motor cmd")
    ax.legend(fontsize=7)
    ax.set_title("Motor Output Variability")

    fig.savefig(os.path.join(out_dir, "plot3_motors.png"))
    plt.close(fig)
    print("  Plot 3 saved.")


# ---------------------------------------------------------------------------
# Plot 4 – PSD
# ---------------------------------------------------------------------------
def plot_psd(fd: FlightData, out_dir: str):
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))

    # Use sensor_combined gyro (high-rate) for PSD
    if len(fd.sc_ts) < 64:
        print("  Plot 4 skipped – insufficient sensor data.")
        plt.close(fig)
        return

    median_dt = np.median(np.diff(fd.sc_ts))
    fs = 1.0 / median_dt if median_dt > 0 else 200.0

    h1 = fd.phases[0]
    h2 = fd.phases[3]

    # (0,0): Roll gyro PSD – Hover1 vs Hover2
    ax = axes[0, 0]
    for phase, c, lbl in [(h1, h1.color, "Hover1"), (h2, h2.color, "Hover2")]:
        mask = _phase_mask(fd.sc_ts, phase)
        gyro_roll = fd.sc_gyro[mask, 0]
        if len(gyro_roll) < 32:
            continue
        nperseg = min(128, len(gyro_roll))
        f, pxx = signal.welch(gyro_roll, fs=fs, nperseg=nperseg)
        ax.semilogy(f, pxx, lw=0.9, color=c, label=lbl)
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("PSD [rad²/s²/Hz]")
    ax.set_title("Roll Gyro PSD")
    ax.legend()

    # (0,1): Pitch gyro PSD
    ax = axes[0, 1]
    for phase, c, lbl in [(h1, h1.color, "Hover1"), (h2, h2.color, "Hover2")]:
        mask = _phase_mask(fd.sc_ts, phase)
        gyro_pitch = fd.sc_gyro[mask, 1]
        if len(gyro_pitch) < 32:
            continue
        nperseg = min(128, len(gyro_pitch))
        f, pxx = signal.welch(gyro_pitch, fs=fs, nperseg=nperseg)
        ax.semilogy(f, pxx, lw=0.9, color=c, label=lbl)
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("PSD [rad²/s²/Hz]")
    ax.set_title("Pitch Gyro PSD")
    ax.legend()

    # (1,0): All 4 phases roll gyro PSD
    ax = axes[1, 0]
    for phase in fd.phases:
        mask = _phase_mask(fd.sc_ts, phase)
        gyro_roll = fd.sc_gyro[mask, 0]
        if len(gyro_roll) < 32:
            continue
        nperseg = min(128, len(gyro_roll))
        f, pxx = signal.welch(gyro_roll, fs=fs, nperseg=nperseg)
        ax.semilogy(f, pxx, lw=0.9, color=phase.color, label=phase.key)
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("PSD [rad²/s²/Hz]")
    ax.set_title("Roll Gyro PSD – All Phases")
    ax.legend(fontsize=7)

    # (1,1): Accel Y PSD (lateral → wall effect)
    ax = axes[1, 1]
    for phase, c, lbl in [(h1, h1.color, "Hover1"), (h2, h2.color, "Hover2")]:
        mask = _phase_mask(fd.sc_ts, phase)
        acc_y = fd.sc_accel[mask, 1]
        if len(acc_y) < 32:
            continue
        nperseg = min(128, len(acc_y))
        f, pxx = signal.welch(acc_y, fs=fs, nperseg=nperseg)
        ax.semilogy(f, pxx, lw=0.9, color=c, label=lbl)
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("PSD [m²/s⁴/Hz]")
    ax.set_title("Lateral Accel (Y) PSD")
    ax.legend()

    fig.suptitle(f"Frequency Analysis – {fd.filename}", fontsize=11, y=1.01)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "plot4_psd.png"))
    plt.close(fig)
    print("  Plot 4 saved.")


# ---------------------------------------------------------------------------
# Plot 5 – Multi-run statistics
# ---------------------------------------------------------------------------
def _collect_phase_stats(flights: list[FlightData]) -> pd.DataFrame:
    """Compute per-phase summary metrics across multiple flights."""
    rows = []
    for fd in flights:
        if len(fd.phases) < 4:
            continue
        for p in fd.phases:
            row = {"file": fd.filename, "phase": p.key}

            # Roll angle
            m = _phase_mask(fd.att_ts, p)
            roll = fd.att_roll[m]
            row["roll_mean"] = np.mean(roll) if len(roll) else np.nan
            row["roll_std"] = np.std(roll) if len(roll) else np.nan

            # Roll tracking error
            m_act = _phase_mask(fd.att_ts, p)
            m_sp = _phase_mask(fd.att_sp_ts, p)
            if m_act.sum() > 0 and m_sp.sum() > 0:
                sp_interp = np.interp(fd.att_ts[m_act], fd.att_sp_ts[m_sp],
                                      fd.att_sp_roll[m_sp])
                err = fd.att_roll[m_act] - sp_interp
                row["roll_err_rmse"] = np.sqrt(np.mean(err**2))
                row["roll_err_mae"] = np.mean(np.abs(err))
            else:
                row["roll_err_rmse"] = np.nan
                row["roll_err_mae"] = np.nan

            # Roll rate std
            m = _phase_mask(fd.angvel_ts, p)
            rv = fd.angvel_roll[m]
            row["roll_rate_std"] = np.std(rv) if len(rv) else np.nan

            # Roll torque
            m = _phase_mask(fd.torque_ts, p)
            tq = fd.torque_roll[m]
            row["torque_roll_mean"] = np.mean(tq) if len(tq) else np.nan
            row["torque_roll_std"] = np.std(tq) if len(tq) else np.nan
            row["torque_roll_rms"] = np.sqrt(np.mean(tq**2)) if len(tq) else np.nan

            # Motor asymmetry M1-M3
            m = _phase_mask(fd.motor_ts, p)
            if m.sum() > 0:
                _, a13 = _motor_asymmetry(fd.motors[m])
                row["motor_asym_mean"] = np.mean(a13)
                row["motor_asym_std"] = np.std(a13)
            else:
                row["motor_asym_mean"] = np.nan
                row["motor_asym_std"] = np.nan

            # Rate controller roll integral (mean over phase)
            m = _phase_mask(fd.rcs_ts, p)
            ri = fd.rcs_roll_integ[m]
            row["roll_integ_mean"] = np.mean(ri) if len(ri) else np.nan

            rows.append(row)

    return pd.DataFrame(rows)


BOX_METRICS = [
    ("roll_std", "Roll Angle Std [deg]"),
    ("roll_err_rmse", "Roll Tracking RMSE [deg]"),
    ("roll_rate_std", "Roll Rate Std [deg/s]"),
    ("torque_roll_std", "Roll Torque Std"),
    ("torque_roll_rms", "Roll Torque RMS"),
    ("motor_asym_mean", "Motor Asym M1−M3 (mean)"),
]

TEST_METRICS = [
    ("roll_std", "Roll Angle Std"),
    ("roll_err_rmse", "Roll Track RMSE"),
    ("roll_rate_std", "Roll Rate Std"),
    ("torque_roll_std", "Roll Torque Std"),
    ("torque_roll_rms", "Roll Torque RMS"),
    ("motor_asym_mean", "Motor Asym"),
    ("roll_integ_mean", "Roll Integ Mean"),
]


def _run_paired_tests(df: pd.DataFrame, group_label: str,
                      out_dir: str, suffix: str) -> list[dict]:
    """Wilcoxon signed-rank + Cohen's d for Hover1 vs Hover2."""
    print(f"\n  === Paired Tests: Hover1 vs Hover2 [{group_label}] ===")
    print(f"  {'Metric':<22s} {'Hover1 mean':>12s} {'Hover2 mean':>12s} "
          f"{'delta mean':>10s} {'Cohen d':>9s} {'p (Wilcoxon)':>13s}")
    print("  " + "-" * 82)

    test_rows = []
    for metric, label in TEST_METRICS:
        h1_vals = df.loc[df["phase"] == "hover1", ["file", metric]].dropna()
        h2_vals = df.loc[df["phase"] == "hover2", ["file", metric]].dropna()
        merged = h1_vals.merge(h2_vals, on="file", suffixes=("_h1", "_h2"))
        if len(merged) < 3:
            continue
        v1 = merged[f"{metric}_h1"].values
        v2 = merged[f"{metric}_h2"].values
        diff = v2 - v1
        mean_diff = np.mean(diff)
        pooled_std = np.sqrt((np.var(v1, ddof=1) + np.var(v2, ddof=1)) / 2)
        cohens_d = mean_diff / pooled_std if pooled_std > 1e-12 else np.nan

        try:
            stat_w, p_w = stats.wilcoxon(v1, v2, alternative="two-sided")
        except ValueError:
            p_w = np.nan

        print(f"  {label:<22s} {np.mean(v1):12.5f} {np.mean(v2):12.5f} "
              f"{mean_diff:10.5f} {cohens_d:9.3f} {p_w:13.4f}")
        test_rows.append({
            "group": group_label, "metric": label,
            "hover1_mean": np.mean(v1), "hover2_mean": np.mean(v2),
            "delta": mean_diff, "cohens_d": cohens_d, "p_wilcoxon": p_w,
        })

    if test_rows:
        test_df = pd.DataFrame(test_rows)
        csv_path = os.path.join(out_dir, f"statistical_tests_{suffix}.csv")
        test_df.to_csv(csv_path, index=False)
        print(f"  Saved to {os.path.basename(csv_path)}")
    return test_rows


def plot_statistics(flights: list[FlightData], out_dir: str,
                    group_label: str = "", suffix: str = ""):
    """Phase box-plots + paired tests for a single group of flights."""
    df = _collect_phase_stats(flights)
    if df.empty:
        print(f"  Plot 5 ({group_label}) skipped – no data.")
        return

    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    for idx, (metric, ylabel) in enumerate(BOX_METRICS):
        ax = axes.flat[idx]
        data_by_phase = []
        for pk in PHASE_KEYS:
            vals = df.loc[df["phase"] == pk, metric].dropna().values
            data_by_phase.append(vals)
        bp = ax.boxplot(data_by_phase, tick_labels=PHASE_KEYS, patch_artist=True)
        for patch, c in zip(bp["boxes"], PHASE_COLORS):
            patch.set_facecolor(c)
            patch.set_alpha(0.5)
        ax.set_ylabel(ylabel)
        ax.set_title(ylabel)

    title = f"Phase Statistics – {group_label} ({len(flights)} runs)"
    fig.suptitle(title, fontsize=12, y=1.01)
    fig.tight_layout()
    fname = f"plot5_statistics_{suffix}.png" if suffix else "plot5_statistics.png"
    fig.savefig(os.path.join(out_dir, fname))
    plt.close(fig)
    print(f"  {fname} saved.")

    _run_paired_tests(df, group_label, out_dir, suffix)


def plot_group_comparison(flights_a: list[FlightData],
                          flights_b: list[FlightData],
                          label_a: str, label_b: str,
                          out_dir: str):
    """Cross-group comparison: overlay Group A vs Group B statistics."""
    df_a = _collect_phase_stats(flights_a)
    df_b = _collect_phase_stats(flights_b)
    if df_a.empty or df_b.empty:
        print("  Cross-group comparison skipped – insufficient data.")
        return

    df_a["group"] = label_a
    df_b["group"] = label_b
    df_all = pd.concat([df_a, df_b], ignore_index=True)

    group_colors = {"hover1": "#4e79a7", "hover2": "#76b7b2"}
    hatches = {label_a: "", label_b: "//"}

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))

    for idx, (metric, ylabel) in enumerate(BOX_METRICS):
        ax = axes.flat[idx]
        positions = []
        data_list = []
        colors = []
        hatch_list = []
        labels = []
        pos = 0
        for pk in ["hover1", "hover2"]:
            for grp, lbl_grp in [(label_a, label_a), (label_b, label_b)]:
                vals = df_all.loc[(df_all["phase"] == pk) & (df_all["group"] == grp),
                                  metric].dropna().values
                data_list.append(vals)
                positions.append(pos)
                colors.append(group_colors[pk])
                hatch_list.append(hatches[grp])
                labels.append(f"{pk}\n{grp}")
                pos += 1
            pos += 0.5

        bp = ax.boxplot(data_list, positions=positions, widths=0.6,
                        patch_artist=True)
        for patch, c, h in zip(bp["boxes"], colors, hatch_list):
            patch.set_facecolor(c)
            patch.set_alpha(0.5)
            patch.set_hatch(h)
        ax.set_xticks(positions)
        ax.set_xticklabels(labels, fontsize=7)
        ax.set_ylabel(ylabel)
        ax.set_title(ylabel)

    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor="#4e79a7", alpha=0.5, label="Hover1"),
        Patch(facecolor="#76b7b2", alpha=0.5, label="Hover2"),
        Patch(facecolor="white", edgecolor="black", label=label_a),
        Patch(facecolor="white", edgecolor="black", hatch="//", label=label_b),
    ]
    fig.legend(handles=legend_elements, loc="upper right",
               bbox_to_anchor=(0.99, 0.99), fontsize=8)

    fig.suptitle(f"Group Comparison: {label_a} vs {label_b}", fontsize=12, y=1.01)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "plot7_group_comparison.png"))
    plt.close(fig)
    print("  plot7_group_comparison.png saved.")

    # Cross-group unpaired test: Hover2(A) vs Hover2(B) for each metric
    print(f"\n  === Cross-Group Test: Hover2 [{label_a}] vs Hover2 [{label_b}] ===")
    print(f"  {'Metric':<22s} {label_a+' mean':>14s} {label_b+' mean':>14s} "
          f"{'delta':>10s} {'Cohen d':>9s} {'p (MWU)':>10s}")
    print("  " + "-" * 85)

    cross_rows = []
    for metric, label in TEST_METRICS:
        va = df_a.loc[df_a["phase"] == "hover2", metric].dropna().values
        vb = df_b.loc[df_b["phase"] == "hover2", metric].dropna().values
        if len(va) < 2 or len(vb) < 2:
            continue
        mean_diff = np.mean(vb) - np.mean(va)
        pooled_std = np.sqrt((np.var(va, ddof=1) + np.var(vb, ddof=1)) / 2)
        cohens_d = mean_diff / pooled_std if pooled_std > 1e-12 else np.nan
        try:
            _, p_mwu = stats.mannwhitneyu(va, vb, alternative="two-sided")
        except ValueError:
            p_mwu = np.nan
        print(f"  {label:<22s} {np.mean(va):14.5f} {np.mean(vb):14.5f} "
              f"{mean_diff:10.5f} {cohens_d:9.3f} {p_mwu:10.4f}")
        cross_rows.append({
            "metric": label, f"{label_a}_hover2_mean": np.mean(va),
            f"{label_b}_hover2_mean": np.mean(vb),
            "delta": mean_diff, "cohens_d": cohens_d, "p_mannwhitney": p_mwu,
        })

    # Also compare Hover1(A) vs Hover1(B)
    print(f"\n  === Cross-Group Test: Hover1 [{label_a}] vs Hover1 [{label_b}] ===")
    print(f"  {'Metric':<22s} {label_a+' mean':>14s} {label_b+' mean':>14s} "
          f"{'delta':>10s} {'Cohen d':>9s} {'p (MWU)':>10s}")
    print("  " + "-" * 85)

    for metric, label in TEST_METRICS:
        va = df_a.loc[df_a["phase"] == "hover1", metric].dropna().values
        vb = df_b.loc[df_b["phase"] == "hover1", metric].dropna().values
        if len(va) < 2 or len(vb) < 2:
            continue
        mean_diff = np.mean(vb) - np.mean(va)
        pooled_std = np.sqrt((np.var(va, ddof=1) + np.var(vb, ddof=1)) / 2)
        cohens_d = mean_diff / pooled_std if pooled_std > 1e-12 else np.nan
        try:
            _, p_mwu = stats.mannwhitneyu(va, vb, alternative="two-sided")
        except ValueError:
            p_mwu = np.nan
        print(f"  {label:<22s} {np.mean(va):14.5f} {np.mean(vb):14.5f} "
              f"{mean_diff:10.5f} {cohens_d:9.3f} {p_mwu:10.4f}")
        cross_rows.append({
            "metric": label + " (H1)",
            f"{label_a}_hover1_mean": np.mean(va),
            f"{label_b}_hover1_mean": np.mean(vb),
            "delta": mean_diff, "cohens_d": cohens_d, "p_mannwhitney": p_mwu,
        })

    if cross_rows:
        cross_df = pd.DataFrame(cross_rows)
        cross_df.to_csv(os.path.join(out_dir, "cross_group_tests.csv"), index=False)
        print("  Saved to cross_group_tests.csv")

    # --- Difference-in-Differences (DiD) ---
    _plot_did(df_a, df_b, label_a, label_b, out_dir)


def _plot_did(df_a: pd.DataFrame, df_b: pd.DataFrame,
              label_a: str, label_b: str, out_dir: str):
    """Difference-in-Differences: wall effect magnitude per group.

    For each run, compute  delta_i = metric(Hover2) - metric(Hover1).
    This cancels trim bias.  Then compare delta across groups.
    """
    from matplotlib.patches import Patch

    df_a = df_a.copy()
    df_b = df_b.copy()
    df_a["group"] = label_a
    df_b["group"] = label_b

    did_metrics = [
        ("roll_std", "Roll Angle Std"),
        ("roll_rate_std", "Roll Rate Std"),
        ("torque_roll_std", "Roll Torque Std"),
        ("torque_roll_rms", "Roll Torque RMS"),
        ("motor_asym_mean", "Motor Asym M1-M3"),
        ("roll_integ_mean", "Roll Integ Mean"),
    ]

    print(f"\n  === Difference-in-Differences: Wall Effect (Hover2 − Hover1) ===")
    print(f"  {'Metric':<22s} {label_a+' DiD':>14s} {label_b+' DiD':>14s} "
          f"{'DiD diff':>10s} {'Cohen d':>9s} {'p (MWU)':>10s}")
    print("  " + "-" * 85)

    did_rows = []
    did_data_a = {}
    did_data_b = {}

    for metric, label in did_metrics:
        deltas = {}
        for df_grp, grp_label, store in [
            (df_a, label_a, did_data_a), (df_b, label_b, did_data_b)
        ]:
            h1 = df_grp.loc[df_grp["phase"] == "hover1", ["file", metric]].dropna()
            h2 = df_grp.loc[df_grp["phase"] == "hover2", ["file", metric]].dropna()
            merged = h1.merge(h2, on="file", suffixes=("_h1", "_h2"))
            d = merged[f"{metric}_h2"].values - merged[f"{metric}_h1"].values
            deltas[grp_label] = d
            store[metric] = d

        da = deltas[label_a]
        db = deltas[label_b]

        if len(da) < 2 or len(db) < 2:
            continue

        did_diff = np.mean(db) - np.mean(da)
        pooled_std = np.sqrt((np.var(da, ddof=1) + np.var(db, ddof=1)) / 2)
        cohens_d = did_diff / pooled_std if pooled_std > 1e-12 else np.nan
        try:
            _, p_mwu = stats.mannwhitneyu(da, db, alternative="two-sided")
        except ValueError:
            p_mwu = np.nan

        print(f"  {label:<22s} {np.mean(da):14.5f} {np.mean(db):14.5f} "
              f"{did_diff:10.5f} {cohens_d:9.3f} {p_mwu:10.4f}")
        did_rows.append({
            "metric": label,
            f"{label_a}_did_mean": np.mean(da),
            f"{label_b}_did_mean": np.mean(db),
            "did_diff": did_diff, "cohens_d": cohens_d, "p_mannwhitney": p_mwu,
        })

    if did_rows:
        did_df = pd.DataFrame(did_rows)
        did_df.to_csv(os.path.join(out_dir, "did_analysis.csv"), index=False)
        print("  Saved to did_analysis.csv")

    # --- DiD plot ---
    fig, axes = plt.subplots(2, 3, figsize=(16, 9))

    grp_a_color = "#e15759"
    grp_b_color = "#4e79a7"

    for idx, (metric, label) in enumerate(did_metrics):
        ax = axes.flat[idx]
        da = did_data_a.get(metric, np.array([]))
        db = did_data_b.get(metric, np.array([]))

        if len(da) == 0 and len(db) == 0:
            ax.set_visible(False)
            continue

        data = []
        colors_bp = []
        tick_labs = []
        if len(da) > 0:
            data.append(da)
            colors_bp.append(grp_a_color)
            tick_labs.append(label_a)
        if len(db) > 0:
            data.append(db)
            colors_bp.append(grp_b_color)
            tick_labs.append(label_b)

        bp = ax.boxplot(data, tick_labels=tick_labs, patch_artist=True, widths=0.5)
        for patch, c in zip(bp["boxes"], colors_bp):
            patch.set_facecolor(c)
            patch.set_alpha(0.5)

        # Overlay individual points
        for i, (vals, c) in enumerate(zip(data, colors_bp)):
            jitter = np.random.default_rng(42).normal(0, 0.04, len(vals))
            ax.scatter(np.full_like(vals, i + 1) + jitter, vals,
                       color=c, alpha=0.7, s=25, zorder=3, edgecolors="white", lw=0.5)

        ax.axhline(0, color="gray", ls="--", lw=0.8)
        ax.set_ylabel(f"Δ {label}")
        ax.set_title(f"Wall Effect: {label}\n(Hover2 − Hover1)")

    legend_elements = [
        Patch(facecolor=grp_a_color, alpha=0.5, label=label_a),
        Patch(facecolor=grp_b_color, alpha=0.5, label=label_b),
    ]
    fig.legend(handles=legend_elements, loc="upper right",
               bbox_to_anchor=(0.99, 0.99), fontsize=9)

    fig.suptitle("Difference-in-Differences: Wall Effect by Group\n"
                 "(positive = Hover2 > Hover1, trim-bias cancelled)",
                 fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "plot8_did_analysis.png"))
    plt.close(fig)
    print("  plot8_did_analysis.png saved.")


# ---------------------------------------------------------------------------
# Plot 6 – Control effort
# ---------------------------------------------------------------------------
def plot_control_effort(fd: FlightData, out_dir: str):
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # 1) Rate controller integral (roll)
    ax = axes[0]
    ax.plot(fd.rcs_ts, fd.rcs_roll_integ, lw=0.9, label="Roll integ")
    ax.plot(fd.rcs_ts, fd.rcs_pitch_integ, lw=0.9, label="Pitch integ")
    ax.plot(fd.rcs_ts, fd.rcs_yaw_integ, lw=0.9, label="Yaw integ")
    _add_phase_spans(ax, fd.phases)
    ax.set_ylabel("Rate ctrl integral")
    ax.legend(loc="upper right", ncol=3)
    ax.set_title(f"Control Effort – {fd.filename}")

    # 2) Torque RMS (rolling window)
    ax = axes[1]
    win = 10
    if len(fd.torque_roll) > win:
        rms_roll = pd.Series(fd.torque_roll**2).rolling(win, center=True).mean().apply(np.sqrt)
        rms_pitch = pd.Series(fd.torque_pitch**2).rolling(win, center=True).mean().apply(np.sqrt)
        ax.plot(fd.torque_ts, rms_roll, lw=0.8, label="Roll torque RMS")
        ax.plot(fd.torque_ts, rms_pitch, lw=0.8, label="Pitch torque RMS")
    _add_phase_spans(ax, fd.phases)
    ax.set_ylabel("Rolling RMS torque")
    ax.legend(loc="upper right")
    ax.set_title("Torque RMS (rolling window)")

    # 3) Estimated mechanical power ∝ Σ motor_cmd²
    ax = axes[2]
    if len(fd.motors) > 0:
        power_proxy = np.sum(fd.motors**2, axis=1)
        ax.plot(fd.motor_ts, power_proxy, lw=0.7, color="#333", label="Σ motor²")
        _add_phase_spans(ax, fd.phases)
    ax.set_ylabel("Power proxy (Σ cmd²)")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right")
    ax.set_title("Motor Power Proxy")

    fig.align_ylabels(axes)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "plot6_control_effort.png"))
    plt.close(fig)
    print("  Plot 6 saved.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Wall-effect analysis for ULog flight logs",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--log-dir",
                        default=os.path.join(os.path.dirname(__file__), "logs"),
                        help="Directory containing .ulg files")
    parser.add_argument("--out-dir",
                        default=os.path.join(os.path.dirname(__file__), "analysis_output"),
                        help="Output directory for plots and CSV")
    parser.add_argument("--single", default=None, metavar="FILE",
                        help="Analyse a single .ulg file instead of the whole directory")
    parser.add_argument("--representative", default=None, metavar="FILE",
                        help="Which log to use for single-run plots (default: last consistent)")
    parser.add_argument("--show", action="store_true",
                        help="Show plots interactively (default: save only)")
    args = parser.parse_args()

    plt.rcParams.update(_RC_PARAMS)
    os.makedirs(args.out_dir, exist_ok=True)

    # Discover log files
    if args.single:
        log_files = [args.single]
    else:
        log_files = sorted(
            [os.path.join(args.log_dir, f)
             for f in os.listdir(args.log_dir) if f.endswith(".ulg")])

    if not log_files:
        print("No .ulg files found.", file=sys.stderr)
        return 1

    print(f"Found {len(log_files)} log file(s).\n")

    # Load all flights
    flights: list[FlightData] = []
    for lf in log_files:
        print(f"Loading {os.path.basename(lf)} ...", end=" ")
        fd = load_flight(lf)
        if len(fd.phases) == 4:
            flights.append(fd)
            print(f"OK  ({len(fd.phases)} phases)")
        else:
            print(f"SKIP  (detected {len(fd.phases)} phases)")

    if not flights:
        print("No valid flights found.", file=sys.stderr)
        return 1

    # Split into groups
    group_a = [f for f in flights if f.filename.startswith(GROUP_A_PREFIXES)]
    group_b = [f for f in flights if f.filename.startswith(GROUP_B_PREFIXES)]
    print(f"\nGroup A (baseline?): {len(group_a)} runs  "
          f"{[f.filename for f in group_a]}")
    print(f"Group B (morphed?) : {len(group_b)} runs  "
          f"{[f.filename for f in group_b]}")

    # Pick representative run for single-run plots (one per group)
    if args.representative:
        rep_b = next((f for f in flights if args.representative in f.filename), flights[-1])
    else:
        rep_b = group_b[-1] if group_b else flights[-1]
    rep_a = group_a[-1] if group_a else None

    print(f"\nRepresentative run (Group B): {rep_b.filename}")
    if rep_a:
        print(f"Representative run (Group A): {rep_a.filename}")

    # --- Single-run plots (Group B representative) ---
    print("\nGenerating single-run plots (Group B)...")
    plot_timeline(rep_b, args.out_dir)
    plot_roll_detail(rep_b, args.out_dir)
    plot_motors(rep_b, args.out_dir)
    plot_psd(rep_b, args.out_dir)
    plot_control_effort(rep_b, args.out_dir)

    # --- Single-run plots (Group A representative) ---
    if rep_a:
        print("\nGenerating single-run plots (Group A)...")
        a_dir = os.path.join(args.out_dir, "group_a")
        os.makedirs(a_dir, exist_ok=True)
        plot_timeline(rep_a, a_dir)
        plot_roll_detail(rep_a, a_dir)
        plot_motors(rep_a, a_dir)
        plot_psd(rep_a, a_dir)
        plot_control_effort(rep_a, a_dir)

    # --- Multi-run statistics per group ---
    if len(group_b) >= 2:
        print(f"\n{'='*60}")
        print(f"Group B statistics ({len(group_b)} runs)...")
        print(f"{'='*60}")
        plot_statistics(group_b, args.out_dir,
                        group_label="Group B (morphed?)", suffix="group_b")

    if len(group_a) >= 2:
        print(f"\n{'='*60}")
        print(f"Group A statistics ({len(group_a)} runs)...")
        print(f"{'='*60}")
        plot_statistics(group_a, args.out_dir,
                        group_label="Group A (baseline?)", suffix="group_a")

    # --- Cross-group comparison ---
    if len(group_a) >= 2 and len(group_b) >= 2:
        print(f"\n{'='*60}")
        print("Cross-group comparison...")
        print(f"{'='*60}")
        plot_group_comparison(group_a, group_b,
                              "GroupA", "GroupB", args.out_dir)

    print(f"\nAll outputs saved to: {args.out_dir}")
    if args.show:
        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
