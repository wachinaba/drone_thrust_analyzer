"""Generate supplementary figures for the Marp slide deck."""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
import matplotlib.patches as mpatches
from pathlib import Path

plt.rcParams.update({
    "font.family": ["Noto Sans JP", "Meiryo", "Yu Gothic", "sans-serif"],
    "font.size": 14,
    "axes.titlesize": 16,
    "figure.facecolor": "white",
})

OUT = Path(__file__).parent / "analysis_output" / "slides"
OUT.mkdir(parents=True, exist_ok=True)


def fig_phase_concept():
    """Phase segmentation concept diagram."""
    fig, ax = plt.subplots(figsize=(14, 4))

    phases = [
        ("Hover1\n(壁なし)", 0, 4, "#3182CE", "white"),
        ("Forward\n(壁に接近)", 4, 3.5, "#E53E3E", "white"),
        ("Brake\n(制動)", 7.5, 1, "#DD6B20", "white"),
        ("Hover2\n(壁近接)", 8.5, 4, "#38A169", "white"),
    ]

    t = np.linspace(0, 12.5, 500)
    pitch = np.zeros_like(t)
    pitch[(t >= 4) & (t < 7.5)] = -0.09
    pitch[(t >= 7.5) & (t < 8.5)] = 0.06

    for name, start, dur, color, tc in phases:
        rect = mpatches.FancyBboxPatch(
            (start, -0.16), dur, 0.32, boxstyle="round,pad=0.02",
            facecolor=color, alpha=0.15, edgecolor=color, linewidth=2
        )
        ax.add_patch(rect)
        ax.text(start + dur / 2, 0.21, name, ha="center", va="bottom",
                fontsize=13, fontweight="bold", color=color)

    ax.plot(t, pitch, color="#1a202c", linewidth=2.5, label="Pitch setpoint")

    trim_h1_start = 0
    trim_h1_end = 1.5
    trim_h2_start = 8.5
    trim_h2_end = 10.0
    ax.axvspan(trim_h1_start, trim_h1_end, color="#3182CE", alpha=0.25, hatch="//")
    ax.axvspan(trim_h2_start, trim_h2_end, color="#38A169", alpha=0.25, hatch="//")
    ax.text(0.75, -0.13, "除外\n(1.5s)", ha="center", va="top", fontsize=10, color="#555")
    ax.text(9.25, -0.13, "除外\n(1.5s)", ha="center", va="top", fontsize=10, color="#555")

    ax.set_xlabel("Time [s]", fontsize=14)
    ax.set_ylabel("Pitch setpoint", fontsize=14)
    ax.set_xlim(-0.3, 13)
    ax.set_ylim(-0.20, 0.38)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.legend(loc="upper right", fontsize=12)
    ax.set_title("飛行フェーズの分割と過渡除去", fontsize=16, fontweight="bold", pad=10)

    fig.tight_layout()
    fig.savefig(OUT / "phase_concept.png", dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> {OUT / 'phase_concept.png'}")


def fig_grouping():
    """Group A vs B parameter comparison."""
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))

    groups = ["Group A\n(baseline?)", "Group B\n(morphed?)"]
    colors = ["#E53E3E", "#3182CE"]

    # Roll cmd
    ax = axes[0]
    a_vals = [-0.10, -0.10, -0.10, -0.16]
    b_vals = [-0.20] * 11
    bp = ax.boxplot([a_vals, b_vals], tick_labels=groups, patch_artist=True,
                    widths=0.5)
    for patch, c in zip(bp["boxes"], colors):
        patch.set_facecolor(c)
        patch.set_alpha(0.3)
        patch.set_edgecolor(c)
    for i, vals in enumerate([a_vals, b_vals]):
        ax.scatter([i + 1] * len(vals), vals, color=colors[i], s=40,
                   zorder=3, alpha=0.7, edgecolors="white", linewidths=0.5)
    ax.set_ylabel("Roll command")
    ax.set_title("Roll trim", fontweight="bold")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    # n
    ax = axes[1]
    bars = ax.bar(groups, [4, 11], color=colors, alpha=0.5, edgecolor=colors,
                  linewidth=2, width=0.5)
    for bar, n in zip(bars, [4, 11]):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.3,
                f"n = {n}", ha="center", fontsize=14, fontweight="bold")
    ax.set_ylabel("Number of runs")
    ax.set_title("サンプル数", fontweight="bold")
    ax.set_ylim(0, 14)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    # Fwd pitch
    ax = axes[2]
    a_pitch = [-0.06, -0.06, -0.07, -0.07]
    b_pitch = [-0.09] * 11
    bp = ax.boxplot([a_pitch, b_pitch], tick_labels=groups, patch_artist=True,
                    widths=0.5)
    for patch, c in zip(bp["boxes"], colors):
        patch.set_facecolor(c)
        patch.set_alpha(0.3)
        patch.set_edgecolor(c)
    for i, vals in enumerate([a_pitch, b_pitch]):
        ax.scatter([i + 1] * len(vals), vals, color=colors[i], s=40,
                   zorder=3, alpha=0.7, edgecolors="white", linewidths=0.5)
    ax.set_ylabel("Forward pitch cmd")
    ax.set_title("前進 Pitch 指令", fontweight="bold")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.suptitle("2グループの制御パラメータ比較", fontsize=16, fontweight="bold", y=1.02)
    fig.tight_layout()
    fig.savefig(OUT / "grouping.png", dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> {OUT / 'grouping.png'}")


def fig_key_results_summary():
    """Bar chart summarizing key wall-effect metrics for both groups."""
    fig, axes = plt.subplots(1, 3, figsize=(14, 5))

    colors_h = {"Hover1": "#63B3ED", "Hover2": "#FC8181"}

    # --- Roll Rate Std ---
    ax = axes[0]
    x = np.arange(2)
    w = 0.35
    h1 = [4.756, 1.287]
    h2 = [10.232, 3.684]
    ax.bar(x - w/2, h1, w, color=colors_h["Hover1"], label="Hover1", edgecolor="white")
    ax.bar(x + w/2, h2, w, color=colors_h["Hover2"], label="Hover2", edgecolor="white")
    ax.set_xticks(x)
    ax.set_xticklabels(["Group A", "Group B"])
    ax.set_ylabel("deg/s")
    ax.set_title("Roll Rate Std", fontweight="bold")
    ax.legend(fontsize=10)
    for i, (v1, v2) in enumerate(zip(h1, h2)):
        ratio = v2 / v1
        ax.text(i + w/2, v2 + 0.2, f"×{ratio:.1f}", ha="center", fontsize=11,
                fontweight="bold", color="#C53030")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    # --- Roll Torque Std ---
    ax = axes[1]
    h1 = [0.052, 0.010]
    h2 = [0.086, 0.029]
    ax.bar(x - w/2, h1, w, color=colors_h["Hover1"], edgecolor="white")
    ax.bar(x + w/2, h2, w, color=colors_h["Hover2"], edgecolor="white")
    ax.set_xticks(x)
    ax.set_xticklabels(["Group A", "Group B"])
    ax.set_title("Roll Torque Std", fontweight="bold")
    for i, (v1, v2) in enumerate(zip(h1, h2)):
        ratio = v2 / v1
        ax.text(i + w/2, v2 + 0.001, f"×{ratio:.1f}", ha="center", fontsize=11,
                fontweight="bold", color="#C53030")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    # --- Motor Asym DiD ---
    ax = axes[2]
    did_a = -0.022
    did_b = 0.008
    bars = ax.bar(["Group A", "Group B"], [did_a, did_b],
                  color=["#E53E3E", "#3182CE"], alpha=0.6,
                  edgecolor=["#E53E3E", "#3182CE"], linewidth=2, width=0.5)
    ax.axhline(0, color="#888", linewidth=1, linestyle="--")
    ax.set_title("Motor Asym Δ (DiD)", fontweight="bold")
    ax.set_ylabel("Hover2 − Hover1")
    ax.text(0, did_a - 0.003, f"{did_a:+.003f}", ha="center", fontsize=12, fontweight="bold")
    ax.text(1, did_b + 0.002, f"{did_b:+.003f}", ha="center", fontsize=12, fontweight="bold")

    ax.annotate("方向反転\nd=4.21, p=0.001",
                xy=(0.5, 0), xytext=(0.5, 0.025),
                ha="center", fontsize=11, fontweight="bold", color="#C53030",
                arrowprops=dict(arrowstyle="->", color="#C53030", lw=1.5))
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.suptitle("壁効果の主要指標: Hover1 vs Hover2", fontsize=16, fontweight="bold", y=1.02)
    fig.tight_layout()
    fig.savefig(OUT / "key_results.png", dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> {OUT / 'key_results.png'}")


def fig_trimming_effect():
    """Show the effect of hover trimming on Group B p-values."""
    fig, ax = plt.subplots(figsize=(10, 5))

    metrics = ["Roll Rate\nStd", "Roll Torque\nStd", "Roll Torque\nRMS",
               "Motor Asym\nM1−M3", "Roll Integ\nMean"]
    p_before = [0.102, 0.148, 0.365, 0.001, 0.465]
    p_after  = [0.003, 0.002, 0.007, 0.007, 0.042]

    x = np.arange(len(metrics))
    w = 0.35

    bars1 = ax.bar(x - w/2, p_before, w, color="#CBD5E0", edgecolor="#A0AEC0",
                   linewidth=1.5, label="トリミング前")
    bars2 = ax.bar(x + w/2, p_after, w, color="#3182CE", edgecolor="#2C5282",
                   linewidth=1.5, label="トリミング後 (1.5s除去)")

    ax.axhline(0.05, color="#C53030", linewidth=2, linestyle="--", label="α = 0.05")

    for bar, p in zip(bars1, p_before):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                f"{p:.3f}", ha="center", fontsize=10, color="#666")
    for bar, p in zip(bars2, p_after):
        c = "#C53030" if p < 0.05 else "#666"
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                f"{p:.3f}", ha="center", fontsize=10, fontweight="bold", color=c)

    ax.set_xticks(x)
    ax.set_xticklabels(metrics, fontsize=12)
    ax.set_ylabel("p-value (Wilcoxon)", fontsize=13)
    ax.set_ylim(0, 0.55)
    ax.legend(fontsize=12, loc="upper right")
    ax.set_title("過渡除去の効果: Group B の有意性の改善", fontsize=16, fontweight="bold")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.tight_layout()
    fig.savefig(OUT / "trimming_effect.png", dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> {OUT / 'trimming_effect.png'}")


def fig_did_all_metrics():
    """DiD bar chart for ALL 6 metrics with significance annotations."""
    metrics = [
        "Roll Angle\nStd",
        "Roll Rate\nStd",
        "Roll Torque\nStd",
        "Roll Torque\nRMS",
        "Motor Asym\nM1−M3",
        "Roll Integ\nMean",
    ]
    did_a = [+0.026, +5.476, +0.034, +0.034, -0.022, -0.004]
    did_b = [+0.008, +2.398, +0.019, +0.018, +0.008, +0.002]
    cohens = [-0.32, -0.78, -0.56, -0.61, +4.21, +2.37]
    pvals  = [1.000, 0.280, 0.343, 0.343, 0.001, 0.001]

    fig, axes = plt.subplots(2, 3, figsize=(15, 7.5))
    axes = axes.flatten()

    colors = {"A": "#E53E3E", "B": "#3182CE"}

    for i, (m, da, db, d, p) in enumerate(zip(metrics, did_a, did_b, cohens, pvals)):
        ax = axes[i]
        bars = ax.bar(
            ["Group A", "Group B"], [da, db],
            color=[colors["A"], colors["B"]], alpha=0.55,
            edgecolor=[colors["A"], colors["B"]], linewidth=2, width=0.55,
        )
        ax.axhline(0, color="#888", linewidth=1, linestyle="--")
        ax.set_title(m.replace("\n", " "), fontsize=13, fontweight="bold")
        ax.set_ylabel("Δ (Hover2 − Hover1)", fontsize=10)

        for bar, v in zip(bars, [da, db]):
            offset = max(abs(da), abs(db)) * 0.10
            y = v + offset if v >= 0 else v - offset
            ax.text(bar.get_x() + bar.get_width() / 2, y,
                    f"{v:+.3f}", ha="center", va="bottom" if v >= 0 else "top",
                    fontsize=10, fontweight="bold")

        sig = "***" if p < 0.001 else "**" if p < 0.01 else "*" if p < 0.05 else "n.s."
        sig_color = "#C53030" if p < 0.05 else "#888"
        ax.text(0.5, 0.97, f"d = {d:+.2f},  p = {p:.3f}  {sig}",
                transform=ax.transAxes, ha="center", va="top",
                fontsize=11, fontweight="bold", color=sig_color,
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec=sig_color, alpha=0.8))

        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    fig.suptitle("Difference-in-Differences: 全指標の壁効果 Δ（Hover2 − Hover1）",
                 fontsize=16, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(OUT / "did_all_metrics.png", dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> {OUT / 'did_all_metrics.png'}")


def fig_hover_comparison_all():
    """Hover1 vs Hover2 grouped bar for all 6 metrics, both groups."""
    metrics = [
        "Roll Angle\nStd [deg]",
        "Roll Rate\nStd [deg/s]",
        "Roll Torque\nStd",
        "Roll Torque\nRMS",
        "Motor Asym\nM1−M3",
        "Roll Integ\nMean",
    ]
    # Group A: [Hover1, Hover2], Group B: [Hover1, Hover2]
    data = {
        "A_h1": [0.090, 4.756, 0.052, 0.053, +0.000, -0.002],
        "A_h2": [0.116, 10.232, 0.086, 0.086, -0.021, -0.006],
        "B_h1": [0.126, 1.287, 0.010, 0.013, -0.003, -0.005],
        "B_h2": [0.134, 3.684, 0.029, 0.031, +0.005, -0.003],
    }
    p_a = [0.250, 0.125, 0.125, 0.125, 0.125, 0.125]
    p_b = [0.520, 0.003, 0.002, 0.007, 0.007, 0.042]

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    axes = axes.flatten()

    c = {"A_h1": "#F6AD55", "A_h2": "#E53E3E", "B_h1": "#63B3ED", "B_h2": "#2B6CB0"}

    for i, m in enumerate(metrics):
        ax = axes[i]
        x = np.arange(2)
        w = 0.35

        vals_h1 = [data["A_h1"][i], data["B_h1"][i]]
        vals_h2 = [data["A_h2"][i], data["B_h2"][i]]

        ax.bar(x - w / 2, vals_h1, w, color=[c["A_h1"], c["B_h1"]],
               edgecolor="white", label="Hover1" if i == 0 else "")
        ax.bar(x + w / 2, vals_h2, w, color=[c["A_h2"], c["B_h2"]],
               edgecolor="white", label="Hover2" if i == 0 else "")

        ax.set_xticks(x)
        ax.set_xticklabels(["Group A", "Group B"], fontsize=12)
        ax.set_title(m.replace("\n", " "), fontsize=13, fontweight="bold")

        # Significance stars
        for j, (pa, pb) in enumerate([(p_a[i], p_b[i])]):
            pass
        for j, pv in enumerate([p_a[i], p_b[i]]):
            sig = "***" if pv < 0.001 else "**" if pv < 0.01 else "*" if pv < 0.05 else ""
            if sig:
                ymax = max(abs(vals_h1[j]), abs(vals_h2[j]))
                ax.text(j, ymax * 1.08, sig, ha="center", fontsize=14,
                        fontweight="bold", color="#C53030")

        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    # Legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor=c["A_h1"], label="Group A Hover1"),
        Patch(facecolor=c["A_h2"], label="Group A Hover2"),
        Patch(facecolor=c["B_h1"], label="Group B Hover1"),
        Patch(facecolor=c["B_h2"], label="Group B Hover2"),
    ]
    fig.legend(handles=legend_elements, loc="lower center", ncol=4, fontsize=12,
               bbox_to_anchor=(0.5, -0.02))

    fig.suptitle("全指標: Hover1 vs Hover2 × Group A / B",
                 fontsize=17, fontweight="bold", y=1.01)
    fig.tight_layout()
    fig.savefig(OUT / "hover_all_metrics.png", dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> {OUT / 'hover_all_metrics.png'}")


if __name__ == "__main__":
    print("Generating slide figures...")
    fig_phase_concept()
    fig_grouping()
    fig_key_results_summary()
    fig_trimming_effect()
    fig_did_all_metrics()
    fig_hover_comparison_all()
    print("Done.")
