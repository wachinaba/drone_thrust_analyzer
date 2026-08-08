#!/usr/bin/env python3
"""Aggregate fold-level SVGP validation metrics into a reportable summary."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


METRICS = ("rmse", "mae", "bias", "r2", "mean_predictive_std", "coverage_95pct")


def main() -> int:
    parser = argparse.ArgumentParser(description="Aggregate condition-grouped SVGP cross-validation summaries.")
    parser.add_argument("--cv-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    summaries = sorted(args.cv_dir.glob("fold_*/validation/heldout_validation_summary.json"))
    if len(summaries) < 2:
        raise FileNotFoundError(f"expected at least two fold summaries beneath {args.cv_dir}")

    records = [json.loads(path.read_text(encoding="utf-8")) for path in summaries]
    report: dict[str, object] = {
        "protocol": {
            "validation": "condition-grouped cross-validation",
            "fold_count": len(records),
            "interpretation": (
                "Fold mean ± sample SD. This evaluates unseen condition groups, "
                "not an independently collected external dataset."
            ),
        }
    }
    for metric_set in ("raw_feature_metrics", "deployed_force_policy_metrics"):
        aggregate: dict[str, dict[str, float]] = {}
        for metric in METRICS:
            values = [
                float(record[metric_set][metric])
                for record in records
                if metric in record[metric_set]
            ]
            if values:
                aggregate[metric] = {
                    "mean": float(np.mean(values)),
                    "sample_sd": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
                }
        report[metric_set] = aggregate

    output = args.output or args.cv_dir / "grouped_cv_validation_summary.json"
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
