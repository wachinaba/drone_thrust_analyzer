#!/usr/bin/env python3
"""Create leakage-resistant condition-group folds for SVGP evaluation.

Rows with the same experimental condition are always assigned to the same
fold.  This prevents repeated measurements of a distance/pose/thrust setting
from appearing in both the fitting and evaluation datasets.
"""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import pandas as pd
from sklearn.model_selection import GroupKFold


FEATURE_COLUMNS = [
    "distance",
    "wall_spacing",
    "force_z",
    "alpha",
    "beta",
    "prop_spacing_x",
    "prop_spacing_y",
]
TARGET_COLUMN = "normalized_moment"
DEFAULT_GROUP_COLUMNS = [
    "distance",
    "wall_spacing",
    "target_thrust",
    "tilt_angle",
    "slant_angle",
    "fold_angle",
    "alpha",
    "beta",
    "prop_spacing_x",
    "prop_spacing_y",
    "height",
    "keyword",
]


def comma_columns(value: str) -> list[str]:
    columns = [column.strip() for column in value.split(",") if column.strip()]
    if not columns:
        raise argparse.ArgumentTypeError("at least one column is required")
    return columns


def group_labels(frame: pd.DataFrame, columns: list[str]) -> pd.Series:
    """Create stable labels without treating numerical condition values as ranks."""
    values = frame.loc[:, columns].copy()
    for column in columns:
        if pd.api.types.is_numeric_dtype(values[column]):
            values[column] = values[column].map(
                lambda value: "<MISSING>" if pd.isna(value) else format(float(value), ".12g")
            )
        else:
            values[column] = values[column].fillna("<MISSING>").astype(str)
    return values.agg("\x1f".join, axis=1)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Create condition-grouped train/test CSVs for leakage-resistant SVGP cross-validation."
    )
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--n-splits", type=int, default=5)
    parser.add_argument(
        "--group-columns",
        type=comma_columns,
        default=DEFAULT_GROUP_COLUMNS,
        help="Condition columns that must never be split between training and testing.",
    )
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    if args.n_splits < 2:
        raise ValueError("--n-splits must be at least 2")
    if not args.input_csv.is_file():
        raise FileNotFoundError(f"input CSV not found: {args.input_csv}")
    if args.output_dir.exists():
        if not args.overwrite:
            raise FileExistsError(
                f"output directory already exists: {args.output_dir}; use --overwrite to replace it"
            )
        shutil.rmtree(args.output_dir)

    frame = pd.read_csv(args.input_csv, low_memory=False)
    required = FEATURE_COLUMNS + [TARGET_COLUMN] + args.group_columns
    missing = sorted(set(required) - set(frame.columns))
    if missing:
        raise ValueError(f"input CSV is missing columns: {', '.join(missing)}")

    numeric_columns = FEATURE_COLUMNS + [TARGET_COLUMN]
    frame = frame.copy()
    for column in numeric_columns:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    rows_before = len(frame)
    # Feature and target values must be valid.  Some metadata fields, such as
    # ``keyword``, are absent for legitimate measurements; retain those rows
    # and encode the missing metadata value explicitly in the group label.
    frame = frame.dropna(subset=numeric_columns).reset_index(names="source_row")
    if frame.empty:
        raise ValueError("no valid rows remain after required-column filtering")

    groups = group_labels(frame, args.group_columns)
    if groups.nunique() < args.n_splits:
        raise ValueError(
            f"only {groups.nunique()} condition groups are available for {args.n_splits} folds"
        )

    splitter = GroupKFold(n_splits=args.n_splits)
    frame["cv_fold"] = -1
    for fold_index, (_, test_index) in enumerate(
        splitter.split(frame, frame[TARGET_COLUMN], groups=groups), start=1
    ):
        frame.loc[test_index, "cv_fold"] = fold_index
    if (frame["cv_fold"] < 1).any():
        raise RuntimeError("failed to assign every row to a fold")

    args.output_dir.mkdir(parents=True, exist_ok=False)
    assignment = frame[["source_row", "cv_fold"] + args.group_columns].copy()
    assignment.to_csv(args.output_dir / "fold_assignments.csv", index=False)

    fold_summary: list[dict[str, int]] = []
    for fold in range(1, args.n_splits + 1):
        fold_dir = args.output_dir / f"fold_{fold:02d}"
        fold_dir.mkdir()
        train = frame.loc[frame["cv_fold"] != fold].drop(columns=["cv_fold"])
        test = frame.loc[frame["cv_fold"] == fold].drop(columns=["cv_fold"])
        train_groups = set(group_labels(train, args.group_columns))
        test_groups = set(group_labels(test, args.group_columns))
        if train_groups & test_groups:
            raise RuntimeError(f"group leakage detected in fold {fold}")
        train.to_csv(fold_dir / "train.csv", index=False)
        test.to_csv(fold_dir / "test.csv", index=False)
        fold_summary.append(
            {
                "fold": fold,
                "train_rows": len(train),
                "test_rows": len(test),
                "train_groups": len(train_groups),
                "test_groups": len(test_groups),
            }
        )

    manifest = {
        "input_csv": str(args.input_csv),
        "rows_before_filtering": rows_before,
        "rows_after_filtering": len(frame),
        "dropped_rows": rows_before - len(frame),
        "n_splits": args.n_splits,
        "feature_columns": FEATURE_COLUMNS,
        "target_column": TARGET_COLUMN,
        "group_columns": args.group_columns,
        "group_count": int(groups.nunique()),
        "folds": fold_summary,
        "interpretation": (
            "Each fold holds out complete experimental condition groups. "
            "Do not report this as external validation; it is grouped cross-validation."
        ),
    }
    (args.output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
