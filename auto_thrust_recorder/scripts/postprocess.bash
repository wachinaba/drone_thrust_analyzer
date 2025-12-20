#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

[ -d corrected ] || mkdir corrected

"$PYTHON_BIN" "$SCRIPT_DIR/merge_front_back_bias.py" \
  -k raw -d . \
  --output-dir corrected/ \
  --dropna-mode none \
  --step-warmup 0.2 \
  --bias-scope per-step \
  --param-rename dir:direction

# NOTE:
# - '--default-column' は csv_concat_4.py のオプション（merge_csv.py には存在しない）
# - slant_angle が欠損してるデータでも下流（回帰/可視化）が動くようにここで補完する
"$PYTHON_BIN" "$SCRIPT_DIR/csv_concat_4.py" \
  -k biascorr -d corrected/ \
  --output concat.csv \
  --dropna-mode none \
  --default-column slant_angle=0 \
  --default-column-mode missing

# morphing drone 由来の派生パラメータ列 + 計算列を concat.csv に追加（上書き出力）
"$PYTHON_BIN" "$SCRIPT_DIR/add_calculated_columns_to_csv.py" \
  --input concat.csv \
  --output concat.csv

# 親ディレクトリ配下にある concat.csv をまとめて1つにマージ（出力は親に作る）
"$PYTHON_BIN" "$SCRIPT_DIR/merge_csv.py" \
  --files $(find ../ -name 'concat.csv' | xargs) \
  --output ../concat_merged.csv

"$PYTHON_BIN" "$SCRIPT_DIR/kernel_ridge_regression.py" \
  ../concat_merged.csv \
  --output ../krr.png \
  --target normalized_moment \
  --target-expr '`torque_x` / `target_thrust` / `prop_spacing` * 100' \
  --do-grid-search \
  --pairwise-heatmaps \
  --std-heatmaps \
  --plot-raw-fit \
  --curve-x distance \
  --row-group-by wall_spacing \
  --col-group-by tilt_angle,fold_angle,slant_angle \
  --normalize \
  --features distance,tilt_angle,fold_angle,slant_angle,force_z,wall_spacing
