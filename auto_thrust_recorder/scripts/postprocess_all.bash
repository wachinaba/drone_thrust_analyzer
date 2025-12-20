#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
postprocess_all.bash

実行したフォルダ配下のサブフォルダに対して、postprocess 相当の前処理(1)-(3)を並列実行し、
最後に親フォルダで(4)(5)を1回だけ実行します（出力衝突を防ぐため）。

Usage:
  ./postprocess_all.bash [-j N] [-r ROOT] [-l LOG_DIR]

Options:
  -j N    並列数（default: nproc）
  -r ROOT 探索ルート（default: .）
  -l LOG_DIR  ログ出力先ディレクトリ（指定時、各サブフォルダの処理ログをファイルに保存）

Env:
  PYTHON_BIN   python 実行コマンド（default: python3）

Notes:
  - 各サブフォルダ内に '*raw*.csv' が見つかったものだけ処理対象にします。
  - 並列実行のため、処理順は入れ替わりますが、[i/N] 形式で進捗を表示します。
  - 生成物:
      <subdir>/corrected/...
      <subdir>/concat.csv
      <ROOT>/concat_merged.csv
      <ROOT>/krr.png
EOF
}

JOBS="$(nproc)"
ROOT="."
LOG_DIR=""

while getopts ":j:r:l:h" opt; do
  case "$opt" in
    j) JOBS="$OPTARG" ;;
    r) ROOT="$OPTARG" ;;
    l) LOG_DIR="$OPTARG" ;;
    h) usage; exit 0 ;;
    \?) echo "Unknown option: -$OPTARG" >&2; usage; exit 2 ;;
    :) echo "Option -$OPTARG requires an argument" >&2; usage; exit 2 ;;
  esac
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

sanitize_name() {
  # path -> safe filename (no slashes/spaces)
  local s="$1"
  s="${s#/}"                # strip leading /
  s="${s//\//__}"            # / -> __
  s="${s// /_}"              # space -> _
  s="${s//[^A-Za-z0-9_.-]/_}" # others -> _
  echo "$s"
}

run_one_dir() {
  local dir="$1"
  local idx="${2:-}"
  local total="${3:-}"
  local log_path=""
  local prefix=""
  if [ -n "$idx" ] && [ -n "$total" ]; then
    prefix="[$idx/$total] "
  fi
  if [ -n "${LOG_DIR:-}" ]; then
    mkdir -p "$LOG_DIR"
    log_path="$LOG_DIR/$(sanitize_name "$dir").log"
    echo "${prefix}[postprocess] start: $dir -> $log_path"
  else
    echo "${prefix}[postprocess] start: $dir"
  fi

  if [ -n "$log_path" ]; then
    (
      set -euo pipefail
      cd "$dir"

      [ -d corrected ] || mkdir corrected

      "$PYTHON_BIN" "$SCRIPT_DIR/merge_front_back_bias.py" \
        -k raw -d . \
        --output-dir corrected/ \
        --dropna-mode none \
        --step-warmup 0.3 \
        --bias-scope per-step \
        --param-rename dir:direction

      "$PYTHON_BIN" "$SCRIPT_DIR/csv_concat_4.py" \
        -k biascorr -d corrected/ \
        --output concat.csv \
        --dropna-mode none \
        --default-column slant_angle=0 \
        --default-column-mode missing

      # morphing drone 由来の派生パラメータ列を concat.csv に追加（上書き出力）
      "$PYTHON_BIN" "$SCRIPT_DIR/add_morph_params_to_csv.py" \
        --input concat.csv \
        --output concat.csv
    ) >"$log_path" 2>&1
  else
    (
      set -euo pipefail
      cd "$dir"

      [ -d corrected ] || mkdir corrected

      "$PYTHON_BIN" "$SCRIPT_DIR/merge_front_back_bias.py" \
        -k raw -d . \
        --output-dir corrected/ \
        --dropna-mode none \
        --step-warmup 0.3 \
        --bias-scope per-step \
        --param-rename dir:direction

      "$PYTHON_BIN" "$SCRIPT_DIR/csv_concat_4.py" \
        -k biascorr -d corrected/ \
        --output concat.csv \
        --dropna-mode none \
        --default-column slant_angle=0 \
        --default-column-mode missing

      # morphing drone 由来の派生パラメータ列を concat.csv に追加（上書き出力）
      "$PYTHON_BIN" "$SCRIPT_DIR/add_morph_params_to_csv.py" \
        --input concat.csv \
        --output concat.csv
    )
  fi

  echo "${prefix}[postprocess] done : $dir"
}

export -f run_one_dir
export -f sanitize_name
export SCRIPT_DIR PYTHON_BIN LOG_DIR

ROOT_ABS="$(cd -- "$ROOT" && pwd)"

mapfile -d '' TARGET_DIRS < <(
  find "$ROOT_ABS" -mindepth 1 -maxdepth 1 -type d -print0
)

if [ "${#TARGET_DIRS[@]}" -eq 0 ]; then
  echo "No subdirectories found under: $ROOT_ABS" >&2
  exit 1
fi

# '*raw*.csv' がありそうなディレクトリだけ対象にする（無駄な失敗ログを減らす）
FILTERED_DIRS=()
for d in "${TARGET_DIRS[@]}"; do
  if find "$d" -maxdepth 1 -type f -name '*raw*.csv' -print -quit >/dev/null 2>&1; then
    FILTERED_DIRS+=("$d")
  fi
done

if [ "${#FILTERED_DIRS[@]}" -eq 0 ]; then
  echo "No candidate subdirectories containing '*raw*.csv' under: $ROOT_ABS" >&2
  exit 1
fi

TOTAL_DIRS="${#FILTERED_DIRS[@]}"
IDX=0
while [ "$IDX" -lt "$TOTAL_DIRS" ]; do
  i1=$((IDX + 1))
  printf '%s\0' "${i1}"$'\t'"${TOTAL_DIRS}"$'\t'"${FILTERED_DIRS[$IDX]}"
  IDX=$((IDX + 1))
done \
  | xargs -0 -n 1 -P "$JOBS" bash -c '
      IFS=$'\''\t'\'' read -r idx total dir <<< "$1"
      run_one_dir "$dir" "$idx" "$total"
    ' _

if [ -n "$LOG_DIR" ]; then
  mkdir -p "$LOG_DIR"
  echo "[postprocess] merging concat.csv into: $ROOT_ABS/concat_merged.csv -> $LOG_DIR/merge_concat.log"
else
  echo "[postprocess] merging concat.csv into: $ROOT_ABS/concat_merged.csv"
fi
mapfile -d '' CONCAT_FILES < <(find "$ROOT_ABS" -name 'concat.csv' -print0)
if [ "${#CONCAT_FILES[@]}" -eq 0 ]; then
  echo "No concat.csv found under: $ROOT_ABS" >&2
  exit 1
fi

if [ -n "$LOG_DIR" ]; then
  (
    "$PYTHON_BIN" "$SCRIPT_DIR/merge_csv.py" \
      --files "${CONCAT_FILES[@]}" \
      --output "$ROOT_ABS/concat_merged.csv"
  ) >"$LOG_DIR/merge_concat.log" 2>&1
else
  "$PYTHON_BIN" "$SCRIPT_DIR/merge_csv.py" \
    --files "${CONCAT_FILES[@]}" \
    --output "$ROOT_ABS/concat_merged.csv"
fi

if [ -n "$LOG_DIR" ]; then
  echo "[postprocess] training/plotting -> $ROOT_ABS/krr.png -> $LOG_DIR/krr.log"
  (
    "$PYTHON_BIN" "$SCRIPT_DIR/kernel_ridge_regression.py" \
      "$ROOT_ABS/concat_merged.csv" \
      --output "$ROOT_ABS/krr.png" \
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
  ) >"$LOG_DIR/krr.log" 2>&1
else
  echo "[postprocess] training/plotting -> $ROOT_ABS/krr.png"
  "$PYTHON_BIN" "$SCRIPT_DIR/kernel_ridge_regression.py" \
    "$ROOT_ABS/concat_merged.csv" \
    --output "$ROOT_ABS/krr.png" \
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
fi

echo "[postprocess] all done"


