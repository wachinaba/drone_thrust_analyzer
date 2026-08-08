#!/usr/bin/env bash
# Train the final all-data SVGP after grouped-CV protocol is frozen.
#
# Usage:
#   PYTHON_BIN=python bash train_reference_svgp.bash /path/to/merged.csv /path/to/model.pkl
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 INPUT_MERGED_CSV OUTPUT_MODEL_PKL" >&2
  exit 2
fi

INPUT_CSV="$1"
OUTPUT_MODEL="$2"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

"${PYTHON_BIN}" "${SCRIPT_DIR}/gaussian_process_regression.py" "${INPUT_CSV}" \
  --features distance,wall_spacing,force_z,alpha,beta,prop_spacing_x,prop_spacing_y \
  --target normalized_moment \
  --backend gpytorch \
  --alpha 0.001 \
  --num-inducing 512 \
  --epochs 300 \
  --batch-size 1024 \
  --no-internal-test \
  --pairwise-heatmaps f \
  --plot-raw-fit f \
  --save-model "${OUTPUT_MODEL}" \
  --output "$(dirname "${OUTPUT_MODEL}")/gpr_moment_ab_reference.png"
