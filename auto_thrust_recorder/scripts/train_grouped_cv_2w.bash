#!/usr/bin/env bash
# Train and externally evaluate one SVGP per condition-grouped fold.
#
# Usage:
#   bash train_grouped_cv_2w.bash /path/to/grouped_cv_5fold
#
# The fold directory must be produced by prepare_grouped_cv_datasets.py.
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 /path/to/grouped_cv_output" >&2
  exit 2
fi

DATASET_DIR="$(cd "$1" && pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRAINER="${SCRIPT_DIR}/gaussian_process_regression.py"
SIM_ROOT="$(cd "${SCRIPT_DIR}/../../../morphing_drone_sim" && pwd)"
VALIDATOR="${SIM_ROOT}/scripts/validate_gpr_sim_transfer.py"
PYTHON_BIN="${PYTHON_BIN:-python3}"

for FOLD_DIR in "${DATASET_DIR}"/fold_*; do
  [[ -d "${FOLD_DIR}" ]] || continue
  FOLD_NAME="$(basename "${FOLD_DIR}")"
  MODEL_PATH="${FOLD_DIR}/gpr_moment_ab_${FOLD_NAME}.pkl"
  VALIDATION_DIR="${FOLD_DIR}/validation"

  "${PYTHON_BIN}" "${TRAINER}" "${FOLD_DIR}/train.csv" \
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
    --save-model "${MODEL_PATH}" \
    --trust-model t \
    --output "${FOLD_DIR}/gpr_moment_ab_${FOLD_NAME}.png"

  "${PYTHON_BIN}" "${VALIDATOR}" \
    --holdout-csv "${FOLD_DIR}/test.csv" \
    --model-path "${MODEL_PATH}" \
    --output-dir "${VALIDATION_DIR}" \
    --group-columns distance,tilt_angle,slant_angle,fold_angle,target_thrust,wall_spacing \
    --holdout-description "condition-grouped cross-validation ${FOLD_NAME}; all matching condition rows excluded during fitting"
done
