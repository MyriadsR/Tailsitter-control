#!/usr/bin/env bash
set -euo pipefail

ENV_NAME="${ENV_NAME:-vtol_sim}"
PYTHON_BIN="${PYTHON_BIN:-python3.12}"
CONDA_PYTHON_VERSION="${CONDA_PYTHON_VERSION:-3.12}"
PACKAGE_TYPE="${1:-pip}"
REQUIREMENTS_FILE="requirements-vtol_sim.txt"

if [[ ! -f "$REQUIREMENTS_FILE" ]]; then
  echo "Error: ${REQUIREMENTS_FILE} not found in current directory."
  exit 1
fi

case "$PACKAGE_TYPE" in
  pip)
    "$PYTHON_BIN" -m venv "$ENV_NAME"
    # shellcheck disable=SC1090
    source "$ENV_NAME/bin/activate"
    python -m pip install --upgrade pip
    pip install -r "$REQUIREMENTS_FILE"
    echo "Done. Activate with: source ${ENV_NAME}/bin/activate"
    ;;
  conda)
    conda create -y -n "$ENV_NAME" "python=${CONDA_PYTHON_VERSION}"
    conda run -n "$ENV_NAME" pip install -r "$REQUIREMENTS_FILE"
    echo "Done. Activate with: conda activate ${ENV_NAME}"
    ;;
  *)
    echo "Unsupported package type: $PACKAGE_TYPE"
    echo "Usage: ./setup_vtol_sim_env.sh [pip|conda]"
    exit 1
    ;;
esac
