#!/usr/bin/env bash
set -euo pipefail

# Usage examples:
#   bash setup_vtol_sim_env.sh
#   bash setup_vtol_sim_env.sh --name vtol_sim --python 3.12.12
#   bash setup_vtol_sim_env.sh --recreate

ENV_NAME="vtol_sim"
PYTHON_VERSION="3.12.12"
RECREATE="false"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --name)
            ENV_NAME="$2"
            shift 2
            ;;
        --python)
            PYTHON_VERSION="$2"
            shift 2
            ;;
        --recreate)
            RECREATE="true"
            shift
            ;;
        -h|--help)
            echo "Usage: bash setup_vtol_sim_env.sh [--name <env_name>] [--python <version>] [--recreate]"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Package groups by installer type.
CONDA_PACKAGES=(
    "python=${PYTHON_VERSION}"
    "numpy"
    "scipy"
    "matplotlib"
)

PIP_PACKAGES=(
    "pyvista>=0.46,<0.49"
    "imageio"
    "imageio-ffmpeg"
    "gymnasium>=1.2,<2"
    "stable-baselines3>=2.7,<3"
)

if ! command -v conda >/dev/null 2>&1; then
    echo "Error: conda command not found. Please install Miniconda/Anaconda first."
    exit 1
fi

CONDA_BASE="$(conda info --base)"
# shellcheck disable=SC1091
source "${CONDA_BASE}/etc/profile.d/conda.sh"

if conda env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
    if [[ "${RECREATE}" == "true" ]]; then
        echo "[INFO] Removing existing environment: ${ENV_NAME}"
        conda env remove -y -n "${ENV_NAME}"
        echo "[INFO] Creating environment: ${ENV_NAME}"
        conda create -y -n "${ENV_NAME}" "${CONDA_PACKAGES[@]}"
    else
        echo "[INFO] Environment exists, updating packages: ${ENV_NAME}"
        conda install -y -n "${ENV_NAME}" "${CONDA_PACKAGES[@]}"
    fi
else
    echo "[INFO] Creating environment: ${ENV_NAME}"
    conda create -y -n "${ENV_NAME}" "${CONDA_PACKAGES[@]}"
fi

conda activate "${ENV_NAME}"

if [[ ${#PIP_PACKAGES[@]} -gt 0 ]]; then
    echo "[INFO] Installing pip packages into ${ENV_NAME}"
    python -m pip install --upgrade pip
    python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
    python -m pip install "${PIP_PACKAGES[@]}"
fi

echo ""
echo "[DONE] Python environment is ready: ${ENV_NAME}"
echo "[INFO] Python version target: ${PYTHON_VERSION}"
echo ""
echo "Package type summary:"
echo "- conda:"
for pkg in "${CONDA_PACKAGES[@]}"; do
    echo "  - ${pkg}"
done

echo "- pip:"
if [[ ${#PIP_PACKAGES[@]} -eq 0 ]]; then
    echo "  - (none)"
else
    for pkg in "${PIP_PACKAGES[@]}"; do
        echo "  - ${pkg}"
    done
fi

echo ""
echo "To activate manually: conda activate ${ENV_NAME}"
