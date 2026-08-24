#!/usr/bin/env bash
# Wrapper for the GPU-enabled darknet build.
#
# The CUDA toolkit here came from pip (nvidia-cuda-nvcc / -runtime / -cccl)
# inside realsense_env rather than a system install, because this account has
# no sudo. Those shared libraries are not on the default loader path, so every
# invocation needs LD_LIBRARY_PATH pointed at them.
#
# Usage: ./darknet.sh detector train obj.data unitree-go1-tiny.cfg ...
set -euo pipefail

VENV=/home/ssingh/Finroc/singh_files/aor_robotics_group/realsense_camera/realsense_env
SITE="$VENV/lib/python3.12/site-packages/nvidia"
DARKNET="$HOME/darknet/build-gpu/src-cli/darknet"

if [ ! -x "$DARKNET" ]; then
  echo "GPU darknet not found at $DARKNET" >&2
  echo "Rebuild it, or use the CPU build at ~/darknet/build/src-cli/darknet" >&2
  exit 1
fi

export LD_LIBRARY_PATH="$SITE/cu13/lib:$SITE/cudnn/lib:${LD_LIBRARY_PATH:-}"

exec "$DARKNET" "$@"
