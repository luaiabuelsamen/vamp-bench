#!/bin/bash
# Runs a command in the v2.1 PyRoki venv on the Jetson GPU (or the CPU with JAX_PLATFORMS=cpu).
# - PYTHONPATH is replaced (the login shell's points into other projects' PyTorch trees) by bench/pyroki_site, whose
#   sitecustomize routes LU to JAX's pure-XLA lowering (the plugin's LU kernels have no sm_87 image).
# - cuDNN 9.8 from pip (nvidia-cudnn-cu12, venv only): jax-cuda12-plugin 0.6.2 requires >= 9.8; JetPack 6.2 ships 9.3.
# - XLA autotuning off: its redzone kernel (RepeatBufferKernel) has no sm_87 image in the PyPI plugin and aborts.
#   Autotuning only picks among GEMM/conv algorithms; results are unaffected, speed may be.
R="$(cd "$(dirname "$0")/.." && pwd)"
CUDNN="$R/.venv-pyroki/lib/python3.10/site-packages/nvidia/cudnn/lib"
cmd="$1"; shift
[ "$cmd" = python ] && cmd="$R/.venv-pyroki/bin/python"
exec env PYTHONPATH="$R/bench/pyroki_site" LD_LIBRARY_PATH="$CUDNN:/usr/local/cuda/lib64" \
  XLA_FLAGS="--xla_gpu_autotune_level=0 ${XLA_FLAGS:-}" XLA_PYTHON_CLIENT_PREALLOCATE=false "$cmd" "$@"
