"""Jetson Orin (sm_87) shim for the PyPI jax-cuda12-plugin 0.6.2, loaded by bench/pyroki_env.sh.

jaxlib's hand-written CUDA kernels (LU pivots-to-permutation, batch-pointer setup for batched LU) ship SASS for
sm_50-sm_120 except sm_87 and fail on this board with "no kernel image is available". XLA-compiled code is unaffected
(XLA compiles for the device). These two ops are routed to JAX's own pure-XLA lowerings, which JAX already uses on
platforms without the custom kernels: LU by partial pivoting (_lu_python) and the pivot-to-permutation loop.
Same algorithm (partial pivoting), compiled by XLA instead of cuSOLVER getrf.
Batched triangular solves: XLA lowers them to cuBLAS trsmBatched after a precompiled MakeBatchPointers kernel, which
has the same problem, so a batched solve runs as a loop (lax.map) of unbatched cuBLAS trsm calls. CPU lowerings are
untouched.
"""
from jax._src.interpreters import mlir
import math

from jax import lax
from jax._src.lax import linalg as _l

mlir.register_lowering(_l.lu_p, mlir.lower_fun(_l._lu_python, multiple_results=True), platform="cuda")
mlir.register_lowering(_l.lu_pivots_to_permutation_p,
                       mlir.lower_fun(_l._generic_lu_pivots_to_permutation, multiple_results=False), platform="cuda")

_tri = _l._triangular_solve_lowering


def _tri_unbatched(a, b, **kw):
    batch = a.shape[:-2]
    if not batch:
        return _l.triangular_solve_p.bind(a, b, **kw)
    n = math.prod(batch)
    out = lax.map(lambda ab: _l.triangular_solve_p.bind(ab[0], ab[1], **kw),
                  (a.reshape((n,) + a.shape[-2:]), b.reshape((n,) + b.shape[-2:])))
    return out.reshape(b.shape)


def _tri_rule(ctx, a, b, **kw):
    if len(ctx.avals_in[0].shape) <= 2:
        return _tri(ctx, a, b, **kw)
    return mlir.lower_fun(_tri_unbatched, multiple_results=False)(ctx, a, b, **kw)


mlir.register_lowering(_l.triangular_solve_p, _tri_rule, platform="cuda")
