"""Jetson Orin (sm_87) shim for the PyPI jax-cuda12-plugin 0.6.2, loaded by bench/pyroki_env.sh.

jaxlib's hand-written CUDA kernels (LU pivots-to-permutation, batch-pointer setup for batched LU) ship SASS for
sm_50-sm_120 except sm_87 and fail on this board with "no kernel image is available". XLA-compiled code is unaffected
(XLA compiles for the device). These two ops are routed to JAX's own pure-XLA lowerings, which JAX already uses on
platforms without the custom kernels: LU by partial pivoting (_lu_python) and the pivot-to-permutation loop.
Same algorithm (partial pivoting), compiled by XLA instead of cuSOLVER getrf.
Batched triangular solves and Cholesky factorisations: XLA lowers them to batched cuBLAS/cuSOLVER calls after a
precompiled MakeBatchPointers kernel, which has the same problem, so batched
operands use plain-XLA Cholesky-Banachiewicz and substitution, vectorised over the batch (same algorithms). CPU lowerings are
untouched.
"""
from jax._src.interpreters import mlir
import jax.numpy as jnp
from jax import lax
from jax._src.lax import linalg as _l

mlir.register_lowering(_l.lu_p, mlir.lower_fun(_l._lu_python, multiple_results=True), platform="cuda")
mlir.register_lowering(_l.lu_pivots_to_permutation_p,
                       mlir.lower_fun(_l._generic_lu_pivots_to_permutation, multiple_results=False), platform="cuda")




def _chol_xla(a):
    """Cholesky-Banachiewicz, lower, vectorised over the batch; n sequential column steps."""
    n = a.shape[-1]; idx = jnp.arange(n)

    def body(j, L):
        lj = L[..., j, :]
        d = jnp.sqrt(a[..., j, j] - jnp.sum(lj * lj, -1))
        col = (a[..., :, j] - jnp.einsum("...ik,...k->...i", L, lj)) / d[..., None]
        col = jnp.where(idx > j, col, jnp.where(idx == j, d[..., None], 0.0))
        return L.at[..., :, j].set(col)
    return lax.fori_loop(0, n, body, jnp.zeros_like(a))


def _tri_xla(a, b, *, left_side, lower, transpose_a, conjugate_a, unit_diagonal):
    """Substitution, vectorised over the batch; n sequential row steps. Only the selected triangle of a is read."""
    n = a.shape[-1]
    a = jnp.tril(a) if lower else jnp.triu(a)
    if unit_diagonal:
        a = a * (1 - jnp.eye(n, dtype=a.dtype)) + jnp.eye(n, dtype=a.dtype)
    if transpose_a:
        a = jnp.swapaxes(a, -1, -2); lower = not lower
    if not left_side:  # X A = B  <=>  A^T X^T = B^T
        return jnp.swapaxes(_tri_xla(jnp.swapaxes(a, -1, -2), jnp.swapaxes(b, -1, -2), left_side=True, lower=not lower,
                                     transpose_a=False, conjugate_a=False, unit_diagonal=False), -1, -2)
    order = jnp.arange(n) if lower else jnp.arange(n)[::-1]

    def body(k, x):
        i = order[k]
        xi = (b[..., i, :] - jnp.einsum("...k,...km->...m", a[..., i, :], x)) / a[..., i, i][..., None]
        return x.at[..., i, :].set(xi)
    return lax.fori_loop(0, n, body, jnp.zeros_like(b))


def _batched_xla(prim, unbatched_rule, fn):
    """cuda lowering: unbatched operands keep the library call; batched ones use fn (plain XLA, compiled for sm_87)."""
    def rule(ctx, *xs, **kw):
        if len(ctx.avals_in[0].shape) <= 2:
            return unbatched_rule(ctx, *xs, **kw)
        return mlir.lower_fun(fn, multiple_results=False)(ctx, *xs, **kw)
    mlir.register_lowering(prim, rule, platform="cuda")


_batched_xla(_l.triangular_solve_p, _l._triangular_solve_lowering, _tri_xla)
_batched_xla(_l.cholesky_p, _l._cholesky_lowering, _chol_xla)
