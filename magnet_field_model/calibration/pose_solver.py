"""Per-frame pose solving, batched across all frames at once.

Every frame's 6-DOF pose is independent of every other frame's, so instead of
looping scipy.optimize.least_squares over frames (the old approach: one
Python-level nonlinear solve per frame, warm-started in a chain, and
inherently sequential), this runs one Levenberg-Marquardt iteration for
*all* frames simultaneously - the per-frame 6x6 normal equations solve
batches cleanly through numpy.

That is the same independence the Schur complement exploits (see
bundle_params.covariance_shared), and it is only affordable because
bundle_geometry supplies an analytic pose Jacobian.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .bundle_geometry import APPROX_REST_T_MM, BundleGeometry, N_POSE_PARAMS


@dataclass(frozen=True)
class PoseSolveResult:
    poses: NDArray[np.float64]      # (n_frames, 6) [t, rotvec]
    residual_mT: NDArray[np.float64]  # (n_frames, 9) prediction - measurement
    converged: NDArray[np.bool_]    # (n_frames,)
    iterations: int


def solve_poses(
    geometry: BundleGeometry,
    measured: NDArray[np.float64],
    *,
    sigma: NDArray[np.float64] | None = None,
    initial: NDArray[np.float64] | None = None,
    max_iter: int = 60,
    tol: float = 1e-12,
) -> PoseSolveResult:
    """Fit one pose per frame against a fixed geometry.

    measured: (n_frames, 9). sigma: (n_frames, 9) or None for unweighted.
    initial: (n_frames, 6) starting poses, or None to start every frame from
    the nominal rest pose - fine here because the knob is mechanically
    constrained to stay within about a millimetre and 15 degrees of rest.
    """
    measured = np.asarray(measured, dtype=float)
    n = len(measured)
    w = np.ones_like(measured) if sigma is None else 1.0 / sigma

    if initial is None:
        poses = np.zeros((n, N_POSE_PARAMS))
        poses[:, :3] = APPROX_REST_T_MM
    else:
        poses = np.array(initial, dtype=float)

    lam = np.full(n, 1e-3)
    eye = np.eye(N_POSE_PARAMS)

    def cost_of(p: NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        r = (geometry.predict(p[:, :3], p[:, 3:]) - measured) * w
        return 0.5 * np.sum(r**2, axis=1), r

    cost, _ = cost_of(poses)
    iterations = 0

    for iterations in range(1, max_iter + 1):
        pred, j_pose, _ = geometry.predict_and_jacobians(poses[:, :3], poses[:, 3:])
        r = (pred - measured) * w
        j = j_pose * w[:, :, None]

        h = np.einsum("nka,nkb->nab", j, j)
        g = np.einsum("nka,nk->na", j, r)
        # Marquardt scaling: damp proportionally to each parameter's own
        # curvature, so translation (mm) and rotation (rad) are damped alike.
        diag = np.maximum(np.einsum("naa->na", h), 1e-12)
        h_damped = h + lam[:, None, None] * diag[:, :, None] * eye

        try:
            step = -np.linalg.solve(h_damped, g[:, :, None])[:, :, 0]
        except np.linalg.LinAlgError:
            break

        trial = poses + step
        trial_cost, _ = cost_of(trial)

        improved = trial_cost < cost
        poses = np.where(improved[:, None], trial, poses)
        gain = cost - trial_cost
        cost = np.where(improved, trial_cost, cost)
        lam = np.where(improved, np.maximum(lam / 3.0, 1e-9), np.minimum(lam * 3.0, 1e9))

        if np.all(~improved | (gain < tol * np.maximum(cost, 1.0))):
            break

    residual = geometry.predict(poses[:, :3], poses[:, 3:]) - measured
    converged = lam < 1e3
    return PoseSolveResult(poses=poses, residual_mT=residual, converged=converged,
                           iterations=iterations)
