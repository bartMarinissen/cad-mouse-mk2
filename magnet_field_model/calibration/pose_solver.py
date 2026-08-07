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

from .bundle_geometry import APPROX_REST_T_MM, N_POSE_PARAMS, BundleGeometry


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
    retry: bool = True,
) -> PoseSolveResult:
    """Fit one pose per frame against a fixed geometry.

    measured: (n_frames, 9). sigma: (n_frames, 9) or None for unweighted.
    initial: (n_frames, 6) starting poses, or None to start every frame from
    the nominal rest pose - fine here because the knob is mechanically
    constrained to stay within about a millimetre and 15 degrees of rest.

    With `retry`, any frame whose residual ends up far worse than its peers is
    solved again from the median of the frames that did work. A cold start can
    occasionally drop a single frame into a nonsense local minimum (a pose
    tens of degrees away that still stalls the LM), and since `converged`
    gates frame selection downstream, letting one through would quietly poison
    the fit.
    """
    measured = np.asarray(measured, dtype=float)
    n = len(measured)
    w = np.ones_like(measured) if sigma is None else 1.0 / sigma

    if initial is None:
        poses = np.zeros((n, N_POSE_PARAMS))
        poses[:, :3] = APPROX_REST_T_MM
    else:
        poses = np.array(initial, dtype=float)

    poses, iterations = _run_lm(geometry, measured, w, poses, max_iter, tol)
    good = _residual_ok(geometry, measured, w, poses)

    if retry and not good.all():
        centre = np.median(poses[good], axis=0) if good.any() else poses.mean(axis=0)
        bad = ~good
        best = poses[bad]
        best_cost = _frame_residual(geometry, measured[bad], w[bad], best)

        for seed in _retry_seeds(centre):
            trial = np.repeat(seed[None, :], int(bad.sum()), axis=0)
            trial, _ = _run_lm(geometry, measured[bad], w[bad], trial, max_iter, tol)
            cost = _frame_residual(geometry, measured[bad], w[bad], trial)
            improved = cost < best_cost
            best = np.where(improved[:, None], trial, best)
            best_cost = np.where(improved, cost, best_cost)

        poses[bad] = best
        good = _residual_ok(geometry, measured, w, poses)

    residual = geometry.predict(poses[:, :3], poses[:, 3:]) - measured
    return PoseSolveResult(poses=poses, residual_mT=residual, converged=good,
                           iterations=iterations)


def _retry_seeds(centre: NDArray[np.float64]) -> list[NDArray[np.float64]]:
    """Starting poses to try for a frame that failed from a cold start.

    A frame that ends up badly wrong is nearly always one caught close to a
    magnet and well tilted, where the residual surface has a second basin a
    few degrees away. Nudging the starting orientation around is enough to
    find the right one; the translation matters far less, since the knob
    cannot travel far.
    """
    seeds = [centre.copy()]
    rest = np.concatenate([APPROX_REST_T_MM, np.zeros(3)])
    seeds.append(rest)
    for axis in range(3):
        for sign in (+1.0, -1.0):
            seed = centre.copy()
            seed[3 + axis] += sign * 0.15  # ~8.6 degrees
            seeds.append(seed)
    return seeds


def _frame_residual(
    geometry: BundleGeometry,
    measured: NDArray[np.float64],
    w: NDArray[np.float64],
    poses: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Per-frame weighted RMS residual."""
    r = (geometry.predict(poses[:, :3], poses[:, 3:]) - measured) * w
    return np.sqrt(np.mean(r**2, axis=1))


# How far above the typical frame a residual has to sit before it is treated as
# a solver failure rather than ordinary model mismatch. Deliberately generous:
# the two are separated by more than an order of magnitude in practice (a frame
# in the wrong basin lands at tens of mT, roughly half its own signal, while a
# well-solved frame on real data sits at ~2% of signal). A tight bar here would
# reject the near-magnet frames, which legitimately fit worse *and* carry the
# most information - exactly the ones worth keeping.
_FAILURE_RESIDUAL_FACTOR = 10.0


def _residual_ok(
    geometry: BundleGeometry,
    measured: NDArray[np.float64],
    w: NDArray[np.float64],
    poses: NDArray[np.float64],
) -> NDArray[np.bool_]:
    """Which frames actually solved, as opposed to landing in a wrong basin.

    Judged against the median rather than an absolute threshold, because the
    achievable residual depends on the geometry being solved against - it is
    ~1e-14 for exact synthetic data and ~0.4 mT for a real capture. The
    absolute floor stops an unreachably tight bar when the median is ~0.
    """
    per_frame = _frame_residual(geometry, measured, w, poses)
    scale = float(np.abs(measured * w).mean())
    bar = max(_FAILURE_RESIDUAL_FACTOR * float(np.median(per_frame)), 1e-6 * scale)
    return np.asarray(per_frame <= bar, dtype=np.bool_)


def _run_lm(
    geometry: BundleGeometry,
    measured: NDArray[np.float64],
    w: NDArray[np.float64],
    poses: NDArray[np.float64],
    max_iter: int,
    tol: float,
) -> tuple[NDArray[np.float64], int]:
    """Levenberg-Marquardt over every frame at once. Returns (poses, iterations)."""
    n = len(measured)
    lam = np.full(n, 1e-3)
    eye = np.eye(N_POSE_PARAMS)

    def cost_of(p: NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        r = (geometry.predict(p[:, :3], p[:, 3:]) - measured) * w
        return 0.5 * np.sum(r**2, axis=1), r

    cost, _ = cost_of(poses)
    iterations = 0

    for iterations in range(1, max_iter + 1):  # noqa: B007 (used after the loop, as the return count)
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

    return poses, iterations
