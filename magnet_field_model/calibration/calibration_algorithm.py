"""The bundle calibration math.

A joint (bundle-adjustment-style) nonlinear least-squares fit over:

  - 36 shared calibration parameters (per magnet/sensor pair: magnet
    position offset, magnet rotation offset, a 5-free-entry upper-triangular
    gain matrix, magnet strength offset) - each ridge-regularized around its
    nominal/expected value (0) using a per-parameter expected stddev, and
  - one 6-DOF knob pose (translation + rotation vector) per captured
    frame - free, unregularized.

against the raw field measurements collected for all 7 poses. See
bundle_params.py (BundleCalibrationProblem) for how the combined, scaled
residual/Jacobian is built - minimizing its sum of squares is a MAP estimate
under the regularization priors. The Jacobian is hand-assembled from
per-frame finite-difference blocks (bundle_geometry.full_jacobian) rather
than left to scipy's own dense finite-difference fallback, since a global
dense Jacobian would re-evaluate every frame's residual for every one of the
~2,350 unknowns (O(n_frames^2)) - this runs offline, once, so there's no
need for anything fancier than finite differences, just not a dense global
sweep.
"""

from __future__ import annotations

import functools
import multiprocessing
from collections.abc import Callable, Iterable
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import least_squares

from .bundle_geometry import (
    APPROX_REST_T_MM,
    N_POSE_PARAMS,
    N_SHARED_PARAMS,
    BundleGeometry,
    unpack_shared,
)
from .bundle_params import BundleCalibrationProblem, RegularizationSigmas
from .protocol import CalibStep


@dataclass(frozen=True)
class BundleCalibrationResult:
    geometry: BundleGeometry                            # fitted (nominal + offset) bundle geometry
    shared_offsets: NDArray[np.float64]                  # the raw 36 fitted offsets, for reporting
    frame_poses: NDArray[np.float64]                     # (n_frames, 6) fitted [t, rotvec] per frame
    cost: float                                          # scipy's final 0.5 * sum(residuals**2)
    success: bool
    message: str
    field_residual_rms_mT: float                         # unscaled RMS field residual, for sanity-checking the fit


def _solve_single_frame_pose(
    measured_field: NDArray[np.float64],
    geometry: BundleGeometry,
    t0: NDArray[np.float64],
    rotvec0: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """A cheap, unregularized single-frame solve against nominal geometry.

    Only used to seed initial guesses for the joint fit below - not part
    of the actual calibration result.
    """

    def residual(x: NDArray[np.float64]) -> NDArray[np.float64]:
        return geometry.predict_field(x[:3], x[3:]) - measured_field

    result = least_squares(residual, np.concatenate([t0, rotvec0]))
    return result.x[:3], result.x[3:]


def _solve_step_poses(
    measured_frames: NDArray[np.float64],
    nominal_geometry: BundleGeometry,
) -> NDArray[np.float64]:
    """Warm-started chain of single-frame solves for one calibration step.

    Warm-started frame-to-frame (adjacent frames are ~50ms apart during
    RECORDING, so the true pose barely moves) rather than resetting to the
    rest pose every time - this matters a lot for the dynamic steps
    (PITCH/ROLL/TWIST/HEAVE/RANDOM), where a plain rest-pose guess would be
    far from the true pose. This makes the chain inherently sequential
    within a step - see _initial_frame_poses for the parallel axis instead.
    """
    poses = []
    t_guess, rotvec_guess = APPROX_REST_T_MM, np.zeros(3)
    for measured_field in measured_frames:
        t_guess, rotvec_guess = _solve_single_frame_pose(measured_field, nominal_geometry, t_guess, rotvec_guess)
        poses.append(np.concatenate([t_guess, rotvec_guess]))
    return np.array(poses)


def _initial_frame_poses(
    measured_by_step: dict[CalibStep, NDArray[np.float64]],
    nominal_geometry: BundleGeometry,
    *,
    workers: Callable[[Callable, Iterable], Iterable] = map,
) -> NDArray[np.float64]:
    """Seed per-frame pose guesses via independent single-frame solves.

    Each step's warm-started chain (see _solve_step_poses) is sequential,
    but the 7 steps are fully independent of each other - each starts fresh
    from the rest pose - so `workers` (e.g. a multiprocessing.Pool's .map)
    parallelizes across steps instead.
    """
    steps = sorted(measured_by_step, key=int)
    task = functools.partial(_solve_step_poses, nominal_geometry=nominal_geometry)
    step_poses = workers(task, (measured_by_step[step] for step in steps))
    return np.concatenate(list(step_poses))


def run_bundle_calibration(
    datasets: dict[CalibStep, list[list[float]]],
    *,
    sigma_field_mT: float = 0.5,
    sigmas: RegularizationSigmas = RegularizationSigmas(),
) -> BundleCalibrationResult:
    """Fit the magnet bundle model from collected calibration data.

    datasets maps each CalibStep (STATIONARY, FLAT_CIRCLE, PITCH, ROLL,
    TWIST, HEAVE, RANDOM) to a list of raw sensor frames recorded during
    that pose, where each frame is 9 floats (3 sensors x Bx/By/Bz).
    """
    # Filter out empty steps - CalibrationSession.datasets always has an
    # entry for every CalibStep (including COMPLETE), but COMPLETE never
    # actually gets any frames, and an empty array would break the
    # concatenation below.
    measured_by_step = {
        step: np.array(frames[::8], dtype=float) for step, frames in datasets.items() if frames
    }
    measured = np.concatenate([measured_by_step[step] for step in sorted(measured_by_step, key=int)])
    n_frames = len(measured)

    nominal_geometry = unpack_shared(np.zeros(N_SHARED_PARAMS))

    # One pool, reused for both the initial-pose seeding (parallel across
    # steps - see _initial_frame_poses) and the Jacobian (parallel across
    # frames - see bundle_geometry.poses_jacobian). The Jacobian is supplied
    # explicitly below (jac=), so scipy never falls back to its own dense
    # finite-difference path at all.
    with multiprocessing.Pool() as pool:
        frame_x0 = _initial_frame_poses(measured_by_step, nominal_geometry, workers=pool.map)
        x0 = np.concatenate([np.zeros(N_SHARED_PARAMS), frame_x0.ravel()])

        problem = BundleCalibrationProblem(
            measured=measured, sigma_field_mT=sigma_field_mT, sigmas=sigmas, workers=pool.map
        )
        result = least_squares(
            problem.residual, x0, jac=problem.jacobian, x_scale="jac", verbose=2, tr_solver='lsmr',
            xtol=1e-5, ftol=1e-6, gtol=1e-5,
            max_nfev=1000,
            tr_options={'regularize': False}
        )
    print(result)

    unscaled = problem.unscale_residual(result.fun)
    field_residual_rms_mT = float(np.sqrt(np.mean(unscaled[: 9 * n_frames] ** 2)))

    return BundleCalibrationResult(
        geometry=unpack_shared(result.x[:N_SHARED_PARAMS]),
        shared_offsets=result.x[:N_SHARED_PARAMS],
        frame_poses=result.x[N_SHARED_PARAMS:].reshape(n_frames, N_POSE_PARAMS),
        cost=result.cost,
        success=result.success,
        message=result.message,
        field_residual_rms_mT=field_residual_rms_mT,
    )
