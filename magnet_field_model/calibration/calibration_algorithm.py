"""The bundle calibration math.

A joint (bundle-adjustment-style) nonlinear least-squares fit over:

  - 30 shared calibration parameters (per magnet/sensor pair: sensor
    position offset, magnet position offset, magnet rotation offset,
    sensor gain offset) - each ridge-regularized around its nominal/
    expected value (0) using a per-parameter expected stddev, and
  - one 6-DOF knob pose (translation + rotation vector) per captured
    frame - free, unregularized.

against the raw field measurements collected for all 7 poses. The
combined residual vector is [field residuals / sigma_field] followed by
[shared offsets / their sigma] - minimizing its sum of squares is a MAP
estimate under those priors. Solved with scipy.optimize.least_squares
using its default finite-difference Jacobian: this runs offline, once, so
there's no need to hand-derive an analytic one.
"""

from __future__ import annotations

from dataclasses import dataclass
import multiprocessing

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import least_squares

from .bundle_geometry import APPROX_REST_T_MM, BundleGeometry
from .bundle_params import N_SHARED_PARAMS, RegularizationSigmas, unpack_shared
from .protocol import CalibStep

N_POSE_PARAMS = 6  # translation (3) + rotation vector (3)


@dataclass(frozen=True)
class BundleCalibrationResult:
    geometry: BundleGeometry                            # fitted (nominal + offset) bundle geometry
    shared_offsets: NDArray[np.float64]                  # the raw 30 fitted offsets, for reporting
    frame_poses: NDArray[np.float64]                     # (n_frames, 6) fitted [t, rotvec] per frame
    cost: float                                          # scipy's final 0.5 * sum(residuals**2)
    success: bool
    message: str


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


def _initial_frame_poses(
    measured_by_step: dict[CalibStep, NDArray[np.float64]],
    nominal_geometry: BundleGeometry,
) -> NDArray[np.float64]:
    """Seed per-frame pose guesses via independent single-frame solves.

    Warm-started frame-to-frame within each step (adjacent frames are
    ~50ms apart during RECORDING, so the true pose barely moves) rather
    than resetting to the rest pose every time - this matters a lot for
    the dynamic steps (PITCH/ROLL/TWIST/HEAVE/RANDOM), where a plain
    rest-pose guess would be far from the true pose.
    """
    poses = []
    for step in sorted(measured_by_step, key=int):
        t_guess, rotvec_guess = APPROX_REST_T_MM, np.zeros(3)
        for measured_field in measured_by_step[step]:
            t_guess, rotvec_guess = _solve_single_frame_pose(measured_field, nominal_geometry, t_guess, rotvec_guess)
            poses.append(np.concatenate([t_guess, rotvec_guess]))
    return np.array(poses)


def residuals(x: NDArray[np.float64], *, measured: np.array, n_frames: int, sigma_vector: NDArray[np.float64], sigma_field_mT: float,) -> NDArray[np.float64]:
    shared = x[:N_SHARED_PARAMS]
    frame_params = x[N_SHARED_PARAMS:].reshape(n_frames, N_POSE_PARAMS)
    geometry = unpack_shared(shared)

    field_residuals = np.empty(9 * n_frames)
    for i in range(n_frames):
        t, rotvec = frame_params[i, :3], frame_params[i, 3:]
        field_residuals[9 * i : 9 * i + 9] = (geometry.predict_field(t, rotvec) - measured[i]) / sigma_field_mT

    reg_residuals = shared / sigma_vector
    return np.concatenate([field_residuals, reg_residuals])

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
        step: np.array(frames, dtype=float) for step, frames in datasets.items() if frames
    }
    measured = np.concatenate([measured_by_step[step] for step in sorted(measured_by_step, key=int)])
    n_frames = len(measured)

    nominal_geometry = unpack_shared(np.zeros(N_SHARED_PARAMS))
    frame_x0 = _initial_frame_poses(measured_by_step, nominal_geometry)
    x0 = np.concatenate([np.zeros(N_SHARED_PARAMS), frame_x0.ravel()])

    sigma_vector = sigmas.as_vector()

    params = {
        "measured": measured,
        "n_frames": n_frames,
        "sigma_vector": sigma_vector,
        "sigma_field_mT": sigma_field_mT,
    }

    with multiprocessing.Pool() as pool:
        result = least_squares(residuals, x0, verbose=2, xtol=1e-4, kwargs=params, workers=pool.map)

    return BundleCalibrationResult(
        geometry=unpack_shared(result.x[:N_SHARED_PARAMS]),
        shared_offsets=result.x[:N_SHARED_PARAMS],
        frame_poses=result.x[N_SHARED_PARAMS:].reshape(n_frames, N_POSE_PARAMS),
        cost=result.cost,
        success=result.success,
        message=result.message,
    )
