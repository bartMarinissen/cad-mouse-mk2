"""The bundle calibration itself: a regularized least-squares fit.

Structure of the fit:

  frames  ->  approximate poses (seed)  ->  one joint solve  ->  result

It is a bundle adjustment - a joint fit over a small set of *shared*
parameters (the per-magnet geometry and per-sensor gain that describe this
particular unit) and one free 6-DOF pose per captured frame. Four things
about how it is actually solved are worth knowing:

1. The forward model's frame convention matches the firmware's exactly (see
   bundle_geometry.py's "Frame conventions" note and parameterization.py's
   gauge notes for magnet position and gain/strength).
2. The Jacobian is analytic (bundle_geometry.predict_and_jacobians), not
   finite-differenced.
3. Residuals are weighted by a per-observation sigma that grows with the
   measured field magnitude, so near-magnet frames don't dominate.
4. Every shared parameter is solved for jointly, in a single
   least_squares call - NOT in stages. An earlier version froze parameters
   in a ladder on the theory that a cold start needed the help; ablated
   against real and synthetic data, a single unstaged solve converges to the
   same optimum every time (see bundle_params.SolveStage's docstring for the
   measurement). SolveStage/the `stages` parameter still exist, but only as
   a way to isolate a parameter subset for testing.

The result carries a posterior sigma per parameter, computed from the fit's
own Jacobian, so it is visible which parameters the data actually determined
and which just sat at their prior.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import least_squares

from .bundle_geometry import NOMINAL_GEOMETRY, BundleGeometry
from .bundle_params import (
    ALL_STAGE,
    BundleCalibrationProblem,
    RegularizationSigmas,
    ResidualWeights,
    SolveStage,
)
from .parameterization import N_SHARED_PARAMS
from .pose_solver import solve_poses
from .protocol import CalibStep

# Measured sweep over the captured runs: 30, 60, 120, 200 and all 387 frames
# all land within 0.03 percentage points of the same residual, and the
# run-to-run spread of the fitted parameters does not improve past ~60 either
# - the limit is systematic model error, not sample noise, so there is
# nothing to buy by fitting every frame. An earlier version therefore picked
# a diverse subset by farthest-point sampling; that machinery is gone now -
# *which* frames survive barely matters (see the measurement above), so
# _decimate() below just thins evenly. The cap itself still earns its keep,
# though, for a different reason than accuracy: solving all ~387 frames
# unstaged takes 40+ seconds (more least_squares iterations at ~6x the
# problem dimension), against ~4s at 60.
DEFAULT_N_FRAMES = 60


def _decimate(indices: NDArray[np.int_], n_max: int) -> NDArray[np.int_]:
    """Evenly thin `indices` down to at most n_max entries."""
    if len(indices) <= n_max:
        return indices
    pick = np.linspace(0, len(indices) - 1, n_max).round().astype(np.int_)
    return indices[pick]


@dataclass(frozen=True)
class StageReport:
    name: str
    n_free_shared: int
    cost: float
    field_rms_mT: float
    field_rel_pct: float
    success: bool
    message: str
    seconds: float


@dataclass(frozen=True)
class BundleCalibrationResult:
    geometry: BundleGeometry
    shared_offsets: NDArray[np.float64]        # full-length (N_SHARED_PARAMS) offsets from nominal
    posterior_sigma: NDArray[np.float64]       # same length; inf for parameters never freed
    prior_sigma: NDArray[np.float64]
    frame_poses: NDArray[np.float64]           # (n_selected, 6)
    selected_indices: NDArray[np.int_]
    field_residual_mT: NDArray[np.float64]     # (n_selected, 9)
    field_rms_mT: float
    field_rel_pct: float
    stages: tuple[StageReport, ...] = ()
    initial_rms_mT: float = float("nan")
    initial_rel_pct: float = float("nan")

    @property
    def information_gain(self) -> NDArray[np.float64]:
        """1 - posterior/prior sigma, per parameter.

        ~0 means the data said nothing and the parameter is sitting at its
        prior; ~1 means the data pinned it down far tighter than the prior did.

        NaN where there is no prior to compare against - an unregularized
        parameter is determined entirely by the data, so "how much did the data
        add over the prior" is not a question with an answer. Read its
        posterior sd directly instead.
        """
        with np.errstate(invalid="ignore", divide="ignore"):
            gain = 1.0 - self.posterior_sigma / self.prior_sigma
        return np.where(np.isfinite(self.prior_sigma), gain, np.nan)


def _rel_pct(residual: NDArray[np.float64], measured: NDArray[np.float64]) -> float:
    """RMS residual as a percentage of the mean per-sensor field magnitude."""
    rms = float(np.sqrt(np.mean(residual**2)))
    scale = float(np.linalg.norm(measured.reshape(-1, 3, 3), axis=2).mean())
    return 100.0 * rms / scale


def flatten_datasets(
    datasets: dict[CalibStep, list[list[float]]],
) -> tuple[NDArray[np.float64], NDArray[np.object_]]:
    """All frames in step order, plus a parallel array of step names."""
    steps = sorted((s for s, f in datasets.items() if f), key=int)
    frames = np.concatenate([np.array(datasets[s], dtype=float) for s in steps])
    labels = np.concatenate([np.array([s.name] * len(datasets[s])) for s in steps])
    return frames, labels


def run_bundle_calibration(
    datasets: dict[CalibStep, list[list[float]]],
    *,
    n_frames: int = DEFAULT_N_FRAMES,
    sigmas: RegularizationSigmas | None = None,
    weights: ResidualWeights | None = None,
    stages: tuple[SolveStage, ...] = ALL_STAGE,
    verbose: bool = True,
) -> BundleCalibrationResult:
    """Fit the magnet bundle model from collected calibration data.

    datasets maps each CalibStep to the list of 9-float frames recorded during
    that pose. n_frames caps how many converged frames the joint fit actually
    uses (evenly decimated if there are more) - see DEFAULT_N_FRAMES for why
    a cap exists at all despite frame choice barely affecting the result.
    """
    sigmas = sigmas or RegularizationSigmas()
    weights = weights or ResidualWeights()

    measured_all, _labels = flatten_datasets(datasets)

    # --- 1. approximate poses against nominal geometry, to seed the real fit ---
    seed = solve_poses(NOMINAL_GEOMETRY, measured_all)
    initial_rms = float(np.sqrt(np.mean(seed.residual_mT**2)))
    initial_rel = _rel_pct(seed.residual_mT, measured_all)
    if verbose:
        print(f"seeded {len(measured_all)} poses in {seed.iterations} LM iterations "
              f"({seed.converged.sum()}/{len(measured_all)} converged); "
              f"nominal residual {initial_rms:.3f} mT ({initial_rel:.2f}%)")

    # --- 2. use every converged frame, capped for runtime (see _decimate) ---
    idx = _decimate(np.flatnonzero(seed.converged), n_frames)
    measured = measured_all[idx]
    poses = seed.poses[idx]
    rot_deg = np.degrees(np.linalg.norm(poses[:, 3:], axis=1))
    if verbose:
        print(f"using {len(idx)} of {len(measured_all)} frames "
              f"(t span {np.ptp(poses[:, 0]):.2f}/{np.ptp(poses[:, 1]):.2f}/"
              f"{np.ptp(poses[:, 2]):.2f} mm, rotation up to {rot_deg.max():.1f} deg)")

    # --- 3. the fit ---
    x_shared = np.zeros(N_SHARED_PARAMS)
    reports: list[StageReport] = []
    problem: BundleCalibrationProblem | None = None
    x: NDArray[np.float64] | None = None

    geometry = NOMINAL_GEOMETRY
    for stage in stages:
        t0 = time.monotonic()
        problem = stage.build_problem(measured, x_shared, sigmas, weights)
        x0 = problem.pack(x_shared, poses)
        # Tolerances are deliberately not tighter than this: the model itself
        # carries ~2% systematic error, so chasing parameter changes below
        # ~1e-8 relative is grinding on noise. Tighter settings just burned
        # the evaluation budget without moving the answer.
        result = least_squares(
            problem.residual, x0, jac=problem.jacobian,
            method="trf", tr_solver="lsmr", x_scale="jac",
            xtol=1e-8, ftol=1e-8, gtol=1e-8, max_nfev=400,
        )
        x = result.x
        x_shared, geometry, poses = problem.unpack(x)

        resid = problem.field_residual_mT(x)
        rms = float(np.sqrt(np.mean(resid**2)))
        rel = _rel_pct(resid, measured)
        reports.append(StageReport(
            name=stage.name, n_free_shared=problem.n_free_shared, cost=float(result.cost),
            field_rms_mT=rms, field_rel_pct=rel, success=bool(result.success),
            message=str(result.message), seconds=time.monotonic() - t0,
        ))
        if verbose:
            print(f"  stage '{stage.name}' ({problem.n_free_shared} free): "
                  f"residual {rms:.3f} mT ({rel:.2f}%) in {reports[-1].seconds:.1f}s")

    assert problem is not None and x is not None, "at least one stage is required"

    # --- 4. posterior uncertainty on whatever the last stage left free ---
    posterior = np.full(N_SHARED_PARAMS, np.inf)
    try:
        cov = problem.covariance_shared(x)
        posterior[problem.free_mask] = np.sqrt(np.maximum(np.diag(cov), 0.0))
    except np.linalg.LinAlgError:
        if verbose:
            print("  (posterior covariance is singular - reporting no uncertainties)")

    resid = geometry.predict(poses[:, :3], poses[:, 3:]) - measured
    return BundleCalibrationResult(
        geometry=geometry,
        shared_offsets=x_shared,
        posterior_sigma=posterior,
        prior_sigma=sigmas.as_vector(),
        frame_poses=poses,
        selected_indices=idx,
        field_residual_mT=resid,
        field_rms_mT=float(np.sqrt(np.mean(resid**2))),
        field_rel_pct=_rel_pct(resid, measured),
        stages=tuple(reports),
        initial_rms_mT=initial_rms,
        initial_rel_pct=initial_rel,
    )
