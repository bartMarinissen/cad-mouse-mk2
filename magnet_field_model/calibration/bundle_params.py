"""Priors, residual weighting, and the least-squares problem scipy solves.

Three ideas live here:

**Weighting.** The sensors span roughly 9-70 mT over a calibration run, and
the model's error is part absolute (sensor noise, ~0.1 mT measured at rest)
and part relative (unmodelled field-shape error, ~2%). A single flat
sigma_field therefore massively overweights the close-to-the-magnet frames.
`ResidualWeights` uses sigma = hypot(absolute, relative * |B_measured|) per
sensor per frame instead. Note it keys off the *measured* magnitude, never
the predicted one - a weight that depended on the parameters would no longer
be a least-squares problem and would bias the fit toward shrinking |B|.

**Priors.** Every shared parameter is an offset from nominal with expected
value 0, so a ridge term (offset / sigma) reads directly as "how many prior
stddevs from nominal". Priors are per *group* (`RegularizationSigmas`,
walking `parameterization.BLOCKS`), because the groups mean physically
different things and are trusted very differently - some (like
`magnet_strength_mean`) deliberately carry none at all, see its docstring.
The magnet-position and gain/strength gauges are fixed structurally now
(see `parameterization.py`), not by these priors - they only have to be
defensible as beliefs about the hardware, not as a load-bearing anchor.

**The problem.** `BundleCalibrationProblem` wraps measured data, a parameter
mask, priors and weights into the residual(x)/jacobian(x) pair
`scipy.optimize.least_squares` needs.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import cast

import numpy as np
import scipy.sparse as sp
from numpy.typing import NDArray

from .bundle_geometry import N_POSE_PARAMS, N_SENSORS, BundleGeometry
from .parameterization import BLOCKS, GROUP_SLICES, N_SHARED_PARAMS


@dataclass(frozen=True)
class RegularizationSigmas:
    """Expected stddev of each parameter group's offset from nominal.

    Priors, not measurements - they encode how far from the nominal/CAD value
    each quantity is plausibly allowed to drift.

    A group may be set to None, meaning *no prior at all*: that parameter is
    then determined by the data alone. Use it when the nominal value is not
    actually a belief worth holding, since a prior centred on a number nobody
    stands behind quietly pulls the answer toward it.
    """

    # Magnet placement in the knob. Kept deliberately: the knob is 3D printed
    # (see ../enclosure/), and a few tenths of a millimetre is a well-founded
    # statement about that process - unlike the 600mT figure below, this is a
    # real belief about the parts rather than a round number nobody stands
    # behind. The position gauge is now fixed structurally (see
    # parameterization.MAGNET_POS_BASIS), so this prior is not load-bearing
    # and only has to be defensible on its own merits.
    magnet_pos_mm: float | None = 0.3
    # Magnet axis tilt, ~1.7 deg.
    magnet_tilt_rad: float | None = 0.03
    # Common-mode magnet strength: the absolute field scale. UNREGULARIZED by
    # default. The nominal it would be centred on is local_field.py's 600mT
    # polarization, which is a round guess rather than a measurement of these
    # magnets - so a prior on it would be asserting a belief nobody actually
    # holds, and would drag the fitted field scale toward an arbitrary number.
    # Left free, this parameter reports what the data alone says the scale is,
    # with an honest posterior sd next to it.
    magnet_strength_mean: float | None = None
    # How much individual magnets differ from that mean. Kept, unlike the mean
    # above, because its justification is independent of the 600mT figure: it
    # says magnets cut from one batch are graded to about a percent of each
    # other, which is a real belief about the parts. Setting this very small
    # approaches "assume all magnets identical"; setting it to None drops the
    # assumption entirely.
    magnet_strength_diff: float | None = 0.01
    # DC offset on the raw reading: Hall zero-point plus ambient field. The
    # firmware's own hand-tuned Config::sensor_offset_mT reaches 1.6 mT, so
    # this is deliberately loose enough not to fight it.
    sensor_offset_mT: float | None = 1.5
    # No gain_iso here: gain carries no isotropic/scale parameter at all (see
    # parameterization.py's "gain: 8 exactly-traceless basis matrices") -
    # magnet_strength_mean owns that job instead.
    # Per-axis sensitivity spread at fixed overall scale.
    gain_aniso: float | None = 0.05
    # Cross-axis skew and sensor-frame misalignment: weakly observable given
    # how little the field direction varies over a run, so kept tight.
    gain_sym: float | None = 0.03
    gain_rot: float | None = 0.03

    def as_vector(self) -> NDArray[np.float64]:
        """Full-length (N_SHARED_PARAMS,) sigma, one entry per shared parameter."""
        per_group = {
            "magnet_pos": self.magnet_pos_mm,
            "magnet_tilt": self.magnet_tilt_rad,
            "magnet_strength_mean": self.magnet_strength_mean,
            "magnet_strength_diff": self.magnet_strength_diff,
            "sensor_offset": self.sensor_offset_mT,
            "gain_aniso": self.gain_aniso,
            "gain_sym": self.gain_sym,
            "gain_rot": self.gain_rot,
        }
        out = np.empty(N_SHARED_PARAMS)
        for block in BLOCKS:
            sigma = per_group[block.name]
            # None means "no prior": an infinite sigma makes both the penalty
            # (offset / sigma) and its Jacobian row exactly zero, and
            # contributes no precision to the posterior.
            out[GROUP_SLICES[block.name]] = np.inf if sigma is None else sigma
        return out


@dataclass(frozen=True)
class ResidualWeights:
    """Per-observation sigma model: hypot(absolute floor, relative fraction).

    absolute_mT defaults a little above the ~0.1 mT read noise measured on a
    stationary knob, leaving room for quantisation; relative covers the
    residual field-shape error the calibration cannot remove.
    """

    absolute_mT: float = 0.2
    relative: float = 0.02

    def sigma(self, measured: NDArray[np.float64]) -> NDArray[np.float64]:
        """(n_frames, 9) sigma, constant per sensor within a frame."""
        per_sensor = np.linalg.norm(measured.reshape(-1, N_SENSORS, 3), axis=2)
        sigma = np.hypot(self.absolute_mT, self.relative * per_sensor)
        return np.repeat(sigma, 3, axis=1)


@dataclass(frozen=True)
class SolveStage:
    """A named subset of shared parameter groups to free in one solve.

    Despite the name, `run_bundle_calibration` no longer solves in stages by
    default (see ALL_STAGE below) - an earlier version froze parameters in a
    ladder (offset, then geometry, then gain, then a separate gauge-transfer
    pass for magnet strength), on the theory that a cold start freeing
    everything at once would trade a badly-scaled gain against a magnet
    position it couldn't yet see. That was never actually tested until it was
    ablated: a single unstaged solve over every free parameter converges to
    the *same* optimum, to within optimizer tolerance (~1e-3 on real capture
    data, bit-identical on synthetic ground-truth recovery), for both real and
    synthetic data. `scipy.optimize.least_squares`'s `x_scale="jac"` already
    renormalizes each parameter by its own Jacobian norm every iteration,
    which was apparently already doing the job the ladder was defending
    against.

    What SolveStage is still for: isolating a specific subset of parameters in
    a test (e.g. "solve magnet_strength alone, with nothing else free, to
    check it's recoverable when nothing competes with it") - a genuinely
    useful diagnostic capability, kept for that reason even though the default
    calibration no longer walks a ladder of them.
    """

    name: str
    free_groups: tuple[str, ...]

    def mask(self) -> NDArray[np.bool_]:
        m = np.zeros(N_SHARED_PARAMS, dtype=bool)
        for group in self.free_groups:
            m[GROUP_SLICES[group]] = True
        return m

    def build_problem(
        self,
        measured: NDArray[np.float64],
        x_shared_base: NDArray[np.float64],
        sigmas: RegularizationSigmas,
        weights: ResidualWeights,
    ) -> BundleCalibrationProblem:
        """Convenience: build the Problem this stage implies, without the
        caller needing to know a Problem is constructed from a mask rather
        than the stage itself."""
        return BundleCalibrationProblem(measured, x_shared_base, self.mask(), sigmas, weights)


# What run_bundle_calibration solves by default: one stage, everything free.
ALL_STAGE: tuple[SolveStage, ...] = (
    SolveStage("everything", tuple(b.name for b in BLOCKS)),
)


class BundleCalibrationProblem:
    """The scaled residual/Jacobian pair scipy.optimize.least_squares works on.

    The optimization vector `x` is [free shared params, then 6 pose params
    per frame]. Shared parameters outside `mask` stay frozen at
    `x_shared_base` and simply do not appear in `x`.

    A plain class, not a dataclass: `_sigma_field` and `_sigma_prior` are
    real derived state computed once from the constructor arguments (not
    plain stored fields), so an explicit `__init__` is a more honest
    description of what this object actually is than dataclass syntax would be.
    """

    def __init__(
        self,
        measured: NDArray[np.float64],
        x_shared_base: NDArray[np.float64],
        mask: NDArray[np.bool_],
        sigmas: RegularizationSigmas | None = None,
        weights: ResidualWeights | None = None,
    ) -> None:
        self.measured = measured
        self.x_shared_base = x_shared_base
        self.mask = mask
        self.sigmas = sigmas or RegularizationSigmas()
        self.weights = weights or ResidualWeights()
        self._sigma_field = self.weights.sigma(measured)
        self._sigma_prior = self.sigmas.as_vector()

    @property
    def n_frames(self) -> int:
        return len(self.measured)

    @property
    def n_free_shared(self) -> int:
        """How many shared parameters `mask` leaves free."""
        return int(self.mask.sum())

    @property
    def free_mask(self) -> NDArray[np.bool_]:
        """Which entries of the full shared vector this problem leaves free."""
        return self.mask.copy()

    def pack(
        self, x_shared: NDArray[np.float64], poses: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Build the solver vector `x` from a full shared-parameter vector and
        per-frame poses - the inverse of unpack()."""
        return np.concatenate([x_shared[self.mask], poses.ravel()])

    def unpack(
        self, x: NDArray[np.float64]
    ) -> tuple[NDArray[np.float64], BundleGeometry, NDArray[np.float64]]:
        """Solver vector `x` -> (full shared-parameter vector, the geometry it
        encodes, per-frame poses).

        Frozen shared parameters come from `x_shared_base`; free ones from
        `x` itself, via `mask`. Every caller that used to do
        `unpack(x)` followed immediately by `unpack_shared(x_shared)` now
        gets the geometry directly - this is the one place that combination
        happens.
        """
        x_shared = self.x_shared_base.copy()
        x_shared[self.mask] = x[: self.n_free_shared]
        poses = x[self.n_free_shared :].reshape(self.n_frames, N_POSE_PARAMS)
        return x_shared, BundleGeometry.from_shared(x_shared), poses

    def residual(self, x: NDArray[np.float64]) -> NDArray[np.float64]:
        """The full scaled residual: field residuals (in sigma units) followed
        by prior residuals (in prior-sigma units) for the free parameters."""
        x_shared, geometry, poses = self.unpack(x)
        predicted = geometry.predict(poses[:, :3], poses[:, 3:])
        field = ((predicted - self.measured) / self._sigma_field).ravel()
        prior = (x_shared / self._sigma_prior)[self.mask]
        return np.concatenate([field, prior])

    def jacobian(self, x: NDArray[np.float64]) -> sp.csr_array:
        """The scaled residual's Jacobian: dense in the shared-parameter
        columns, block-diagonal in the per-frame pose columns (perturbing one
        frame's pose cannot touch another frame's residual), stacked over the
        prior rows' diagonal contribution."""
        _, geometry, poses = self.unpack(x)
        _, j_pose, j_shared = geometry.predict_and_jacobians(poses[:, :3], poses[:, 3:])
        inv_sigma = (1.0 / self._sigma_field)[:, :, None]
        j_pose = j_pose * inv_sigma
        j_shared = j_shared[:, :, self.mask] * inv_sigma

        n, p = self.n_frames, self.n_free_shared
        shared_block = sp.csr_array(j_shared.reshape(9 * n, p))
        pose_block = sp.block_diag([sp.coo_array(b) for b in j_pose], format="csr")
        field_jac = sp.hstack([shared_block, pose_block], format="csr")

        prior_jac = sp.hstack([
            sp.diags_array(1.0 / self._sigma_prior[self.mask]),
            sp.csr_array((p, N_POSE_PARAMS * n)),
        ], format="csr")
        return cast(sp.csr_array, sp.vstack([field_jac, prior_jac], format="csr"))

    def field_residual_mT(self, x: NDArray[np.float64]) -> NDArray[np.float64]:
        """The field part of the residual in real mT (unscaled), for reporting."""
        _, geometry, poses = self.unpack(x)
        predicted = geometry.predict(poses[:, :3], poses[:, 3:])
        return predicted - self.measured

    def covariance_shared(self, x: NDArray[np.float64]) -> NDArray[np.float64]:
        """Posterior covariance of the free shared parameters (a Laplace/
        Gaussian approximation around x), for reporting only.

        Reuses this same object's own jacobian(x) - the exact Jacobian the
        solver itself trusts - so the reported uncertainty can't drift out of
        sync with what was actually fitted. The top-left p x p block of
        inv(J^T J) is the marginal covariance of the shared parameters with
        the per-frame poses marginalized out; that block equality is a
        textbook Schur-complement identity, not an approximation.

        An earlier version computed that block directly via a hand-rolled
        per-frame Schur elimination (O(n_frames) instead of forming the full
        (p + 6n)-sized matrix). That reduction is the right tool at a much
        larger scale than this tool ever runs at (it's also exactly what
        would make an eventual on-device streaming solve tractable - see
        git history for the implementation if that becomes real work) - but
        at this problem's actual size (a few hundred to a couple thousand
        unknowns) a plain dense inverse is simpler and still well under a
        second, so that's what this does.
        """
        j = self.jacobian(x)
        h = (j.transpose() @ j).toarray()
        p = self.n_free_shared
        return np.linalg.inv(h)[:p, :p]
