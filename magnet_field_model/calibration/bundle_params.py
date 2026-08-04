"""Priors, residual weighting, and the staged least-squares problem.

Three ideas live here, and they are what turn the raw forward model in
bundle_geometry.py into something that actually converges to a sensible
answer:

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
stddevs from nominal". Priors are per *group*, because the groups mean
physically different things and are trusted very differently. They also do
the gauge fixing: a global translation/rotation of all magnets is exactly
degenerate with a compensating per-frame pose change, and anchoring the
magnet offsets to nominal is what removes those 6 dead directions.

**Staging.** Fitting all 42 shared parameters at once from a cold start
invites the solver to trade a badly-scaled gain against a magnet position it
cannot yet see. `SolveStage` frees one group of parameters at a time, each
stage warm-starting from the last - the gain scale first (it is the dominant
term by far), then magnet geometry, then the weak cross-axis gain terms.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import scipy.sparse as sp
from numpy.typing import NDArray

from .bundle_geometry import (
    GROUP_SLICES,
    N_POSE_PARAMS,
    N_SENSORS,
    N_SHARED_PARAMS,
    PARAM_GROUPS,
    unpack_shared,
)


@dataclass(frozen=True)
class RegularizationSigmas:
    """Expected stddev of each parameter group's offset from nominal.

    Priors, not measurements - they encode how far from the nominal/CAD value
    each quantity is plausibly allowed to drift.
    """

    # Magnet placement in the knob: press-fit/glued, so a few tenths of a mm.
    magnet_pos_mm: float = 0.3
    # Magnet axis tilt, ~1.7 deg.
    magnet_tilt_rad: float = 0.03
    # Common-mode magnet strength: the absolute field scale. Loose, because
    # the nominal 600mT polarization is itself only an estimate, and this
    # carries the whole scale once the det(G)=1 gauge is applied.
    magnet_strength_mean: float = 0.15
    # How much individual magnets differ from that mean. Deliberately ~15x
    # tighter: magnets cut from one batch are graded to about a percent of
    # each other, whereas their common absolute remanence is not pinned at
    # all. Setting this very small approaches "assume all magnets identical",
    # which pushes per-sensor scale differences into geometry and gain
    # instead - see the note in the README about that trade.
    magnet_strength_diff: float = 0.01
    # DC offset on the raw reading: Hall zero-point plus ambient field. The
    # firmware's own hand-tuned Config::sensor_offset_mT reaches 1.6 mT, so
    # this is deliberately loose enough not to fight it.
    sensor_offset_mT: float = 1.5
    # Isotropic sensor gain. Deliberately loose: the firmware's own hand-tuned
    # Config::magnet_gains span -0.96..-1.2, i.e. offsets up to 0.2 from the
    # nominal, so a tight prior here would fight known-real hardware spread.
    gain_iso: float = 0.15
    # Per-axis sensitivity spread at fixed overall scale.
    gain_aniso: float = 0.05
    # Cross-axis skew and sensor-frame misalignment: weakly observable given
    # how little the field direction varies over a run, so kept tight.
    gain_sym: float = 0.03
    gain_rot: float = 0.03

    def as_vector(self) -> NDArray[np.float64]:
        per_group = {
            "magnet_pos": self.magnet_pos_mm,
            "magnet_tilt": self.magnet_tilt_rad,
            "magnet_strength_mean": self.magnet_strength_mean,
            "magnet_strength_diff": self.magnet_strength_diff,
            "sensor_offset": self.sensor_offset_mT,
            "gain_iso": self.gain_iso,
            "gain_aniso": self.gain_aniso,
            "gain_sym": self.gain_sym,
            "gain_rot": self.gain_rot,
        }
        out = np.empty(N_SHARED_PARAMS)
        for name, _ in PARAM_GROUPS:
            out[GROUP_SLICES[name]] = per_group[name]
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
    """One step of the hierarchical solve: which parameter groups are free."""

    name: str
    free_groups: tuple[str, ...]

    def mask(self) -> NDArray[np.bool_]:
        m = np.zeros(N_SHARED_PARAMS, dtype=bool)
        for group in self.free_groups:
            m[GROUP_SLICES[group]] = True
        return m


# The default ladder. Each stage inherits everything the previous one freed.
#
# Two things to note about the ordering. `sensor_offset` is freed immediately:
# a DC offset is a pure constant across every pose, so it is both easy to
# separate and badly corrupting if left for later - the geometry stages would
# otherwise contort themselves to absorb it. And `gain_iso` carries scale
# throughout, with `magnet_strength` frozen; the final stage swaps them, after
# renormalize_gauge() has moved the scale across (see GAUGE_STAGE).
DEFAULT_STAGES: tuple[SolveStage, ...] = (
    SolveStage("gain scale + offset", ("gain_iso", "sensor_offset")),
    SolveStage("magnet geometry",
               ("gain_iso", "sensor_offset", "magnet_pos", "magnet_tilt")),
    SolveStage("full gain",
               ("gain_iso", "sensor_offset", "magnet_pos", "magnet_tilt",
                "gain_aniso", "gain_sym", "gain_rot")),
)

# Run after renormalize_gauge(): identical to the last default stage except
# gain_iso is frozen (det(G) == 1) and magnet_strength is free in its place.
# This both re-optimizes strength and cleans up the cross-talk term that makes
# the gauge transfer only approximate.
GAUGE_STAGE = SolveStage(
    "magnet strength",
    ("magnet_strength_mean", "magnet_strength_diff", "sensor_offset", "magnet_pos", "magnet_tilt",
     "gain_aniso", "gain_sym", "gain_rot"),
)


@dataclass
class BundleCalibrationProblem:
    """The scaled residual/Jacobian pair scipy.optimize.least_squares works on.

    The optimization vector is [free shared params, then 6 pose params per
    frame]. Shared parameters outside the current stage stay frozen at
    `x_shared_base` and simply do not appear in the vector.
    """

    measured: NDArray[np.float64]           # (n_frames, 9)
    x_shared_base: NDArray[np.float64]      # full-length shared vector; frozen entries used as-is
    stage: SolveStage
    sigmas: RegularizationSigmas = field(default_factory=RegularizationSigmas)
    weights: ResidualWeights = field(default_factory=ResidualWeights)

    def __post_init__(self) -> None:
        self._mask = self.stage.mask()
        self._sigma_field = self.weights.sigma(self.measured)
        self._sigma_prior = self.sigmas.as_vector()

    @property
    def n_frames(self) -> int:
        return len(self.measured)

    @property
    def n_free_shared(self) -> int:
        return int(self._mask.sum())

    @property
    def free_mask(self) -> NDArray[np.bool_]:
        """Which entries of the full shared vector this stage leaves free."""
        return self._mask.copy()

    def pack(self, x_shared: NDArray[np.float64], poses: NDArray[np.float64]) -> NDArray[np.float64]:
        return np.concatenate([x_shared[self._mask], poses.ravel()])

    def unpack(self, x: NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        x_shared = self.x_shared_base.copy()
        x_shared[self._mask] = x[: self.n_free_shared]
        poses = x[self.n_free_shared :].reshape(self.n_frames, N_POSE_PARAMS)
        return x_shared, poses

    def residual(self, x: NDArray[np.float64]) -> NDArray[np.float64]:
        x_shared, poses = self.unpack(x)
        predicted = unpack_shared(x_shared).predict(poses[:, :3], poses[:, 3:])
        field = ((predicted - self.measured) / self._sigma_field).ravel()
        prior = (x_shared / self._sigma_prior)[self._mask]
        return np.concatenate([field, prior])

    def jacobian(self, x: NDArray[np.float64]) -> sp.spmatrix:
        x_shared, poses = self.unpack(x)
        _, j_pose, j_shared = unpack_shared(x_shared).predict_and_jacobians(
            poses[:, :3], poses[:, 3:]
        )
        inv_sigma = (1.0 / self._sigma_field)[:, :, None]
        j_pose = j_pose * inv_sigma
        j_shared = j_shared[:, :, self._mask] * inv_sigma

        n, p = self.n_frames, self.n_free_shared
        # Dense shared columns stacked on top of a block-diagonal pose part -
        # perturbing one frame's pose cannot touch another frame's residual.
        shared_block = sp.csr_array(j_shared.reshape(9 * n, p))
        pose_block = sp.block_diag([sp.coo_array(b) for b in j_pose], format="csr")
        field_jac = sp.hstack([shared_block, pose_block], format="csr")

        prior_jac = sp.hstack([
            sp.diags_array(1.0 / self._sigma_prior[self._mask]),
            sp.csr_array((p, N_POSE_PARAMS * n)),
        ], format="csr")
        return sp.vstack([field_jac, prior_jac], format="csr")

    def field_residual_mT(self, x: NDArray[np.float64]) -> NDArray[np.float64]:
        """The field part of the residual in real mT, for reporting."""
        x_shared, poses = self.unpack(x)
        predicted = unpack_shared(x_shared).predict(poses[:, :3], poses[:, 3:])
        return predicted - self.measured

    def covariance_shared(self, x: NDArray[np.float64]) -> NDArray[np.float64]:
        """Posterior covariance of the free shared parameters, via the Schur complement.

        Each frame's 6 pose parameters appear only in that frame's 9 residuals,
        so the pose-pose block of the normal equations is block-diagonal and
        can be eliminated one frame at a time, leaving a system only in the
        shared parameters. That keeps this O(n_frames) in time and O(p^2) in
        memory instead of forming the full (42 + 6n)^2 matrix - and it is the
        same reduction that would make an on-device port tractable.
        """
        x_shared, poses = self.unpack(x)
        _, j_pose, j_shared = unpack_shared(x_shared).predict_and_jacobians(
            poses[:, :3], poses[:, 3:]
        )
        inv_sigma = (1.0 / self._sigma_field)[:, :, None]
        a = j_pose * inv_sigma                       # (n, 9, 6)  pose columns
        b = j_shared[:, :, self._mask] * inv_sigma   # (n, 9, p)  shared columns

        p = self.n_free_shared
        h = np.diag(1.0 / self._sigma_prior[self._mask] ** 2)  # prior contribution
        h = h + np.einsum("nka,nkb->ab", b, b)

        h_pp = np.einsum("nka,nkb->nab", a, a)               # (n, 6, 6)
        h_sp = np.einsum("nka,nkb->nab", b, a)               # (n, p, 6)
        # Ridge-guard each frame's 6x6 before inverting: a frame whose pose is
        # poorly determined would otherwise blow up the reduction.
        h_pp = h_pp + 1e-9 * np.eye(N_POSE_PARAMS)
        reduced = h - np.einsum("nab,nbc,ndc->ad", h_sp, np.linalg.inv(h_pp), h_sp)

        return np.linalg.inv(reduced)

    def prior_sigma_free(self) -> NDArray[np.float64]:
        return self._sigma_prior[self._mask]

    def free_param_names(self) -> list[str]:
        names: list[str] = []
        for group, _ in PARAM_GROUPS:
            sl = GROUP_SLICES[group]
            width = (sl.stop - sl.start) // N_SENSORS
            for unit in range(N_SENSORS):
                for k in range(width):
                    idx = sl.start + width * unit + k
                    if self._mask[idx]:
                        names.append(f"{group}[{unit + 1}]{'xyz'[k] if width == 3 else k}")
        return names
