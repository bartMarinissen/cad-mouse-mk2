"""The scaled residual/Jacobian problem that scipy.optimize.least_squares
actually optimizes over.

Every regularized parameter is represented as an *offset* from its nominal
value (see bundle_geometry.py: unpack_shared() is where "offset" turns into
a concrete BundleGeometry): the free variable the solver sees is always
"how far from nominal", with expected value 0, so the ridge penalty
(offset / sigma) reads directly as "how many prior-stddevs away from
expected is this".

BundleCalibrationProblem ties a BundleGeometry parametrization, a set of
regularization priors, and the measured data together into the
residual(x)/jacobian(x) pair least_squares needs - it does its own scaling
(dividing by sigma_field_mT / the per-parameter sigmas) rather than letting
scipy do it, since `x_scale='jac'` is a different, complementary kind of
scaling (of the trust-region step, not of the residual itself).
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass, field

import numpy as np
import scipy.sparse as sp
from numpy.typing import NDArray

from .bundle_geometry import (
    N_MAGNETS,
    N_POSE_PARAMS,
    N_SHARED_PARAMS,
    full_jacobian,
    unpack_shared,
)


@dataclass(frozen=True)
class RegularizationSigmas:
    """Expected stddev of each regularized parameter's offset from nominal.

    These are priors, not measurements - tune them to reflect how much
    you actually trust the nominal/CAD geometry for each quantity.
    """

    magnet_pos_mm: float = 0.2
    magnet_rotation_rad: float = 0.03
    # gain[0,0] is fixed (not regularized - see bundle_geometry.py); the
    # remaining diagonal entries ([1,1], [2,2]) and off-diagonal (cross-talk)
    # entries get separate priors since they mean different things.
    gain_diag: float = 0.05
    gain_offdiag: float = 0.05
    # TODO derive better sensor gain / magnet strength stddevs from datasheets
    magnet_strength: float = 0.05

    def as_vector(self) -> NDArray[np.float64]:
        # Order must match bundle_geometry.unpack_shared's per-pair layout:
        # magnet_pos(3), magnet axis tilt(2 - not 3, see bundle_geometry.py),
        # gain free entries in [0,1] [0,2] [1,1] [1,2] [2,2] order,
        # magnet_strength(1).
        per_pair = np.array(
            [self.magnet_pos_mm] * 3
            + [self.magnet_rotation_rad] * 2
            + [self.gain_offdiag, self.gain_offdiag, self.gain_diag, self.gain_offdiag, self.gain_diag]
            + [self.magnet_strength]
        )
        return np.tile(per_pair, N_MAGNETS)


@dataclass
class BundleCalibrationProblem:
    """Wraps measured data + priors into least_squares' residual(x)/jacobian(x)."""

    measured: NDArray[np.float64]  # (n_frames, 9)
    sigma_field_mT: float = 0.5
    sigmas: RegularizationSigmas = field(default_factory=RegularizationSigmas)
    workers: Callable[[Callable, Iterable], Iterable] = map

    @property
    def n_frames(self) -> int:
        return len(self.measured)

    def _split(self, x: NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        shared = x[:N_SHARED_PARAMS]
        frame_params = x[N_SHARED_PARAMS:].reshape(self.n_frames, N_POSE_PARAMS)
        return shared, frame_params

    def residual(self, x: NDArray[np.float64]) -> NDArray[np.float64]:
        shared, frame_params = self._split(x)
        geometry = unpack_shared(shared)
        predicted = geometry.predict_fields(frame_params[:, :3], frame_params[:, 3:])
        field_residuals = ((predicted - self.measured) / self.sigma_field_mT).ravel()
        reg_residuals = shared / self.sigmas.as_vector()
        return np.concatenate([field_residuals, reg_residuals])

    def jacobian(self, x: NDArray[np.float64]) -> sp.spmatrix:
        shared, frame_params = self._split(x)
        field_jac = full_jacobian(
            frame_params[:, :3], frame_params[:, 3:], shared, workers=self.workers
        ) / self.sigma_field_mT

        n_x = len(x)
        reg_jac = sp.hstack([
            sp.diags(1.0 / self.sigmas.as_vector()),
            sp.csr_matrix((N_SHARED_PARAMS, n_x - N_SHARED_PARAMS)),
        ])
        return sp.vstack([field_jac, reg_jac], format="csr")

    def unscale_residual(self, scaled_residual: NDArray[np.float64]) -> NDArray[np.float64]:
        """Convert a residual vector (e.g. scipy's OptimizeResult.fun) back
        into real units (mT for the field part, mm/rad/dimensionless for the
        regularization part) - for presentation only, not performance-sensitive.
        """
        n_field = 9 * self.n_frames
        field_residual_mT = scaled_residual[:n_field] * self.sigma_field_mT
        reg_residual = scaled_residual[n_field:] * self.sigmas.as_vector()
        return np.concatenate([field_residual_mT, reg_residual])
