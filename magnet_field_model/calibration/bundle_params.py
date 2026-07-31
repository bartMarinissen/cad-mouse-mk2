"""Packing calibration parameters into/out of the flat vector
scipy.optimize.least_squares works with.

Every regularized parameter is represented as an *offset* from its
nominal value (see bundle_geometry.py): the free variable the solver
sees is always "how far from nominal", with expected value 0, so the
ridge penalty (offset / sigma) reads directly as "how many prior-stddevs
away from expected is this".
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from .bundle_geometry import MAGNET_POS_NOMINAL_KNOB, N_MAGNETS, SENSOR_POS_NOMINAL, BundleGeometry

# Layout of the 30 shared calibration offsets, per magnet/sensor pair (x3):
#   3 sensor position offset (mm) + 3 magnet position offset (mm)
#   + 3 magnet rotation offset (rotation vector, rad) + 1 gain offset
N_SHARED_PARAMS_PER_PAIR = 10
N_SHARED_PARAMS = N_SHARED_PARAMS_PER_PAIR * N_MAGNETS


# TODO: 
"""
We should also build a class here that holds a BundleGeomety, a RegularizationsSigma, and the expected std-dev for the magnets.

This will be responsible for the `residual` and `jacobian` arguments passed to scipy leastsquares.

Hence it should allow for full residual calculation.
this should include scaling everything to their expected std-dev. (we won't let scipy do that, we'll let it use x_scale='jac).
It should also include the regularization penalties.

The class should also allow for full jacobian calculation (including regularization penalties)

Finally, consider if there is added value in allowing this to compute jac_sparsity for the 

It should also have a way to get the residual in real units, unscaled by the stddevs. 
Just for final examination and presentation, not performance critical at all.
We should also be able to do this unscaling for the result from scipy-s least-squares.

"""

@dataclass(frozen=True)
class RegularizationSigmas:
    """Expected stddev of each regularized parameter's offset from nominal.

    These are priors, not measurements - tune them to reflect how much
    you actually trust the nominal/CAD geometry for each quantity.
    """

    sensor_pos_mm: float = 0.1
    magnet_pos_mm: float = 0.3
    magnet_rotation_rad: float = 0.03
    # TODO derive better sensor gain stddev from datasheets
    gain: float = 0.05

    def as_vector(self) -> NDArray[np.float64]:
        per_pair = np.array(
            [self.sensor_pos_mm] * 3
            + [self.magnet_pos_mm] * 3
            + [self.magnet_rotation_rad] * 3
            + [self.gain]
        )
        return np.tile(per_pair, N_MAGNETS)


def unpack_shared(x_shared: NDArray[np.float64]) -> BundleGeometry:
    """Build a concrete BundleGeometry from the 30 shared calibration offsets."""
    pairs = x_shared.reshape(N_MAGNETS, N_SHARED_PARAMS_PER_PAIR)
    sensor_pos_offset = pairs[:, 0:3]
    magnet_pos_offset = pairs[:, 3:6]
    magnet_rotvec_offset = pairs[:, 6:9]
    gain_offset = pairs[:, 9]

    return BundleGeometry(
        sensor_pos=SENSOR_POS_NOMINAL + sensor_pos_offset,
        magnet_pos_knob=MAGNET_POS_NOMINAL_KNOB + magnet_pos_offset,
        magnet_rotation=Rotation.from_rotvec(magnet_rotvec_offset),
        gain=1.0 + gain_offset,
    )
