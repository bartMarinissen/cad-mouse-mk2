"""Pure nominal/CAD geometry - no calibration-parameter concepts here.

Deliberately a leaf module: it imports nothing else from this package, so
both `bundle_geometry.py` (the forward model) and `parameterization.py` (the
calibration parameter layout, which needs these nominal positions to build
the magnet-position gauge basis - see MAGNET_POS_BASIS there) can depend on
it without a circular import.

Mirrors firmware/include/magnet_model/positions.h.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray

N_MAGNETS = 3
N_SENSORS = 3
N_POSE_PARAMS = 6  # translation (3) + rotation vector (3)

_SQRT3 = math.sqrt(3.0)
_TRIANGLE_SIDE_MM = 28.58
_TRIANGLE_R_MM = _TRIANGLE_SIDE_MM / _SQRT3
_MAGNET_Z_FROM_PIVOT_MM = 14.0
_MAGNET_REST_DISTANCE_MM = 6.0

# World frame: origin at bundle center, +X right, +Y towards USB-C, sensors
# in the z=0 plane. Fixed - not a calibration parameter. Real sensor
# placement error is absorbed by the magnet position offsets instead (the
# two are related by a per-frame pose anyway).
SENSOR_POS: NDArray[np.float64] = np.array([
    [0.0, -_TRIANGLE_R_MM, 0.0],                                # sensor 1: bottom center
    [-_TRIANGLE_R_MM * _SQRT3 / 2, _TRIANGLE_R_MM * 0.5, 0.0],  # sensor 2: top left
    [_TRIANGLE_R_MM * _SQRT3 / 2, _TRIANGLE_R_MM * 0.5, 0.0],   # sensor 3: top right
])

# Knob-frame magnet positions (BOTTOM-FACE reference, matching positions.h -
# see local_field.py for why the bottom face, not the center, is the origin).
MAGNET_POS_NOMINAL_KNOB: NDArray[np.float64] = SENSOR_POS.copy()
MAGNET_POS_NOMINAL_KNOB[:, 2] = -_MAGNET_Z_FROM_PIVOT_MM

# Seed for the per-frame pose solve: the pivot sits this far above the sensor
# plane so each magnet's bottom face is ~_MAGNET_REST_DISTANCE_MM above its
# sensor. Only an initial guess, never assumed exact.
APPROX_REST_T_MM: NDArray[np.float64] = np.array(
    [0.0, 0.0, _MAGNET_Z_FROM_PIVOT_MM + _MAGNET_REST_DISTANCE_MM]
)
