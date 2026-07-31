"""Physical geometry of the 3-magnet / 3-sensor bundle, and the forward
model that predicts sensor readings for a given knob pose.

Nominal geometry mirrors firmware/include/magnet_model/positions.h. The
magnet itself (6mm x 6mm cylinder, ~N42 remanence) mirrors the one built
in field_approximation.ipynb. Both are kept in sync by hand, not code -
if either changes, this needs updating too.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import magpylib as magpy
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

N_MAGNETS = 3

# --- The physical magnet, from field_approximation.ipynb ---
MAGNET_DIMENSION_MM = (6.0, 6.0)  # (diameter, height)
MAGNET_POLARIZATION_MT = (0.0, 0.0, -600.0)  # ~N42 remanence, along the magnet's own local +z

# --- Nominal bundle geometry, mirrors firmware/include/magnet_model/positions.h ---
_SQRT3 = math.sqrt(3.0)
_TRIANGLE_SIDE_MM = 28.58
_TRIANGLE_R_MM = _TRIANGLE_SIDE_MM / _SQRT3
_MAGNET_Z_FROM_PIVOT_MM = 14.0
_MAGNET_REST_DISTANCE_MM = 6.0

# World frame: origin at bundle center, +X right, +Y towards USB-C, sensors in the z=0 plane.
SENSOR_POS_NOMINAL: NDArray[np.float64] = np.array([
    [0.0, -_TRIANGLE_R_MM, 0.0],                                # sensor 1: bottom center
    [-_TRIANGLE_R_MM * _SQRT3 / 2, _TRIANGLE_R_MM * 0.5, 0.0],  # sensor 2: top left
    [_TRIANGLE_R_MM * _SQRT3 / 2, _TRIANGLE_R_MM * 0.5, 0.0],   # sensor 3: top right
])

# Knob frame: magnets assumed to sit directly above their sensor at rest.
MAGNET_POS_NOMINAL_KNOB: NDArray[np.float64] = SENSOR_POS_NOMINAL.copy()
MAGNET_POS_NOMINAL_KNOB[:, 2] = -_MAGNET_Z_FROM_PIVOT_MM

# A reasonable initial guess for the knob's rest pose in world coordinates.
# Not a firmware constant verbatim - derived from the two constants above
# (the knob pivot sits this far above the sensor plane so that, upright,
# each magnet ends up ~_MAGNET_REST_DISTANCE_MM above its sensor). Only
# used to seed the per-frame pose solve, never assumed to be exact.
APPROX_REST_T_MM: NDArray[np.float64] = np.array(
    [0.0, 0.0, _MAGNET_Z_FROM_PIVOT_MM + _MAGNET_REST_DISTANCE_MM]
)


@dataclass(frozen=True)
class BundleGeometry:
    """A concrete bundle geometry (nominal values with calibration offsets applied).
    
    This stores all our callibration parameters. And is the place where we actually get our 
    """

    # These are all the callibration parameters
    # TODO: just drop sensor position for now
    sensor_pos: NDArray[np.float64]       # (3, 3) world-frame sensor positions, mm
    magnet_pos_knob: NDArray[np.float64]  # (3, 3) knob-frame magnet positions, mm
    magnet_rotation: Rotation             # length-3 Rotation: magnet axis offset from the knob frame
    # TODO: we want cross talk on the axis included in this. The actual gains should be a skew-only matrix per sensor.
    gain: NDArray[np.float64]             # (3,) per-sensor multiplicative gain
    # TODO: add magnet strength as a callibration parameter - note not in magpy just as a multiplier on the result

    # TODO: consider vectorizing this function so it will take many t and rotvecs 
    def predict_field(self, t: NDArray[np.float64], rotvec: NDArray[np.float64]) -> NDArray[np.float64]:
        """Predict the 9 field components (sensor1 xyz, sensor2 xyz, sensor3 xyz) for a knob pose.

        t is the knob pivot's translation in world coordinates (mm);
        rotvec is its orientation as a rotation vector (rad).
        """
        r_knob = Rotation.from_rotvec(rotvec)
        field = np.empty(3 * N_MAGNETS)
        # TODO: consider using a magpy scnenario, potentially one that is pre-assembled in the class
        for i in range(N_MAGNETS):
            magnet_world_pos = t + r_knob.apply(self.magnet_pos_knob[i])
            magnet_world_rot = r_knob * self.magnet_rotation[i]
            
            magnet = magpy.magnet.Cylinder(
                polarization=MAGNET_POLARIZATION_MT,
                dimension=MAGNET_DIMENSION_MM,
                position=magnet_world_pos,
                orientation=magnet_world_rot,
            )
            field[3 * i : 3 * i + 3] = self.gain[i] * magnet.getB(self.sensor_pos[i])
        return field

    # TODO: implement
    def predict_fields(self, t: NDArray[np.float64], rotvec: NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """ a (more efficent?) way to get all field predictions given a set of poses for all frames
        """

    # TODO: implement this
    def one_frame_jacobian(self, t: NDArray[np.float64], rotvec: NDArray[np.float64])  -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """A finite differences jacobian implementation for a single frame.

        This differentiates w.r.t to both the pose and the callibration parameters.
        both are returned as separate Jacobians for easier full assembly later
        """
        pass

    #TODO: implement this after implementing the above.
    def full_jacobian(self, ts, rotvecs) -> NDArray[np.float64]:
        """ Assembles the full jacobian of the fields with respect to ts, rotvecs, and the callibration parameters.

        Uses the above `one_frame_jacobian` function to assemble this matrix from parts.
        
        
        """
