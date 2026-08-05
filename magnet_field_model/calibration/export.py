"""Turning a fitted BundleGeometry into a firmware CalibrationParams initializer.

The gain convention is the one genuinely subtle part, so it is worth stating
plainly:

    This calibration applies gain to the **model**   :  pred = G_fit @ B_model + o_fit
    The firmware applies gain to the **measurement** :  corrected = G_fw @ raw - o_fw

Both want `corrected ~ B_model`, so with the installed magnet polarity folded
in (see bundle_geometry.MAGNET_POLARITY):

    G_fw = inv(MAGNET_POLARITY * G_fit)      -> lands near -I, matching Config
    o_fw = G_fw @ o_fit

Fitting on the model side is deliberate. Scaling the measurement instead
would put a fitted parameter on the same side of the residual as the noise,
which lets the optimizer shrink the residual by shrinking the gain rather
than by explaining the data. Keeping the measurement untouched keeps the
residual in raw sensor units, which is exactly what ResidualWeights' sigma
model describes - and the DC offset genuinely belongs on that side, since it
is a property of the raw reading rather than of the modelled field.

The other thing worth stating plainly: gain and magnet strength are no longer
entangled the way they used to be. parameterization.py's GAIN_BASIS is built
from 8 exactly-traceless matrices with no isotropic/scale direction at all, so
`geometry.gain` structurally carries no field scale - none to transfer, no
gauge-fixing step needed here. `geometry.magnet_strength` carries all of it,
as a multiplier on local_field.py's own nominal polarization
(`local_field.MAGNET_POLARIZATION_MT`, 600mT) - see
firmware_magnet_strength_mT() for turning that back into a real mT value.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from .bundle_geometry import (
    MAGNET_POLARITY,
    MAGNET_POS_NOMINAL_KNOB,
    N_MAGNETS,
    BundleGeometry,
)
from .local_field import MAGNET_POLARIZATION_MT


def firmware_sensor_gain(geometry: BundleGeometry) -> NDArray[np.float64]:
    """(3, 3, 3) per-sensor gain matrices for SensorController.

    Inverse of the fitted model-side gain, with the installed magnet polarity
    folded back in so the result lands near -I like Config::magnet_gains.
    """
    return np.linalg.inv(MAGNET_POLARITY * geometry.gain)


def firmware_sensor_offset(geometry: BundleGeometry) -> NDArray[np.float64]:
    """(3, 3) per-sensor offsets in read_mT()'s output space, i.e. G_fw @ o_fit."""
    result: NDArray[np.float64] = np.einsum(
        "iab,ib->ia", firmware_sensor_gain(geometry), geometry.sensor_offset
    )
    return result


def firmware_magnet_strength_mT(geometry: BundleGeometry) -> NDArray[np.float64]:
    """(3,) per-magnet polarization (remanence, Br), in mT.

    Feeds firmware/include/CalibrationParams.h's magnet_strength_mT, which
    MagnetModel::evaluate divides by its own BICUBIC_FIELD_REFERENCE_MT
    (magnet_model_table.h) to get the ratio it scales the bicubic table by.

    geometry.magnet_strength is dimensionless, a multiplier on local_field.py's
    own nominal polarization (MAGNET_POLARIZATION_MT, 600mT) - 1.0 means
    "exactly that nominal". Multiplying back in gives the magnet's actual
    fitted Br in mT: a real physical quantity, independent of both that 600mT
    modelling choice (which only exists to give the *fit* a closed-form near
    field to linearize around) and of whatever polarization the firmware's
    bicubic table happens to be generated at (BICUBIC_FIELD_REFERENCE_MT,
    currently 1000mT and arbitrary - see bicubic_table.py). Those two
    reference values are unrelated on purpose; this function's job is only to
    report the truth in mT, and MagnetModel does the rest.
    """
    nominal_mt = abs(MAGNET_POLARIZATION_MT[2])
    result: NDArray[np.float64] = nominal_mt * geometry.magnet_strength
    return result


def format_cpp(geometry: BundleGeometry) -> str:
    """A pasteable C++ CalibrationParams initializer.

    Emits a `fittedCalibration()` function shaped like
    Config::defaultCalibration() (firmware/src/Config.cpp), assigning each
    firmware/include/CalibrationParams.h field in turn - a plain function
    rather than an aggregate initializer, since designated initializers are
    C++20 and this firmware does not require that standard.

    Still deliberately just text. Once the device can be written over serial,
    that becomes the real delivery mechanism and this stays for inspection and
    diffing: a snippet you can read is worth keeping even when you no longer
    have to paste it.
    """
    gain = firmware_sensor_gain(geometry)
    offset = firmware_sensor_offset(geometry)
    strength_mt = firmware_magnet_strength_mT(geometry)
    pos = geometry.magnet_pos_knob
    rot = geometry.magnet_rotation.as_matrix()
    off = pos - MAGNET_POS_NOMINAL_KNOB

    def mat3(m: NDArray[np.float64], indent: str) -> str:
        rows = ",\n".join(
            indent + "    " + ", ".join(f"{v: .6f}f" for v in row) for row in m
        )
        return "(Mat3() <<\n" + rows + "\n" + indent + "  ).finished()"

    def vec3(v: NDArray[np.float64]) -> str:
        return "Vec3(" + ", ".join(f"{x: .6f}f" for x in v) + ")"

    lines = [
        "// ---- generated by magnet_field_model/bundle_callibration.py ----",
        "// Drop-in replacement for Config::defaultCalibration() in",
        "// firmware/src/Config.cpp.",
        "CalibrationParams fittedCalibration() {",
        "  CalibrationParams cal{};",
        "",
        "  // Per-sensor gain applied to the RAW reading in SensorController.",
        "  // (inverse of the fitted model-side gain - see calibration/export.py)",
    ]
    for i in range(N_MAGNETS):
        lines.append(f"  cal.sensor_gain[{i}] = " + mat3(gain[i], "  ") + ";")
    lines.append("")
    lines.append("  // Per-sensor DC offset, subtracted after the gain matrix (read_mT's space).")
    for i in range(N_MAGNETS):
        lines.append(f"  cal.sensor_offset_mT[{i}] = " + vec3(offset[i]) + ";")
    lines.append("")
    lines.append("  // Knob-frame magnet positions (BOTTOM-FACE reference, as in positions.h).")
    lines.append("  // Offsets from nominal: " + ", ".join(
        f"[{o[0]:+.3f} {o[1]:+.3f} {o[2]:+.3f}]" for o in off))
    for i in range(N_MAGNETS):
        lines.append(f"  cal.magnet_pos_knob[{i}] = " + vec3(pos[i]) + ";")
    lines.append("")
    lines.append("  // Per-magnet axis tilt (knob frame).")
    for i in range(N_MAGNETS):
        lines.append(f"  cal.magnet_rotation[{i}] = " + mat3(rot[i], "  ") + ";")
    lines.append("")
    lines.append("  // Per-magnet polarization (remanence, Br), in mT. See")
    lines.append("  // MagnetModel::magnet_strength_mT / CalibrationParams.h.")
    for i in range(N_MAGNETS):
        lines.append(f"  cal.magnet_strength_mT[{i}] = {strength_mt[i]:.6f}f;")
    lines.append("")
    lines.append("  return cal;")
    lines.append("}")
    return "\n".join(lines)
