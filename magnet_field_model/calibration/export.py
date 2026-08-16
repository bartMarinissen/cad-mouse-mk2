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

import struct
import zlib

import numpy as np
from numpy.typing import NDArray

from .bundle_geometry import (
    MAGNET_POLARITY,
    MAGNET_POS_NOMINAL_KNOB,
    N_MAGNETS,
    BundleGeometry,
)
from .firmware_struct import PAYLOAD_SIZE, new_params, to_bytes
from .local_field import MAGNET_POLARIZATION_MT

# --- /calibration.bin framing. Mirrors firmware/include/CalibrationStorage.h. ---
#
# Only the wrapper is defined here - magic, version, CRC. The payload's shape
# is not: it is a CalibrationParams, and firmware_struct.py reads that layout
# out of the firmware header itself rather than restating it. So these four
# constants are the entire hand-maintained surface between the two sides.
BLOB_MAGIC = b"CMK2"
BLOB_VERSION = 1
BLOB_HEADER_SIZE = len(BLOB_MAGIC) + 4  # magic + version
BLOB_CRC_SIZE = 4

BLOB_PAYLOAD_SIZE = PAYLOAD_SIZE
BLOB_PAYLOAD_FLOATS = BLOB_PAYLOAD_SIZE // 4
BLOB_SIZE = BLOB_HEADER_SIZE + BLOB_PAYLOAD_SIZE + BLOB_CRC_SIZE


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
    Config::defaultCalibration() and meant to be placed in
    firmware/src/Config.cpp.

    This used to assign each field in turn, because designated initializers
    are C++20 and this firmware does not require that standard. Now that
    CalibrationParams is plain arrays rather than matrix-library types
    (formerly Eigen, now BLA - see TODO/eigen-to-bla-migration.md), ordinary
    (non-designated) nested-brace aggregate initialization does the job and
    has been valid since C++98 - so the emitted snippet is one initializer
    whose shape mirrors the struct, which is easier to eyeball against the
    header than a run of assignments was.

    Still deliberately just text, and no longer the delivery mechanism: the
    device is written over serial or via uploadfs (see format_binary). This
    stays for inspection and diffing, because a snippet you can read is worth
    keeping even when you no longer have to paste it.
    """
    gain = firmware_sensor_gain(geometry)
    offset = firmware_sensor_offset(geometry)
    strength_mt = firmware_magnet_strength_mT(geometry)
    pos = geometry.magnet_pos_knob
    rot = geometry.magnet_rotation.as_matrix()
    off = pos - MAGNET_POS_NOMINAL_KNOB

    def fmt(value: float) -> str:
        return f"{value: .6f}f"

    def vec_block(v: NDArray[np.float64]) -> str:
        return "{" + ", ".join(fmt(x) for x in v) + "}"

    def mat_block(m: NDArray[np.float64], indent: str) -> str:
        """One matrix as {{r0}, {r1}, {r2}}, rows aligned under each other."""
        joiner = ",\n" + indent + " "
        return "{" + joiner.join(vec_block(row) for row in m) + "}"

    body = "          "

    lines = [
        "// ---- generated by magnet_field_model/bundle_callibration.py ----",
        "// Drop-in replacement for Config::defaultCalibration() in",
        "// firmware/src/Config.cpp. see format_cpp()'s docstring.",
        "CalibrationParams fittedCalibration() {",
        "  return CalibrationParams{",
        "      // sensor_gain, applied to the RAW reading in SensorController.",
        "      // (inverse of the fitted model-side gain - see calibration/export.py)",
        "      {",
    ]
    for i in range(N_MAGNETS):
        lines.append(body + mat_block(gain[i], body) + ",")
    lines.append("      },")
    lines.append("")
    lines.append("      // sensor_offset_mT, subtracted after the gain matrix (read_mT's space).")
    lines.append("      {")
    for i in range(N_MAGNETS):
        lines.append(body + vec_block(offset[i]) + ",")
    lines.append("      },")
    lines.append("")
    lines.append("      // magnet_pos_knob (BOTTOM-FACE reference, as in positions.h).")
    lines.append("      // Offsets from nominal: " + ", ".join(
        f"[{o[0]:+.3f} {o[1]:+.3f} {o[2]:+.3f}]" for o in off))
    lines.append("      {")
    for i in range(N_MAGNETS):
        lines.append(body + vec_block(pos[i]) + ",")
    lines.append("      },")
    lines.append("")
    lines.append("      // magnet_rotation, per-magnet axis tilt (knob frame).")
    lines.append("      {")
    for i in range(N_MAGNETS):
        lines.append(body + mat_block(rot[i], body) + ",")
    lines.append("      },")
    lines.append("")
    lines.append("      // magnet_strength_mT, per-magnet polarization (remanence, Br), in mT.")
    lines.append("      // See MagnetModel::magnet_strength_mT / CalibrationParams.h.")
    lines.append("      " + vec_block(strength_mt) + ",")
    lines.append("  };")
    lines.append("}")
    return "\n".join(lines)


def format_binary(geometry: BundleGeometry) -> bytes:
    """The /calibration.bin blob: the same numbers format_cpp() prints as text.

    Framing, from firmware/include/CalibrationStorage.h:

        offset  size  field
        0       4     magic, b"CMK2"
        4       4     version, uint32
        8       300   payload: a CalibrationParams, verbatim
        308     4     crc32 over bytes [0, 308)

    The payload is a real C struct, built through cffi from the declaration in
    firmware/include/CalibrationParams.h (see firmware_struct.py) rather than
    packed by hand. That matters more than it might look: the firmware reads
    this file with a memcpy straight into CalibrationParams, so a field written
    in the wrong order still produces a valid blob with a valid CRC that loads
    into plausible-looking nonsense. Addressing fields by name means the order
    is only ever stated once, in the header.

    Everything little-endian, which is native for both the RP2040 and any host
    that runs this, so neither side byte-swaps. The CRC is stdlib zlib.crc32 -
    standard reflected IEEE 802.3, which the firmware reimplements bitwise.

    Two routes carry these bytes: written to firmware/data/calibration.bin they
    become a LittleFS image for `pio run -t uploadfs`; hex-encoded onto one
    line they become the CAL_UPLOAD serial command. Identical either way.
    """
    params = new_params(
        sensor_gain=np.asarray(firmware_sensor_gain(geometry)).tolist(),
        sensor_offset_mT=np.asarray(firmware_sensor_offset(geometry)).tolist(),
        magnet_pos_knob=np.asarray(geometry.magnet_pos_knob).tolist(),
        magnet_rotation=np.asarray(geometry.magnet_rotation.as_matrix()).tolist(),
        magnet_strength_mT=np.asarray(firmware_magnet_strength_mT(geometry)).tolist(),
    )

    body = BLOB_MAGIC + struct.pack("<I", BLOB_VERSION) + to_bytes(params)
    blob = body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)

    assert len(blob) == BLOB_SIZE, f"blob is {len(blob)} bytes, expected {BLOB_SIZE}"
    return blob
