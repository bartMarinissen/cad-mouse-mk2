"""Tests for the fitted-geometry -> firmware-constants export."""

from __future__ import annotations

import re
import shutil
import struct
import subprocess
import zlib

import numpy as np
import pytest

from calibration.bundle_geometry import (
    GROUP_SLICES,
    MAGNET_POLARITY,
    N_SHARED_PARAMS,
    NOMINAL_GEOMETRY,
    BundleGeometry,
)
from calibration.export import (
    BLOB_MAGIC,
    BLOB_PAYLOAD_FLOATS,
    BLOB_SIZE,
    BLOB_VERSION,
    firmware_magnet_strength_mT,
    firmware_sensor_gain,
    firmware_sensor_offset,
    format_binary,
    format_cpp,
)
from calibration.firmware_struct import (
    CALIBRATION_PARAMS_H,
    FIELD_NAMES,
    PAYLOAD_SIZE,
    ffi,
    struct_source,
)
from calibration.local_field import MAGNET_POLARIZATION_MT


def _unpack_blob(blob: bytes) -> tuple[int, np.ndarray, int]:
    """Split a calibration blob the way the firmware's deserialize() does."""
    version = struct.unpack_from("<I", blob, 4)[0]
    payload = np.array(
        struct.unpack_from(f"<{BLOB_PAYLOAD_FLOATS}f", blob, 8), dtype=np.float32
    )
    crc = struct.unpack_from("<I", blob, BLOB_SIZE - 4)[0]
    return version, payload, crc


def _blob_fields(payload: np.ndarray) -> dict[str, np.ndarray]:
    """Payload floats back into named fields, in the documented order."""
    return {
        "sensor_gain": payload[0:27].reshape(3, 3, 3),
        "sensor_offset_mT": payload[27:36].reshape(3, 3),
        "magnet_pos_knob": payload[36:45].reshape(3, 3),
        "magnet_rotation": payload[45:72].reshape(3, 3, 3),
        "magnet_strength_mT": payload[72:75],
    }


def test_firmware_gain_is_the_inverse():
    """The firmware applies gain to the measurement, the fit applies it to the
    model, so the exported matrix is the inverse of the fitted one - with the
    installed magnet polarity folded back in."""
    rng = np.random.default_rng(6)
    geom = BundleGeometry.from_shared(rng.normal(0.0, 0.03, N_SHARED_PARAMS))
    fw = firmware_sensor_gain(geom)
    for i in range(3):
        assert np.allclose(fw[i] @ (MAGNET_POLARITY * geom.gain[i]), np.eye(3), atol=1e-10)


def test_nominal_gain_is_identity_and_polarity_carries_the_sign():
    """Nominal gain is +I, with the raw sensors' sign flip carried by the
    magnet's installed polarity instead. What matters is that the exported
    firmware gain still lands near -I, matching Config::magnet_gains - getting
    that sign wrong is what made the previous calibrator unfittable."""
    assert np.allclose(NOMINAL_GEOMETRY.gain, np.eye(3)[None, :, :])
    assert MAGNET_POLARITY == -1.0
    assert np.allclose(firmware_sensor_gain(NOMINAL_GEOMETRY), -np.eye(3)[None, :, :])


def test_firmware_offset_mapping():
    """o_fw = G_fw @ o_fit, i.e. the offset transforms into read_mT's space."""
    rng = np.random.default_rng(11)
    x = np.zeros(N_SHARED_PARAMS)
    x[GROUP_SLICES["sensor_offset"]] = rng.normal(0.0, 1.0, 9)
    geom = BundleGeometry.from_shared(x)
    expected = np.einsum("iab,ib->ia", firmware_sensor_gain(geom), geom.sensor_offset)
    assert np.allclose(firmware_sensor_offset(geom), expected)


def test_nominal_strength_is_local_field_polarization():
    """geometry.magnet_strength == 1.0 (nominal) means exactly
    local_field.py's own modelled polarization - so the exported mT value at
    nominal must equal that polarization's magnitude, not some other
    constant (e.g. NOT firmware/bicubic_table.py's unrelated
    BICUBIC_FIELD_REFERENCE_MT, which this function must not need to know
    about at all)."""
    assert np.allclose(NOMINAL_GEOMETRY.magnet_strength, 1.0)
    expected = abs(MAGNET_POLARIZATION_MT[2])
    assert np.allclose(firmware_magnet_strength_mT(NOMINAL_GEOMETRY), expected)


def test_strength_mT_scales_linearly_with_the_fitted_multiplier():
    """A magnet fitted at k times the nominal multiplier must report k times
    the nominal mT - the whole point of exporting a real physical unit is
    that this relationship has no hidden gauge dependence."""
    rng = np.random.default_rng(23)
    x = np.zeros(N_SHARED_PARAMS)
    x[GROUP_SLICES["magnet_strength_mean"]] = rng.normal(0.0, 0.05)
    x[GROUP_SLICES["magnet_strength_diff"]] = rng.normal(0.0, 0.02, 2)
    geom = BundleGeometry.from_shared(x)
    nominal_mt = abs(MAGNET_POLARIZATION_MT[2])
    assert np.allclose(firmware_magnet_strength_mT(geom), nominal_mt * geom.magnet_strength)


def test_format_cpp_emits_every_field_in_struct_order():
    """The snippet is now one aggregate initializer, so its float literals are
    the struct's fields in declaration order. Parsing them back and slicing by
    field catches a dropped magnet, a field emitted in the wrong place, or a
    matrix written column-major - none of which the C++ compiler would notice,
    since every arrangement is a valid initializer for the same POD type."""
    rng = np.random.default_rng(31)
    geom = BundleGeometry.from_shared(rng.normal(scale=0.02, size=N_SHARED_PARAMS))
    cpp = format_cpp(geom)

    assert "CalibrationParams fittedCalibration() {" in cpp
    assert "return CalibrationParams{" in cpp
    assert cpp.rstrip().endswith("}")

    # The "f" suffix is what separates real literals from the offsets-from-
    # nominal figures in the comment above magnet_pos_knob.
    literals = [float(v) for v in re.findall(r"(-?\s*\d+\.\d+)f", cpp)]
    assert len(literals) == 75, f"expected 75 float literals, found {len(literals)}"

    values = np.array(literals)
    expected = np.concatenate([
        np.asarray(firmware_sensor_gain(geom)).ravel(),
        np.asarray(firmware_sensor_offset(geom)).ravel(),
        np.asarray(geom.magnet_pos_knob).ravel(),
        np.asarray(geom.magnet_rotation.as_matrix()).ravel(),
        np.asarray(firmware_magnet_strength_mT(geom)).ravel(),
    ])
    # 1e-5 because the snippet prints six decimals; this is about ordering.
    assert np.allclose(values, expected, atol=1e-5)


def test_format_cpp_and_format_binary_agree():
    """Both exports render the same calibration, so their numbers must match
    field for field. format_cpp is lossy at six decimals, hence the tolerance -
    what is being pinned here is that the two orderings are the same one."""
    rng = np.random.default_rng(37)
    geom = BundleGeometry.from_shared(rng.normal(scale=0.02, size=N_SHARED_PARAMS))

    from_text = np.array(
        [float(v) for v in re.findall(r"(-?\s*\d+\.\d+)f", format_cpp(geom))]
    )
    _, from_binary, _ = _unpack_blob(format_binary(geom))
    assert np.allclose(from_text, from_binary, atol=1e-5)


# --- /calibration.bin blob. The other end of this contract is
# --- firmware/include/CalibrationStorage.h; nothing but these tests checks it.


def test_crc32_matches_the_standard_check_value():
    """The firmware reimplements CRC-32 bitwise, so both ends have to agree on
    exactly which CRC-32 this is. 0xCBF43926 over b"123456789" is the published
    check value for CRC-32/ISO-HDLC (reflected, poly 0xEDB88320, init and final
    XOR 0xFFFFFFFF) - the one zlib computes. If the firmware's crc32() does not
    produce this for the same input, the two implementations are not the same
    algorithm and no stored calibration will ever validate."""
    assert zlib.crc32(b"123456789") & 0xFFFFFFFF == 0xCBF43926


def test_blob_has_the_documented_size_and_header():
    blob = format_binary(NOMINAL_GEOMETRY)
    assert len(blob) == BLOB_SIZE
    assert blob[:4] == BLOB_MAGIC == b"CMK2"
    version, payload, _ = _unpack_blob(blob)
    assert version == BLOB_VERSION
    assert payload.size == BLOB_PAYLOAD_FLOATS == 75


def test_blob_crc_covers_the_header_and_payload():
    """The CRC is over everything preceding it, not just the payload - the
    firmware checks crc32(blob, 308) against the last four bytes, so a magic or
    version corrupted in flash has to fail the checksum too."""
    blob = format_binary(NOMINAL_GEOMETRY)
    _, _, crc = _unpack_blob(blob)
    assert crc == zlib.crc32(blob[: BLOB_SIZE - 4]) & 0xFFFFFFFF


def test_corrupting_any_region_breaks_the_crc():
    blob = format_binary(NOMINAL_GEOMETRY)
    stored = struct.unpack_from("<I", blob, BLOB_SIZE - 4)[0]

    # One byte from the magic, the version, and three spread through the
    # payload - every region the checksum is supposed to be covering.
    for offset in (0, 5, 8, 150, BLOB_SIZE - 5):
        corrupted = bytearray(blob)
        corrupted[offset] ^= 0xFF
        recomputed = zlib.crc32(bytes(corrupted[: BLOB_SIZE - 4])) & 0xFFFFFFFF
        assert recomputed != stored, f"flipping byte {offset} left the CRC unchanged"


def test_blob_matrices_are_row_major():
    """Row-major is the load-bearing convention: the firmware serializes
    through m(r, c) accessors specifically so this side can stay a plain
    .ravel() regardless of which C++ matrix library reads it (Eigen stored
    Matrix3f column-major; BLA's Matrix -- TODO/eigen-to-bla-migration.md --
    happens to be row-major natively, but the accessor-based serialization
    doesn't depend on that). A transposed gain matrix would still pass the
    CRC and still look plausible, so nothing downstream would catch it -
    only this test would."""
    rng = np.random.default_rng(53)
    geom = BundleGeometry.from_shared(rng.normal(scale=0.02, size=N_SHARED_PARAMS))
    _, payload, _ = _unpack_blob(format_binary(geom))

    gain = firmware_sensor_gain(geom)
    # Element [0][1] must land at index 1, where a column-major writer would
    # have put element [1][0] instead.
    assert payload[1] == np.float32(gain[0][0][1])
    assert payload[3] == np.float32(gain[0][1][0])
    # And each magnet's 9 floats are contiguous, not interleaved.
    assert np.array_equal(payload[9:18], np.asarray(gain[1], dtype=np.float32).ravel())


def test_blob_round_trips_to_the_same_numbers_format_cpp_prints():
    """The binary and text exports must be two renderings of one calibration.
    They share the firmware_* helpers, so this is really checking that the
    payload order and field slicing match what the firmware will read back."""
    rng = np.random.default_rng(67)
    geom = BundleGeometry.from_shared(rng.normal(scale=0.02, size=N_SHARED_PARAMS))
    _, payload, _ = _unpack_blob(format_binary(geom))
    fields = _blob_fields(payload)

    expected = {
        "sensor_gain": firmware_sensor_gain(geom),
        "sensor_offset_mT": firmware_sensor_offset(geom),
        "magnet_pos_knob": geom.magnet_pos_knob,
        "magnet_rotation": geom.magnet_rotation.as_matrix(),
        "magnet_strength_mT": firmware_magnet_strength_mT(geom),
    }
    for name, want in expected.items():
        # Exact, not approximate: struct.pack("<f") and np.float32 round
        # identically, so any difference here is a layout bug rather than
        # precision loss.
        assert np.array_equal(fields[name], np.asarray(want, dtype=np.float32)), name


def test_nominal_blob_is_pinned():
    """A regression pin on the whole format: the stored CRC changes if the
    layout, the field order, the version, or the nominal geometry changes. The
    first three silently break every device with a stored calibration, so they
    should not move without a version bump and a deliberate update here.

    Pinning the *stored* CRC rather than crc32 over the whole 312 bytes is
    deliberate. A message with its own CRC-32 appended always checksums to the
    residue constant 0x2144DF1C, whatever the message was, so pinning that
    would assert nothing about the content."""
    blob = format_binary(NOMINAL_GEOMETRY)
    _, _, stored = _unpack_blob(blob)
    assert stored == 0xE03758E3
    assert zlib.crc32(blob) & 0xFFFFFFFF == 0x2144DF1C  # the residue, for contrast


# --- The layout, and the checks that both ends of it agree. ---


def test_struct_source_extracts_the_real_declaration():
    """The layout is read out of the firmware header, so the extraction itself
    is load-bearing. A header reformat that broke it into something still
    parseable but wrong would silently change every blob this produces."""
    assert CALIBRATION_PARAMS_H.is_file(), f"header not found at {CALIBRATION_PARAMS_H}"

    source = struct_source()
    assert source.startswith("struct CalibrationParams {")
    assert source.rstrip().endswith("};")
    assert "//" not in source, "comments should be stripped before reaching cdef"
    for field in FIELD_NAMES:
        assert field in source, f"{field} missing from the extracted declaration"


def test_struct_source_refuses_a_header_without_the_struct(tmp_path):
    """Failing loudly is the point - a silent fallback to a built-in copy would
    reintroduce exactly the duplication this module removes."""
    empty = tmp_path / "CalibrationParams.h"
    empty.write_text("#pragma once\n// nothing here\n")
    with pytest.raises(ValueError, match="no `struct CalibrationParams"):
        struct_source(empty)

    with pytest.raises(FileNotFoundError):
        struct_source(tmp_path / "does_not_exist.h")


def test_payload_layout_is_pinned():
    """The offsets the format depends on. Inserting or reordering a field
    changes these, and would otherwise only show up as a device that loads a
    valid-CRC blob into the wrong values."""
    assert PAYLOAD_SIZE == 300
    assert FIELD_NAMES == (
        "sensor_gain",
        "sensor_offset_mT",
        "magnet_pos_knob",
        "magnet_rotation",
        "magnet_strength_mT",
    )
    assert {name: ffi.offsetof("struct CalibrationParams", name) for name in FIELD_NAMES} == {
        "sensor_gain": 0,
        "sensor_offset_mT": 108,
        "magnet_pos_knob": 144,
        "magnet_rotation": 180,
        "magnet_strength_mT": 288,
    }


# Appended to the extracted declaration so the compiler confirms the size the
# format depends on, rather than the test asserting it separately.
_CPP_SIZE_ASSERT = (
    f'\nstatic_assert(sizeof(CalibrationParams) == {PAYLOAD_SIZE},'
    ' "payload size is the format");\n'
)

_CPP_MAIN = """
#include <cstdio>
int main() {
  CalibrationParams c = fittedCalibration();
  fwrite(&c, 1, sizeof(c), stdout);
  return 0;
}
"""


@pytest.mark.skipif(shutil.which("g++") is None, reason="needs a host C++ compiler")
def test_cpp_struct_layout_matches_the_blob_payload(tmp_path):
    """Compile the emitted snippet and compare the struct's own bytes against
    format_binary()'s payload.

    This is the only check that both ends of the format agree. The firmware
    reads /calibration.bin with a memcpy straight into CalibrationParams, so
    everything rests on Python laying the floats out in exactly the order the
    C++ struct declares them - and a wrong order still produces a valid blob
    with a valid CRC that loads into plausible-looking garbage.

    Both sides are now derived from the same header - Python's payload via
    cffi, this program via struct_source() - so what is really being checked
    is that cffi's ABI model agrees with a real compiler's. That is worth
    checking, and it is why this test survives the move to cffi.

    Only possible because the struct is plain arrays: with matrix-library
    (formerly Eigen, now BLA -- see TODO/eigen-to-bla-migration.md) members
    it would be neither trivially copyable nor host-compilable standalone.
    """
    rng = np.random.default_rng(71)
    geom = BundleGeometry.from_shared(rng.normal(scale=0.02, size=N_SHARED_PARAMS))

    source = tmp_path / "layout.cpp"
    source.write_text(struct_source() + _CPP_SIZE_ASSERT + format_cpp(geom) + _CPP_MAIN)
    binary = tmp_path / "layout"

    subprocess.run(
        ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-o", str(binary), str(source)],
        check=True, capture_output=True,
    )
    from_cpp = np.array(
        struct.unpack(
            f"<{BLOB_PAYLOAD_FLOATS}f",
            subprocess.run([str(binary)], check=True, capture_output=True).stdout,
        )
    )

    _, from_python, _ = _unpack_blob(format_binary(geom))

    # atol covers format_cpp's six printed decimals - the snippet is a lossy
    # rendering of the same numbers. Ordering has no tolerance: a transposed
    # matrix or a swapped field lands whole units away, not 1e-6.
    assert np.allclose(from_cpp, from_python, atol=1e-5)
