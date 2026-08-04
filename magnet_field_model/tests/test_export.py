"""Tests for the fitted-geometry -> firmware-constants export."""

from __future__ import annotations

import numpy as np

from calibration.bundle_geometry import (
    GROUP_SLICES,
    MAGNET_POLARITY,
    N_SHARED_PARAMS,
    NOMINAL_GEOMETRY,
    BundleGeometry,
)
from calibration.export import firmware_sensor_gain, firmware_sensor_offset


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
