"""Tests for the calibration parameter layout itself (parameterization.py).

The gain/strength gauge fix rests entirely on GAIN_BASIS having zero trace;
see calibration/parameterization.py's module docstring for the full gauge
argument, and tests/test_calibration_algorithm.py for its consequences under
a real fit.
"""

from __future__ import annotations

import numpy as np

from calibration.bundle_geometry import GAIN_BASIS


def test_gain_basis_is_exactly_traceless():
    """The whole gauge fix rests on this: GAIN_BASIS must have zero trace in
    every row, so that no combination of the free gain parameters can move
    det(G) away from 1 to first order. parameterization._gain_basis() asserts
    this itself at import time; this test pins it down independently too."""
    assert np.allclose(np.einsum("kaa->k", GAIN_BASIS), 0.0)
