"""Validate the analytic Jacobian against central finite differences.

The Python counterpart of firmware/test/test_jacobian.cpp, and the safety net
for every closed-form derivative in bundle_geometry.py. Run it after touching
anything in bundle_geometry.py or local_field.py.

    uv run python -m pytest tests/ -q
"""

from __future__ import annotations

import numpy as np
import pytest

from calibration.bundle_geometry import (
    N_POSE_PARAMS,
    N_SHARED_PARAMS,
    APPROX_REST_T_MM,
    unpack_shared,
)
from calibration.local_field import local_field_and_gradient, _SOURCE


def _central_diff(fn, x0, h=1e-6):
    f0 = np.asarray(fn(x0))
    jac = np.empty(f0.shape + (len(x0),))
    for i in range(len(x0)):
        step = h * max(1.0, abs(x0[i]))
        xp, xm = np.array(x0, dtype=float), np.array(x0, dtype=float)
        xp[i] += step
        xm[i] -= step
        jac[..., i] = (np.asarray(fn(xp)) - np.asarray(fn(xm))) / (2 * step)
    return jac


def _sample_poses(rng, n):
    ts = APPROX_REST_T_MM + rng.uniform(-1.5, 1.5, (n, 3))
    rotvecs = rng.uniform(-0.25, 0.25, (n, 3))
    return ts, rotvecs


def test_local_field_matches_magpylib():
    """The (r, z) decomposition must reproduce magpylib's own field exactly."""
    rng = np.random.default_rng(1)
    pts = np.stack([
        rng.uniform(-8, 8, 200),
        rng.uniform(-8, 8, 200),
        rng.uniform(-20, -2, 200),
    ], axis=-1)
    b, _ = local_field_and_gradient(pts)
    assert np.allclose(b, _SOURCE.getB(pts), rtol=1e-10, atol=1e-10)


def test_local_field_gradient():
    """Local gradient vs. brute-force 3D finite differences, including on-axis."""
    rng = np.random.default_rng(2)
    pts = np.stack([
        rng.uniform(-8, 8, 60),
        rng.uniform(-8, 8, 60),
        rng.uniform(-20, -2, 60),
    ], axis=-1)
    # on-axis points exercise the r -> 0 limit branch
    pts = np.concatenate([pts, np.stack([
        np.zeros(5), np.zeros(5), np.linspace(-18, -3, 5)], axis=-1)])

    _, j = local_field_and_gradient(pts)
    h = 1e-5
    j_ref = np.empty_like(j)
    for b in range(3):
        d = np.zeros(3)
        d[b] = h
        j_ref[..., :, b] = (_SOURCE.getB(pts + d) - _SOURCE.getB(pts - d)) / (2 * h)
    scale = np.maximum(np.abs(j_ref).max(), 1.0)
    assert np.abs(j - j_ref).max() / scale < 1e-6


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_pose_jacobian(seed):
    """d(prediction)/d(pose) against finite differences, at a perturbed geometry."""
    rng = np.random.default_rng(seed)
    x_shared = rng.normal(0.0, 0.03, N_SHARED_PARAMS)
    geom = unpack_shared(x_shared)
    ts, rotvecs = _sample_poses(rng, 4)

    _, j_pose, _ = geom.predict_and_jacobians(ts, rotvecs)

    for k in range(len(ts)):
        ref = _central_diff(
            lambda p: geom.predict(p[None, :3], p[None, 3:])[0],
            np.concatenate([ts[k], rotvecs[k]]),
        )
        scale = max(np.abs(ref).max(), 1.0)
        assert np.abs(j_pose[k] - ref).max() / scale < 2e-5, f"frame {k}"


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_shared_jacobian(seed):
    """d(prediction)/d(shared params) against finite differences."""
    rng = np.random.default_rng(seed + 10)
    x_shared = rng.normal(0.0, 0.03, N_SHARED_PARAMS)
    ts, rotvecs = _sample_poses(rng, 3)

    _, _, j_shared = unpack_shared(x_shared).predict_and_jacobians(ts, rotvecs)

    for k in range(len(ts)):
        ref = _central_diff(
            lambda x: unpack_shared(x).predict(ts[k][None, :], rotvecs[k][None, :])[0],
            x_shared,
        )
        scale = max(np.abs(ref).max(), 1.0)
        worst = np.abs(j_shared[k] - ref).max() / scale
        assert worst < 2e-5, f"frame {k}, worst rel err {worst:.2e}"


def test_shared_jacobian_at_nominal():
    """Same check exactly at nominal (zero offsets), where tilt rotations are identity."""
    rng = np.random.default_rng(99)
    x_shared = np.zeros(N_SHARED_PARAMS)
    ts, rotvecs = _sample_poses(rng, 2)
    _, _, j_shared = unpack_shared(x_shared).predict_and_jacobians(ts, rotvecs)
    for k in range(len(ts)):
        ref = _central_diff(
            lambda x: unpack_shared(x).predict(ts[k][None, :], rotvecs[k][None, :])[0],
            x_shared,
        )
        scale = max(np.abs(ref).max(), 1.0)
        assert np.abs(j_shared[k] - ref).max() / scale < 2e-5


def test_predict_shapes_and_batching():
    """Batched prediction must equal per-frame prediction."""
    rng = np.random.default_rng(7)
    geom = unpack_shared(rng.normal(0.0, 0.02, N_SHARED_PARAMS))
    ts, rotvecs = _sample_poses(rng, 5)
    batched = geom.predict(ts, rotvecs)
    assert batched.shape == (5, 9)
    for k in range(5):
        one = geom.predict(ts[k][None, :], rotvecs[k][None, :])[0]
        assert np.allclose(batched[k], one, rtol=1e-12, atol=1e-12)


def test_magnet_spin_is_not_a_parameter():
    """x_shared must carry only 2 tilt DOF per magnet - spin about the magnet's
    own polarization axis is unobservable by construction."""
    from calibration.bundle_geometry import GROUP_SLICES, N_MAGNETS
    assert GROUP_SLICES["magnet_tilt"].stop - GROUP_SLICES["magnet_tilt"].start == 2 * N_MAGNETS
    # and the assembled tilt vector's z component is always exactly zero
    rng = np.random.default_rng(3)
    geom = unpack_shared(rng.normal(0.0, 0.05, N_SHARED_PARAMS))
    assert np.all(geom.magnet_tilt[:, 2] == 0.0)
