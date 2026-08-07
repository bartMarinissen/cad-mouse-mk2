"""Unit tests for the batched Levenberg-Marquardt pose solver."""

from __future__ import annotations

import numpy as np

from calibration.bundle_geometry import (
    APPROX_REST_T_MM,
    N_SHARED_PARAMS,
    NOMINAL_GEOMETRY,
    BundleGeometry,
)
from calibration.pose_solver import solve_poses


def _random_poses(rng, n, spread_mm=1.0, spread_rad=0.15):
    ts = APPROX_REST_T_MM + rng.uniform(-spread_mm, spread_mm, (n, 3))
    rotvecs = rng.uniform(-spread_rad, spread_rad, (n, 3))
    return np.concatenate([ts, rotvecs], axis=1)


def test_pose_solver_recovers_known_poses():
    """Batched LM must recover poses it generated, from a cold rest-pose start."""
    rng = np.random.default_rng(0)
    poses = _random_poses(rng, 40)
    measured = NOMINAL_GEOMETRY.predict(poses[:, :3], poses[:, 3:])

    result = solve_poses(NOMINAL_GEOMETRY, measured)

    assert result.converged.all()
    assert np.abs(result.residual_mT).max() < 1e-6
    assert np.abs(result.poses - poses).max() < 1e-4


def test_pose_solver_handles_perturbed_geometry():
    """Poses still solve when the geometry is not nominal."""
    rng = np.random.default_rng(1)
    geom = BundleGeometry.from_shared(rng.normal(0.0, 0.02, N_SHARED_PARAMS))
    poses = _random_poses(rng, 20)
    measured = geom.predict(poses[:, :3], poses[:, 3:])

    result = solve_poses(geom, measured)
    assert result.converged.all()
    assert np.abs(result.residual_mT).max() < 1e-5
