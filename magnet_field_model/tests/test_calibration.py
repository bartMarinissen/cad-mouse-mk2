"""End-to-end and component tests for the calibration pipeline.

The synthetic round-trip tests are the important ones: they generate data
from a *known* geometry and check the fit recovers it. That is the only check
that can catch a sign/convention error like the one this rewrite fixed - a
fit can look perfectly converged and still be describing the wrong magnet.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from calibration.bundle_geometry import (
    APPROX_REST_T_MM,
    GROUP_SLICES,
    N_SHARED_PARAMS,
    NOMINAL_GEOMETRY,
    unpack_shared,
)
from calibration.calibration_algorithm import run_bundle_calibration
from calibration.export import firmware_sensor_gain
from calibration.frame_selection import magnet_world_positions, select_diverse_frames
from calibration.pose_solver import solve_poses
from calibration.protocol import CalibStep

RUNS_DIR = Path(__file__).resolve().parent.parent / "calibration_runs"


def _random_poses(rng, n, spread_mm=1.0, spread_rad=0.15):
    ts = APPROX_REST_T_MM + rng.uniform(-spread_mm, spread_mm, (n, 3))
    rotvecs = rng.uniform(-spread_rad, spread_rad, (n, 3))
    return np.concatenate([ts, rotvecs], axis=1)


# --------------------------------------------------------------------------- #
# pose solver
# --------------------------------------------------------------------------- #

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
    geom = unpack_shared(rng.normal(0.0, 0.02, N_SHARED_PARAMS))
    poses = _random_poses(rng, 20)
    measured = geom.predict(poses[:, :3], poses[:, 3:])

    result = solve_poses(geom, measured)
    assert result.converged.all()
    assert np.abs(result.residual_mT).max() < 1e-5


# --------------------------------------------------------------------------- #
# frame selection
# --------------------------------------------------------------------------- #

def test_frame_selection_count_and_bounds():
    rng = np.random.default_rng(2)
    poses = _random_poses(rng, 200)
    idx = select_diverse_frames(poses, 25)
    assert len(idx) == 25
    assert len(np.unique(idx)) == 25
    assert idx.min() >= 0 and idx.max() < 200
    assert np.all(np.diff(idx) > 0), "indices should come back sorted"


def test_frame_selection_returns_all_when_asked_for_too_many():
    rng = np.random.default_rng(3)
    poses = _random_poses(rng, 10)
    assert len(select_diverse_frames(poses, 50)) == 10


def test_frame_selection_beats_decimation_on_spread():
    """Farthest-point selection should cover more pose volume than plain
    decimation of the same size - that is its entire reason to exist."""
    rng = np.random.default_rng(4)
    # A run-like dataset: mostly clustered near rest, with a few excursions.
    poses = np.concatenate([_random_poses(rng, 180, 0.2, 0.02),
                            _random_poses(rng, 20, 2.0, 0.25)])
    n = 20
    chosen = magnet_world_positions(poses)[select_diverse_frames(poses, n)]
    decimated = magnet_world_positions(poses)[:: len(poses) // n][:n]

    def spread(x):
        return float(np.linalg.norm(x - x.mean(axis=0), axis=1).mean())

    assert spread(chosen) > 1.5 * spread(decimated)


def test_frame_selection_respects_valid_mask():
    rng = np.random.default_rng(5)
    poses = _random_poses(rng, 50)
    valid = np.zeros(50, dtype=bool)
    valid[10:20] = True
    idx = select_diverse_frames(poses, 5, valid=valid)
    assert set(idx).issubset(set(range(10, 20)))


# --------------------------------------------------------------------------- #
# export
# --------------------------------------------------------------------------- #

def test_firmware_gain_is_the_inverse():
    """The firmware applies gain to the measurement, the fit applies it to the
    model, so the exported matrix must be the inverse of the fitted one."""
    rng = np.random.default_rng(6)
    geom = unpack_shared(rng.normal(0.0, 0.03, N_SHARED_PARAMS))
    fw = firmware_sensor_gain(geom)
    for i in range(3):
        assert np.allclose(fw[i] @ geom.gain[i], np.eye(3), atol=1e-10)


def test_nominal_gain_is_negative_identity():
    """A zero offset vector must mean exactly -I, not +I - this is the
    convention error that made the previous calibrator unfittable."""
    assert np.allclose(NOMINAL_GEOMETRY.gain, -np.eye(3)[None, :, :])


# --------------------------------------------------------------------------- #
# synthetic round trip
# --------------------------------------------------------------------------- #

def _synthetic_datasets(geom, rng, n_per_step=40):
    """Fabricate a capture that mimics the real step structure."""
    datasets = {}
    specs = [
        (CalibStep.STATIONARY, 0.02, 0.005),
        (CalibStep.FLAT_CIRCLE, 0.8, 0.05),
        (CalibStep.PITCH, 0.4, 0.12),
        (CalibStep.ROLL, 0.4, 0.12),
        (CalibStep.TWIST, 0.3, 0.20),
        (CalibStep.HEAVE, 1.6, 0.03),
        (CalibStep.RANDOM, 1.0, 0.10),
    ]
    for step, dt, drot in specs:
        poses = _random_poses(rng, n_per_step, dt, drot)
        field = geom.predict(poses[:, :3], poses[:, 3:])
        field = field + rng.normal(0.0, 0.1, field.shape)  # ~measured read noise
        datasets[step] = field.tolist()
    return datasets


def test_synthetic_round_trip_recovers_gain_and_geometry():
    """Generate from a known geometry, fit, and check the answer comes back."""
    rng = np.random.default_rng(7)
    truth = np.zeros(N_SHARED_PARAMS)
    truth[GROUP_SLICES["gain_iso"]] = [0.06, -0.04, 0.02]
    truth[GROUP_SLICES["magnet_pos"]] = [0.15, -0.20, 0.05,
                                         -0.10, 0.12, -0.04,
                                         0.08, 0.18, 0.03]
    truth[GROUP_SLICES["magnet_tilt"]] = [0.01, -0.008, 0.006, 0.012, -0.011, 0.004]
    geom = unpack_shared(truth)

    result = run_bundle_calibration(
        _synthetic_datasets(geom, rng), n_frames=60, verbose=False
    )

    # residual should fall to about the injected noise level
    assert result.field_rms_mT < 0.2

    got = result.shared_offsets
    gain_err = np.abs(got[GROUP_SLICES["gain_iso"]] - truth[GROUP_SLICES["gain_iso"]])
    pos_err = np.abs(got[GROUP_SLICES["magnet_pos"]] - truth[GROUP_SLICES["magnet_pos"]])
    assert gain_err.max() < 0.03, f"gain not recovered: {gain_err}"
    assert pos_err.max() < 0.12, f"magnet positions not recovered: {pos_err}"


def test_synthetic_round_trip_at_nominal_stays_near_zero():
    """Data generated from nominal geometry must not invent large offsets."""
    rng = np.random.default_rng(8)
    result = run_bundle_calibration(
        _synthetic_datasets(NOMINAL_GEOMETRY, rng), n_frames=60, verbose=False
    )
    assert result.field_rms_mT < 0.2
    assert np.abs(result.shared_offsets[GROUP_SLICES["gain_iso"]]).max() < 0.03
    assert np.abs(result.shared_offsets[GROUP_SLICES["magnet_pos"]]).max() < 0.15


# --------------------------------------------------------------------------- #
# real captured data
# --------------------------------------------------------------------------- #

def _real_runs():
    return sorted(RUNS_DIR.glob("raw_*.json"))


@pytest.mark.skipif(not _real_runs(), reason="no captured calibration runs available")
def test_real_run_residual_improves():
    """On real hardware data the fit must beat the nominal model substantially."""
    raw = json.loads(_real_runs()[0].read_text())
    datasets = {CalibStep[k]: v for k, v in raw.items()}

    result = run_bundle_calibration(datasets, n_frames=60, verbose=False)

    assert result.initial_rel_pct < 3.0, "nominal model should already be ~2%"
    assert result.field_rel_pct < 0.8, "fit should reach well under 1%"
    assert result.field_rel_pct < result.initial_rel_pct / 2


@pytest.mark.skipif(len(_real_runs()) < 2, reason="need at least two runs")
def test_real_runs_agree_with_each_other():
    """Three captures of the same hardware must produce the same parameters,
    well inside the priors. This is the check that would have caught the old
    calibrator immediately."""
    results = []
    for path in _real_runs():
        raw = json.loads(path.read_text())
        datasets = {CalibStep[k]: v for k, v in raw.items()}
        results.append(run_bundle_calibration(datasets, n_frames=60, verbose=False))

    x = np.array([r.shared_offsets for r in results])
    prior = results[0].prior_sigma
    spread = x.std(axis=0)
    # every parameter should reproduce far tighter than its own prior width
    assert np.all(spread < 0.5 * prior), f"worst {np.max(spread / prior):.2f} of prior"

    # and the physically meaningful ones should be genuinely tight
    pos = spread[GROUP_SLICES["magnet_pos"]]
    assert pos.max() < 0.08, f"magnet positions vary {pos.max():.3f} mm between runs"


@pytest.mark.skipif(not _real_runs(), reason="no captured calibration runs available")
def test_real_run_gain_stays_near_minus_one():
    """A physically plausible answer: sensor gains within ~20% of -1, magnet
    offsets sub-millimetre, tilts a fraction of a degree. The previous
    calibrator failed exactly this (gains of 2.3, strengths of 2.6)."""
    raw = json.loads(_real_runs()[0].read_text())
    datasets = {CalibStep[k]: v for k, v in raw.items()}
    result = run_bundle_calibration(datasets, n_frames=60, verbose=False)

    diag = np.array([np.diag(result.geometry.gain[i]) for i in range(3)])
    assert np.all(diag < 0), "gain must stay on the correct side of zero"
    assert np.abs(np.abs(diag) - 1.0).max() < 0.2

    offsets = result.shared_offsets
    assert np.abs(offsets[GROUP_SLICES["magnet_pos"]]).max() < 1.0
    assert np.degrees(np.abs(offsets[GROUP_SLICES["magnet_tilt"]])).max() < 3.0
