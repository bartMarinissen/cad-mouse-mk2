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
    magnet_pos_offsets,
    renormalize_gauge,
    set_magnet_pos_offsets,
    set_strength_vector,
    strength_vector,
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
    model, so the exported matrix is the inverse of the fitted one - with the
    installed magnet polarity folded back in."""
    from calibration.bundle_geometry import MAGNET_POLARITY

    rng = np.random.default_rng(6)
    geom = unpack_shared(rng.normal(0.0, 0.03, N_SHARED_PARAMS))
    fw = firmware_sensor_gain(geom)
    for i in range(3):
        assert np.allclose(fw[i] @ (MAGNET_POLARITY * geom.gain[i]), np.eye(3), atol=1e-10)


def test_nominal_gain_is_identity_and_polarity_carries_the_sign():
    """Nominal gain is +I, with the raw sensors' sign flip carried by the
    magnet's installed polarity instead. What matters is that the exported
    firmware gain still lands near -I, matching Config::magnet_gains - getting
    that sign wrong is what made the previous calibrator unfittable."""
    from calibration.bundle_geometry import MAGNET_POLARITY

    assert np.allclose(NOMINAL_GEOMETRY.gain, np.eye(3)[None, :, :])
    assert MAGNET_POLARITY == -1.0
    assert np.allclose(firmware_sensor_gain(NOMINAL_GEOMETRY), -np.eye(3)[None, :, :])


def test_firmware_offset_mapping():
    """o_fw = G_fw @ o_fit, i.e. the offset transforms into read_mT's space."""
    from calibration.export import firmware_sensor_offset

    rng = np.random.default_rng(11)
    x = np.zeros(N_SHARED_PARAMS)
    x[GROUP_SLICES["sensor_offset"]] = rng.normal(0.0, 1.0, 9)
    geom = unpack_shared(x)
    expected = np.einsum("iab,ib->ia", firmware_sensor_gain(geom), geom.sensor_offset)
    assert np.allclose(firmware_sensor_offset(geom), expected)


# --------------------------------------------------------------------------- #
# gauge: det(G) = 1
# --------------------------------------------------------------------------- #

def test_gain_matrix_to_params_inverts_assembly():
    """gain_matrix_to_params must be the exact inverse of unpack_shared's
    gain assembly - renormalize_gauge depends on that round trip."""
    from calibration.bundle_geometry import NOMINAL_GAIN_SIGN, gain_matrix_to_params

    rng = np.random.default_rng(12)
    x = np.zeros(N_SHARED_PARAMS)
    for group in ("gain_iso", "gain_aniso", "gain_sym", "gain_rot"):
        sl = GROUP_SLICES[group]
        x[sl] = rng.normal(0.0, 0.05, sl.stop - sl.start)

    a = NOMINAL_GAIN_SIGN * unpack_shared(x).gain - np.eye(3)
    recovered = gain_matrix_to_params(a)
    for group, vals in recovered.items():
        assert np.allclose(vals.ravel(), x[GROUP_SLICES[group]], atol=1e-12), group


def test_renormalize_gauge_gives_unit_determinant():
    """After the transfer every sensor gain must have |det| == 1."""
    rng = np.random.default_rng(13)
    x = np.zeros(N_SHARED_PARAMS)
    for group in ("gain_iso", "gain_aniso", "gain_sym", "gain_rot"):
        sl = GROUP_SLICES[group]
        x[sl] = rng.normal(0.0, 0.06, sl.stop - sl.start)
    set_strength_vector(x, rng.normal(0.0, 0.02, 3))

    geom = unpack_shared(renormalize_gauge(x))
    assert np.allclose(np.abs(np.linalg.det(geom.gain)), 1.0, atol=1e-10)


def test_renormalize_gauge_nearly_preserves_predictions():
    """The transfer re-attributes scale rather than changing the model. It is
    only exact without cross-talk, so allow the ~1% cross-talk term - but it
    must be far smaller than the scale being moved."""
    rng = np.random.default_rng(14)
    x = np.zeros(N_SHARED_PARAMS)
    x[GROUP_SLICES["gain_iso"]] = [0.05, -0.03, 0.04]
    poses = _random_poses(rng, 12)

    before = unpack_shared(x).predict(poses[:, :3], poses[:, 3:])
    after = unpack_shared(renormalize_gauge(x)).predict(poses[:, :3], poses[:, 3:])

    scale = np.abs(before).mean()
    assert np.abs(after - before).max() / scale < 0.01


def test_gauge_moves_scale_from_gain_into_strength():
    """gain_iso should end up at zero and its scale show up in strength."""
    x = np.zeros(N_SHARED_PARAMS)
    x[GROUP_SLICES["gain_iso"]] = [0.05, -0.03, 0.04]

    y = renormalize_gauge(x)
    assert np.abs(y[GROUP_SLICES["gain_iso"]]).max() < 1e-10
    # to first order det(G)^(1/3) == 1 + gain_iso, so strength picks it up
    assert np.allclose(strength_vector(y), [0.05, -0.03, 0.04], atol=2e-3)


# --------------------------------------------------------------------------- #
# synthetic round trip
# --------------------------------------------------------------------------- #

# (step, xy spread mm, z spread mm, rotation spread rad) - chosen to match the
# pose ranges actually measured off the captured runs, since how well a
# parameter can be recovered depends entirely on how much the poses move. In
# particular the knob barely translates in xy (about a millimetre total) while
# HEAVE supplies roughly 3mm of z travel.
_STEP_SPECS = (
    (CalibStep.STATIONARY, 0.02, 0.02, 0.005),
    (CalibStep.FLAT_CIRCLE, 0.50, 0.25, 0.045),
    (CalibStep.PITCH, 0.25, 0.15, 0.130),
    (CalibStep.ROLL, 0.25, 0.15, 0.130),
    (CalibStep.TWIST, 0.20, 0.50, 0.260),
    (CalibStep.HEAVE, 0.30, 1.50, 0.030),
    (CalibStep.RANDOM, 0.90, 0.50, 0.110),
)


def _synthetic_datasets(geom, rng, n_per_step=40, z_scale=1.0):
    """Fabricate a capture that mimics the real step structure and pose ranges.

    z_scale exaggerates the heave travel, for tests that need to show what
    *would* become identifiable with more z motion than the hardware gives.
    """
    datasets = {}
    for step, dxy, dz, drot in _STEP_SPECS:
        ts = APPROX_REST_T_MM + np.stack([
            rng.uniform(-dxy, dxy, n_per_step),
            rng.uniform(-dxy, dxy, n_per_step),
            rng.uniform(-dz * z_scale, dz * z_scale, n_per_step),
        ], axis=1)
        rotvecs = rng.uniform(-drot, drot, (n_per_step, 3))
        field = geom.predict(ts, rotvecs)
        field = field + rng.normal(0.0, 0.1, field.shape)  # ~measured read noise
        datasets[step] = field.tolist()
    return datasets


def test_synthetic_round_trip_recovers_gain_and_geometry():
    """Generate from a known geometry, fit, and check the answer comes back."""
    rng = np.random.default_rng(7)
    truth = np.zeros(N_SHARED_PARAMS)
    truth[GROUP_SLICES["gain_iso"]] = [0.06, -0.04, 0.02]
    # Projected onto the gauge-fixed subspace on the way in, so the "truth"
    # compared against below is the gauge representative, not the raw numbers.
    set_magnet_pos_offsets(truth, [[0.15, -0.20, 0.05],
                                   [-0.10, 0.12, -0.04],
                                   [0.08, 0.18, 0.03]])
    truth[GROUP_SLICES["magnet_tilt"]] = [0.01, -0.008, 0.006, 0.012, -0.011, 0.004]
    geom = unpack_shared(truth)

    # gain_iso is what the gauge stage deliberately moves into strength, so
    # compare against the scale-free fit here.
    result = run_bundle_calibration(
        _synthetic_datasets(geom, rng), n_frames=60,
        estimate_strength=False, verbose=False,
    )

    # residual should fall to about the injected noise level
    assert result.field_rms_mT < 0.2

    got = result.shared_offsets
    gain_err = np.abs(got[GROUP_SLICES["gain_iso"]] - truth[GROUP_SLICES["gain_iso"]])
    pos_err = magnet_pos_offsets(got) - magnet_pos_offsets(truth)
    assert gain_err.max() < 0.04, f"gain not recovered: {gain_err}"

    # Magnet offsets are checked in aggregate, not per-component: the knob
    # barely translates in xy over a real capture, so individual in-plane
    # offsets are only weakly determined (the fit reports ~16% information
    # gain for this group). RMS is still a real test - a fit that learned
    # nothing and left every offset at zero would score 0.12 here.
    assert np.sqrt(np.mean(pos_err**2)) < 0.08, f"magnet positions not recovered: {pos_err}"


def test_synthetic_round_trip_recovers_sensor_offsets():
    """A DC offset is constant across every pose while the field is not, so it
    should be one of the better-determined things in the whole fit."""
    rng = np.random.default_rng(21)
    truth = np.zeros(N_SHARED_PARAMS)
    truth[GROUP_SLICES["sensor_offset"]] = [0.8, -1.2, 0.3,
                                            -1.5, 0.4, -0.2,
                                            0.2, 0.9, -0.7]
    geom = unpack_shared(truth)

    result = run_bundle_calibration(
        _synthetic_datasets(geom, rng), n_frames=60, verbose=False
    )
    err = np.abs(result.shared_offsets[GROUP_SLICES["sensor_offset"]]
                 - truth[GROUP_SLICES["sensor_offset"]])
    assert err.max() < 0.25, f"offsets not recovered: {err}"


def test_gauge_stage_holds_det_one_and_fits_the_field():
    """Whatever the gauge does to attribution, it must not damage the fit."""
    rng = np.random.default_rng(22)
    truth = np.zeros(N_SHARED_PARAMS)
    set_strength_vector(truth, [0.07, -0.05, 0.03])
    geom = unpack_shared(truth)
    assert np.allclose(np.abs(np.linalg.det(geom.gain)), 1.0)  # truth is in-gauge

    result = run_bundle_calibration(
        _synthetic_datasets(geom, rng), n_frames=60, verbose=False
    )
    assert result.field_rms_mT < 0.2
    assert np.allclose(np.abs(np.linalg.det(result.geometry.gain)), 1.0, atol=1e-6)


def test_magnet_strength_is_weakly_identified_at_realistic_z_travel():
    """Strength is NOT well determined by a real capture, and the fit must say so.

    Scaling a magnet and moving it closer both scale |B|; only the *shape* of
    |B| versus distance separates them, and this hardware's HEAVE step supplies
    only ~3mm of travel. Anisotropic gain can absorb much of the rest, since
    the field is Bz-dominated. So the honest outcome is a low information gain,
    and this test pins that down - it would fail if the fit ever started
    claiming confidence it has not earned.
    """
    rng = np.random.default_rng(23)
    truth = np.zeros(N_SHARED_PARAMS)
    set_strength_vector(truth, [0.07, -0.05, 0.03])

    result = run_bundle_calibration(
        _synthetic_datasets(unpack_shared(truth), rng), n_frames=60, verbose=False
    )
    # The common mode carries no prior by default, so the honest signal is a
    # large posterior sd rather than a low information gain: the fit should be
    # saying "I cannot pin the absolute field scale", not quietly picking one.
    post = result.posterior_sigma[GROUP_SLICES["magnet_strength_mean"]][0]
    assert post > 0.1, f"strength claims sd {post:.3f} - too confident for this motion"
    assert np.isnan(result.information_gain[GROUP_SLICES["magnet_strength_mean"]][0]), (
        "an unregularized parameter has no prior to compare against"
    )


def test_tight_differential_prior_makes_magnets_equal():
    """The default differential prior is ~15x tighter than the common-mode one,
    which is a deliberate statement that magnets from one batch are near
    identical. Check it actually binds: the fitted strengths should come out
    the same to well under a percent even when the truth says otherwise."""
    rng = np.random.default_rng(25)
    truth = np.zeros(N_SHARED_PARAMS)
    set_strength_vector(truth, [0.06, -0.04, 0.02])

    result = run_bundle_calibration(
        _synthetic_datasets(unpack_shared(truth), rng), n_frames=60, verbose=False
    )
    strengths = result.geometry.magnet_strength
    assert np.ptp(strengths) < 0.01, f"magnets not pulled together: {strengths}"


def test_loosening_the_differential_prior_lets_magnets_differ():
    """The companion: the constraint is the prior, not the parameterization."""
    from calibration.bundle_params import RegularizationSigmas

    rng = np.random.default_rng(26)
    truth = np.zeros(N_SHARED_PARAMS)
    set_strength_vector(truth, [0.06, -0.04, 0.02])

    result = run_bundle_calibration(
        _synthetic_datasets(unpack_shared(truth), rng), n_frames=60,
        sigmas=RegularizationSigmas(magnet_strength_diff=0.2), verbose=False,
    )
    assert np.ptp(result.geometry.magnet_strength) > 0.03


def test_magnet_strength_is_recoverable_when_nothing_competes():
    """The companion to the test above: the machinery is sound, the *data* is
    the limit.

    Free strength alone - no gain anisotropy, no magnet geometry to soak up a
    scale change - and it comes straight back. So the weak result above is
    about what a real capture can distinguish, not about a broken Jacobian or
    a broken gauge.
    """
    from calibration.bundle_params import SolveStage

    rng = np.random.default_rng(24)
    truth = np.zeros(N_SHARED_PARAMS)
    set_strength_vector(truth, [0.07, -0.05, 0.03])

    result = run_bundle_calibration(
        _synthetic_datasets(unpack_shared(truth), rng),
        n_frames=60,
        stages=(SolveStage("strength only",
                           ("magnet_strength_mean", "magnet_strength_diff")),),
        estimate_strength=False,
        verbose=False,
    )
    err = strength_vector(result.shared_offsets) - strength_vector(truth)
    # The differential part is held near zero by its (deliberately tight)
    # prior, so only the common mode is expected to come back - hence the
    # signed mean of the error, not its magnitude.
    assert abs(err.mean()) < 0.02, f"mean strength not recovered: {err}"
    # And with nothing to trade against, the posterior tightens by roughly the
    # order of magnitude that separates "measured" from "guessed" here.
    post = result.posterior_sigma[GROUP_SLICES["magnet_strength_mean"]][0]
    assert post < 0.06, f"strength posterior sd {post:.3f} - expected it to tighten"


def test_synthetic_round_trip_at_nominal_stays_near_zero():
    """Data generated from nominal geometry must not invent large offsets."""
    rng = np.random.default_rng(8)
    result = run_bundle_calibration(
        _synthetic_datasets(NOMINAL_GEOMETRY, rng), n_frames=60, verbose=False
    )
    assert result.field_rms_mT < 0.2
    assert np.abs(result.shared_offsets[GROUP_SLICES["gain_iso"]]).max() < 0.03
    assert np.abs(magnet_pos_offsets(result.shared_offsets)).max() < 0.15


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
def test_real_run_stays_physically_plausible():
    """Sensor gains within ~20% of nominal and on the right side of zero,
    magnet offsets sub-millimetre, tilts a fraction of a degree, strengths
    near 1. The previous calibrator failed exactly this (gains of 2.3,
    strengths of 2.6)."""
    raw = json.loads(_real_runs()[0].read_text())
    datasets = {CalibStep[k]: v for k, v in raw.items()}
    result = run_bundle_calibration(datasets, n_frames=60, verbose=False)

    diag = np.array([np.diag(result.geometry.gain[i]) for i in range(3)])
    assert np.all(diag > 0), "fitted gain must stay on the correct side of zero"
    assert np.abs(diag - 1.0).max() < 0.2

    # and what actually ships lands near -I, matching Config::magnet_gains
    fw_diag = np.array([np.diag(g) for g in firmware_sensor_gain(result.geometry)])
    assert np.all(fw_diag < 0)
    assert np.abs(np.abs(fw_diag) - 1.0).max() < 0.2

    offsets = result.shared_offsets
    assert np.abs(magnet_pos_offsets(offsets)).max() < 1.0
    assert np.degrees(np.abs(offsets[GROUP_SLICES["magnet_tilt"]])).max() < 3.0
    # Magnet strength carries no prior, and the data barely constrains the
    # absolute field scale, so the right check is consistency with nominal
    # given the fit's *own* stated uncertainty - not a fixed window.
    post = result.posterior_sigma[GROUP_SLICES["magnet_strength_mean"]][0]
    assert np.abs(result.geometry.magnet_strength - 1.0).max() < 3 * post
    assert np.abs(result.geometry.sensor_offset).max() < 5.0
