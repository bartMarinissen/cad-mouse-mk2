"""End-to-end tests for the bundle calibration fit (calibration_algorithm.py).

The synthetic round-trip tests are the important ones: they generate data
from a *known* geometry and check the fit recovers it. That is the only check
that can catch a sign/convention error like the one this rewrite fixed - a
fit can look perfectly converged and still be describing the wrong magnet.
"""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path

import numpy as np
import pytest

from calibration.bundle_geometry import (
    ALL_MAGNETS,
    APPROX_REST_T_MM,
    GROUP_SLICES,
    N_SHARED_PARAMS,
    NOMINAL_GEOMETRY,
    PAIRED_ONLY,
    BundleGeometry,
    magnet_pos_offsets,
    set_magnet_pos_offsets,
    set_strength_vector,
    strength_vector,
)
from calibration.calibration_algorithm import _decimate, run_bundle_calibration
from calibration.protocol import CalibStep

RUNS_DIR = Path(__file__).resolve().parent.parent / "calibration_runs"


# --------------------------------------------------------------------------- #
# frame decimation
# --------------------------------------------------------------------------- #
#
# No diversity-selection logic to test here deliberately - an earlier version
# picked frames by farthest-point sampling, ablated away once the measured
# "30 to 387 frames land within 0.03pp" result showed the specific frames
# chosen barely matter. _decimate() only exists to bound runtime.

def test_decimate_caps_and_spans_the_range():
    idx = np.arange(200)
    out = _decimate(idx, 25)
    assert len(out) == 25
    assert len(np.unique(out)) == 25
    assert out[0] == 0 and out[-1] == 199, "should span the full range, not just a prefix"


def test_decimate_passes_through_when_under_the_cap():
    idx = np.arange(10)
    assert np.array_equal(_decimate(idx, 50), idx)


# --------------------------------------------------------------------------- #
# gauge: gain has no isotropic/scale parameter, so det(G) = 1 structurally
# --------------------------------------------------------------------------- #

def test_gain_has_no_scale_direction():
    """Randomizing every free gain parameter at its realistic prior scale must
    still leave det(G) within the expected second-order tolerance of 1 - there
    is no way to reach a different determinant, because there is no free
    parameter that scales.

    tr(A) = 0 exactly (test_parameterization.test_gain_basis_is_exactly_traceless),
    so det(I+A) = 1 - tr(A^2)/2 + det(A): a genuine but second-order-small
    deviation at typical fitted magnitudes, not an exact identity. The
    magnitude used here matches RegularizationSigmas' prior widths - a much
    larger stress magnitude would show a larger (still second-order, still
    real) deviation, which is not what this test is checking.
    """
    rng = np.random.default_rng(12)
    x = np.zeros(N_SHARED_PARAMS)
    sigmas = {"gain_aniso": 0.05, "gain_sym": 0.03, "gain_rot": 0.03}
    for group, sigma in sigmas.items():
        sl = GROUP_SLICES[group]
        x[sl] = rng.normal(0.0, sigma, sl.stop - sl.start)

    geom = BundleGeometry.from_shared(x)
    assert np.allclose(np.abs(np.linalg.det(geom.gain)), 1.0, atol=2e-2)


def test_magnet_strength_free_from_the_start_no_gauge_transfer_needed():
    """With gain carrying no scale, a fit that only frees magnet_strength (no
    gain parameters at all) should already land close to the true value -
    there is no separate gauge-transfer stage required to get there."""
    rng = np.random.default_rng(13)
    truth = np.zeros(N_SHARED_PARAMS)
    set_strength_vector(truth, [0.05, -0.03, 0.04])
    geom = BundleGeometry.from_shared(truth)

    from calibration.bundle_params import SolveStage

    result = run_bundle_calibration(
        _synthetic_datasets(geom, rng), n_frames=60,
        stages=(SolveStage("strength only",
                           ("magnet_strength_mean", "magnet_strength_diff")),),
        verbose=False,
    )
    assert result.field_rms_mT < 0.2


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
    truth[GROUP_SLICES["gain_aniso"]] = [0.03, -0.02, 0.05, -0.03, -0.02, 0.04]
    # Projected onto the gauge-fixed subspace on the way in, so the "truth"
    # compared against below is the gauge representative, not the raw numbers.
    set_magnet_pos_offsets(truth, [[0.15, -0.20, 0.05],
                                   [-0.10, 0.12, -0.04],
                                   [0.08, 0.18, 0.03]])
    truth[GROUP_SLICES["magnet_tilt"]] = [0.01, -0.008, 0.006, 0.012, -0.011, 0.004]
    geom = BundleGeometry.from_shared(truth)

    result = run_bundle_calibration(
        _synthetic_datasets(geom, rng), n_frames=60, verbose=False,
    )

    # residual should fall to about the injected noise level
    assert result.field_rms_mT < 0.2

    got = result.shared_offsets
    gain_err = np.abs(got[GROUP_SLICES["gain_aniso"]] - truth[GROUP_SLICES["gain_aniso"]])
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
    geom = BundleGeometry.from_shared(truth)

    result = run_bundle_calibration(
        _synthetic_datasets(geom, rng), n_frames=60, verbose=False
    )
    err = np.abs(result.shared_offsets[GROUP_SLICES["sensor_offset"]]
                 - truth[GROUP_SLICES["sensor_offset"]])
    assert err.max() < 0.25, f"offsets not recovered: {err}"


def test_fitted_gain_determinant_stays_near_one():
    """Every fitted gain matrix stays within the gauge's second-order
    tolerance of det=1, structurally - not because a fit stage enforces it."""
    rng = np.random.default_rng(22)
    truth = np.zeros(N_SHARED_PARAMS)
    set_strength_vector(truth, [0.07, -0.05, 0.03])
    geom = BundleGeometry.from_shared(truth)
    assert np.allclose(np.abs(np.linalg.det(geom.gain)), 1.0, atol=5e-3)  # truth is in-gauge

    result = run_bundle_calibration(
        _synthetic_datasets(geom, rng), n_frames=60, verbose=False
    )
    assert result.field_rms_mT < 0.2
    assert np.allclose(np.abs(np.linalg.det(result.geometry.gain)), 1.0, atol=5e-3)


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
        _synthetic_datasets(BundleGeometry.from_shared(truth), rng), n_frames=60, verbose=False
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
        _synthetic_datasets(BundleGeometry.from_shared(truth), rng), n_frames=60, verbose=False
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
        _synthetic_datasets(BundleGeometry.from_shared(truth), rng), n_frames=60,
        sigmas=RegularizationSigmas(magnet_strength_diff=0.2), verbose=False,
    )
    assert np.ptp(result.geometry.magnet_strength) > 0.03


def _fit_strength_only(coupling, monkeypatch, seed=24):
    """Fit common-mode strength against synthetic data, under one coupling.

    The coupling has to be patched globally, not just passed to from_shared:
    the fit rebuilds a BundleGeometry from the parameter vector on every
    iteration (via BundleCalibrationProblem.unpack), and those rebuilds read
    SENSOR_MAGNET_COUPLING. Patching it makes data generation and the fit
    agree, which is the whole point - a mismatch between them is exactly the
    hardware bug this coupling switch exists to avoid.
    """
    import calibration.bundle_geometry as bg
    from calibration.bundle_params import SolveStage

    monkeypatch.setattr(bg, "SENSOR_MAGNET_COUPLING", coupling)

    rng = np.random.default_rng(seed)
    truth = np.zeros(N_SHARED_PARAMS)
    set_strength_vector(truth, [0.07, -0.05, 0.03])

    result = run_bundle_calibration(
        _synthetic_datasets(BundleGeometry.from_shared(truth, coupling=coupling), rng),
        n_frames=60,
        stages=(SolveStage("strength only",
                           ("magnet_strength_mean", "magnet_strength_diff")),),
        verbose=False,
    )
    # The differential part is held near zero by its (deliberately tight)
    # prior, so only the common mode is expected to come back - hence the
    # signed mean of the error, not its magnitude.
    err = float((strength_vector(result.shared_offsets) - strength_vector(truth)).mean())
    post = float(result.posterior_sigma[GROUP_SLICES["magnet_strength_mean"]][0])
    return err, post


def test_absolute_strength_needs_cross_magnet_coupling(monkeypatch):
    """Separating magnet *strength* from magnet *distance* is what cross-talk buys.

    Under ALL_MAGNETS a sensor sees its own magnet at ~6mm and the other two
    at ~28mm. A uniform strength change scales all three contributions
    equally; moving the knob changes the near one steeply and the far ones
    barely. Different functional forms, so the two separate cleanly.

    Under PAIRED_ONLY only the near contribution exists, and strength is very
    nearly degenerate with a z shift - broken only by the curvature of a
    single falloff across the ~3mm of heave the hardware actually gives. So
    absolute field scale goes soft. That is a real, accepted cost of matching
    the firmware's model (see SENSOR_MAGNET_COUPLING), not a defect.

    What must hold under *either* model is that the fit knows: the posterior
    sd has to cover the error actually made. A fit that is wrong and says so
    is usable; one that is wrong and confident is not.
    """
    err_paired, post_paired = _fit_strength_only(PAIRED_ONLY, monkeypatch)
    err_all, post_all = _fit_strength_only(ALL_MAGNETS, monkeypatch)

    # Honesty, the non-negotiable part: reported uncertainty covers real error.
    assert abs(err_paired) < post_paired, (
        f"PAIRED_ONLY error {err_paired:+.4f} exceeds its own posterior sd "
        f"{post_paired:.4f} - the fit is wrong *and* overconfident")
    assert abs(err_all) < post_all, (
        f"ALL_MAGNETS error {err_all:+.4f} exceeds its own posterior sd {post_all:.4f}")

    # Cross-talk genuinely determines absolute scale better, and the posterior
    # reflects that rather than the two just differing by noise.
    assert abs(err_all) < abs(err_paired), (
        f"expected cross-talk to recover strength better: "
        f"ALL_MAGNETS {err_all:+.4f} vs PAIRED_ONLY {err_paired:+.4f}")
    assert post_all < post_paired, (
        f"expected cross-talk to tighten the strength posterior: "
        f"ALL_MAGNETS {post_all:.4f} vs PAIRED_ONLY {post_paired:.4f}")

    # With cross-talk the machinery still recovers scale tightly - which is
    # what shows the Jacobian and gauge are sound and the PAIRED_ONLY spread
    # above is the data talking, not a broken fit.
    assert abs(err_all) < 0.05, f"mean strength not recovered under cross-talk: {err_all:+.4f}"
    assert post_all < 0.08, f"strength posterior sd {post_all:.3f} - expected it to tighten"


def test_paired_only_isolates_each_sensor_to_its_own_magnet(monkeypatch):
    """The mask does what it claims: under PAIRED_ONLY, magnet j is invisible
    to every sensor but j.

    Checked behaviourally rather than by inspecting the mask, because the
    mask is applied once to the local field and then flows through several
    einsums - this asserts the property that actually matters downstream.
    """
    rng = np.random.default_rng(5)
    ts = APPROX_REST_T_MM + rng.uniform(-0.3, 0.3, (4, 3))
    rotvecs = rng.uniform(-0.05, 0.05, (4, 3))

    # Perturb magnet 0's position on the geometry directly, NOT through
    # set_magnet_pos_offsets(): that projects onto MAGNET_POS_BASIS, whose
    # zero-mean gauge constraint spreads a single-magnet bump across all
    # three - which would move the "cross" sensors for reasons that have
    # nothing to do with coupling and make this test vacuous.
    bumped_pos = NOMINAL_GEOMETRY.magnet_pos_knob.copy()
    bumped_pos[0] += np.array([0.4, 0.4, 0.0])

    for coupling, cross_should_move in ((PAIRED_ONLY, False), (ALL_MAGNETS, True)):
        base = dataclasses.replace(NOMINAL_GEOMETRY, coupling=coupling)
        bumped = dataclasses.replace(
            NOMINAL_GEOMETRY, coupling=coupling, magnet_pos_knob=bumped_pos
        )

        delta = np.abs(bumped.predict(ts, rotvecs) - base.predict(ts, rotvecs))
        # Sensor 0 is magnet 0's pair; sensors 1 and 2 are the cross terms.
        assert delta[:, 0:3].max() > 1e-3, "magnet 0 should move its own sensor"
        cross_moved = delta[:, 3:9].max()
        if cross_should_move:
            assert cross_moved > 1e-6, (
                f"ALL_MAGNETS: magnet 0 should reach sensors 1/2, moved {cross_moved:.2e}")
        else:
            assert cross_moved == 0.0, (
                f"PAIRED_ONLY: magnet 0 must not reach sensors 1/2, moved {cross_moved:.2e}")


def test_synthetic_round_trip_at_nominal_stays_near_zero():
    """Data generated from nominal geometry must not invent large offsets."""
    rng = np.random.default_rng(8)
    result = run_bundle_calibration(
        _synthetic_datasets(NOMINAL_GEOMETRY, rng), n_frames=60, verbose=False
    )
    assert result.field_rms_mT < 0.2
    assert np.abs(result.shared_offsets[GROUP_SLICES["gain_aniso"]]).max() < 0.03
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
    from calibration.export import firmware_sensor_gain

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
