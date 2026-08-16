#pragma once
// PROTOTYPE / SKETCH -- not wired into the firmware build (see README.md).
//
// The P=45 shared-parameter vector: what each column means, and how one
// sensor's 3 rows of the Jacobian get assembled into it. This is the step
// between the raw per-quantity derivatives (bundle_shared_jacobian.h,
// bundle_linear_jacobian.h) and the accumulator that consumes P-wide rows
// (schur_normal_equations.h's add_sensor).
//
// --- Ordering: unit-major, deliberately NOT matching the PC side ----------
//
// magnet_field_model/calibration/parameterization.py orders type-major (all
// tilts together, then all offsets, then all gains). That is close to the
// worst case for block structure: one sensor's live columns land in 8
// disjoint runs. The two layouts differ by a fixed permutation applied at the
// storage boundary, which is cheap and round-trip testable, and the on-device
// layout is not required to match -- so it is chosen for the accumulator
// instead.
//
// Ordering by UNIT gives two runs per sensor: a border every sensor touches,
// and one contiguous block only its own sensor touches.
//
//   [ 0.. 2]  magnet_pos          3   border
//   [ 3    ]  magnet_strength_mean 1  border
//   [ 4.. 5]  magnet_strength_diff 2  border
//   [ 6..11]  magnet_tilt          6  border   (2 per magnet)
//   [12..22]  sensor_offset_0 (3) + gain_0 (8) = 11   unit block 0
//   [23..33]  sensor_offset_1 (3) + gain_1 (8) = 11   unit block 1
//   [34..44]  sensor_offset_2 (3) + gain_2 (8) = 11   unit block 2
//
// H_ss is then bordered block-diagonal: H_ss[U_i, U_j] = 0 exactly for i != j,
// because that entry needs a sensor whose rows touch both blocks and
// sensor_offset/gain belong to one physical sensor each.
//
// --- Why magnet_tilt sits in the border, where it does not yet belong -----
//
// Under today's PAIRED_ONLY coupling sensor i sees only magnet i, so tilt_i is
// touched by exactly one sensor and *could* live in unit block i -- giving a
// 6-wide border, 13-wide blocks, and a bigger sparsity win (~5.6x against
// dense, vs ~3.8x here).
//
// It is in the border anyway, because modelling cross-magnet interference is
// an intended direction (TODO/cross-magnet-interference.md) and that is the
// one group it moves: with every sensor seeing every magnet, tilt_i is touched
// by all three. Nothing else moves -- sensor_offset and gain stay per-sensor
// no matter how the magnets couple, since no field cross-talk makes sensor j's
// reading depend on sensor i's gain matrix.
//
// Placing it in the border now makes the block geometry INVARIANT across that
// switch: turning on cross-magnet coupling changes only which entries happen
// to be zero, not the layout, so the accumulator needs no restructuring and no
// re-verification. The cost is computing some structurally-zero border entries
// today. That trade is taken deliberately; see TODO/on-device-calibration.md.

#include "math3D.h"
#include "bundle_shared_jacobian.h"
#include "bundle_linear_jacobian.h"
#include "bundle_magnet_pos_gauge.h"
#include "bundle_gnomonic_chart.h"

static constexpr int N_MAGNETS = 3;
static constexpr int N_SENSORS = 3;

// Border: the columns every sensor's rows touch.
static constexpr int COL_MAGNET_POS      = 0;   // 3
static constexpr int COL_STRENGTH_MEAN   = 3;   // 1
static constexpr int COL_STRENGTH_DIFF   = 4;   // 2
static constexpr int COL_MAGNET_TILT     = 6;   // 6  (2 per magnet)
static constexpr int BORDER_WIDTH        = 12;

// Unit blocks: columns only their own sensor's rows touch.
static constexpr int UNIT_BLOCK_WIDTH    = 11;  // offset 3 + gain 8
static constexpr int UNIT_OFFSET_SENSOR_OFFSET = 0;   // 3
static constexpr int UNIT_OFFSET_GAIN          = 3;   // 8

static constexpr int N_SHARED_PARAMS = BORDER_WIDTH + N_SENSORS * UNIT_BLOCK_WIDTH;  // 45

// First column of sensor i's own block.
inline constexpr int unit_block_start(int sensor_index) {
    return BORDER_WIDTH + UNIT_BLOCK_WIDTH * sensor_index;
}

using SharedRow = Eigen::Matrix<float, 3, N_SHARED_PARAMS>;

// STRENGTH_DIFF_BASIS's row for one magnet: parameterization.py's
// [[1,0],[0,1],[-1,-1]] -- magnet 2 carries the negative sum so the three
// per-magnet deviations are constrained to be zero-sum, leaving the absolute
// level entirely to magnet_strength_mean.
inline Vec2 strength_diff_coefficients(int magnet_index) {
    if (magnet_index == 0) return Vec2(1.0f, 0.0f);
    if (magnet_index == 1) return Vec2(0.0f, 1.0f);
    return Vec2(-1.0f, -1.0f);
}

// Prior stddevs, per parameter, in THIS file's column order. Values are
// bundle_params.py's RegularizationSigmas -- the same beliefs about the same
// hardware, not independently chosen. They cannot simply be copied across,
// because that file lays its vector out type-major and this one unit-major;
// filling by group here is the permutation, done once.
//
// Two units notes:
//  - magnet_tilt's 0.06 is radians there, against a rotation-vector
//    parameterization. Here tilt is the gnomonic chart, whose radial
//    coordinate is tan(theta) -- equal to theta to third order, so at a
//    tolerance-sized tilt the number transfers unchanged. The chart is also
//    rotated 90 degrees relative to that one, which an isotropic prior does
//    not notice.
//  - magnet_strength_mean is deliberately UNREGULARIZED (infinity), matching
//    None there. The nominal it would be centred on is a round guess, and
//    this has to work for magnets nobody has measured -- a prior there would
//    drag the fitted field scale toward a number nobody stands behind.
//    Safe despite the parameter being near-degenerate with position and gain
//    (see TODO/cross-magnet-interference.md): a flat direction only survives
//    if it lies ENTIRELY in unregularized coordinates, and every parameter
//    this one trades against is itself strongly regularized, so the prior on
//    the partners supplies curvature along the whole combined direction.
inline Eigen::Matrix<float, N_SHARED_PARAMS, 1> nominal_prior_sigma() {
    Eigen::Matrix<float, N_SHARED_PARAMS, 1> s;
    const float unregularized = std::numeric_limits<float>::infinity();

    s.segment<3>(COL_MAGNET_POS).setConstant(1.0f);        // mm, 3D-printed knob
    s[COL_STRENGTH_MEAN] = unregularized;                   // see above
    s.segment<2>(COL_STRENGTH_DIFF).setConstant(0.1f);     // one batch, graded ~1%
    s.segment<6>(COL_MAGNET_TILT).setConstant(0.06f);      // ~1.7 degrees

    for (int i = 0; i < N_SENSORS; ++i) {
        const int base = unit_block_start(i);
        s.segment<3>(base + UNIT_OFFSET_SENSOR_OFFSET).setConstant(1.8f);   // mT
        // Gain basis index order is aniso {0,1}, sym {2,3,4}, rot {5,6,7} --
        // GAIN_GROUP_BASIS_INDICES in parameterization.py, mirrored by
        // GAIN_BASIS in bundle_linear_jacobian.h.
        s.segment<2>(base + UNIT_OFFSET_GAIN + 0).setConstant(0.25f);   // aniso
        s.segment<3>(base + UNIT_OFFSET_GAIN + 2).setConstant(0.03f);   // sym
        s.segment<3>(base + UNIT_OFFSET_GAIN + 5).setConstant(0.03f);   // rot
    }
    return s;
}

// Accumulates the columns that depend on ONE MAGNET's parameters. Call once
// per magnet contributing to this sensor's reading -- today exactly once
// (PAIRED_ONLY, magnet_index == sensor_index), and once per magnet if
// cross-magnet coupling is turned on, which is why this accumulates (+=)
// rather than assigns. Caller zeroes the row first.
//
// chart_j is that magnet's gnomonic chart Jacobian, built once per magnet per
// solver iteration (bundle_gnomonic_chart.h) -- NOT per sensor or per frame.
//
// `gain` is this sensor's CURRENT gain matrix, and it is not optional. The
// prediction is G*B + offset, so every derivative that acts through the field
// carries a factor of G:  d(G*B)/d(magnet param) = G * dB/d(magnet param).
// evaluate_bundle_jacobian returns the pre-gain dB, exactly as
// VirtualSensor::evaluate returns a pre-gain pose Jacobian -- Math.md 4.E
// flags the same trap there, where [B_g]_x must be built from the pre-gain
// field. Note the asymmetry with add_sensor_columns below: the gain's own
// columns are already complete derivatives and must NOT be multiplied again,
// so this cannot be done by scaling the assembled row.
inline void add_magnet_columns(
    SharedRow& row, int magnet_index, const Mat3& gain, float nominal_strength_mT,
    const SharedJacobianBlock& J, const Eigen::Matrix<float, 3, 2>& chart_j
) {
    row.block<3, 3>(0, COL_MAGNET_POS) +=
        gain * project_magnet_pos(J.d_magnet_pos, magnet_index);

    // Strength enters the prediction as a scalar multiplier, so its whole
    // effect on these 3 rows is the single column J.d_strength, weighted by
    // this magnet's coefficient in each strength basis.
    //
    // nominal_strength_mT is the chain-rule factor, and it is easy to drop:
    // J.d_strength is d(B)/d(strength_mT), an ABSOLUTE derivative, while the
    // fitted parameter is a dimensionless multiplier (see
    // BundleSolver::rebuild_from_params), so
    //   d(B)/d(multiplier) = d(B)/d(strength_mT) * d(strength_mT)/d(multiplier)
    //                      = J.d_strength * nominal_strength_mT.
    // Omitting it makes these columns wrong by exactly the nominal strength --
    // a factor of ~1000 -- which LM absorbs by mis-fitting everything else
    // rather than by failing outright.
    const Vec3 g_strength = gain * J.d_strength * nominal_strength_mT;
    row.block<3, 1>(0, COL_STRENGTH_MEAN) += g_strength;   // MEAN_BASIS is all-ones
    const Vec2 diff = strength_diff_coefficients(magnet_index);
    row.block<3, 1>(0, COL_STRENGTH_DIFF + 0) += g_strength * diff.x();
    row.block<3, 1>(0, COL_STRENGTH_DIFF + 1) += g_strength * diff.y();

    row.block<3, 2>(0, COL_MAGNET_TILT + 2 * magnet_index) +=
        gain * project_magnet_tilt(J.d_magnet_tilt, chart_j);
}

// Accumulates the columns belonging to this SENSOR's own hardware. Called
// exactly once per sensor regardless of coupling -- gain and offset correct a
// sensor's own reading, so no amount of magnetic cross-talk makes them
// depend on another sensor's magnets.
//
// B_field_global is the sensor's total predicted field (summed over whatever
// magnets contribute), since gain multiplies the assembled prediction.
inline void add_sensor_columns(
    SharedRow& row, int sensor_index, const Vec3& B_field_global
) {
    const int base = unit_block_start(sensor_index);
    row.block<3, 3>(0, base + UNIT_OFFSET_SENSOR_OFFSET) += Mat3::Identity();
    for (int k = 0; k < 8; ++k) {
        row.block<3, 1>(0, base + UNIT_OFFSET_GAIN + k) += d_gain(k, B_field_global);
    }
}
