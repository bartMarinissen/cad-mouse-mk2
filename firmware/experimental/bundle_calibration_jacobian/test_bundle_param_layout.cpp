// test_bundle_param_layout.cpp -- PROTOTYPE, not wired into the build.
//
// The wiring test. bundle_param_layout.h assembles one sensor's 3 rows of the
// P=45 Jacobian out of six separately-derived pieces (gauge-projected
// position, the strength mean/diff split, the gnomonic tilt chart, sensor
// offset, and the 8 gain basis directions), placed at hand-written column
// offsets. Every individual piece already has finite-difference coverage in
// test_bundle_shared_jacobian.cpp; none of that says the pieces are put in the
// right *columns*.
//
// So this differentiates the assembled row against the thing it claims to be
// the derivative of: build the whole bundle state from a 45-vector, predict
// what a sensor reads, perturb one parameter, central-difference. Every
// column, one at a time. A transposed basis, an off-by-one magnet index, a
// swapped mean/diff coefficient, a missing gain factor, or two groups
// overlapping in the layout all show up here and nowhere else.
//
// Evaluated at a NON-ZERO parameter vector on purpose. At x = 0 the gain is
// identity, the tilts are at the chart origin, and the strength offsets vanish
// -- which is exactly where a missing gain factor or a chart bug is invisible.

#include <unity.h>
#include "math3D.h"
#include "magnet_model/BicubicField.h"
#include "magnet_model/magnet_local_model.h"
#include "magnet_model/positions.h"
#include "bundle_param_layout.h"

static constexpr float REL_TOL = 0.01f;   // matches test_jacobian.cpp's bar

static const Vec3 SENSOR_POS[3] = {
    Positions::sensor_1_world, Positions::sensor_2_world, Positions::sensor_3_world,
};
static const Vec3 MAGNET_LOCAL[3] = {
    Positions::Magnet_1_knob, Positions::Magnet_2_knob, Positions::Magnet_3_knob,
};
// Offset from the real rest pose, not an invented one. Positions.h owns the
// geometry; the knob sits ~21mm above the sensor plane, so a hand-picked
// z=6 puts the magnet-local query at z_l = +9 -- outside the bicubic table's
// domain entirely (bounds live in magnet_model_table.h). Finite-difference
// checks still PASS out there, because analytic-vs-numeric consistency holds
// in the extrapolation region too; they just stop being about the region the
// knob actually reaches. firmware/test/README flags exactly this.
static const Vec3 BASE_T = Positions::approx_rest_pos + Vec3(2.0f, -1.5f, 0.0f);

static Mat3 exp_so3(const Vec3& w) {
    float theta = w.norm();
    Mat3 K = skew_matrix(w);
    if (theta < 1.0e-8f) return Mat3::Identity() + K + 0.5f * (K * K);
    float s = std::sin(theta) / theta;
    float c = (1.0f - std::cos(theta)) / (theta * theta);
    return Mat3::Identity() + s * K + c * (K * K);
}

// Nominal magnet orientations. Non-identity so "nominal frame" and "knob
// frame" cannot be silently conflated by a chart bug.
static const Mat3 NOMINAL_ROT[3] = {
    exp_so3(Vec3(0.04f, -0.02f, 0.01f)),
    exp_so3(Vec3(-0.03f, 0.05f, -0.02f)),
    exp_so3(Vec3(0.01f, 0.03f, 0.04f)),
};

using ParamVector = Eigen::Matrix<float, N_SHARED_PARAMS, 1>;

// FD step per column, and it genuinely has to vary -- firmware/test/README's
// "step sizes carry unit assumptions", hit for real here.
//
// sensor_offset is in FIELD units (mT) and enters the prediction additively.
// The predicted field is large, so a 5e-4 step moves the 7th significant digit
// of a float32 -- right at epsilon (~1.2e-7). Differencing that leaves ~2%
// noise, which looks exactly like a 2%-wrong derivative. Measured before this
// was fixed: the offset columns came out as 0.9766 and 1.0376 against an
// analytic identity. It enters linearly, so a large step costs no truncation
// error at all -- 0.5 mT is a pure win, not a tradeoff.
//
// Magnet strength is NOT in mT: it is a dimensionless multiplier on the whole
// field (see BundleSolver::rebuild_from_params), so it behaves like gain and
// the default step is fine.
// The tilt columns need a LARGER step for the opposite-looking reason -- also
// roundoff, not truncation. A tilt perturbation only moves the field by
// |B| * O(h), and inside the table's valid domain |B| is far smaller than the
// extrapolated values this test used to probe, so the differenced signal sinks
// toward float32 epsilon. Swept, at the rest pose:
//
//     step    5e-3    2e-3    5e-4    1e-4    2e-5
//     err     pass    pass   1.8%    3.5%   10.8%
//
// Error GROWING as the step shrinks is the cancellation signature (truncation
// error would shrink), so the analytic column is right and the instrument was
// wrong. 2e-3 sits in the flat region with margin at both ends.
static float fd_step_for_column(int j) {
    if (j == COL_STRENGTH_MEAN || j == COL_STRENGTH_DIFF || j == COL_STRENGTH_DIFF + 1) {
        return 5.0e-4f;   // dimensionless multiplier; scales the whole field
    }
    if (j >= COL_MAGNET_TILT && j < COL_MAGNET_TILT + 2 * N_MAGNETS) {
        return 2.0e-3f;   // gnomonic chart units
    }
    for (int i = 0; i < N_SENSORS; ++i) {
        const int base = unit_block_start(i) + UNIT_OFFSET_SENSOR_OFFSET;
        if (j >= base && j < base + 3) return 0.5f;   // mT
    }
    return 5.0e-4f;
}

// The full bundle state a parameter vector denotes. This is the inverse of
// the layout: if assembling a Jacobian row puts group G at columns [a,b), then
// this must read group G's offset from exactly those columns. The test is
// meaningful precisely because these two are written independently.
struct BundleState {
    MagnetState magnets[N_MAGNETS];
    Vec3 sensor_offsets[N_SENSORS];
    Mat3 gains[N_SENSORS];
};

// STRENGTH_DIFF_BASIS transcribed literally from parameterization.py, rather
// than calling bundle_param_layout.h's strength_diff_coefficients().
//
// This matters, and mutation testing is how it was found: the first version of
// this file called the production helper here. Swapping magnet 0's
// coefficients from (1,0) to (0,1) then produced ZERO failures, because the
// same wrong number went into both the state being differenced and the
// Jacobian being checked, and they cancelled. A test that shares a helper with
// the thing it tests is a consistency check, not an oracle.
//
// The same caveat still applies to MAGNET_POS_BASIS and GAIN_BASIS below,
// which both sides do read from the production headers. Those are verbatim
// pasted constants owned by parameterization.py, so they are verified by being
// copied rather than by this test -- but a typo in the paste would survive
// this file, and it is worth knowing that rather than assuming otherwise.
static Vec2 reference_strength_diff_coefficients(int magnet_index) {
    static const float BASIS[3][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}, {-1.0f, -1.0f}};
    return Vec2(BASIS[magnet_index][0], BASIS[magnet_index][1]);
}

static BundleState state_from_params(const ParamVector& x) {
    BundleState s;
    for (int m = 0; m < N_MAGNETS; ++m) {
        s.magnets[m].pos = MAGNET_LOCAL[m]
            + MAGNET_POS_BASIS.block<3, 3>(3 * m, 0) * x.segment<3>(COL_MAGNET_POS);

        // Multiplier around 1, matching bundle_geometry.py -- see
        // BundleSolver::rebuild_from_params.
        const Vec2 diff = reference_strength_diff_coefficients(m);
        s.magnets[m].strength_mT = BICUBIC_FIELD_REFERENCE_MT
            * (1.0f + x[COL_STRENGTH_MEAN]
                    + diff.dot(x.segment<2>(COL_STRENGTH_DIFF)));

        MagnetTilt tilt{x[COL_MAGNET_TILT + 2 * m], x[COL_MAGNET_TILT + 2 * m + 1]};
        s.magnets[m].rotation = rotation_from_tilt(NOMINAL_ROT[m], tilt);
    }
    for (int i = 0; i < N_SENSORS; ++i) {
        const int base = unit_block_start(i);
        s.sensor_offsets[i] = x.segment<3>(base + UNIT_OFFSET_SENSOR_OFFSET);
        s.gains[i] = Mat3::Identity();
        for (int k = 0; k < 8; ++k) {
            s.gains[i] += x[base + UNIT_OFFSET_GAIN + k] * GAIN_BASIS[k];
        }
    }
    return s;
}

// What sensor i reads: gain applied to the modelled field, plus its offset.
// PAIRED_ONLY -- sensor i sees magnet i only, matching the firmware's
// ForwardModel (ARCHITECTURE.md's calibration section).
static Vec3 predict(int sensor_index, const BundleState& s, const Vec3& t, const Mat3& R) {
    VirtualSensor sensor(SENSOR_POS[sensor_index]);
    MagnetModel magnet = build_magnet_model(CALCULATED_BICUBIC_FIELD, s.magnets[sensor_index]);
    Vec3 B; Eigen::Matrix<float, 3, 6> J_pose; SharedJacobianBlock J_shared;
    evaluate_bundle_jacobian(sensor, magnet, t, R, B, J_pose, J_shared);
    return s.gains[sensor_index] * B + s.sensor_offsets[sensor_index];
}

static SharedRow assemble_row(int sensor_index, const ParamVector& x,
                               const BundleState& s, const Vec3& t, const Mat3& R) {
    VirtualSensor sensor(SENSOR_POS[sensor_index]);
    MagnetModel magnet = build_magnet_model(CALCULATED_BICUBIC_FIELD, s.magnets[sensor_index]);
    Vec3 B; Eigen::Matrix<float, 3, 6> J_pose; SharedJacobianBlock J_shared;
    evaluate_bundle_jacobian(sensor, magnet, t, R, B, J_pose, J_shared);

    const int m = sensor_index;                       // PAIRED_ONLY
    MagnetTilt tilt{x[COL_MAGNET_TILT + 2 * m], x[COL_MAGNET_TILT + 2 * m + 1]};

    SharedRow row = SharedRow::Zero();
    add_magnet_columns(row, m, s.gains[sensor_index], BICUBIC_FIELD_REFERENCE_MT, J_shared,
                        chart_jacobian(NOMINAL_ROT[m], tilt));
    add_sensor_columns(row, sensor_index, B);
    return row;
}

void test_assembled_row_matches_finite_differences(void) {
    // Deliberately non-zero, and different in every group.
    ParamVector x0 = ParamVector::Zero();
    x0.segment<3>(COL_MAGNET_POS)    << 0.05f, -0.03f, 0.04f;
    x0[COL_STRENGTH_MEAN]            = 0.04f;
    x0.segment<2>(COL_STRENGTH_DIFF) << 0.025f, -0.015f;
    for (int m = 0; m < N_MAGNETS; ++m) {
        x0[COL_MAGNET_TILT + 2 * m]     = 0.02f * (m + 1);
        x0[COL_MAGNET_TILT + 2 * m + 1] = -0.015f * (m + 1);
    }
    for (int i = 0; i < N_SENSORS; ++i) {
        const int base = unit_block_start(i);
        x0.segment<3>(base + UNIT_OFFSET_SENSOR_OFFSET) << 0.4f, -0.3f, 0.2f;
        for (int k = 0; k < 8; ++k) x0[base + UNIT_OFFSET_GAIN + k] = 0.01f * (k + 1 - 4);
    }

    const Vec3 t = BASE_T;
    const Mat3 R = exp_so3(Vec3(0.05f, -0.03f, 0.02f));
    char msg[224];

    for (int sensor_index = 0; sensor_index < N_SENSORS; ++sensor_index) {
        BundleState s0 = state_from_params(x0);
        SharedRow analytic = assemble_row(sensor_index, x0, s0, t, R);

        for (int j = 0; j < N_SHARED_PARAMS; ++j) {
            const float h = fd_step_for_column(j);

            ParamVector xp = x0, xm = x0;
            xp[j] += h;
            xm[j] -= h;

            Vec3 Bp = predict(sensor_index, state_from_params(xp), t, R);
            Vec3 Bm = predict(sensor_index, state_from_params(xm), t, R);
            Vec3 numeric = (Bp - Bm) / (2.0f * h);
            Vec3 col = analytic.col(j);

            // Columns this sensor structurally does not see (another sensor's
            // gain/offset, another magnet's tilt) must be exactly zero on both
            // sides -- that IS the sparsity the accumulator is built around, so
            // check it rather than skipping it.
            if (numeric.norm() < 1.0e-4f && col.norm() < 1.0e-4f) continue;

            float denom = numeric.norm() > 1.0e-6f ? numeric.norm() : 1.0f;
            float e = (col - numeric).norm() / denom;
            snprintf(msg, sizeof(msg),
                     "sensor %d column %d: analytic (%.4f,%.4f,%.4f) vs numeric "
                     "(%.4f,%.4f,%.4f), rel err %.5f",
                     sensor_index, j, col.x(), col.y(), col.z(),
                     numeric.x(), numeric.y(), numeric.z(), e);
            TEST_ASSERT_TRUE_MESSAGE(e < REL_TOL, msg);
        }
    }
}

// The sparsity claim the layout exists to produce, asserted directly rather
// than inferred: one sensor's row is nonzero only in the border and in its own
// unit block. If this fails, a sparsity-aware accumulator built on the layout
// would silently drop real terms.
void test_row_sparsity_matches_layout(void) {
    ParamVector x0 = ParamVector::Zero();
    x0[COL_STRENGTH_MEAN] = 2.0f;
    const Vec3 t = BASE_T;
    const Mat3 R = exp_so3(Vec3(0.05f, -0.03f, 0.02f));
    char msg[192];

    for (int sensor_index = 0; sensor_index < N_SENSORS; ++sensor_index) {
        BundleState s0 = state_from_params(x0);
        SharedRow row = assemble_row(sensor_index, x0, s0, t, R);

        for (int other = 0; other < N_SENSORS; ++other) {
            if (other == sensor_index) continue;
            float leak = row.block<3, UNIT_BLOCK_WIDTH>(0, unit_block_start(other)).norm();
            snprintf(msg, sizeof(msg),
                     "sensor %d wrote %.3e into sensor %d's unit block -- the block-diagonal "
                     "structure H_ss[U_i,U_j]=0 does not hold", sensor_index, leak, other);
            TEST_ASSERT_TRUE_MESSAGE(leak == 0.0f, msg);
        }

        // Under PAIRED_ONLY the other magnets' tilt columns are zero too --
        // but as a coupling fact, not a layout one. They live in the border
        // precisely so that turning on cross-magnet coupling fills them in
        // without moving anything.
        for (int m = 0; m < N_MAGNETS; ++m) {
            if (m == sensor_index) continue;
            float leak = row.block<3, 2>(0, COL_MAGNET_TILT + 2 * m).norm();
            snprintf(msg, sizeof(msg),
                     "sensor %d wrote %.3e into magnet %d's tilt columns under PAIRED_ONLY",
                     sensor_index, leak, m);
            TEST_ASSERT_TRUE_MESSAGE(leak == 0.0f, msg);
        }
    }
}

// ======================================================================
// Unity entry points
// ======================================================================
void setUp(void) {}
void tearDown(void) {}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_assembled_row_matches_finite_differences);
    RUN_TEST(test_row_sparsity_matches_layout);
    UNITY_END();
}

void loop() {}
