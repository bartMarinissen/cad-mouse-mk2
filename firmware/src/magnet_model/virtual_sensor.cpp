#include "magnet_model/virtual_sensor.h"

VirtualSensor::VirtualSensor(Vec3 sensor_pos_global)
    : sensor_pos_global(sensor_pos_global)
    {}

// Evaluate the field a sensor sees from a given magnet, and a given translation from the global frame to the knob frame
void __not_in_flash_func(VirtualSensor::evaluate)(const MagnetModel &magnet, const Vec3& t,
        const Mat3& R,
        Vec3 &B_field_global,
        Matrix3x6f &J) const {

    // R_total = R * R_mag maps magnet-local straight to global. Both places the
    // magnet's own tilt used to appear collapse onto it:
    //   R * (R_mag * B_local)             ->  R_total * B_local
    //   R * (R_mag * J * R_mag^T) * R^T   ->  R_total * J * R_total^T
    // which takes the 3x3 product count from four down to two.
    const Mat3 R_total = R * magnet.magnet_rotation;
    const Mat3 R_total_T = ~R_total;

    // 1. Global geometry
    const Vec3 v = sensor_pos_global - t;

    // 2. Sensor position in the magnet-local frame, per Math.md 3.2. The
    // R_mag^T * m term is a frozen calibration constant, precomputed in
    // MagnetModel rather than re-rotated here every call.
    const Vec3 sensor_magnet_rel = R_total_T * v - magnet.magnet_offset_local;

    // 3. Evaluate local field and Jacobian
    Mat3 J_local;
    const Vec3 B_local = magnet.evaluate(sensor_magnet_rel, J_local);

    // 4. Straight from magnet-local to global
    B_field_global = R_total * B_local;

    // 6. Assemble the Jacobian blocks
    const Mat3 M = R_total * J_local * R_total_T;

    J.Submatrix<3, 3>(0, 0) = -M;

    // J_rot = M * [v]_x - [B_field_global]_x. A skew matrix has a zero
    // diagonal, so a naive 3x3 product spends a third of its multiplies on
    // structural zeros -- but this build sets -ffinite-math-only
    // project-wide (platformio.ini), which licenses GCC to fold
    // x * 0.0f -> 0.0f, and that's enough for the general form below to
    // reach the same soft-float call count as writing the products out by
    // hand (see TODO/Performance.md's "-O3 project-wide" pass; an earlier
    // attempt scoping the flag to just this file via a pragma measured 51
    // calls, not the 33 the project-wide flag actually reaches -- scoping
    // via #pragma/__attribute__((optimize(...))) doesn't reliably combine
    // an -O level with an -f flag the way real command-line flags do).
    // Note this uses the pre-gain physical field for the B term, as Math.md
    // 4.E requires. The -ffinite-math-only fold this relies on can, in
    // principle, discard a genuine NaN arising in M/v/B_field_global before
    // it reaches solve_pose.cpp's all_finite(dx) safety net -- that residual
    // risk is inherent to the flag and documented in TODO/Performance.md,
    // not something this form or a check downstream can fully close.
    J.Submatrix<3, 3>(0, 3) = M * skew_matrix(v) - skew_matrix(B_field_global);
}
