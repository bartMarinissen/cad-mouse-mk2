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

    // J_rot = M * [v]_x - [B_field_global]_x, with both skew products written
    // out. A skew matrix has a zero diagonal, so a general 3x3 product spends a
    // third of its multiplies on structural zeros; column j of [a]_x has only
    // two non-zero entries.
    //
    // A file-scoped `#pragma GCC optimize("finite-math-only")` was tried here
    // to let GCC fold the clean skew_matrix()-based form itself (see
    // TODO/Performance.md's "-O3, scoped -ffinite-math-only" pass) -- but
    // measured on the real build, it doesn't reach parity (51 calls vs. this
    // hand form's 33), unlike the whole-TU command-line-flag measurement it
    // was extrapolated from. Kept written out until a real per-file
    // build-flag override is tried instead of a pragma.
    const float vx = v(0), vy = v(1), vz = v(2);
    for (int i = 0; i < 3; ++i) {
        const float m0 = M(i, 0), m1 = M(i, 1), m2 = M(i, 2);
        J(i, 3) = m1 * vz - m2 * vy;
        J(i, 4) = m2 * vx - m0 * vz;
        J(i, 5) = m0 * vy - m1 * vx;
    }

    // ... then -[B]_x, which touches six entries rather than nine. Note this
    // uses the pre-gain physical field, as Math.md 4.E requires.
    const float Bx = B_field_global(0), By = B_field_global(1), Bz = B_field_global(2);
    J(0, 4) += Bz;   J(0, 5) -= By;
    J(1, 3) -= Bz;   J(1, 5) += Bx;
    J(2, 3) += By;   J(2, 4) -= Bx;
}
