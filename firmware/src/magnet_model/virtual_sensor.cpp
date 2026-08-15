#include "magnet_model/virtual_sensor.h"

VirtualSensor::VirtualSensor(Vec3 sensor_pos_global)
    : sensor_pos_global(sensor_pos_global)
    {}

// Evaluate the field a sensor sees from all three magnets, and a given translation from the global frame to the knob frame
void __not_in_flash_func(VirtualSensor::evaluate)(const MagnetModel &paired,
        const MagnetPlacement &paired_placement,
        const MagnetPlacement &cross_a,
        const MagnetPlacement &cross_b,
        const Vec3& t,
        Eigen::Matrix<float, 3, 1> &B_field_global,
        Eigen::Matrix<float, 3, 6> &J) const {

    // 1. Global geometry. Shared by every magnet: all three ride the same
    // rigid knob, so there is one v per sensor, not one per pair.
    const Vec3 v = sensor_pos_global - t;

    // 2. Paired magnet, through the interpolated near field.
    //
    // R_total = R * R_mag maps magnet-local straight to global, and is built
    // once per magnet by MagnetModel::place rather than here -- with three
    // sensors now looking at it, computing it per pair would build the same
    // matrix three times. Both places the magnet's own tilt used to appear
    // still collapse onto it:
    //   R * (R_mag * B_local)             ->  R_total * B_local
    //   R * (R_mag * J * R_mag^T) * R^T   ->  R_total * J * R_total^T
    const Mat3 R_total_T = paired_placement.R_total.transpose();

    // Sensor position in the magnet-local frame, per Math.md 3.2. The
    // R_mag^T * m term is a frozen calibration constant, precomputed in
    // MagnetModel rather than re-rotated here every call.
    const Vec3 sensor_magnet_rel = R_total_T * v - paired.magnet_offset_local;

    Mat3 J_local;
    const Vec3 B_local = paired.evaluate(sensor_magnet_rel, J_local);

    // Straight from magnet-local to global. M is the field's gradient in world
    // coordinates (Math.md 4.E).
    Vec3 B_total = paired_placement.R_total * B_local;
    Mat3 M_total = paired_placement.R_total * J_local * R_total_T;

    // 3. The other two magnets, as point dipoles.
    //
    // These come back already in world coordinates, so there is no frame to
    // transform into, nothing to rotate back, and no R J R^T congruence -- the
    // three operations that dominate the paired branch above. That is what
    // makes six extra pairs affordable; see dipole_field().
    Mat3 M_cross_a, M_cross_b;
    const Vec3 B_cross_a = dipole_field(cross_a.moment_world,
                                        (sensor_pos_global - cross_a.centre_world).eval(),
                                        M_cross_a);
    const Vec3 B_cross_b = dipole_field(cross_b.moment_world,
                                        (sensor_pos_global - cross_b.centre_world).eval(),
                                        M_cross_b);

    // 4. Superposition, before assembly rather than after. Fields add, and
    // both Jacobian blocks below are linear in B and M, so one assembly on the
    // summed quantities is exact and costs a third of three assemblies.
    B_total += B_cross_a + B_cross_b;
    M_total += M_cross_a + M_cross_b;

    B_field_global = B_total;

    // 5. Assemble the Jacobian blocks
    J.block<3, 3>(0, 0) = -M_total;

    // J_rot = M * [v]_x - [B_field_global]_x, with both skew products written
    // out. A skew matrix has a zero diagonal, so a general 3x3 product spends a
    // third of its multiplies on structural zeros; column j of [a]_x has only
    // two non-zero entries.
    const float vx = v.x(), vy = v.y(), vz = v.z();
    for (int i = 0; i < 3; ++i) {
        const float m0 = M_total(i, 0), m1 = M_total(i, 1), m2 = M_total(i, 2);
        J(i, 3) = m1 * vz - m2 * vy;
        J(i, 4) = m2 * vx - m0 * vz;
        J(i, 5) = m0 * vy - m1 * vx;
    }

    // ... then -[B]_x, which touches six entries rather than nine. Note this
    // uses the pre-gain physical field, as Math.md 4.E requires, and the
    // summed field rather than the paired magnet's alone.
    const float Bx = B_total.x(), By = B_total.y(), Bz = B_total.z();
    J(0, 4) += Bz;   J(0, 5) -= By;
    J(1, 3) -= Bz;   J(1, 5) += Bx;
    J(2, 3) += By;   J(2, 4) -= Bx;
}
