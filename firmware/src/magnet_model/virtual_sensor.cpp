#include "magnet_model/virtual_sensor.h"

VirtualSensor::VirtualSensor(Vec3 sensor_pos_world)
    : sensor_pos_world(sensor_pos_world)
    {}

// Evaluate the field a sensor sees from all three magnets, and a given transformation from the global frame to the knob frame
// The transformation from the global to the knob frame (a translation + a rotation) is encoded in the 3 MagnetPlacements
// t_world also captures the translation part, but is only used for the rotation part of the jacobian
void __not_in_flash_func(VirtualSensor::evaluate)(
        const MagnetPlacement &paired_placement,
        const MagnetPlacement &cross_a,
        const MagnetPlacement &cross_b,
        const Vec3& t_world,
        Vec3 &B_world,
        Matrix3x6f &J) const {


    /** -----
    * Get the local field and the displacement jacobian
    *   ------
    */
    // Get the main contribution from the near magnet.
    Mat3 J_displacement_world; // displacement Jacobian accumulator
    B_world = paired_placement.near_approx_world(sensor_pos_world, J_displacement_world);

    // The other two magnets, as point dipoles.
    // All in the same world frame, so we can accumulate directly.
    Mat3 J_cross_a, J_cross_b;
    B_world += cross_a.far_approx_world(sensor_pos_world, J_cross_a);
    B_world += cross_b.far_approx_world(sensor_pos_world, J_cross_b);
    J_displacement_world += J_cross_a + J_cross_b;

    /**  -----
     * Assemble the full jacobian from the displacement jacobian
     *   -----
     *
     * We now need to take a jacobian w.r.t. sensor_pos_world
     * and use it to build the full 3x6 jacobina w.r.t
     * the knob translation t and the knob rotation w.
     *
     * w here is a rotation vector. We don't use that vector to represent rotation
     * internally instead we use a 3x3 matrix R.
     *
     * Eventually in the solver we solve for a small update to the rotation vector w
     * and use that to update the rotation matrix R.
     *
     * The rotation matrix R isn't passed into this function, its already processed in the
     * MagnetPlacement's passed in.
     */

    // The position of the sensor relative to the knob origin in the world frame.
    // This is the 'lever' with which knob rotation moves the sensor
    const Vec3 v_world = sensor_pos_world - t_world;

    // the knob displacement t is in the opposite direction as sensor_pos_world.
    // So we flip the sign since ∂(v_world)/∂t = -1
    J.Submatrix<3, 3>(0, 0) = -J_displacement_world;

    // The rotation jacobian: J_rot = J_displacement_world * [v_world]_x - [B_world]_x. A
    // skew matrix has a zero diagonal, so a naive 3x3 product spends a third of its
    // multiplies on structural zeros -- but this build sets -ffinite-math-only
    // project-wide (platformio.ini), which licenses GCC to fold x * 0.0f -> 0.0f, and
    // that's enough for the general form below to optimize that out
    // (see TODO/Performance.md's "-O3 project-wide" pass,
    // re-verified for this summed-input call shape in the pass covering the cross-magnet
    // port). Note this uses the pre-gain physical field for the B term, as Math.md 4.E
    // requires -- the sensor gain is applied at the real sensors in SensorController, not
    // here. The -ffinite-math-only fold this relies on can, in
    // principle, discard a genuine NaN arising in J_displacement_world/v_world/B_world
    // before it reaches solve_pose.cpp's all_finite(dx) safety net -- that residual risk
    // is inherent to the flag and documented in TODO/Performance.md, not something this
    // form or a check downstream can fully close.
    J.Submatrix<3, 3>(0, 3) = J_displacement_world * skew_matrix(v_world) - skew_matrix(B_world);
}
