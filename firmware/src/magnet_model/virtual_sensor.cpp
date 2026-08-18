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
        Eigen::Matrix<float, 3, 1> &B_world,
        Eigen::Matrix<float, 3, 6> &J) const {


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
    // So we flip the sign since ∂v_world/∂t = -1
    J.block<3, 3>(0, 0) = -J_displacement_world;

    // The rotation jacobian is much more complicated. See math.md for more details behind the formula.
    // J_rot = J_displacement_world * [v_world]_x - [B_world]_x
    //
    // Here we compute this in two steps by hand-rolling the skew-matrices to take full advantage
    // of the zeroes it has. 
    
    // A skew matrix has a zero diagonal, so a general 3x3 product spends a
    // third of its multiplies on structural zeros; column j of [a]_x has only
    // two non-zero entries.
    // So here we calculate J_displacement_world * [v_world]_x
    for (int i = 0; i < 3; ++i) {
        const float m0 = J_displacement_world(i, 0), m1 = J_displacement_world(i, 1), m2 = J_displacement_world(i, 2);
        J(i, 3) = m1 * v_world.z() - m2 * v_world.y();
        J(i, 4) = m2 * v_world.x() - m0 * v_world.z();
        J(i, 5) = m0 * v_world.y() - m1 * v_world.x();
    }
    // ... then -[B]_x, which touches six entries rather than nine.
    // Note, since we do not apply gain in the forward model when solving the pose, we don't need to worry about it here
    // despite the note in math.md
    J(0, 4) += B_world.z();   J(0, 5) -= B_world.y();
    J(1, 3) -= B_world.z();   J(1, 5) += B_world.x();
    J(2, 3) += B_world.y();   J(2, 4) -= B_world.x();
}
