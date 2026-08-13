#include "bundle_shared_jacobian.h"

void evaluate_shared_jacobian(
    const Sensor& sensor, const MagnetModel& magnet,
    const Vec3& t, const Mat3& R,
    const Vec3& B_field_global, const Eigen::Matrix<float, 3, 6>& J_pose,
    SharedJacobianBlock& J_shared
) {
    // Recovered, not recomputed: Sensor::evaluate already built this to fill
    // J_pose's translation block.
    const Mat3 neg_M = J_pose.block<3, 3>(0, 0);

    const Vec3 v = sensor.sensor_pos_global - t;
    const Vec3 d = R.transpose() * v - magnet.magnet_pos_knob;
    const Vec3 b_knob = R.transpose() * B_field_global;

    // Position: d(B)/dm_j = -M @ R = neg_M @ R
    J_shared.d_magnet_pos = neg_M * R;

    // Tilt: d(B)/d(eps) = M @ R @ [d]_x - R @ [b_knob]_x
    //                   = -(neg_M @ R) @ [d]_x - R @ [b_knob]_x
    J_shared.d_magnet_tilt =
        -J_shared.d_magnet_pos * skew_matrix(d) - R * skew_matrix(b_knob);

    // Strength: the field is exactly linear in magnet_strength_mT (it enters
    // only as a multiplier inside MagnetModel::evaluate), so the derivative
    // is just the prediction itself, rescaled.
    J_shared.d_strength = B_field_global / magnet.magnet_strength_mT;
}

// --- Sketch of how this composes at the ForwardModel level ----------------
//
// ForwardModel::evaluate's existing loop (forward_model.cpp) already calls
// sensors_[i].evaluate(magnets_[i], t, R, B_i, J_i) once per sensor and
// copies the results into the flat 9x1/9x6 outputs. The bundle-calibration
// version is the same loop with one more (cheap) call added per iteration:
//
//   for (int i = 0; i < 3; ++i) {
//       sensors_[i].evaluate(magnets_[i], t, R, B_i, Jp_i);      // unchanged
//       evaluate_shared_jacobian(sensors_[i], magnets_[i], t, R,
//                                 B_i, Jp_i, J_shared[i]);        // new
//       B_field.block<3,1>(3*i, 0) = B_i;
//       J_pose.block<3,6>(3*i, 0) = Jp_i;
//   }
//
// J_shared[i] is naturally block-sparse in magnet index already -- sensor i
// only ever sees magnet i (PAIRED_ONLY coupling, which is the only mode the
// firmware's ForwardModel implements today), so there is no 9x9 dense shared
// block to assemble, just 3 independent 3x7 ones (3 pos + 3 tilt raw + 1
// strength), exactly mirroring bundle_geometry.py's per-sensor structure.
//
// Not shown: BicubicField::evaluate and MagnetModel::evaluate are called
// exactly once per sensor per frame here, same as today's pose-only solve --
// the whole point of routing through Sensor::evaluate's own output rather
// than a second forward pass.
