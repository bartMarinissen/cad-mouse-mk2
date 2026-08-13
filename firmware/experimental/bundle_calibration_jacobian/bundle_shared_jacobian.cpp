#include "bundle_shared_jacobian.h"

void evaluate_bundle_jacobian(
    const Sensor& sensor, const MagnetModel& magnet,
    const Vec3& t, const Mat3& R,
    Vec3& B_field_global, Eigen::Matrix<float, 3, 6>& J_pose, SharedJacobianBlock& J_shared
) {
    sensor.evaluate(magnet, t, R, B_field_global, J_pose);   // the actual call

    // Recovered, not recomputed: Sensor::evaluate (above) already built this
    // to fill J_pose's translation block.
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

// --- How this composes across a whole solver iteration ---------------------
//
// The two-level structure matters: MagnetState is per-magnet, shared across
// every frame; a frame's pose is per-frame. Building MagnetModel is the
// OUTER loop (3 calls), evaluating it against each frame's pose is the INNER
// loop (3 sensors x ~60 frames = ~180 calls) -- reusing the same 3 built
// objects, not rebuilding one per (sensor, frame) pair. See the cost
// comment above MagnetState for the measured reason this split exists.
//
//   MagnetState trial_magnets[3] = { ... current LM iterate ... };
//   MagnetModel magnets[3] = {
//       build_magnet_model(CALCULATED_BICUBIC_FIELD, trial_magnets[0]),
//       build_magnet_model(CALCULATED_BICUBIC_FIELD, trial_magnets[1]),
//       build_magnet_model(CALCULATED_BICUBIC_FIELD, trial_magnets[2]),
//   };                                                        // 3x per iteration
//
//   for (int f = 0; f < n_frames; ++f) {                      // ~60x per iteration
//       for (int i = 0; i < 3; ++i) {                         // x3 sensors
//           evaluate_bundle_jacobian(sensors[i], magnets[i],
//                                     frame[f].t, frame[f].R,
//                                     B_i, Jp_i, J_shared[f][i]);
//           B_field[f].block<3,1>(3*i, 0) = B_i;
//           J_pose[f].block<3,6>(3*i, 0) = Jp_i;
//       }
//   }
//   // ... LM computes a step from all of the above, updates trial_magnets[i]
//   // and every frame[f].t/.R in place, and the next iteration rebuilds
//   // magnets[3] once (not 180 times) before looping over frames again ...
//
// J_shared[f][i] is naturally block-sparse in magnet index already -- sensor
// i only ever sees magnet i (PAIRED_ONLY coupling, which is the only mode
// the firmware's ForwardModel implements today), so there is no 9x9 dense
// shared block to assemble, just 3 independent 3x7 ones (3 pos + 3 tilt raw
// + 1 strength) per frame, exactly mirroring bundle_geometry.py's per-sensor
// structure.
