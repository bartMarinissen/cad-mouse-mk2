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

void evaluate_bundle_jacobian(
    const Sensor& sensor, const BicubicField& field, const MagnetState& magnet_state,
    const Vec3& t, const Mat3& R,
    Vec3& B_field_global, Eigen::Matrix<float, 3, 6>& J_pose, SharedJacobianBlock& J_shared
) {
    // The trial MagnetModel: immutable once built, like every MagnetModel,
    // but built fresh from whatever the solver's mutable MagnetState holds
    // *right now* -- this is the actual answer to "how do the calibration
    // parameters get fed through the call". Cheap: see the header comment on
    // MagnetState for why this construction doesn't compete with the
    // BicubicField lookup evaluate() is about to do anyway.
    MagnetModel magnet(field, magnet_state.pos, magnet_state.rotation, magnet_state.strength_mT);

    sensor.evaluate(magnet, t, R, B_field_global, J_pose);   // the actual call
    evaluate_shared_jacobian(sensor, magnet, t, R, B_field_global, J_pose, J_shared);
}

// --- How this composes at the ForwardModel level ---------------------------
//
// ForwardModel::evaluate's existing loop (forward_model.cpp) calls
// sensors_[i].evaluate(magnets_[i], t, R, B_i, J_i) once per sensor, against
// its own const magnets_[3]. A bundle-calibration solver instead holds its
// own MagnetState[3] (the parameters actually being fit) alongside the
// existing Sensor[3] (never fit -- sensor positions are fixed, see
// CalibrationParams.h), and evaluates through that mutable state every
// iteration:
//
//   MagnetState trial_magnets[3] = { ... current LM iterate ... };
//   for (int i = 0; i < 3; ++i) {
//       evaluate_bundle_jacobian(sensors[i], CALCULATED_BICUBIC_FIELD,
//                                 trial_magnets[i], t, R,
//                                 B_i, Jp_i, J_shared[i]);
//       B_field.block<3,1>(3*i, 0) = B_i;
//       J_pose.block<3,6>(3*i, 0) = Jp_i;
//   }
//   // ... LM computes a step, updates trial_magnets[i].pos/.rotation/
//   // .strength_mT (and t, R) in place, and the next iteration calls
//   // evaluate_bundle_jacobian again with the same objects, new values ...
//
// J_shared[i] is naturally block-sparse in magnet index already -- sensor i
// only ever sees magnet i (PAIRED_ONLY coupling, which is the only mode the
// firmware's ForwardModel implements today), so there is no 9x9 dense shared
// block to assemble, just 3 independent 3x7 ones (3 pos + 3 tilt raw + 1
// strength), exactly mirroring bundle_geometry.py's per-sensor structure.
//
// BicubicField::evaluate and MagnetModel::evaluate still run exactly once
// per sensor per trial evaluation here, same as today's pose-only solve --
// reconstructing MagnetModel from MagnetState each call doesn't add a second
// forward pass, it's what makes the one forward pass see this iteration's
// actual trial parameters instead of frozen construction-time ones.
