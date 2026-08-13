#pragma once
// PROTOTYPE / SKETCH -- not wired into the firmware build (platformio.ini's
// src_dir/include_dir/test_dir point at firmware/src, firmware/include,
// firmware/test specifically; this directory is deliberately none of those,
// so it stays invisible to the build until it's promoted on purpose). Written to
// answer one question: can the bundle-calibration Jacobian's magnet-position and
// magnet-tilt columns be computed as a thin wrapper around Sensor::evaluate(),
// reusing its output instead of re-deriving the forward pass through
// MagnetModel/BicubicField a second time? Answer below: yes.

#include "math3D.h"
#include "magnet_model/sensor.h"
#include "magnet_model/magnet_local_model.h"

// Per-(sensor, magnet) RAW derivative of the predicted field w.r.t. this
// magnet's own placement and strength. "Raw" meaning: before the gauge-fixing
// bases in magnet_field_model/calibration/parameterization.py project these
// down onto the bundle's actual free parameters (3 magnet_pos + 2 magnet_tilt
// shared across all three magnets, not 9+6 independent numbers per magnet).
// That projection is a handful of small, fixed matrix multiplies -- constants
// computable once on the PC side and pasted in as constexpr data, same as
// BICUBIC_INTERPOLATION_TABLE already is -- done once per magnet per solver
// iteration, not inside this per-sensor function.
struct SharedJacobianBlock {
    Mat3 d_magnet_pos;   // dB/dm_j,  knob-frame, all 3 raw columns live
    Mat3 d_magnet_tilt;  // dB/d(eps); column 2 (spin about the magnet's own
                          // polarization axis) is structurally dead -- left
                          // populated here (matches whatever the skew product
                          // produces) and dropped by the caller's projection,
                          // exactly like the PC side's TILT_UNIT_BASIS does.
    Vec3 d_strength;      // dB/d(magnet_strength_mT)
};

// Computes the three calibration-parameter derivatives above for one
// sensor/magnet pair, from Sensor::evaluate()'s OWN output -- called after
// it, not instead of it. It never touches MagnetModel::evaluate or
// BicubicField again: every expensive part of the forward pass (the bicubic
// table lookup) already happened inside the evaluate() call that produced
// B_field_global/J_pose, and everything this function needs is recoverable
// from that output plus data the caller already had (t, R, and the Sensor/
// MagnetModel objects' own public fields).
//
// --- The math -------------------------------------------------------------
//
// Sensor::evaluate already builds M := R_total * J_local * R_total^T (the
// world-frame field gradient) to fill J_pose's translation block as -M. That
// same M is exactly what every shared derivative below is built from too --
// recovered as `neg_M = J_pose.block<3,3>(0,0)`, not recomputed.
//
//   d(B)/d(m_j)   = -M @ R                              (magnet position)
//   d(B)/d(eps)   =  M @ R @ [d]_x  -  R @ [b_knob]_x    (magnet tilt)
//   d(B)/d(s_mT)  =  B_field_global / s_mT               (magnet strength)
//
// where, in the knob frame:
//   d       = R^T (sensor.sensor_pos_global - t) - magnet.magnet_pos_knob
//             (magnet -> sensor vector)
//   b_knob  = R^T @ B_field_global
//             (the predicted field, expressed in the knob frame)
//
// Both are one 3x3-matrix/vector product away from data Sensor::evaluate's
// *caller* already has -- no need to reach into Sensor's private internals
// (v, sensor_magnet_rel, B_local) at all, even though the derivation below
// used them as scratch quantities to get here. (Full derivation: B_world =
// R R_mag B_local(R_mag^T(R^T v - m_j)); differentiate w.r.t. m_j and w.r.t.
// a left rotation perturbation of R_mag, matching bundle_geometry.py's
// predict_and_jacobians() term for term. b_knob's simplification -- R_mag
// cancelling out entirely -- falls out of B_local = R_mag^T R^T B_world.)
//
// --- Why no SO(3) left-Jacobian correction ---------------------------------
//
// bundle_geometry.py needs one (so3_left_jacobian) because scipy parameterizes
// each frame's pose as a rotation *vector* relative to a fixed reference, and
// its analytic derivatives are naturally against a left perturbation instead
// -- the correction reconciles the two. A device-side solver built the way
// solve_pose.cpp already is (Gauss-Newton stepping directly in the se(3)
// tangent space, re-exponentiating R every iteration, never holding a global
// rotation-vector parameterization at all) doesn't introduce that mismatch in
// the first place, so there is nothing to correct for -- not "the correction
// is small here", it structurally does not apply.
void evaluate_shared_jacobian(
    const Sensor& sensor, const MagnetModel& magnet,
    const Vec3& t, const Mat3& R,
    const Vec3& B_field_global, const Eigen::Matrix<float, 3, 6>& J_pose,
    SharedJacobianBlock& J_shared
);

// --- Feeding calibration parameters through the (immutable) forward model -
//
// MagnetModel's magnet_pos_knob/magnet_rotation/magnet_strength_mT are const
// members, fixed at construction (magnet_local_model.h) -- deliberately:
// ForwardModel's own comment calls this out as what makes "the calibration
// is constant for the lifetime of the controllers" a compiler-enforced fact
// rather than a convention, for the RUNTIME/shipped calibration. A solver
// actively fitting these parameters needs a separate, genuinely mutable
// representation of "the trial value right now" -- that's MagnetState below
// -- and builds a fresh (still-immutable) MagnetModel from it on every
// evaluation. That is not fighting the const design, it's what the design
// is asking for: during a fit there is no single "the" MagnetModel yet, only
// a sequence of trial ones, and each one should still be immutable once
// built. The reconstruction itself is not the expensive part: MagnetModel's
// constructor is one 3x3-matrix/vector product (magnet_offset_local),
// dwarfed by the ~260-op BicubicField lookup Sensor::evaluate performs on
// every call regardless, trial or not.
//
// sensor_offset and gain have no equivalent struct here because they never
// reach this deep: neither MagnetModel nor Sensor holds them at all -- gain
// lives in SensorController, applied to the raw reading before the model
// ever sees it; sensor_offset is likewise applied outside ForwardModel/
// Sensor entirely (see CalibrationParams.h and TODO/sensor-gain-calibration.md).
// Their derivatives (bundle_linear_jacobian.h) are linear post-multiplies of
// B_field_global, with no MagnetModel/Sensor construction involved at all --
// there is nothing to feed through here for those two groups, structurally,
// not just as a simplification.
struct MagnetState {
    Vec3 pos;            // magnet_pos_knob
    Mat3 rotation;        // magnet_rotation
    float strength_mT;    // magnet_strength_mT
};

// The actual entry point a solver calls: builds the trial MagnetModel from
// MagnetState and calls Sensor::evaluate() itself -- not left as something
// the caller must remember to do separately and thread the results in by
// hand, which is what the (still available, now internal-use) function
// above required. One call in (a sensor, a field table, a trial magnet
// state, a trial pose), everything a solver needs for one (sensor, magnet)
// pair at that trial point, out.
void evaluate_bundle_jacobian(
    const Sensor& sensor, const BicubicField& field, const MagnetState& magnet_state,
    const Vec3& t, const Mat3& R,
    Vec3& B_field_global, Eigen::Matrix<float, 3, 6>& J_pose, SharedJacobianBlock& J_shared
);

// This file covers magnet_pos, magnet_tilt, and magnet_strength -- the
// parameter groups that need real chain rule through the forward model.
// The other five of parameterization.py's eight groups (sensor_offset,
// gain_aniso, gain_sym, gain_rot, and the strength mean/diff split) are
// linear post-multiplies of B_field_global with no chain-rule content, so
// they don't belong in a function that's about reusing Sensor::evaluate's
// forward pass -- see bundle_linear_jacobian.h for those, in full.
