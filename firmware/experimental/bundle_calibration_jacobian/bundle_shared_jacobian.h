#pragma once
// PROTOTYPE / SKETCH -- not wired into the firmware build (platformio.ini's
// src_dir/include_dir/test_dir point at firmware/src, firmware/include,
// firmware/test specifically; this directory is deliberately none of those,
// so it stays invisible to the build until it's promoted on purpose). Written to
// answer one question: can the bundle-calibration Jacobian's magnet-position and
// magnet-tilt columns be computed as a thin wrapper around VirtualSensor::evaluate(),
// reusing its output instead of re-deriving the forward pass through
// MagnetModel/BicubicField a second time? Answer below: yes.

#include "math3D.h"
#include "magnet_model/virtual_sensor.h"
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

// --- Feeding calibration parameters through the (immutable) forward model -
//
// MagnetModel's magnet_pos_knob/magnet_rotation/magnet_strength_mT are const
// members, fixed at construction (magnet_local_model.h) -- deliberately:
// ForwardModel's own comment calls this out as what makes "the calibration
// is constant for the lifetime of the controllers" a compiler-enforced fact
// rather than a convention, for the RUNTIME/shipped calibration. A solver
// actively fitting these parameters needs a separate, genuinely mutable
// representation of "the trial value right now" -- that's MagnetState below.
//
// IMPORTANT: MagnetState is per-MAGNET, per-SOLVER-ITERATION -- there are 3
// of them, shared across every frame. A frame's pose (t, R) is the thing
// that's per-FRAME (there are ~60 of those). Build each trial MagnetModel
// ONCE per iteration from build_magnet_model() below, then reuse those same
// 3 objects across all ~60 frames' worth of VirtualSensor::evaluate calls -- do NOT
// rebuild one per (frame, sensor) pair. This isn't a micro-optimization:
// measured (host x86 -O2, static instruction count, same method as
// TODO/Performance.md's tallies), MagnetModel's constructor is 155
// instructions against 523 for one full VirtualSensor::evaluate chain
// (MagnetModel::evaluate 111 + BicubicField::evaluate 412) -- about 30% of
// one evaluation's cost, not the "dwarfed, doesn't matter" this comment used
// to claim without having measured it. Reconstructing per-frame instead of
// per-iteration means paying that 30% up to 60x more often than necessary --
// roughly 22% of an entire iteration's compute wasted on rebuilding the same
// 3 magnets over and over. std::move does not help here: there is no heap
// allocation anywhere in this chain (EIGEN_NO_MALLOC, fixed-size types only)
// for a move to avoid copying -- the 155 instructions are Eigen's generic-
// assignment-kernel overhead computing R^T @ m, a computation, not a
// transfer, and moving a Vec3/Mat3 compiles to the same copy either way.
// The only real lever is not doing the computation 60x more than needed.
//
// sensor_offset and gain have no equivalent struct here because they never
// reach this deep: neither MagnetModel nor VirtualSensor holds them at all --
// both are applied to the raw reading in SensorController::read_mT(), before
// the model ever sees it (ARCHITECTURE.md, "Calibration subsystem -- two
// independent layers", owns that split; CalibrationParams.h owns the stored
// layout and the det(G)=1 gauge).
// Their derivatives (bundle_linear_jacobian.h) are linear post-multiplies of
// B_field_global, with no MagnetModel/VirtualSensor construction involved at all --
// there is nothing to feed through here for those two groups, structurally,
// not just as a simplification.
struct MagnetState {
    Vec3 pos;            // magnet_pos_knob
    Mat3 rotation;        // magnet_rotation
    float strength_mT;    // magnet_strength_mT
};

// Builds the trial (immutable) MagnetModel for one magnet from its current
// mutable state. Call once per magnet per solver iteration (3 times), in
// the OUTER loop -- not once per frame. See the comment above MagnetState
// for the measured cost of getting this wrong.
inline MagnetModel build_magnet_model(const BicubicField& field, const MagnetState& state) {
    return MagnetModel(field, state.pos, state.rotation, state.strength_mT);
}

// The per-(sensor, frame) entry point: takes an ALREADY-BUILT MagnetModel
// (from build_magnet_model, called once per magnet per iteration, not here),
// calls VirtualSensor::evaluate() itself, and computes SharedJacobianBlock from
// that same call's output -- it never touches MagnetModel::evaluate or
// BicubicField a second time, since every expensive part of the forward
// pass (the bicubic table lookup) already happened inside the evaluate()
// call above. Call this once per (sensor, frame) pair -- 3 sensors x ~60
// frames per iteration -- passing the SAME 3 MagnetModel objects (built
// once, outside the frame loop) to every frame, since the magnets don't
// change within one iteration, only the poses do.
//
// --- The math -------------------------------------------------------------
//
// VirtualSensor::evaluate already builds M := R_total * J_local * R_total^T (the
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
// Both are one 3x3-matrix/vector product away from data already in hand at
// this point -- no need to reach into VirtualSensor's private internals (v,
// sensor_magnet_rel, B_local) at all, even though the derivation used them
// as scratch quantities to get here. (Full derivation: B_world =
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
void evaluate_bundle_jacobian(
    const VirtualSensor& sensor, const MagnetModel& magnet,
    const Vec3& t, const Mat3& R,
    Vec3& B_field_global, Eigen::Matrix<float, 3, 6>& J_pose, SharedJacobianBlock& J_shared
);

// This file covers magnet_pos, magnet_tilt, and magnet_strength -- the
// parameter groups that need real chain rule through the forward model.
// The other five of parameterization.py's eight groups (sensor_offset,
// gain_aniso, gain_sym, gain_rot, and the strength mean/diff split) are
// linear post-multiplies of B_field_global with no chain-rule content, so
// they don't belong in a function that's about reusing VirtualSensor::evaluate's
// forward pass -- see bundle_linear_jacobian.h for those, in full.
