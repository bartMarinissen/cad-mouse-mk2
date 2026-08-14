#pragma once
// PROTOTYPE / SKETCH -- companion to bundle_shared_jacobian.h. That file
// covers the two parameter groups that need real chain rule through the
// forward model (magnet_pos, magnet_tilt) plus magnet_strength. This file is
// the other five groups from parameterization.py's BLOCKS: sensor_offset,
// gain_aniso, gain_sym, gain_rot, and the magnet_strength mean/diff split.
// None of them touch VirtualSensor/MagnetModel/BicubicField at all -- they are
// fixed linear maps of quantities the forward pass already produced.

#include "math3D.h"

// GAIN_BASIS: the same 8 exactly-traceless 3x3 matrices as
// magnet_field_model/calibration/parameterization.py's GAIN_BASIS -- literal
// constants, not derived on-device (nothing here runs the SVD/trace
// construction that file uses to build them; that happens once, on the PC,
// same as MAGNET_POS_BASIS/TILT_UNIT_BASIS below).
//
// Index groups mirror GAIN_GROUP_BASIS_INDICES exactly:
//   gain_aniso: {0, 1}      gain_sym: {2, 3, 4}      gain_rot: {5, 6, 7}
inline const Mat3 GAIN_BASIS[8] = {
    (Mat3() <<  1, 0,  0,   0, 0,  0,   0, 0, -1).finished(),   // aniso 0
    (Mat3() <<  0, 0,  0,   0, 1,  0,   0, 0, -1).finished(),   // aniso 1
    (Mat3() <<  0, 1,  0,   1, 0,  0,   0, 0,  0).finished(),   // sym (0,1)
    (Mat3() <<  0, 0,  1,   0, 0,  0,   1, 0,  0).finished(),   // sym (0,2)
    (Mat3() <<  0, 0,  0,   0, 0,  1,   0, 1,  0).finished(),   // sym (1,2)
    (Mat3() <<  0, 0,  0,   0, 0, -1,   0, 1,  0).finished(),   // rot (skew e0)
    (Mat3() <<  0, 0,  1,   0, 0,  0,  -1, 0,  0).finished(),   // rot (skew e1)
    (Mat3() <<  0,-1,  0,   1, 0,  0,   0, 0,  0).finished(),   // rot (skew e2)
};

// d(B_i)/d(gain param at basis index k) = GAIN_BASIS[k] @ B_field_global_i.
// Per-sensor, not per-magnet: gain corrects a sensor's own reading, so this
// only ever needs sensor i's own prediction -- the same B_field_global
// VirtualSensor::evaluate already returned for the pose solve.
inline Vec3 d_gain(int basis_index, const Vec3& B_field_global_i) {
    return GAIN_BASIS[basis_index] * B_field_global_i;
}

// d(B_i)/d(sensor_offset_i) = Identity. Doesn't warrant a function -- any
// caller assembling a Jacobian column for sensor i's own offset just writes
// a 3x3 identity block directly, same as parameterization.py's
// _replicate_per_unit(np.eye(3), N_SENSORS) does.

// --- Magnet strength: mean + differential -----------------------------
//
// Each magnet's raw d_strength (from bundle_shared_jacobian.h) is nonzero
// only in its own sensor's 3 rows, under PAIRED_ONLY coupling -- so the
// mean/diff split is pure bookkeeping across the three already-computed
// per-magnet columns, done once per frame:
//
//   d(pred)/d(mean)  = [ d_strength[0] ;  d_strength[1] ;  d_strength[2] ]
//   d(pred)/d(diff0) = [ d_strength[0] ;  0              ; -d_strength[2] ]
//   d(pred)/d(diff1) = [ 0              ;  d_strength[1] ; -d_strength[2] ]
//
// (STRENGTH_MEAN_BASIS / STRENGTH_DIFF_BASIS in parameterization.py, applied
// to the 3-vector of per-magnet strengths -- here that 3-vector is 3
// separate Vec3 field-columns instead of 3 scalars, because strength's
// effect on the 9-vector prediction is a whole 3x1 block per magnet, not a
// single number; the basis coefficients themselves are identical.)

// --- Magnet position / tilt gauge, for completeness -----------------------
//
// bundle_shared_jacobian.h's d_magnet_pos (3 raw columns) and d_magnet_tilt
// (3 raw columns, 1 dead) are pre-gauge. Projecting them onto the bundle's
// actual free shape/tilt parameters is MAGNET_POS_BASIS (9x3) and
// TILT_UNIT_BASIS (3x2), applied once per magnet per frame -- again fixed
// numeric constants computed on the PC (parameterization.py's SVD-derived
// nullspace basis), pasted in rather than recomputed on-device. Not
// reproduced here since the values are the same kind of pasted-constant
// pattern as GAIN_BASIS above, just with a different shape.
