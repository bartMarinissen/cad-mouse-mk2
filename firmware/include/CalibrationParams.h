#pragma once

#include "math3D.h"

// Everything the bundle calibration (magnet_field_model/) fits for one physical
// unit: the per-sensor correction that turns a raw reading into mT, and the
// knob/magnet geometry the forward model solves against.
//
// This is constant for the lifetime of every controller that uses it.
// sensorController() and motionController() (Controllers.h) are each
// *constructed from* it and hold it const, rather than being reconfigured
// later: it lets the compiler fold the values into the hot paths, and it makes
// reassigning one a compile error rather than a bug to find later.
//
// GAIN CONVENTION. The one genuinely subtle part, quoted from
// magnet_field_model/calibration/export.py:
//
//   This calibration applies gain to the **model**   :  pred = G_fit @ B_model + o_fit
//   The firmware applies gain to the **measurement** :  corrected = G_fw @ raw - o_fw
//
// so with the installed magnet polarity folded in,
//   G_fw = inv(MAGNET_POLARITY * G_fit)   -> lands near -I
//   o_fw = G_fw @ o_fit
//
// The values in this struct are already in the firmware-side (G_fw, o_fw) form;
// export.py does that conversion before emitting them.
//
// Sensor world positions are deliberately NOT here. calibration/bundle_geometry.py
// fixes them as the definition of the world frame and absorbs real sensor
// placement error into the magnet position offsets instead, so they are not a
// fitted quantity. They stay in magnet_model/positions.h.
//
// PLAIN ARRAYS, NOT MATRIX TYPES. This struct is exactly what CalibrationStorage
// copies to and from flash -- the stored file is a magic/version header, these
// 300 bytes verbatim, and a CRC. That only works if the struct is trivially
// copyable, and BLA::Matrix is not: it declares non-trivial constructors, so
// memcpy'ing into a struct of Mat3/Vec3 would be undefined behaviour even
// though the storage underneath really is a bare float array.
//
// Two more things fall out of it. The layout is now the language's guarantee
// rather than the matrix library's implementation detail, so `sizeof == 300`
// is a fact instead of an observation. And the ordering is ROW-major,
// matching how numpy ravels on the Python side that writes these files --
// BLA::Matrix's own storage happens to be row-major too so toMat3()/toVec3()
// below are now a direct per-element copy rather than a reinterpret-and-reorder.
//
// Consumers convert with toMat3()/toVec3() below, at construction, which is
// where they already copied these values out.
struct CalibrationParams {
  // --- Sensor correction, applied by SensorController to the raw reading. ---
  // corrected = sensor_gain[i] * raw[i] - sensor_offset_mT[i]
  // Indexed [sensor][row][col].
  float sensor_gain[3][3][3];
  // Per-sensor DC offset (Hall zero point plus ambient field), in mT, subtracted
  // after the gain matrix -- i.e. in read_mT()'s output space, not raw counts.
  // Indexed [sensor][xyz].
  float sensor_offset_mT[3][3];

  // --- Knob/magnet geometry, consumed by the forward model. ---
  // Knob-frame magnet positions, bottom-face reference. Replaces the nominal
  // Positions::Magnet_i_knob values as the model's actual geometry.
  // Indexed [magnet][xyz].
  float magnet_pos_knob[3][3];
  // Per-magnet axis tilt in the knob frame. VirtualSensor::evaluate already rotates
  // into and out of the magnet frame with this; every construction site simply
  // passed identity until now. Indexed [magnet][row][col].
  float magnet_rotation[3][3][3];

  // Per-magnet polarization (remanence, Br), in mT. Consumed by
  // MagnetModel::evaluate, which scales the six cylindrical field quantities
  // by magnet_strength_mT[i] / BICUBIC_FIELD_REFERENCE_MT
  // (magnet_model/magnet_model_table.h) before the x/y decomposition -- that
  // reference is the polarization the bicubic table itself was generated at,
  // an arbitrary but fixed number the notebook bakes into the generated
  // header, so this field can be a real physical quantity in mT rather than a
  // dimensionless multiplier tied to that arbitrary choice.
  //
  // These are only meaningful together with sensor_gain, under the det(G)=1
  // gauge that produced them (calibration/bundle_geometry.py's
  // renormalize_gauge). Gain scale and magnet strength are the same degree of
  // freedom -- a sensor reading 5% high and its magnet being 5% strong differ
  // only through cross-talk -- and the fit deliberately moves that freedom out
  // of the gain and into here, leaving det(G) == 1. So do not take a
  // sensor_gain from one calibration and a magnet_strength_mT from another,
  // and do not "normalize" one without the other.
  //
  // These now mean roughly what they say. The fit and this firmware both
  // model cross-magnet field -- every sensor sees the other two magnets,
  // ~28.58mm away -- so it is no longer absorbed into these strengths. They
  // were EFFECTIVE values while it was, and older stored calibrations are
  // still those values: they were fitted to compensate a term the firmware
  // did not have, and are not interchangeable with ones fitted since.
  // Recalibrate rather than reusing a blob from before that change.
  //
  // Still not a substitute for measuring a magnet, since the det(G)=1 gauge
  // above is what fixes the scale.
  //
  // Config::defaultCalibration() sets these to BICUBIC_FIELD_REFERENCE_MT,
  // which is the consistent default rather than a placeholder: it makes the
  // ratio above exactly 1.0, matching that Config::magnet_gains is a
  // hand-tuned scalar per sensor that already carries the whole field scale
  // (the gain-carries-scale gauge, not the fit's).
  float magnet_strength_mT[3];
};

// The stored file is a raw copy of the struct above, so this has to hold.
// If it ever fires, the format changed -- bump kVersion in CalibrationStorage.h
// rather than quietly updating the number.
static_assert(sizeof(CalibrationParams) == 300,
              "CalibrationParams is the on-disk payload; its size is the format");

// --- Turning the stored arrays into the BLA types the model works in. ---
//
// BLA::Matrix's storage is natively row-major (storage[i*Cols+j] -- see
// math3D.h / TODO/eigen-to-bla-migration.md), which is exactly how
// float[3][3] already lays out, and matches numpy's default on the Python
// side that writes these files. So this is
// just a direct per-element copy -- no reordering, and stated here once
// rather than by every caller.
inline Mat3 toMat3(const float m[3][3]) {
  Mat3 out;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      out(i, j) = m[i][j];
  return out;
}

inline Vec3 toVec3(const float v[3]) {
  return Vec3(v[0], v[1], v[2]);
}
