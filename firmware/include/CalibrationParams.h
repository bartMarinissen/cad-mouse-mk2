#pragma once

#include "math3D.h"

// Everything the bundle calibration (magnet_field_model/) fits for one physical
// unit: the per-sensor correction that turns a raw reading into mT, and the
// knob/magnet geometry the forward model solves against.
//
// This is constant for the lifetime of every controller that uses it. The
// calibrated controllers are *constructed from* it and hold it const -- see
// CalibratedControllers in Controllers.h -- rather than being reconfigured
// later, so there is no window in which half the pipeline is running on one
// calibration and half on another.
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
struct CalibrationParams {
  // --- Sensor correction, applied by SensorController to the raw reading. ---
  // corrected = sensor_gain[i] * raw[i] - sensor_offset_mT[i]
  Mat3 sensor_gain[3];
  // Per-sensor DC offset (Hall zero point plus ambient field), in mT, subtracted
  // after the gain matrix -- i.e. in read_mT()'s output space, not raw counts.
  Vec3 sensor_offset_mT[3];

  // --- Knob/magnet geometry, consumed by the forward model. ---
  // Knob-frame magnet positions, bottom-face reference. Replaces the nominal
  // Positions::Magnet_i_knob values as the model's actual geometry.
  Vec3 magnet_pos_knob[3];
  // Per-magnet axis tilt in the knob frame. Sensor::evaluate already rotates
  // into and out of the magnet frame with this; every construction site simply
  // passed identity until now.
  Mat3 magnet_rotation[3];

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
  // KNOWN GAP: magnet_field_model/calibration/export.py still emits a
  // dimensionless multiplier centred on 1.0 (its geometry.magnet_strength,
  // relative to local_field.py's own 600mT guess), not an mT value on this
  // scale. Pasting that output straight into this field is wrong -- it needs
  // multiplying by whatever Br the calibration's 1.0 actually represents
  // first. Left as-is deliberately; updating export.py is a separate,
  // not-yet-done piece of work.
  //
  // Config::defaultCalibration() sets these to BICUBIC_FIELD_REFERENCE_MT,
  // which is the consistent default rather than a placeholder: it makes the
  // ratio above exactly 1.0, matching that Config::magnet_gains is a
  // hand-tuned scalar per sensor that already carries the whole field scale
  // (the gain-carries-scale gauge, not the fit's).
  float magnet_strength_mT[3];
};
