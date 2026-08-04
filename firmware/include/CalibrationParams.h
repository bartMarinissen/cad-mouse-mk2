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

  // Per-magnet polarization multiplier.
  //
  // NOT YET CONSUMED: MagnetModel::evaluate does not scale its output by this,
  // so these must stay at 1.0 for now -- which is consistent rather than a
  // placeholder, because today's Config::magnet_gains still carries the whole
  // field scale. It is carried here so the on-flash format does not need a
  // version bump when it does get wired up.
  //
  // When it does: it belongs in MagnetModel::evaluate, scaling Br, Bz and the
  // four derivatives before the x/y decomposition (6 multiplies, exact, since
  // the field enters linearly). Folding it into sensor_gain instead is free
  // today -- ForwardModel pairs sensor i with magnet i one-to-one, so scaling
  // magnet i by s_i is identical to scaling sensor i by 1/s_i -- but that
  // equivalence dies the moment cross-magnet interference is modelled (see
  // TODO/cross-magnet-interference.md), and it muddles MagnetModel's units in
  // the meantime. Note also that these values are only meaningful under the
  // det(G)=1 gauge that produced them (calibration/bundle_geometry.py's
  // renormalize_gauge): gain scale and magnet strength are the same degree of
  // freedom, and the fit deliberately moved it out of the gain and into here.
  float magnet_strength[3];
};
