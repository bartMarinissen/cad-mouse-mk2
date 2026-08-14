#pragma once
#include "math3D.h"
#include "BicubicField.h"

// The actual Bi-cubic-field interpolation table
constexpr BicubicField CALCULATED_BICUBIC_FIELD(BICUBIC_INTERPOLATION_TABLE, BICUBIC_ORIGIN, BICUBIC_FAR);

struct MagnetModel {
    const Vec3 magnet_pos_knob;
    const Mat3 magnet_rotation;
    // This magnet's polarization (remanence, Br), in mT. Defaults to
    // BICUBIC_FIELD_REFERENCE_MT (magnet_model_table.h) -- i.e. "exactly the
    // magnet the bicubic table was generated for" -- so a magnet weaker or
    // stronger than that reads out proportionally weaker/stronger; the bundle
    // calibration fits the real per-unit value.
    //
    // This is the per-magnet manufacturing tolerance that causes "Phantom Tilt"
    // (a magnet 5% strong reads, to geometry-only math, as a magnet that moved
    // closer), so it belongs to the magnet rather than to the sensor watching
    // it. Correcting it on the sensor side would be numerically identical today
    // -- ForwardModel pairs sensor i with magnet i one-to-one -- but stops being
    // so the moment cross-magnet interference is modelled, since one scalar per
    // sensor cannot then undo three different magnet strengths.
    const float magnet_strength_mT;

    // magnet_rotation^T * magnet_pos_knob, precomputed.
    //
    // Math.md 3.2 writes the sensor position in the magnet-local frame as
    //   v_l = R_total^T (s - t) - R_mag^T m
    // where the second term depends only on frozen calibration constants. Doing
    // it that way lets VirtualSensor::evaluate subtract a constant vector instead of
    // performing a second 3x3 rotation on every call.
    const Vec3 magnet_offset_local;

    // We assume BicubicField is passed by reference to avoid copying the grid
    MagnetModel(const BicubicField& field_model, const Vec3& m_local,
                const Mat3 &magnet_rotation = Mat3::Identity(),
                float magnet_strength_mT = BICUBIC_FIELD_REFERENCE_MT);

    // Evaluates the local field and populates the 3x3 local Jacobian
    Vec3 evaluate(const Vec3& v_l, Mat3& J_local) const;

private:
    const BicubicField& field_model_;
    // magnet_strength_mT / BICUBIC_FIELD_REFERENCE_MT, precomputed once so
    // evaluate() does a multiply on every call instead of a divide.
    const float strength_ratio_;
};
