#pragma once
#include "math3D.h"
#include "BicubicField.h"

// The actual Bi-cubic-field interpolation table
constexpr BicubicField CALCULATED_BICUBIC_FIELD(BICUBIC_INTERPOLATION_TABLE, BICUBIC_ORIGIN, BICUBIC_FAR);

struct MagnetModel {
    const Vec3 magnet_pos_knob;
    const Mat3 magnet_rotation;
    // Polarization multiplier for this specific magnet, relative to the single
    // magnet the bicubic table was generated for. 1.0 means "exactly the
    // modelled magnet"; the bundle calibration fits the real spread.
    //
    // This is the per-magnet manufacturing tolerance that causes "Phantom Tilt"
    // (a magnet 5% strong reads, to geometry-only math, as a magnet that moved
    // closer), so it belongs to the magnet rather than to the sensor watching
    // it. Correcting it on the sensor side would be numerically identical today
    // -- ForwardModel pairs sensor i with magnet i one-to-one -- but stops being
    // so the moment cross-magnet interference is modelled, since one scalar per
    // sensor cannot then undo three different magnet strengths.
    const float magnet_strength;

    // We assume BicubicField is passed by reference to avoid copying the grid
    MagnetModel(const BicubicField& field_model, const Vec3& m_local,
                const Mat3 &magnet_rotation = Mat3::Identity(),
                float magnet_strength = 1.0f);

    // Evaluates the local field and populates the 3x3 local Jacobian
    Vec3 evaluate(const Vec3& v_l, Mat3& J_local) const;

private:
    const BicubicField& field_model_;
};
