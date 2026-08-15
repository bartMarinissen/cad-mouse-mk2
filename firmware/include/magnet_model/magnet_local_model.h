#pragma once
#include "math3D.h"
#include "BicubicField.h"

// The actual Bi-cubic-field interpolation table
constexpr BicubicField CALCULATED_BICUBIC_FIELD(BICUBIC_INTERPOLATION_TABLE, BICUBIC_ORIGIN, BICUBIC_FAR);

// Field and gradient of an ideal point dipole, in whatever frame the caller
// expresses m and r in.
//
//   m [in]  dipole moment, mT*mm^3 (see DIPOLE_MOMENT_AT_REFERENCE_MT_MM3)
//   r [in]  displacement from the dipole to the field point, mm
//   J [out] dB_a/dr_b, in that same frame
//   returns the field there, mT
//
// Being frame-agnostic is the point rather than a nicety. A magnet's
// orientation reaches this formula entirely through m -- one rotated vector --
// so a caller holding a world-frame moment gets a world-frame gradient
// straight out, skipping the transform-in, rotate-out and R J R^T congruence
// that the interpolated path needs (Math.md 4.F). The table cannot do that:
// it is tabulated in (r, z), so it must be handed magnet-local coordinates.
//
// Deliberately no r -> 0 branch, unlike MagnetModel::evaluate. This models the
// magnets a sensor does NOT sit under, which the knob's geometry keeps a
// triangle side away; it is never evaluated near its own source.
Vec3 dipole_field(const Vec3& m, const Vec3& r, Mat3& J);

// One magnet's frozen geometry combined with the pose being evaluated: what
// the magnet looks like from the world frame right now.
//
// Every field here depends on the magnet and the pose but NOT on which sensor
// is looking, which is the whole reason the type exists. Once each sensor sees
// all three magnets there are nine (sensor, magnet) pairs per evaluation but
// still only three magnets, so ForwardModel builds these once per magnet and
// passes them down rather than letting VirtualSensor rebuild them per pair.
struct MagnetPlacement {
    // R * magnet_rotation, mapping magnet-local directly to world. The
    // interpolated path needs it to get into and out of the magnet's frame.
    Mat3 R_total;
    // The dipole's location: the magnet's geometric centre, in world
    // coordinates. Cross terms measure their displacement from here.
    Vec3 centre_world;
    // The dipole's moment vector in world coordinates, magnitude
    // moment_mT_mm3 along the magnet's own -z (its polarization axis).
    // Carrying orientation as this one rotated vector is what lets the far
    // field skip the frame machinery entirely -- see dipole_field().
    Vec3 moment_world;
};

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
    // it. That distinction used to be cosmetic, when each sensor saw exactly
    // one magnet and a per-sensor scalar could have absorbed it. It no longer
    // is: every sensor now sees all three magnets, and one scalar per sensor
    // cannot undo three different magnet strengths.
    const float magnet_strength_mT;

    // magnet_rotation^T * magnet_pos_knob, precomputed.
    //
    // Math.md 3.2 writes the sensor position in the magnet-local frame as
    //   v_l = R_total^T (s - t) - R_mag^T m
    // where the second term depends only on frozen calibration constants. Doing
    // it that way lets VirtualSensor::evaluate subtract a constant vector instead of
    // performing a second 3x3 rotation on every call.
    const Vec3 magnet_offset_local;

    // --- The same magnet, as the far-field model sees it ---
    //
    // Both are what a *cross* sensor needs: one that this magnet is not paired
    // with, and so evaluates through dipole_field() rather than the table.
    // Precomputed here because they are frozen calibration constants, and
    // because they depend only on the magnet -- ForwardModel lifts them out of
    // the sensor loop, so each is built once per solve rather than per pair.

    // This magnet's geometric centre in the knob frame, half a magnet along
    // its own axis above magnet_pos_knob, which is the BOTTOM FACE. The
    // equivalent dipole belongs at the centre; siting it at the bottom face
    // instead is a tens-of-percent error at cross-magnet range, not a rounding
    // one.
    const Vec3 centre_knob;

    // Dipole moment magnitude, mT*mm^3: the table's reference moment scaled by
    // this magnet's own strength, through the same strength_ratio_ the
    // interpolated path uses. So a magnet 5% strong is 5% strong in both
    // models by construction, rather than by two constants being kept in
    // agreement.
    //
    // Magnitude only. The polarization points along the magnet's local -z, so
    // the moment VECTOR is -moment_mT_mm3 times the magnet's own axis, which
    // in world coordinates is the third column of R * magnet_rotation.
    const float moment_mT_mm3;

    // We assume BicubicField is passed by reference to avoid copying the grid
    MagnetModel(const BicubicField& field_model, const Vec3& m_local,
                const Mat3 &magnet_rotation = Mat3::Identity(),
                float magnet_strength_mT = BICUBIC_FIELD_REFERENCE_MT);

    // Evaluates the local field and populates the 3x3 local Jacobian
    Vec3 evaluate(const Vec3& v_l, Mat3& J_local) const;

    // This magnet as seen from the world frame at pose (t, R). Cheap, and
    // called once per magnet per forward-model evaluation rather than once
    // per sensor/magnet pair -- see MagnetPlacement.
    MagnetPlacement place(const Vec3& t, const Mat3& R) const;

private:
    const BicubicField& field_model_;
    // magnet_strength_mT / BICUBIC_FIELD_REFERENCE_MT, precomputed once so
    // evaluate() does a multiply on every call instead of a divide.
    const float strength_ratio_;
};
