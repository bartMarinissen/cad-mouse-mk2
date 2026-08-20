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
// Being frame-agnostic allows us to calculate this in the world frame instead
// of needing to go to the magnet frame and back. That means a single rotation
// instead of two rotations.
//
// Deliberately no r -> 0 branch, unlike MagnetModel::evaluate. This models the
// magnets a sensor does NOT sit under, which the knob's geometry keeps a
// triangle side away; it is never evaluated near its own source.
Vec3 dipole_field(const Vec3& m, const Vec3& r, Mat3& J);

// Forward declaration
struct MagnetPlacement;

struct MagnetModel {
    // The magnet's origin in the knob frame -- its geometric centre, which
    // is both where the bicubic table's own (r, z)=(*, 0) sits and where
    // the far-field dipole belongs, so no separate centre point is needed.
    const Vec3 magnet_pos_knob;
    // The orientation of the magnet compared to the knob frame.
    // At perfect manufacturing this would be the identity.
    const Mat3 magnet_rotation;
    // This magnet's polarization (remanence, Br), in mT. Defaults to
    // BICUBIC_FIELD_REFERENCE_MT (magnet_model_table.h) -- i.e. "exactly the
    // magnet the bicubic table was generated for" -- so a magnet weaker or
    // stronger than that reads out proportionally weaker/stronger; the bundle
    // calibration fits the real per-unit value.
    const float magnet_strength_mT;

    // --- The magnet info as required for the dipole model ------

    // The dipole model is used for what a *cross* sensor needs.
    // Precomputed here because they are frozen calibration constants, and
    // because they depend only on the magnet -- ForwardModel lifts them out of
    // the sensor loop, so each is built once per solve rather than per pair.

    // Dipole moment magnitude, mT*mm^3: the table's reference moment scaled by
    // this magnet's own strength, through the same strength_ratio. 
    //
    // Magnitude only. The polarization points along the magnet's local -z, so
    // the moment VECTOR is -moment_mT_mm3 times the magnet's own axis, which
    // in world coordinates is the third column of R * magnet_rotation.
    const float moment_mT_mm3;

    // We assume BicubicField is passed by reference to avoid copying the grid
    MagnetModel(const BicubicField& field_model, const Vec3& m_local,
                const Mat3 &magnet_rotation = identity3(),
                float magnet_strength_mT = BICUBIC_FIELD_REFERENCE_MT);

    /**
     * Evaluate the magnetic field acording to this model at position p_local in the local magnet frame
     * 
     * Returns the magnetic field, and outputs the jacobian in J_local.
     * 
     * Note, since this works in the magnet local frame, magnet_pos_knob and magnet_rotation
     * are ignored here
     */
    Vec3 evaluate(const Vec3& p_local, Mat3& J_local) const;

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


// One magnet's frozen geometry combined with the pose being evaluated: what
// the magnet looks like from the world frame right now.
//
// Due to cross-magnet modeling it is more efficient to pre-calculate this 
// once and re-use it.
//
// Note: the way one should create these is though MagnetModel.place()
struct MagnetPlacement {
    // R * magnet_rotation, mapping magnet-local directly to world. The
    // interpolated path needs it to get into and out of the magnet's frame.
    Mat3 R_total;
    // The magnet-local origin in the world frame -- the magnet's geometric
    // centre. Both the near (table) and far (dipole) models measure their
    // displacement from here, since it's where each of them puts the magnet.
    Vec3 origin_world;
    // The dipole's moment vector in world coordinates, magnitude
    // moment_mT_mm3 along the magnet's own -z (its polarization axis).
    // Carrying orientation as this one rotated vector is what lets the far
    // field skip the frame machinery entirely -- see dipole_field().
    Vec3 moment_world;

    // The underlying magnet model.
    // Note, we know that the magnet-model outlives this because they live 
    // for effectively the entire lifetime of the program.
    const MagnetModel& magnet;

    /**
     * Get the field of this magnet using the far approximation in the world frame
     * at position p_world in the world frame. This uses a dipole model,
     * placed at this->origin_world -- the magnet's geometric centre.
     *
     * Returns the magnetic field, outputs the jacobian w.r.t. p_world in J_world.
     */
    Vec3 far_approx_world(const Vec3& p_world, Mat3& J_world) const;
    
    /**
     * Get the field of this magnet using the near approximation in the world frame
     * at position p_world in the world frame. This uses interpolation based on 
     * an exact model.
     * 
     * Returns the magnetic field,  outputs the jacobian w.r.t. p_world  in J_world.
     */
    Vec3 near_approx_world(const Vec3& p_world, Mat3& J_world) const;
};
