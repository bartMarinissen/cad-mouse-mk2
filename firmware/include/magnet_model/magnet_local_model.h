#pragma once
#include "math3D.h"
#include "BicubicField.h"

// The actual Bi-cubic-field interpolation table
constexpr BicubicField CALCULATED_BICUBIC_FIELD(BICUBIC_INTERPOLATION_TABLE, BICUBIC_ORIGIN, BICUBIC_FAR);

struct MagnetModel {
    const Vec3 magnet_pos_knob;
    const Mat3 magnet_rotation;


    // We assume BicubicField is passed by reference to avoid copying the grid
    MagnetModel(const BicubicField& field_model, const Vec3& m_local, const Mat3 &magnet_rotation = Mat3::Identity());

    // Evaluates the local field and populates the 3x3 local Jacobian
    Vec3 evaluate(const Vec3& v_l, Mat3& J_local) const;

private:
    const BicubicField& field_model_;
};
