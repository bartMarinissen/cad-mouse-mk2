#pragma once
#include "math3D.h"
#include "BicubicField.h"
#include "magnet_model_table.h"

// The actual Bi-cubic-field interpolation table
constexpr BicubicField CALCULATED_BICUBIC_FIELD(BICUBIC_INTERPOLATION_TABLE, BICUBIC_ORIGIN, BICUBIC_FAR);

class MagnetModel {
public:
    // We assume BicubicField is passed by reference to avoid copying the grid
    MagnetModel(const BicubicField& field_model, const Vec3& m_local);

    const Vec3& get_m_local() const;

    // Evaluates the local field and populates the 3x3 local Jacobian
    Vec3 evaluate(const Vec3& v_l, Mat3& J_local) const;

private:
    const BicubicField& field_model_;
    Vec3 m_local_; 
};
