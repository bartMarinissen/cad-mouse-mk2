#include "magnet_model/magnet_local_model.h"
#include "magnet_model/magnet_model_table.h"
#include <cmath>


MagnetModel::MagnetModel(const BicubicField& field_model, const Vec3& m_local)
    : field_model_(field_model), m_local_(m_local) {}

const Vec3& MagnetModel::get_m_local() const {
    return m_local_;
}

/**
 * p[in] is the position where we evaluate the field relative to the magnet
 * J_local[out] the jacobian
 * returns the field at p[in] oriented along the magnet axis
 */
Vec3 MagnetModel::evaluate(const Vec3& p, Mat3& J_local) const {

    // get radius of the given point compared to the z axis
    float r = std::sqrt(p[0]*p[0] + p[1]*p[1]);

    Vec3 B_val, d_dr, d_dz; 
    field_model_.evaluate(r, p[2], B_val, d_dr, d_dz);

    // Decompose the measured field
    // Because we need to reconsitute the magnetic field in the x and y direction based on Br
    float Br = B_val[0];
    // skip y which is stupidly zero
    float Bz = B_val[2];
    
    float dBr_dr = d_dr[0];
    // skip y which is stupidly zero
    float dBz_dr = d_dr[2];
    float dBr_dz = d_dz[0];
    // skip y which is stupidly zero
    float dBz_dz = d_dz[2];

    Vec3 B_local;
    const float EPSILON = 1e-6f;

    if (r < EPSILON) {
        B_local = Vec3(0.0f, 0.0f, Bz);
        
        J_local = Vec3(dBr_dr, dBr_dr, dBz_dz).asDiagonal();

    } else {
        // Cosines to rotate our 2d vector back to 3d in line with x, y
        float cx = p[0] / r;
        float cy = p[1] / r;
        // Higher order terms for the jacobian
        float cx2 = cx * cx;
        float cy2 = cy * cy;
        float cxcy = cx * cy;
        float Br_over_r = Br / r;

        B_local = Vec3(Br * cx, Br * cy, Bz);

        float Br_dr_minus_Br_r = dBr_dr - Br_over_r;

        J_local <<
            dBr_dr * cx2 + Br_over_r * cy2,  Br_dr_minus_Br_r * cxcy,         dBr_dz * cx,
            Br_dr_minus_Br_r * cxcy,         dBr_dr * cy2 + Br_over_r * cx2,  dBr_dz * cy,
            dBz_dr * cx,                     dBz_dr * cy,                     dBz_dz
        ;
    }

    return B_local;
}