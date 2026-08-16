#include "magnet_model/magnet_local_model.h"
#include "magnet_model/magnet_model_table.h"
#include <cmath>


MagnetModel::MagnetModel(const BicubicField& field_model, const Vec3& m_local,
                         const Mat3 &magnet_rotation, float magnet_strength_mT)
    : magnet_pos_knob(m_local), magnet_rotation(magnet_rotation),
      magnet_strength_mT(magnet_strength_mT),
      magnet_offset_local(~magnet_rotation * m_local),
      field_model_(field_model),
      strength_ratio_(magnet_strength_mT / BICUBIC_FIELD_REFERENCE_MT) {}


/**
 * p[in] is the position where we evaluate the field relative to the magnet
 * J_local[out] the jacobian
 * returns the field at p[in] oriented along the magnet axis
 */
Vec3 __not_in_flash_func(MagnetModel::evaluate)(const Vec3& p, Mat3& J_local) const {
    // return value
    Vec3 B_local;
    // get radius of the given point compared to the z axis
    // TODO: possible optimization, do the bicubic interpolation on Z, r^2? 
    // This also solves the r < EPSILON singularity.
    float r = sqrtf(p(0)*p(0) + p(1)*p(1));

    Vec2 B_cylindrical, d_dr, d_dz;
    field_model_.evaluate(r, p(2), B_cylindrical, d_dr, d_dz);


    // Decompose the measured field
    // Because we need to reconsitute the magnetic field in the x and y direction based on Br
    //
    // Also Apply this magnet's strength ratio here
    float Br = B_cylindrical(0) * strength_ratio_;
    // skip y which is stupidly zero
    float Bz = B_cylindrical(1) * strength_ratio_;

    float dBr_dr = d_dr(0) * strength_ratio_;
    float dBz_dr = d_dr(1) * strength_ratio_;
    float dBr_dz = d_dz(0) * strength_ratio_;
    float dBz_dz = d_dz(1) * strength_ratio_;


    const float EPSILON = 1e-6f;

    if (r < EPSILON) {
        B_local = Vec3(
            p(0) * dBr_dr,
            p(1) * dBr_dr,
            Bz
        );

        // dBr_dr, dBr_dr, dBz_dz on the diagonal, zero elsewhere. No
        // .asDiagonal() equivalent in BLA; this is the only call site, so a
        // direct fill beats a shared helper.
        J_local = Mat3(
            dBr_dr, 0.0f,   0.0f,
            0.0f,   dBr_dr, 0.0f,
            0.0f,   0.0f,   dBz_dz
        );

    } else {
        // optimization to avoid 3 divisions;
        float r_reciprocal = 1.0f/r;
        // Cosines to rotate our 2d vector back to 3d in line with x, y
        float cx = p(0) * r_reciprocal;
        float cy = p(1) * r_reciprocal;
        // Higher order terms for the jacobian
        float cx2 = cx * cx;
        float cy2 = cy * cy;
        float cxcy = cx * cy;
        float Br_over_r = Br * r_reciprocal;

        B_local = Vec3(Br * cx, Br * cy, Bz);

        float Br_dr_minus_Br_r = dBr_dr - Br_over_r;

        // BLA's variadic constructor fills row-major, same order the comma
        // operator used to.
        J_local = Mat3(
            dBr_dr * cx2 + Br_over_r * cy2,  Br_dr_minus_Br_r * cxcy,         dBr_dz * cx,
            Br_dr_minus_Br_r * cxcy,         dBr_dr * cy2 + Br_over_r * cx2,  dBr_dz * cy,
            dBz_dr * cx,                     dBz_dr * cy,                     dBz_dz
        );
    }

    return B_local;
}
