#include "magnet_model/magnet_local_model.h"
#include "magnet_model/magnet_model_table.h"
#include <cmath>


MagnetModel::MagnetModel(const BicubicField& field_model, const Vec3& m_local,
                         const Mat3 &magnet_rotation, float magnet_strength_mT)
    : magnet_pos_knob(m_local), magnet_rotation(magnet_rotation),
      magnet_strength_mT(magnet_strength_mT),
      magnet_offset_local(magnet_rotation.transpose() * m_local),
      // The centre is half a magnet along the magnet's OWN axis, not the
      // knob's z, so the tilt has to be applied to the offset before adding.
      centre_knob(m_local + magnet_rotation * Vec3(0.0f, 0.0f, MAGNET_HALF_HEIGHT_MM)),
      moment_mT_mm3((magnet_strength_mT / BICUBIC_FIELD_REFERENCE_MT)
                    * DIPOLE_MOMENT_AT_REFERENCE_MT_MM3),
      field_model_(field_model),
      strength_ratio_(magnet_strength_mT / BICUBIC_FIELD_REFERENCE_MT) {}


// 1/(4*pi), the only constant in the dipole field. Written out rather than
// computed from M_PI so it is a literal in the hot path.
static constexpr float ONE_OVER_FOUR_PI = 0.07957747f;

/**
 * m[in] dipole moment, r[in] displacement to the field point, J[out] dB_a/dr_b.
 * Returns the field. See the header for the frame convention.
 *
 * Straight from Math.md 4.G:
 *   B   = k[ 3(m.r) r rho^-5  -  m rho^-3 ]
 *   J_ab = k[ 3 rho^-5 (m_a r_b + r_a m_b + (m.r) d_ab)  -  15 (m.r) rho^-7 r_a r_b ]
 */
Vec3 __not_in_flash_func(dipole_field)(const Vec3& m, const Vec3& r, Mat3& J) {
    const float rho2 = r.squaredNorm();
    // One divide and one sqrt for the whole chain; every other power of rho
    // below is a multiply, matching how evaluate() handles r_reciprocal.
    const float inv_rho2 = 1.0f / rho2;
    const float inv_rho = sqrtf(inv_rho2);
    const float inv_rho3 = inv_rho2 * inv_rho;
    const float inv_rho5 = inv_rho3 * inv_rho2;

    const float mdotr = m.dot(r);

    const Vec3 B_local = (3.0f * ONE_OVER_FOUR_PI * mdotr * inv_rho5) * r
                       - (ONE_OVER_FOUR_PI * inv_rho3) * m;

    // J is symmetric -- it is minus the Hessian of a scalar potential, the
    // field being curl-free away from its source -- so six entries are
    // computed and three mirrored rather than nine evaluated.
    const float a3 = 3.0f * ONE_OVER_FOUR_PI * inv_rho5;
    const float a15 = 5.0f * a3 * mdotr * inv_rho2;   // 15 k (m.r) rho^-7
    const float diag = a3 * mdotr;

    const float mx = m.x(), my = m.y(), mz = m.z();
    const float rx = r.x(), ry = r.y(), rz = r.z();

    const float jxx = 2.0f * a3 * mx * rx + diag - a15 * rx * rx;
    const float jyy = 2.0f * a3 * my * ry + diag - a15 * ry * ry;
    const float jzz = 2.0f * a3 * mz * rz + diag - a15 * rz * rz;
    const float jxy = a3 * (mx * ry + rx * my) - a15 * rx * ry;
    const float jxz = a3 * (mx * rz + rx * mz) - a15 * rx * rz;
    const float jyz = a3 * (my * rz + ry * mz) - a15 * ry * rz;

    J << jxx, jxy, jxz,
         jxy, jyy, jyz,
         jxz, jyz, jzz;

    return B_local;
}


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
    float r = sqrtf(p[0]*p[0] + p[1]*p[1]);

    Vec2 B_cylindrical, d_dr, d_dz;
    field_model_.evaluate(r, p[2], B_cylindrical, d_dr, d_dz);


    // Decompose the measured field
    // Because we need to reconsitute the magnetic field in the x and y direction based on Br
    //
    // Also Apply this magnet's strength ratio here
    float Br = B_cylindrical[0] * strength_ratio_;
    // skip y which is stupidly zero
    float Bz = B_cylindrical[1] * strength_ratio_;

    float dBr_dr = d_dr[0] * strength_ratio_;
    float dBz_dr = d_dr[1] * strength_ratio_;
    float dBr_dz = d_dz[0] * strength_ratio_;
    float dBz_dz = d_dz[1] * strength_ratio_;


    const float EPSILON = 1e-6f;

    if (r < EPSILON) {
        B_local = Vec3(
            p[0] * dBr_dr,
            p[1] * dBr_dr,
            Bz
        );
        
        J_local = Vec3(dBr_dr, dBr_dr, dBz_dz).asDiagonal();

    } else {
        // optimization to avoid 3 divisions;
        float r_reciprocal = 1.0f/r;
        // Cosines to rotate our 2d vector back to 3d in line with x, y
        float cx = p[0] * r_reciprocal;
        float cy = p[1] * r_reciprocal;
        // Higher order terms for the jacobian
        float cx2 = cx * cx;
        float cy2 = cy * cy;
        float cxcy = cx * cy;
        float Br_over_r = Br * r_reciprocal;

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
