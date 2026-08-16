#include "magnet_model/magnet_local_model.h"
#include "magnet_model/magnet_model_table.h"
#include <cmath>


MagnetModel::MagnetModel(const BicubicField& field_model, const Vec3& m_local,
                         const Mat3 &magnet_rotation, float magnet_strength_mT)
    : magnet_pos_knob(m_local), magnet_rotation(magnet_rotation),
      magnet_strength_mT(magnet_strength_mT),
      // The centre is half a magnet along the magnet's OWN axis, not the
      // knob's z, so the tilt has to be applied to the offset before adding.
      magnet_centre_knob(m_local + magnet_rotation * Vec3(0.0f, 0.0f, MAGNET_HALF_HEIGHT_MM)),
      moment_mT_mm3((magnet_strength_mT / BICUBIC_FIELD_REFERENCE_MT)
                    * DIPOLE_MOMENT_AT_REFERENCE_MT_MM3),
      field_model_(field_model),
      strength_ratio_(magnet_strength_mT / BICUBIC_FIELD_REFERENCE_MT) {}


MagnetPlacement __not_in_flash_func(MagnetModel::place)(const Vec3& t, const Mat3& R) const {
    Mat3 R_total = R * magnet_rotation;
    Vec3 centre_world = t + R * magnet_centre_knob;
    Vec3 origin_world = t + R * magnet_pos_knob;
    // Polarization runs along the magnet's local -z, so the moment vector is
    // -|m| times its own axis -- which, mapped to world, is R_total's third
    // column. A column read and a scale: the entire cost of carrying this
    // magnet's orientation into the far-field model.
    Vec3 moment_world = -moment_mT_mm3 * R_total.col(2);
    return MagnetPlacement{R_total, centre_world, origin_world, moment_world, *this};
}


// 1/(4*pi), the only constant in the dipole field. Written out rather than
// computed from M_PI so it is a literal in the hot path.
static constexpr float ONE_OVER_FOUR_PI = 0.07957747f;

/**
 * m[in] dipole moment, r[in] displacement to the field point, J[out] dB_a/dr_b.
 * Returns the field. See the header for the frame convention.
 *
 * Straight from Math.md 4.E:
 *   B   = k[ 3(m·r) r rho^-5  -  m rho^-3 ]
 *   J_ab = k[ 3 rho^-5 (m_a r_b + r_a m_b + (m·r) d_ab)  -  15 (m.r) rho^-7 r_a r_b ]
 */
Vec3 __not_in_flash_func(dipole_field)(const Vec3& m, const Vec3& r, Mat3& J) {
    const float rho_sq = r.squaredNorm();
    // One divide and one sqrt for the whole chain; every other power of rho
    // below is a multiply, matching how evaluate() handles r_reciprocal.
    const float inv_rho_pow2 = 1.0f / rho_sq;
    const float inv_rho = sqrtf(inv_rho_pow2);
    const float inv_rho_pow3 = inv_rho_pow2 * inv_rho;
    const float inv_rho_pow5 = inv_rho_pow3 * inv_rho_pow2;

    const float mdotr = m.dot(r);

    const Vec3 B_local = (3.0f * ONE_OVER_FOUR_PI * mdotr * inv_rho_pow5) * r
                       - (ONE_OVER_FOUR_PI * inv_rho_pow3) * m;

    // J is symmetric -- it is minus the Hessian of a scalar potential, the
    // field being curl-free away from its source -- so six entries are
    // computed and three mirrored rather than nine evaluated.

    /* We (potentially pre-empting stuff the compiler would optimize anyway) optmize the computation of the jacobian.
     * Distributing K over the sum, and the extracting two constants
     * 
     *  J_ab = k[ 3 rho^-5  (m_a r_b + r_a m_b + (m·r) d_ab) -   15 (m·r) rho^-7 r_a r_b ]
     *       = 3 k 3 rho^-5 (m_a r_b + r_a m_b + (m·r) d_ab) - 15 k (m·r) rho^-7 (r_a r_b)
     *       =     a3       (m_a r_b + r_a m_b + (m·r) d_ab) -          a15      (r_a r_b)
     * Where:
     *  a3  = 3 k rho^-5
     *  a15 = 15 k (m.r) rho^-7  
     *      = 5 a3 (m.r) rho^-2
     */
    const float a3 = 3.0f * ONE_OVER_FOUR_PI * inv_rho_pow5;
    const float a15 = 5.0f * a3 * mdotr * inv_rho_pow2;   // 15 k (m.r) rho^-7

    const float mx = m.x(), my = m.y(), mz = m.z();
    const float rx = r.x(), ry = r.y(), rz = r.z();

    // Diagonal terms where kronecker delta d_ab = 1
    const float jxx = a3 * (2.0f * mx * rx + mdotr) - a15 * rx * rx;
    const float jyy = a3 * (2.0f * my * ry + mdotr) - a15 * ry * ry;
    const float jzz = a3 * (2.0f * mz * rz + mdotr) - a15 * rz * rz;
    // off diagonal terms
    const float jxy = a3 * (mx * ry + rx * my)      - a15 * rx * ry;
    const float jxz = a3 * (mx * rz + rx * mz)      - a15 * rx * rz;
    const float jyz = a3 * (my * rz + ry * mz)      - a15 * ry * rz;


    J << jxx, jxy, jxz,
         jxy, jyy, jyz,
         jxz, jyz, jzz;

    return B_local;
}

Vec3 MagnetPlacement::far_approx_world(const Vec3& p_world, Mat3& J_world) const{
    // The far approximation is just the dipole aproximation. That means we can evaluate this in the 
    // world frame directly
    return dipole_field(moment_world, p_world - centre_world, J_world);
}

Vec3 MagnetPlacement::near_approx_world(const Vec3& p_world, Mat3& J_world) const{
    // The near approximation needs to be computed in the magnet local frame.
    // So we frame swap to the magnet local frame, compute, and frame swap back.

    // --- To the local frame ----
    // Rotation from the world frame to the local frame
    const auto R_total_T = R_total.transpose();
    // Measurement position in the local frame
    const Vec3 p_local = R_total_T * (p_world - origin_world);

    // --- Evaluate in local frame ---
    Mat3 J_local;
    const Vec3 B_local = magnet.evaluate(p_local, J_local);

    // --- Back to world frame ---
    const Vec3 B_world = R_total * B_local;
    // Since this is a matrix we need matrix conjugation
    J_world = R_total * J_local * R_total_T;

    return B_world;
}

/**
 * p[in] is the position where we evaluate the field relative to the magnet
 * J_local[out] the jacobian
 * returns the field at p[in] oriented along the magnet axis
 */
Vec3 __not_in_flash_func(MagnetModel::evaluate)(const Vec3& p_local, Mat3& J_local) const {
    // return value
    Vec3 B_local;
    // get radius of the given point compared to the z axis
    // TODO: possible optimization, do the bicubic interpolation on Z, r^2? 
    // This also solves the r < EPSILON singularity.
    float r = sqrtf(p_local[0]*p_local[0] + p_local[1]*p_local[1]);

    // Get the interpolated field and jacobian.
    // Note the jacobian is returned as two gradient vectors d_dr, d_dz
    Mat2 J_cylindrical;
    Vec2 B_cylindrical = field_model_.evaluate(r, p_local.z(), J_cylindrical);

    J_cylindrical *= strength_ratio_;
    B_cylindrical *= strength_ratio_;


    // Decompose the measured field
    // Because we need to reconsitute the magnetic field in the x and y direction based on Br
    float Br = B_cylindrical[0];
    float Bz = B_cylindrical[1];

    float dBr_dr = J_cylindrical(0, 0);
    float dBz_dr = J_cylindrical(1, 0);
    float dBr_dz = J_cylindrical(0, 1);
    float dBz_dz = J_cylindrical(1, 1);


    const float EPSILON = 1e-6f;

    if (r < EPSILON) {
        B_local = Vec3(
            p_local[0] * dBr_dr,
            p_local[1] * dBr_dr,
            Bz
        );
        
        J_local = Vec3(dBr_dr, dBr_dr, dBz_dz).asDiagonal();

    } else {
        float r_reciprocal = 1.0f/r;
        // Cosines to rotate our 2d vector back to 3d in line with x, y
        float cx = p_local[0] * r_reciprocal;
        float cy = p_local[1] * r_reciprocal;
        // Higher order terms for the jacobian
        float cx_sq = cx * cx;
        float cy_sq = cy * cy;
        float cxcy = cx * cy;
        float Br_over_r = Br * r_reciprocal;

        B_local = Vec3(Br * cx, Br * cy, Bz);

        float Br_dr_minus_Br_r = dBr_dr - Br_over_r;

        J_local <<
            dBr_dr * cx_sq + Br_over_r * cy_sq,  Br_dr_minus_Br_r * cxcy,             dBr_dz * cx,
            Br_dr_minus_Br_r * cxcy,             dBr_dr * cy_sq + Br_over_r * cx_sq,  dBr_dz * cy,
            dBz_dr * cx,                         dBz_dr * cy,                         dBz_dz
        ;
    }

    return B_local;
}
