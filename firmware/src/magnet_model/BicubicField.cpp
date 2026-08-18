#include "magnet_model/BicubicField.h"

// --- small constexpr helpers ------------------------------------
// (these are only needed inside evaluate(), so they live here rather
// than as private class members declared in the header)

static constexpr int iclamp(int v, int lo, int hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

static constexpr float clamp01(float v) noexcept {
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

static constexpr int ifloor(float x) noexcept {
    const int xi = static_cast<int>(x);
    return (x < float(xi)) ? xi - 1 : xi;
}

// The 4-point cubic stencil spans [i0-1, i0+2]. Clamping i0 to
// [1, NR-3] and j0 to [1, NZ-3] would keep the stencil entirely inside
// the real grid with no ghost nodes and no per-fetch bounds checking -
// but on the r-axis, r=0 is the field's physical symmetry axis (the
// grid origin), and the sole caller always queries r=sqrt(x^2+y^2)>=0,
// so the patch touching the axis (i0=0) is a real, commonly-hit region.
// Rather than give that patch up to linear extrapolation, i0 is allowed
// down to 0: the stencil's missing virtual node at column -1 is exactly
// the mirror of column 1, since the axisymmetric field has Br odd and
// Bz even in r (Br(-r,z) = -Br(r,z), Bz(-r,z) = Bz(r,z)). See the
// axis_patch branch in evaluate() below. z has no such symmetry, so j0
// keeps the [1, NZ-3] clamp and the outermost z ring is still only
// covered by extrapolation from the nearest usable patch.
static_assert(NR >= 4, "bicubic stencil needs at least 4 nodes in r");
static_assert(NZ >= 4, "bicubic stencil needs at least 4 nodes in z");

// The axis_patch mirror below is only correct if grid column 0 sits
// exactly on the physical symmetry axis (r=0) - it mirrors column 1
// into the missing virtual column -1, which is only the true field
// value there because r=0 and index 0 coincide. If the grid's r-origin
// ever moves off 0, this assumption breaks silently (wrong, not a
// crash), so pin it down here.
static_assert(BICUBIC_ORIGIN.r == 0.0f,
              "BicubicField's r=0 axis-symmetry mirror assumes the grid's "
              "r-axis origin is exactly 0");


// Value and gradient (d/dr, d/dz, each a Vec2) at (r, z).
Vec2 __not_in_flash_func(BicubicField::evaluate)(float r, float z, Mat2 &jacobian) const noexcept {
    /** This function works on the basis formulation of catmul-rom splines.
     * 
     * We interpolate based on 4 points: f0 f1 f2 f3
     * In the basis formulation, we write the cubic interpolation based on the 4 points as a linear
     * combination of those 4 points. The weights for that formulation depend on the internal coordinate in the cell t
     * (t ranges from 0 to 1).
     * 
     * Catumul Rom gives specifically
     * 
     * 
     *                                           [ 0 -1  2 -1 ] [  1  ]
     * p(t)^T = 1/2 [ p_k-1  p_k  p_k+1  p_k+2 ] [ 2  0 -5  3 ] [  t  ]
     *                                           [ 0  1  4 -3 ] [ t^2 ]
     *                                           [ 0  0 -1  1 ] [ t^3 ]
     * 
     * For standard bicubic interpolation, we first need to do 4 cubic interpolations in the r direction.
     * All of these use the same weights. And we need to do this for two values (B_r and B_z).
     * ********
     * Hence it is much more efficient to only compute these weights once.
     * *******
     * 
     * We then use the same formulation for the final cubic interpolation in the z direction.
     * 
     * The final coefificients for  [ p_k-1  p_k  p_k+1  p_k+2 ] in t we call:
     * a0, a1, a2, a3
     * For u we call them:
     * b0, b1, b2, b3
     * 
     * ==================
     * bounds handling and field symmetry
     * ==================
     * Bicubic derivation needs a 4x4 stencil to work.
     * If we want to interpolate in a cell on the outer edge, that fails.
     * We handle this two ways.
     * 
     * On one side of the grid (the r=0 side) we can exploit the mirror symmetry of the field.
     * Using the facts that B_r(-r, z) = - B_r(r, z) 
     *                      B_z(-r, z) =   B_z(r, z)
     * to use the points at grid index 1 to compute the points missing at grid index -1.
     * 
     * On the other sides, we never use the outer cells for interpolation.
     * Instead we extrapolate based on one cell closer in. This extrapolation we do linearly.
     * The linear extraploation is easily captured in the a and b coeficcients defined above.
     * 
     * We just need the derivateive of the function (that we already calculate) and then 
     */

 
    // --- cell location -------------------------------------------
    // ideal cell indexes within grid
    const float fi = (r - origin_.r) * dr_reciprocal_;
    const float fj = (z - origin_.z) * dz_reciprocal_;


    // Grid cell indexes
    const int i0 = iclamp(ifloor(fi), 0, int(NR) - 3);   // Due to symmetric field, we allow i0 = 0
    const int j0 = iclamp(ifloor(fj), 1, int(NZ) - 3);

    // Local parameter, split into an in-patch part and an overshoot.
    // Inside the box the overshoot is zero and this is exact bicubic
    // Catmull-Rom. Outside, the patch is continued linearly.
    // let tc be the closest grid point, and t_overshoot = tc - t then:
    //
    //     f(t) = f(tc) + t_overshoot * f'(tc)
    //
    const float ti = fi - float(i0);
    const float uj = fj - float(j0);
    // Coordinates inside our current cell
    const float t  = clamp01(ti);
    const float u  = clamp01(uj);
    // Overshoot of coordinates that did not fit our cell (used for linear extrapolation)
    const float t_overshoot = ti - t;
    const float u_overshoot = uj - u;

    // --- end of cell location, yields u, t, t_overshoot, u_overshoot, i0, j0

    // --- Catmull-Rom basis weights and their t-derivatives ---------
    // Powers: t, t^2, t^3
    //         u, u^2, u^3
    const float t2 = t * t, t3 = t2 * t;
    const float u2 = u * u, u3 = u2 * u;

    // Because Eigen matrix can't be constexpr, these are left repeated and not simplified

    // The standard Catmull-ROM basis weights for t
    float a0 =      -0.5f * t  +        t2 - 0.5f * t3;
    float a1 = 1.0f            - 2.5f * t2 + 1.5f * t3;
    float a2 =       0.5f * t  + 2.0f * t2 - 1.5f * t3;
    float a3 =                 - 0.5f * t2 + 0.5f * t3;

    // The standard Catmull-ROM basis weights for u
    float b0 =      -0.5f * u  +        u2 - 0.5f * u3;
    float b1 = 1.0f            - 2.5f * u2 + 1.5f * u3;
    float b2 =       0.5f * u  + 2.0f * u2 - 1.5f * u3;
    float b3 =                 - 0.5f * u2 + 0.5f * u3;

    // The derivatives of the basis weights for t
    const float da0_dt = -0.5f + 2.0f * t - 1.5f * t2;
    const float da1_dt =        -5.0f * t + 4.5f * t2;
    const float da2_dt =  0.5f + 4.0f * t - 4.5f * t2;
    const float da3_dt =        -1.0f * t + 1.5f * t2;

    // The derivatives of the basis weights for u
    const float db0_du = -0.5f + 2.0f * u - 1.5f * u2;
    const float db1_du =        -5.0f * u + 4.5f * u2;
    const float db2_du =  0.5f + 4.0f * u - 4.5f * u2;
    const float db3_du =        -1.0f * u + 1.5f * u2;


    // --- Linear extension outside of the grid ---------------------
    // If there is overshoot (i.e. if t_overshoot or u_overshoot aren't zero) then here we let that
    // go linearly into the weights of the points
    // Note that since our function f(t) is computed as \sum_i a_i(t) * p_i
    // that df(t)/dt = \sum_i da_i(t)/dt * pi
    // (is this worth the extra float multiplications and additions?)
    a0 += t_overshoot * da0_dt;  a1 += t_overshoot * da1_dt;  a2 += t_overshoot * da2_dt;  a3 += t_overshoot * da3_dt;
    b0 += u_overshoot * db0_du;  b1 += u_overshoot * db1_du;  b2 += u_overshoot * db2_du;  b3 += u_overshoot * db3_du;

    // --- contract in r, for each of the 4 rows in z ---------------
    Vec2 row[4], row_deriv[4];
    const bool axis_patch = (i0 == 0);
    if (!axis_patch) {
        // Normal case
        for (int k = 0; k < 4; ++k) {
            const int jj = j0 - 1 + k;
            const Vec2 p0 = grid_[jj][i0 - 1];
            const Vec2 p1 = grid_[jj][i0    ];
            const Vec2 p2 = grid_[jj][i0 + 1];
            const Vec2 p3 = grid_[jj][i0 + 2];
            row[k]       = p0 * a0 + p1 * a1 + p2 * a2 + p3 * a3;
            row_deriv[k] = p0 * da0_dt + p1 * da1_dt + p2 * da2_dt + p3 * da3_dt;
        }
    } else {
        // On-axis patch (r in [0, dr)): the stencil needs a virtual
        // node at column -1 (r = -dr), which by the field's odd/even
        // r-symmetry is exactly the mirror of column 1 - not an
        // approximation. Column 1 is already loaded as p2 elsewhere in
        // this file, so the mirror costs a sign negate, not a fetch.
        for (int k = 0; k < 4; ++k) {
            const int jj = j0 - 1 + k;
            const Vec2 p1 = grid_[jj][0];
            const Vec2 p2 = grid_[jj][1];
            const Vec2 p0(-p2.x(), p2.y());
            const Vec2 p3 = grid_[jj][2];
            row[k]       = p0 * a0 + p1 * a1 + p2 * a2 + p3 * a3;
            row_deriv[k] = p0 * da0_dt + p1 * da1_dt + p2 * da2_dt + p3 * da3_dt;
        }
    }

    // --- contract in z -------------------------------------------
    Vec2 value = row[0] * b0 + row[1] * b1 + row[2] * b2 + row[3] * b3;

    jacobian.col(0)  = ((row_deriv[0] * b0 + row_deriv[1] * b1 + row_deriv[2] * b2 + row_deriv[3] * b3)
            * dr_reciprocal_);

    // row[] is independent of z, so the z-derivative weights apply directly
    jacobian.col(1)  = ((row[0] * db0_du + row[1] * db1_du + row[2] * db2_du + row[3] * db3_du)
            * dz_reciprocal_);
    
    return value;
}
