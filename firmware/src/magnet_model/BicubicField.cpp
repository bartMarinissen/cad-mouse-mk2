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

// Value and gradient (d/dr, d/dz, each a Vec2) at (r, z). Never throws.
void __not_in_flash_func(BicubicField::evaluate)(float r, float z, Vec2& value, Vec2& d_dr, Vec2& d_dz) const noexcept {
    // --- cell location -------------------------------------------
    const float fi = (r - origin_.r) * dr_reciprocal_;
    const float fj = (z - origin_.z) * dz_reciprocal_;

    const int i0 = iclamp(ifloor(fi), 0, int(NR) - 3);
    const int j0 = iclamp(ifloor(fj), 1, int(NZ) - 3);

    // Local parameter, split into an in-patch part and an overshoot.
    // Inside the box the overshoot is zero and this is exact bicubic
    // Catmull-Rom. Outside, the patch is continued linearly:
    //
    //     f(t) = f(tc) + dt * f'(tc)
    //
    // Because f is a linear combination of the four stencil nodes with
    // scalar weights, applying this to the *weights* is equivalent to
    // applying it to the result, so it costs 4 scalar multiplies per
    // axis and no extra Vec2 work. The result is C1 across the boundary
    // and has a constant gradient outside, which keeps the Jacobian
    // continuous for the solver.
    const float ti = fi - float(i0);
    const float uj = fj - float(j0);
    const float t  = clamp01(ti), dt = ti - t;
    const float u  = clamp01(uj), du = uj - u;

    // --- Catmull-Rom basis weights and their t-derivatives ---------
    const float t2 = t * t, t3 = t2 * t;
    float a0 = -0.5f * t  +        t2 - 0.5f * t3;
    float a1 =  1.0f      - 2.5f * t2 + 1.5f * t3;
    float a2 =  0.5f * t  + 2.0f * t2 - 1.5f * t3;
    float a3 =            - 0.5f * t2 + 0.5f * t3;

    const float da0 = -0.5f + 2.0f * t - 1.5f * t2;
    const float da1 =        -5.0f * t + 4.5f * t2;
    const float da2 =  0.5f + 4.0f * t - 4.5f * t2;
    const float da3 =        -1.0f * t + 1.5f * t2;

    a0 += dt * da0;  a1 += dt * da1;  a2 += dt * da2;  a3 += dt * da3;

    // derivative weights, pre-scaled by the axis reciprocal so the
    // chain-rule factor costs 4 scalar multiplies instead of 8
    const float b0 = da0 * dr_reciprocal_;
    const float b1 = da1 * dr_reciprocal_;
    const float b2 = da2 * dr_reciprocal_;
    const float b3 = da3 * dr_reciprocal_;

    const float u2 = u * u, u3 = u2 * u;
    float c0 = -0.5f * u  +        u2 - 0.5f * u3;
    float c1 =  1.0f      - 2.5f * u2 + 1.5f * u3;
    float c2 =  0.5f * u  + 2.0f * u2 - 1.5f * u3;
    float c3 =            - 0.5f * u2 + 0.5f * u3;

    const float e0 = -0.5f + 2.0f * u - 1.5f * u2;
    const float e1 =        -5.0f * u + 4.5f * u2;
    const float e2 =  0.5f + 4.0f * u - 4.5f * u2;
    const float e3 =        -1.0f * u + 1.5f * u2;

    c0 += du * e0;  c1 += du * e1;  c2 += du * e2;  c3 += du * e3;

    // --- contract in r, for each of the 4 rows in z ---------------
    Vec2 row[4], row_deriv[4];
    const bool axis_patch = (i0 == 0);
    if (!axis_patch) {
        for (int k = 0; k < 4; ++k) {
            const int jj = j0 - 1 + k;
            const Vec2 p0 = grid_[jj][i0 - 1];
            const Vec2 p1 = grid_[jj][i0    ];
            const Vec2 p2 = grid_[jj][i0 + 1];
            const Vec2 p3 = grid_[jj][i0 + 2];

            row[k]       = p0 * a0 + p1 * a1 + p2 * a2 + p3 * a3;
            row_deriv[k] = p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
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
            row_deriv[k] = p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
        }
    }

    // --- contract in z -------------------------------------------
    value = row[0] * c0 + row[1] * c1 + row[2] * c2 + row[3] * c3;

    // row_deriv is already d/dr, so blending it in z gives d/dr directly
    d_dr  = row_deriv[0] * c0 + row_deriv[1] * c1
          + row_deriv[2] * c2 + row_deriv[3] * c3;

    // row[] is independent of z, so the z-derivative weights apply directly
    d_dz  = (row[0] * e0 + row[1] * e1 + row[2] * e2 + row[3] * e3)
            * dz_reciprocal_;
}
