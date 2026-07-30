#include "magnet_model/BicubicField.h"



// TODO: fix huge waste.
// This is a 2d function returning 3d values and 3d derivatives.

// Value and gradient (d/dr, d/dz, each a Vec2) at (r, z). Never
// throws; 
void __not_in_flash_func(BicubicField::evaluate)(float r, float z, Vec2& value, Vec2& d_dr, Vec2& d_dz) const noexcept {
    // coordinates within bounding box
    const float fi = (r - origin_.r) * dr_reciprocal_;
    const float fj = (z - origin_.z) * dz_reciprocal_;

    // rounded to integers in the valid range with remainders in t and u
    // note t and u can be outside [0, 1] if (r, z) is outside the bounding box
    const int i0 = iclamp(ifloor(fi), 0, int(NR) - 2);
    const int j0 = iclamp(ifloor(fj), 0, int(NZ) - 2);
    const float t = fi - float(i0);
    const float u = fj - float(j0);

    // do 1D cubic interpolation in r at a fixed z value
    Vec2 row[4]{}, row_deriv[4]{};
    for (int k = 0; k < 4; ++k) {
        const int jj = j0 - 1 + k;
        const Vec2 p0 = point_or_ghost(i0 - 1, jj);
        const Vec2 p1 = point_or_ghost(i0,     jj);
        const Vec2 p2 = point_or_ghost(i0 + 1, jj);
        const Vec2 p3 = point_or_ghost(i0 + 2, jj);
        row[k]       = cubic(p0, p1, p2, p3, t);
        // d_row / dr = d_row / dt * dt / dr = cubic_deriv(...) / dr
        row_deriv[k] = cubic_deriv(p0, p1, p2, p3, t) * dr_reciprocal_;
    }
    // now do another 1d cubic interpolation in z on the 4 results from above
    value = cubic(row[0], row[1], row[2], row[3], u);
    // cubic is a linear function of its inputs, so through the chain rule:
    // so dvalue / dr = dvalue / dt * dt / dr = cubic(cubic_deriv(...)) / dr
    d_dr  = cubic(row_deriv[0], row_deriv[1], row_deriv[2], row_deriv[3], u);
    // row[i] is independent of z so no chain-rule chenanigans
    // dvalue / dz = dvalue / du * du / dz = cubic_deriv(...) / dz
    d_dz  = cubic_deriv(row[0], row[1], row[2], row[3], u) * dz_reciprocal_;
}

// --- small constexpr helpers ------------------------------------
static constexpr int iclamp(int v, int lo, int hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}
static constexpr float clampF(float v, float lo, float hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}
static constexpr int ifloor(float x) noexcept {
    const int xi = static_cast<int>(x);
    return (x < float(xi)) ? xi - 1 : xi;
}

// Plain clamped lookup, no extrapolation.
// Belt and suspenders. No calls should pass out or range indices.
// But compiler will optmizie this away
Vec2 BicubicField::raw(int i, int j) const noexcept {
    return grid_[iclamp(j, 0, int(NZ) - 1)][iclamp(i, 0, int(NR) - 1)];
}

// Lookup allowing i in [-1, NR] and j in [-1, NZ] (exactly one step
// past the real grid, which is all the 4-point cubic stencil ever
// needs at a boundary cell). Out-of-range points are filled by
// linear extrapolation from the two nearest real nodes; the corner
// case combines both directions additively (planar extrapolation).
Vec2 BicubicField::point_or_ghost(int i, int j) const noexcept {
    Vec2 p;
    const bool i_out = i < 0 || i >= int(NR);
    const bool j_out = j < 0 || j >= int(NZ);

    // Read nearest point on the grid
    // Note: raw clamps to the correct range
    p = raw(i, j);

    // Normal case: inside the grid, just return the value
    if (!i_out && !j_out){
        return p;
    } 

    // if we are outside the grid, p is the closest point.
    // in the directions where we need interpolation, we take the neighbor 1 step inside the grid in that direction
    // In the corner case we then automatically do this in both directions, which is the planar extrapolation.
    int known_neigbor_i = (i < 1 ? 1 : int(NR) - 2);
    int known_neigbor_j = (j < 1 ? 1 : int(NZ) - 2);
    if (i_out ){
        p = p + raw(i, j) - raw(known_neigbor_i, j);
    }
    if (j_out ){
        p = p + raw(i, j) - raw(i, known_neigbor_j);
    }
    return p;
}

// 1D Catmull-Rom cubic through 4 points, local param t in [0,1]
// between p1 and p2, and its derivative w.r.t. t.
Vec2 BicubicField::cubic(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, float t) noexcept {
    const float t2 = t * t, t3 = t2 * t;
    return (p1 * 2.0f
            + (p2 - p0) * t
            + (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2
            + (p3 + (p1 - p2) * 3.0f - p0) * t3) * 0.5f;
}
Vec2 BicubicField::cubic_deriv(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, float t) noexcept {
    const float t2 = t * t;
    return ((p2 - p0)
            + (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * (2.0f * t)
            + (p3 + (p1 - p2) * 3.0f - p0) * (3.0f * t2)) * 0.5f;
}

