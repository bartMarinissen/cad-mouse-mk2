#pragma once
#include <cmath>
#include <ArduinoEigenDense.h>

using Vec3 = Eigen::Vector3f;
using Vec2 = Eigen::Vector2f;
using Mat3 = Eigen::Matrix3f;

using Vector9f = Eigen::Matrix<float, 9, 1>;
using Matrix9x6f = Eigen::Matrix<float, 9, 6>;

// Tried __attribute__((always_inline)) here and measured it on cortex-m0plus:
// it does not help. The 24-instruction out-of-line body disappears, but the
// caller grows by 32 (net +8 across the TU) and Eigen's 3x3 product stays a
// separate 221-instruction out-of-line function taking Mat3 by reference --
// so the three structural zeros remain invisible to the multiply either way.
// Exploiting them needs a fused skew-product, not an inlining hint.
inline Mat3 skew_matrix(const Vec3& v) {
    Mat3 m;
    // The comma operator strictly fills row-by-row
    m <<  0.0f,   -v.z(),   v.y(),
          v.z(),   0.0f,   -v.x(),
         -v.y(),   v.x(),   0.0f;
    return m;
}

// One Newton-iteration step toward the nearest orthogonal matrix, valid only
// when R is already close to orthogonal (e.g. drifted by an accumulation of
// small updates, not built from scratch). For R = Q(I+E) with Q exactly
// orthogonal and E small: R^T R = I + 2S + O(E^2), where S=(E+E^T)/2 is E's
// symmetric part, so (3I - R^T R)/2 = I - S + O(E^2), and
//   R*(I-S) = Q(I+E)(I-S) = Q(I + (E-S)) + O(E^2) = Q(I+K) + O(E^2)
// where K=(E-E^T)/2 is E's skew part -- i.e. the symmetric (scale/shear)
// component of the drift is cancelled to first order, leaving only a small
// residual rotation. Quadratically convergent, so one step suffices when
// called every frame against slowly-accumulating drift; ~2 3x3 products, no
// sqrt, no branches, unlike quaternion-normalize-and-rebuild.
inline Mat3 orthonormalize_approx(const Mat3& R) {
    return R * (1.5f * Mat3::Identity() - 0.5f * (R.transpose() * R));
}
