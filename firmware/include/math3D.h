#pragma once
#include <cmath>
#include <ArduinoEigenDense.h>

using Vec3 = Eigen::Vector3f;
using Vec2 = Eigen::Vector2f;
using Mat3 = Eigen::Matrix3f;
using Mat2 = Eigen::Matrix2f;

using Vector6f = Eigen::Matrix<float, 6, 1>;
using Vector9f = Eigen::Matrix<float, 9, 1>;
using Matrix9x6f = Eigen::Matrix<float, 9, 6>;
using Matrix6x6f = Eigen::Matrix<float, 6, 6>;

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
