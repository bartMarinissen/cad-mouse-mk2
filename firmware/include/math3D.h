#pragma once
#include <cmath>
#include <ArduinoEigenDense.h>

using Vec3 = Eigen::Vector3f;
using Vec2 = Eigen::Vector2f;
using Mat3 = Eigen::Matrix3f;

using Vector9f = Eigen::Matrix<float, 9, 1>;
using Matrix9x6f = Eigen::Matrix<float, 9, 6>;

inline Mat3 skew_matrix(const Vec3& v) {
    Mat3 m;
    // The comma operator strictly fills row-by-row
    m <<  0.0f,   -v.z(),   v.y(),
          v.z(),   0.0f,   -v.x(),
         -v.y(),   v.x(),   0.0f;
    return m;
}
