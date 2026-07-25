#pragma once
#include <cmath>
#include <ArduinoEigenDense.h>

using Vec3 = Eigen::Vector3f;
using Mat3 = Eigen::Matrix3f;

inline Vec3 pow_magnitude(Vec3 const &vec, float power) {
    float mag = vec.norm();
    float scaled_mag = powf(mag, power);  
    float ratio = (mag != 0.0f) ? (scaled_mag / mag) : 1.0f;

    return vec * ratio;
}

inline Mat3 skew_matrix(const Vec3& v) {
    Mat3 m;
    // The comma operator strictly fills row-by-row
    m <<  0.0f,   -v.z(),   v.y(),
          v.z(),   0.0f,   -v.x(),
         -v.y(),   v.x(),   0.0f;
    return m;
}

/*
// Fixed-type (float) bicubic interpolator over a uniform 2D grid of
// 3-component vectors. Same structure/boundary handling as the scalar
// version, but a single evaluate() call interpolates all 3 components
// together, reusing one set of basis weights instead of doing 3
// independent scalar evaluations.
struct Vec3 {
    float x{}, y{}, z{};

    static Vec3 zero(){
        return {0.0, 0.0, 0.0};
    }

    constexpr Vec3 operator+(const Vec3& o) const noexcept { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const noexcept { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(float s)       const noexcept { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(float s)       const noexcept { return {x / s, y / s, z / s}; }
    constexpr float operator[](int i) const noexcept {
        return (i == 0) ? x : (i == 1) ? y : z;
    }

    constexpr float norm_sq() const noexcept { return x*x + y*y + z*z; }
    float norm() const noexcept { return std::sqrt(norm_sq()); }

    Vec3 pow_magnitude(float power) const {
        float mag = norm();
        float scaled_mag = powf(mag, power);  
        float ratio = (mag != 0.0f) ? (scaled_mag / mag) : 1.0f;

        return *this * ratio;
    }
};
// Standalone helpers:
constexpr Vec3 operator*(float s, const Vec3& v) noexcept { return v * s; }
constexpr float dot(const Vec3& a, const Vec3& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
    return { a.y * b.z - a.z * b.y,
             a.z * b.x - a.x * b.z,
             a.x * b.y - a.y * b.x };
}
*/

/*
struct Mat3 {
    Eigen::Vector3f row[3];

    static Mat3 Iden() {
        return Mat3{
            Vec3{1.0, 0.0, 0.0},
            Vec3{0.0, 1.0, 0.0},
            Vec3{0.0, 0.0, 1.0}
        };
    }

    constexpr Vec3 col(int idx) const {
        return Vec3{row[0][idx], row[1][idx], row[2][idx]};
    }

    // matrix - vector multiplication
    constexpr Vec3 operator*(const Vec3& o) const noexcept {
        return Vec3{dot(row[0], o), dot(row[1], o), dot(row[2], o)};
    }

    // matrix - matrix multiplication
    constexpr Mat3 operator*(const Mat3& o) const noexcept {
        // Pre-fetch columns to avoid repeated extraction
        const Vec3 c0 = o.col(0);
        const Vec3 c1 = o.col(1);
        const Vec3 c2 = o.col(2);

        return Mat3{{
            {dot(row[0], c0), dot(row[0], c1), dot(row[0], c2)},
            {dot(row[1], c0), dot(row[1], c1), dot(row[1], c2)}, 
            {dot(row[2], c0), dot(row[2], c1), dot(row[2], c2)}
        }};
    }
    // matrix-scalar multipliocation
    constexpr Mat3 operator*(const float s) const noexcept {
        return {row[0] * s, row[1] * s, row[2] * s};
    }

    // matrix addition
    constexpr Mat3 operator+(const Mat3& o) const noexcept {
        Mat3 result;
        result.row[0] = row[0] + o.row[0];
        result.row[1] = row[1] + o.row[1];
        result.row[2] = row[2] + o.row[2];
        return result;
    }


    constexpr Mat3 transpose() const noexcept {
        return Mat3{Vec3{row[0].x, row[1].x, row[2].x},
                Vec3{row[0].y, row[1].y, row[2].y},
                Vec3{row[0].z, row[1].z, row[2].z}};
    }

    // matrix indexing
    constexpr Vec3& operator[](int i) noexcept { return row[i]; }

};
constexpr Mat3 operator*(float s, const Mat3& m) noexcept { return m * s; }

// Helper function to generate a skew-symmetric matrix from a vector
inline Mat3 skew_matrix(const Vec3& v) {
    return Mat3{
       Vec3{ 0.0f, -v.z,  v.y},
       Vec3{ v.z,  0.0f, -v.x},
       Vec3{-v.y,  v.x,  0.0f}
    };
}
*/