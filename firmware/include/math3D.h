#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>
#include <BasicLinearAlgebra.h>

using Vec3 = BLA::Matrix<3, 1, float>;
using Vec2 = BLA::Matrix<2, 1, float>;
using Mat3 = BLA::Matrix<3, 3, float>;

using Vector9f = BLA::Matrix<9, 1, float>;
using Matrix9x6f = BLA::Matrix<9, 6, float>;
using Matrix3x6f = BLA::Matrix<3, 6, float>;

// BLA has no named x()/y()/z() accessors and no operator[] -- every caller
// uses (i) (BLA's own vector-coefficient syntax, via MatrixBase's
// operator()(i, j=0)) instead. See TODO/eigen-to-bla-migration.md.

inline Mat3 identity3() { return BLA::Eye<3, 3, float>(); }

// --- Small helpers Eigen provided that BLA (pinned to the 5.1 release,
// TODO/eigen-to-bla-migration.md) does not. Each has only one or two call
// sites in this tree, so these stay plain free functions rather than
// growing into a shared library of their own. ---

// Dot product. Not in BLA 5.1 -- added to upstream master afterward, not
// yet in a tagged release we can depend on non-vendored.
template <int Dim, typename MatAType, typename MatBType>
inline float dot(const BLA::MatrixBase<MatAType, Dim, 1, float>& a,
                  const BLA::MatrixBase<MatBType, Dim, 1, float>& b) {
    float sum = 0.0f;
    for (int i = 0; i < Dim; ++i) sum += a(i) * b(i);
    return sum;
}

// This build sets -ffinite-math-only (platformio.ini), which licenses GCC to
// assume no float is ever NaN/Inf and fold accordingly -- including folding
// isnan()/isinf()/std::isfinite() themselves into a constant, since the
// compiler is allowed to believe the "not finite" branch is unreachable.
// Reading the raw IEEE-754 bit pattern via memcpy and testing the exponent
// field with plain integer ops isn't a floating-point operation in the sense
// that flag governs, so it can't be folded away the same way: a float is NaN
// or Inf iff its exponent bits are all one.
inline bool is_finite_bits(float x) {
    uint32_t bits;
    memcpy(&bits, &x, sizeof(bits));
    return (bits & 0x7F800000u) != 0x7F800000u;
}

template <int Rows, int Cols, typename MatType>
inline bool all_finite(const BLA::MatrixBase<MatType, Rows, Cols, float>& m) {
    for (int i = 0; i < Rows; ++i)
        for (int j = 0; j < Cols; ++j)
            if (!is_finite_bits(m(i, j))) return false;
    return true;
}

template <int Rows, int Cols, typename MatAType, typename MatBType>
inline BLA::Matrix<Rows, Cols, float> cwise_product(
        const BLA::MatrixBase<MatAType, Rows, Cols, float>& a,
        const BLA::MatrixBase<MatBType, Rows, Cols, float>& b) {
    BLA::Matrix<Rows, Cols, float> out;
    for (int i = 0; i < Rows; ++i)
        for (int j = 0; j < Cols; ++j)
            out(i, j) = a(i, j) * b(i, j);
    return out;
}

template <int Rows, int Cols, typename MatType>
inline BLA::Matrix<Rows, Cols, float> cwise_sqrt(const BLA::MatrixBase<MatType, Rows, Cols, float>& a) {
    BLA::Matrix<Rows, Cols, float> out;
    for (int i = 0; i < Rows; ++i)
        for (int j = 0; j < Cols; ++j)
            out(i, j) = sqrtf(a(i, j));
    return out;
}

inline Mat3 skew_matrix(const Vec3& v) {
    // BLA's variadic constructor fills row-by-row, same order the comma
    // operator used to.
    return Mat3(  0.0f,  -v(2),   v(1),
                 v(2),    0.0f,  -v(0),
                -v(1),    v(0),   0.0f);
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
    return R * (1.5f * BLA::Eye<3, 3, float>() - 0.5f * (~R * R));
}

// Exact SO(3) exponential map (Rodrigues' formula): R_new = exp([w]_x) *
// R_old. Canonical implementation for both solve_knob_pose's rotation
// update (replacing Eigen::AngleAxisf) and test_jacobian.cpp's
// finite-difference perturbation of R -- the latter needs the exact
// exponential rather than the first-order approximation the analytic
// Jacobian linearizes around, or a shared error could cancel and hide.
inline Mat3 exp_so3(const Vec3& w) {
    float theta = BLA::Norm(w);
    Mat3 K = skew_matrix(w);
    if (theta < 1.0e-8f) {
        // Small-angle fallback (also avoids 0/0); accurate to O(theta^2).
        return BLA::Eye<3, 3, float>() + K + 0.5f * (K * K);
    }
    float s = sinf(theta) / theta;
    float c = (1.0f - cosf(theta)) / (theta * theta);
    return BLA::Eye<3, 3, float>() + s * K + c * (K * K);
}
