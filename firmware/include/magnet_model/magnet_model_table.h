
#pragma once
#include "math3D.h"

struct Point { float r; float z; };

using Vec2 = Eigen::Vector2f;
constexpr int NR = 51;
constexpr int NZ = 91;

constexpr Point BICUBIC_ORIGIN = { 0.0, -20.0 };
constexpr Point BICUBIC_FAR    = { 10.0, -0.5 };

// The polarization (remanence, Br) this table was generated at. A real
// magnet is not this strong or weak -- MagnetModel divides its own
// magnet_strength_mT by this to get the ratio it scales the table by.
constexpr float BICUBIC_FIELD_REFERENCE_MT = 1000.0f;

extern const Vec2 BICUBIC_INTERPOLATION_TABLE[NZ][NR];

