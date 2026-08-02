// ============================================================================
// AUTO-GENERATED -- DO NOT EDIT BY HAND.
// Written directly by cell af552cc7 of magnet_field_model/field_approximation.ipynb.
// Re-running that cell overwrites this file; manual edits here will be lost.
// ============================================================================

#pragma once
#include "math3D.h"

struct Point { float r; float z; };

using Vec2 = Eigen::Vector2f;
constexpr int NR = 51;
constexpr int NZ = 91;

constexpr Point BICUBIC_ORIGIN = { 0.0, -20.0 };
constexpr Point BICUBIC_FAR    = { 10.0, -0.5 };

extern const Vec2 BICUBIC_INTERPOLATION_TABLE[NZ][NR];

