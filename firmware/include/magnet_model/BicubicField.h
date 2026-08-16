#pragma once

#include <cstddef>
#include <Arduino.h>

#include "math3D.h"
#include "magnet_model_table.h"


class BicubicField {

public:
    const Vec2 (&grid_)[NZ][NR]{};
    Point origin_{};
    Point far_{};
    float dr_{};
    float dz_{};
    float dr_reciprocal_{};
    float dz_reciprocal_{};

    // values[0][0] = sample at r = origin.r z = origin.z
    // values[NR-1][NZ-1] = sample at r = far.r z = far.z
    // intermediate values are uniformly spaced in r and z.
    // dr, dz derived from the bounding box and grid size.
    constexpr BicubicField(const Vec2 (&values)[NZ][NR],
                            Point origin, Point far) noexcept
        : grid_(values),
          origin_(origin), far_(far),

          dr_((far.r - origin.r) / static_cast<float>(NR - 1)),
          dz_((far.z - origin.z) / static_cast<float>(NZ - 1)),
          dr_reciprocal_(1.0f/dr_),
          dz_reciprocal_(1.0f/dz_)
    {}

    // Value and gradient (d/dr, d/dz, each a Vec2) at (r, z).
    Vec2 evaluate(float r, float z, Mat2 &jacobian) const noexcept;
};
