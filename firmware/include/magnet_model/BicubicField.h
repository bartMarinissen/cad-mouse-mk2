#pragma once

#include <cstddef>
#include <Arduino.h>

#include "math3D.h"
#include "magnet_model_table.h"


class BicubicField {

public:
    const Vec2 (&grid_)[NZ][NR]{};
    // origin_/far_ are (r, z) pairs stored in a Vec2 -- .x() is r, .y() is z
    // (matches how BICUBIC_ORIGIN/BICUBIC_FAR are constructed in
    // magnet_model_table.h: Vec2(r, z)). There used to be a dedicated Point{r,z}
    // struct for this; dropped as a redundant type doing exactly what Vec2
    // already does.
    Vec2 origin_{};
    Vec2 far_{};
    float dr_{};
    float dz_{};
    float dr_reciprocal_{};
    float dz_reciprocal_{};

    // values[0][0] = sample at r = origin.x() z = origin.y()
    // values[NR-1][NZ-1] = sample at r = far.x() z = far.y()
    // intermediate values are uniformly spaced in r and z.
    // dr, dz derived from the bounding box and grid size.
    constexpr BicubicField(const Vec2 (&values)[NZ][NR],
                            Vec2 origin, Vec2 far) noexcept
        : grid_(values),
          origin_(origin), far_(far),

          dr_((far.x() - origin.x()) / static_cast<float>(NR - 1)),
          dz_((far.y() - origin.y()) / static_cast<float>(NZ - 1)),
          dr_reciprocal_(1.0f/dr_),
          dz_reciprocal_(1.0f/dz_)
    {}

    // Value and gradient (d/dr, d/dz, each a Vec2) at (r, z).
    Vec2 evaluate(float r, float z, Mat2 &jacobian) const noexcept;
};
