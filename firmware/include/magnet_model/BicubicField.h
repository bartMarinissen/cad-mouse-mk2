#pragma once

#include <cstddef>
#ifdef ARDUINO
    #include <Arduino.h>
#else
    // __not_in_flash_func is an RP2040/earlephilhower-core macro that only
    // exists once Arduino.h (transitively pico/platform.h) is available.
    // It's a no-op outside that build.
    #define __not_in_flash_func(func_name) func_name
#endif

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
    void evaluate(float r, float z, Vec2& value, Vec2& d_dr, Vec2& d_dz) const noexcept;

    Vec2 value(float r, float z) const noexcept { Vec2 v{}, a{}, b{}; evaluate(r, z, v, a, b); return v; }
    Vec2 ddr  (float r, float z) const noexcept { Vec2 v{}, a{}, b{}; evaluate(r, z, v, a, b); return a; }
    Vec2 ddz  (float r, float z) const noexcept { Vec2 v{}, a{}, b{}; evaluate(r, z, v, a, b); return b; }

private:
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
    Vec2 raw(int i, int j) const noexcept;

    // Lookup allowing i in [-1, NR] and j in [-1, NZ] (exactly one step
    // past the real grid, which is all the 4-point cubic stencil ever
    // needs at a boundary cell). Out-of-range points are filled by
    // linear extrapolation from the two nearest real nodes; the corner
    // case combines both directions additively (planar extrapolation).
    Vec2 point_or_ghost(int i, int j) const noexcept;

    // 1D Catmull-Rom cubic through 4 points, local param t in [0,1]
    // between p1 and p2, and its derivative w.r.t. t.
    static Vec2 cubic(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, float t) noexcept;
    static Vec2 cubic_deriv(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, float t) noexcept;
};
