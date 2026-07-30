#pragma once

#include "math3D.h"

namespace Positions{
    constexpr float SQRT3_F = 1.73205078f;
    constexpr float triangle_sidelength_mm = 28.58f;
    constexpr float R = triangle_sidelength_mm / SQRT3_F;

    // How far the magnets sit below the pivot point of the knob
    // That pivot point is the origin of the knob-frame
    constexpr float magnet_z_pos_from_pivot = 14;
    // The aproximate expected distance form the sensor to the magnet
    constexpr float magnet_rest_distance_sensor = 6;

// World frame (Origin at center, +X Right, +Y towards USB-C)
    inline const Vec3 sensor_1_world = { 0.0f,            -R,       0.0f }; // Bottom Center
    inline const Vec3 sensor_2_world = { -R * SQRT3_F/2,  R * 0.5f, 0.0f }; // Top Left
    inline const Vec3 sensor_3_world = {  R * SQRT3_F/2,  R * 0.5f, 0.0f }; // Top Right

    // Knob frame (Assuming magnets sit perfectly above sensors at rest)
    inline const Vec3 Magnet_1_knob = { 0.0f,            -R,       -magnet_z_pos_from_pivot };
    inline const Vec3 Magnet_2_knob = { -R * SQRT3_F/2,  R * 0.5f, -magnet_z_pos_from_pivot };
    inline const Vec3 Magnet_3_knob = {  R * SQRT3_F/2,  R * 0.5f, -magnet_z_pos_from_pivot };

    inline const Vec3 approx_rest_pos = {0.0f, 0.0f, magnet_z_pos_from_pivot};
}
