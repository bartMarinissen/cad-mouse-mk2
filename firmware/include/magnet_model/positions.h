#pragma once

#include "math3D.h"

namespace Positions{
    const float triangle_sidelength_mm = 28.58f;
    const float vertex_distance_mm = triangle_sidelength_mm / sqrtf(3);

    // World frame has its origin at the centre of the sensors.
    Vec3 sensor_1_world = {vertex_distance_mm * 0.5, vertex_distance_mm *  sqrtf(3.0)/2, 0};
    Vec3 sensor_2_world = {vertex_distance_mm * 0.5, vertex_distance_mm * -sqrtf(3.0)/2, 0};
    Vec3 sensor_3_world = {0.0                     , vertex_distance_mm                , 0};        

    // Knob frame has its origin at the centre of the magnets 
    Vec3 Magnet_1_knob = {vertex_distance_mm * 0.5, vertex_distance_mm *  sqrtf(3.0)/2, 0};
    Vec3 Magnet_2_knob = {vertex_distance_mm * 0.5, vertex_distance_mm * -sqrtf(3.0)/2, 0};
    Vec3 Magnet_3_knob = {0.0                     , vertex_distance_mm                , 0};
}