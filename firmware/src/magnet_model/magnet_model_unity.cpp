// Unity (jumbo) translation unit for the pose-solver hot path.
//
// Including the .cpp files rather than merging them on disk lets GCC see the
// whole solve_knob_pose -> ForwardModel::evaluate -> VirtualSensor::evaluate ->
// MagnetModel::evaluate -> BicubicField::evaluate chain as one translation
// unit, so it can inline across those boundaries and CSE work that is
// currently repeated on either side of a call.
//
// This file is NOT part of the default build. platformio.ini excludes it from
// [env:seeed_xiao_rp2040] and instead excludes the five individual files in
// [env:seeed_xiao_rp2040_unity], so exactly one of the two arrangements is
// compiled at a time. If both were ever active you would get duplicate-symbol
// link errors rather than anything subtle.
//
// Include order matters only in that each file must still see its own headers;
// they all use include guards, so the order below is simply innermost-first for
// readability.

#include "BicubicField.cpp"        // NOLINT(bugprone-suspicious-include)
#include "magnet_local_model.cpp"  // NOLINT(bugprone-suspicious-include)
#include "virtual_sensor.cpp"       // NOLINT(bugprone-suspicious-include)
#include "forward_model.cpp"       // NOLINT(bugprone-suspicious-include)
#include "solve_pose.cpp"          // NOLINT(bugprone-suspicious-include)
