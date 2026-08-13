#!/usr/bin/env bash
# Builds and runs the finite-difference verification for this directory's
# analytic Jacobian code, against the REAL firmware forward model (the actual
# firmware/src/magnet_model/*.cpp, including the generated bicubic field
# table) -- not a reimplementation or a toy stand-in. Requires libeigen3-dev
# (or any Eigen 3.4 headers) and a host g++; no PlatformIO/ARM toolchain
# needed, since none of this touches hardware-specific code.
#
# This exists so "does the Jacobian actually match finite differences" stays
# a one-command, reproducible fact rather than something asserted once in a
# chat transcript and never checked again.
set -euo pipefail
cd "$(dirname "$0")"
FW="../.."   # firmware/
OUT="$(mktemp -d)/test_bundle_shared_jacobian"

g++ -std=c++17 -O2 -DNDEBUG -DEIGEN_NO_MALLOC -D'__not_in_flash_func(x)=x' \
  -I compat -I /usr/include/eigen3 -I "$FW/include" -I . \
  compat/main.cpp \
  test_bundle_shared_jacobian.cpp \
  bundle_shared_jacobian.cpp \
  "$FW/src/magnet_model/BicubicField.cpp" \
  "$FW/src/magnet_model/magnet_local_model.cpp" \
  "$FW/src/magnet_model/magnet_model_table.cpp" \
  "$FW/src/magnet_model/forward_model.cpp" \
  "$FW/src/magnet_model/sensor.cpp" \
  -o "$OUT"

"$OUT"
