#!/usr/bin/env bash
# Builds and runs the exact-algebra verification for schur_normal_equations.h
# (test_schur_normal_equations.cpp) -- a separate binary from verify.sh's
# Jacobian check since each Unity test file defines its own setup()/loop().
# No firmware forward-model sources needed here: this is pure linear algebra,
# independent of BicubicField/MagnetModel/VirtualSensor.
set -euo pipefail
cd "$(dirname "$0")"
OUT="$(mktemp -d)/test_schur_normal_equations"

g++ -std=c++17 -O2 -DNDEBUG -DEIGEN_NO_MALLOC -D'__not_in_flash_func(x)=x' \
  -I compat -I /usr/include/eigen3 -I ../../include -I . \
  compat/main.cpp \
  test_schur_normal_equations.cpp \
  -o "$OUT"

"$OUT"
