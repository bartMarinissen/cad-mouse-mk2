#!/usr/bin/env bash
# Builds and runs the P=45 column-layout wiring check
# (test_bundle_param_layout.cpp), against the REAL firmware forward model --
# same arrangement as verify.sh, separate binary because each Unity test file
# defines its own setup()/loop().
#
# Distinct from verify.sh in what it establishes: verify.sh checks each raw
# derivative is right, this checks they are assembled into the right COLUMNS.
# Both are needed; neither implies the other.
set -euo pipefail
cd "$(dirname "$0")"
FW="../.."   # firmware/
OUT="$(mktemp -d)/test_bundle_param_layout"

g++ -std=c++17 -O2 -DNDEBUG -DEIGEN_NO_MALLOC -D'__not_in_flash_func(x)=x' \
  -I compat -I /usr/include/eigen3 -I "$FW/include" -I . \
  compat/main.cpp \
  test_bundle_param_layout.cpp \
  bundle_shared_jacobian.cpp \
  "$FW/src/magnet_model/BicubicField.cpp" \
  "$FW/src/magnet_model/magnet_local_model.cpp" \
  "$FW/src/magnet_model/magnet_model_table.cpp" \
  "$FW/src/magnet_model/forward_model.cpp" \
  "$FW/src/magnet_model/virtual_sensor.cpp" \
  -o "$OUT"

"$OUT"
