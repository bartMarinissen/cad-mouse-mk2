#!/usr/bin/env bash
# Cross-compiles this prototype for the actual target (Cortex-M0+, no FPU) and
# reports what it costs in flash and static RAM.
#
# This is a COMPILE check, not a run: there is no hardware here, and nothing in
# this directory is in the PlatformIO build, so `pio test` will never reach it.
# verify.sh and verify_schur.sh are what establish the numerics are right; this
# establishes the numerics are right *and buildable for the device*, which the
# host builds cannot show -- they run on x86 with hardware floats.
#
# Needs the earlephilhower toolchain, which `pio run -e seeed_xiao_rp2040`
# fetches. If it isn't there, this script says so and exits 0 rather than
# failing: it is an additional check, not a gate the host suites depend on.
set -euo pipefail
cd "$(dirname "$0")"

TC="$HOME/.platformio/packages/toolchain-rp2040-earlephilhower/bin"
if [ ! -x "$TC/arm-none-eabi-g++" ]; then
  echo "ARM toolchain not found at $TC"
  echo "Install it with:  pip install platformio && pio run -e seeed_xiao_rp2040"
  exit 0
fi

OUT="$(mktemp -d)"

# schur_normal_equations.h is all templates, which compile to nothing until
# something instantiates them -- so an ARM build that only includes the header
# would report a clean pass having checked no code at all. Instantiate at the
# real P before measuring.
cat > "$OUT/instantiate_at_real_size.cpp" <<'EOF'
#include "schur_normal_equations.h"
static constexpr int P = 45;   // shared calibration parameters
template struct FramePoseBlock<P>;
template struct FrameNormalEquations<P>;
template struct SharedNormalEquations<P>;
template Eigen::Matrix<float, 6, 1> solve_frame_pose_update<P>(
    const FramePoseBlock<P>&, const Eigen::Matrix<float, P, 1>&);
EOF

# -mcpu/-mthumb match the target; soft-float is implied (M0+ has no FPU unit
# to target). -O2 rather than -Os because platformio.ini unflags -Os for this
# project's builds.
# -Wstack-usage is a GATE, not decoration. The device gives core0 a 2 KB
# reserved stack inside a 4 KB region (PICO_STACK_SIZE=0x800 in SCRATCH_Y;
# .scratch_y measures 0 bytes in the real firmware, so ~4 KB is reachable
# before the stack runs into core1's at __StackOneTop). Frames NEST, so the
# budget is the deepest path, not any single function.
#
# This caught real bugs that no amount of commenting had: build_frame at
# 19,256 bytes (resetting an accumulator via `*this = T()` built a full-size
# temporary) and solve at 12,536 (Eigen materializing P x P products, and
# H.ldlt() putting an 8.1 KB factorization on the stack). Both were in code
# whose comment already said "must be statically allocated" -- which was true
# and beside the point.
STACK_BUDGET=2048
FLAGS=(-std=c++17 -O2 -DNDEBUG -DEIGEN_NO_MALLOC
       -mcpu=cortex-m0plus -mthumb
       "-D__not_in_flash_func(x)=x"
       -Wstack-usage=$STACK_BUDGET -fstack-usage
       -I compat -I /usr/include/eigen3 -I ../../include -I .)

(cd "$OUT" && :); "$TC/arm-none-eabi-g++" -c "${FLAGS[@]}" bundle_shared_jacobian.cpp -o "$OUT/jacobian.o"
"$TC/arm-none-eabi-g++" -c "${FLAGS[@]}" "$OUT/instantiate_at_real_size.cpp" -o "$OUT/solver.o"

echo "Compiles for cortex-m0plus. Cost, per translation unit:"
echo "  (text = flash, bss = static RAM; neither includes stack)"
"$TC/arm-none-eabi-size" "$OUT/jacobian.o" "$OUT/solver.o"

echo
echo "Deepest stack frames (budget ${STACK_BUDGET} bytes/frame; frames NEST,"
echo "so the true peak is the sum along the deepest call path):"
cat "$OUT"/*.su 2>/dev/null \
  | awk -F'\t' '{n=$1; sub(/.*\//,"",n); printf "%7d  %s\n", $2, substr(n,1,78)}' \
  | sort -rn | head -6
