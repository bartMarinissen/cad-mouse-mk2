#pragma once

#include <cstddef>

// Stand-in for the Arduino framework header, on the include path for
// env:native_test only (platformio.ini). It is not a portability layer and
// nothing in firmware/ is conditionally compiled against it: it exists so the
// magnet_model solver chain, which is pure BLA arithmetic, can be compiled
// and its Jacobians finite-difference-checked without a board attached.
//
// Two things reach that chain through the real Arduino.h:
//
//   1. __not_in_flash_func, below.
//   2. Print, below -- required only because BLA::MatrixBase::printTo()
//      (BasicLinearAlgebra.h) takes a Print& and calls .print() on it. Even
//      after the vendoring patch drops Printable as a *base class* (see
//      TODO/eigen-to-bla-migration.md -- that's what removed the hidden
//      vtable pointer), printTo() itself is still a plain member function
//      that needs the Print type to exist and support the two overloads it
//      calls, char and DType (float here). Nothing in this codebase actually
//      calls printTo() -- it exists for Serial.print(myMatrix) support this
//      firmware doesn't use -- but it still has to typecheck.
//
// If a file outside magnet_model/ ever needs to build here, this is the wrong
// place to grow -- that code wants the real framework, which means it wants
// the hardware test environment.

// On target this is a pico-SDK attribute that pins a function into SRAM rather
// than leaving it in XIP flash, so the solver's hot path does not pay
// flash-cache misses per call (see TODO/Performance.md). It arrives in the
// firmware build through Arduino.h, which is why the host stand-in for
// Arduino.h is where it belongs here too.
//
// Off target there is no XIP flash and no section to place into, so it
// collapses to the identity. The function is compiled exactly as written,
// which is what makes the host build a fair test of the same arithmetic
// rather than of a differently-annotated variant of it.
#define __not_in_flash_func(func_decl) func_decl

// Same idea as __not_in_flash_func above, but for data (pico/platform.h's
// __not_in_flash(group), used by magnet_model_table.cpp to keep
// BICUBIC_INTERPOLATION_TABLE in RAM instead of XIP flash on target). Off
// target there's nothing to place, so this collapses to nothing and the
// declaration is compiled exactly as an ordinary array.
#define __not_in_flash(group)

// Minimal stand-in for Arduino's Print -- just enough surface for
// MatrixBase::printTo() to typecheck (see above). Never actually invoked by
// anything this test suite runs.
struct Print {
    size_t print(char) { return 1; }
    size_t print(float) { return 1; }
};
