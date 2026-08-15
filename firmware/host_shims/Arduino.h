#pragma once

// Stand-in for the Arduino framework header, on the include path for
// env:native_test only (platformio.ini). It is not a portability layer and
// nothing in firmware/ is conditionally compiled against it: it exists so the
// magnet_model solver chain, which is pure Eigen arithmetic, can be compiled
// and its Jacobians finite-difference-checked without a board attached.
//
// Two things reach that chain through the real Arduino.h, and only two:
//
//   1. __not_in_flash_func, below.
//   2. Nothing at all, by way of ArduinoEigen. ArduinoEigenDense.h includes
//      Arduino.h solely to *undo* macros the framework defines (abs, round,
//      A0, B0, ...), and every one of those fixups is #ifdef-guarded. With no
//      framework there are no macros to undo, so an empty header is the
//      correct content rather than a missing one.
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
