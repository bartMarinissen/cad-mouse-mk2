#pragma once

#include <Arduino.h>

// Turns the incoming serial byte stream into whole lines, without blocking.
//
// Transport only. What a line *means* stays with whoever owns the command --
// the same split BundleCalibrationController already assumes, since
// handle_serial_command() takes an already-extracted line and never touches
// Serial itself.
//
// This replaces the Serial.readBytesUntil('\n', ...) that used to live in
// BundleState::update(). Message size is not really the argument -- normal
// operation never sees a long line. The problem is that readBytesUntil blocks
// until the delimiter arrives *or* Stream's 1000ms timeout expires, so any
// line that never completes stalls the ~120Hz loop for a full second and
// stutters HID. A host dying midway through "CAL_ACK 60" does that just as
// well as anything long. Here each update() takes only the bytes already
// buffered and returns, so an unfinished line costs nothing.
//
// A calibration upload is ~640 characters, which is the one case where even a
// complete line takes a visible ~55ms at 115200 baud, but that is a second
// reason rather than the main one.
//
// Output is deliberately not routed through this class. Telemetry,
// SensorController and the bundle protocol keep printing to Serial directly;
// there is no contention to arbitrate, and wrapping writes would be churn for
// its own sake.
class SerialController {
 public:
  void begin();

  // Drain whatever has arrived and assemble it into lines. Called once per
  // loop() from main.cpp, before the state machine runs, so whichever state
  // is active sees a line on the same tick it completed.
  void update();

  // The most recently completed line, or nullptr if none is waiting.
  // Consuming it clears it.
  //
  // The returned pointer is only valid until the next update(), which is to
  // say until the end of the current tick. Anything that needs to outlive
  // that has to copy it.
  const char* takeLine();

 private:
  // "CAL_UPLOAD " + 624 hex characters + terminator is 636, so the largest
  // real command sits at about 62% of this.
  static constexpr size_t kMaxLine = 1024;

  // Bytes to take per tick. USB CDC's own receive buffer is far smaller, so
  // in practice every pending byte is consumed each time; the cap only exists
  // so a pathological flood cannot stretch one tick and stutter HID. Derived
  // from kMaxLine rather than written out, so a complete upload keeps landing
  // in a single tick if that ever changes.
  static constexpr size_t kDrainBudget = kMaxLine * 2;

  // Two buffers on purpose. A completed line sits in line_ until someone
  // takes it, while bytes for the *next* line keep arriving into pending_.
  // Sharing one buffer would let an in-flight line overwrite a completed one
  // before its state got a chance to read it.
  char pending_[kMaxLine];
  size_t pendingLen_ = 0;
  char line_[kMaxLine];
  bool ready_ = false;

  // Set when a line outgrows the buffer. The rest of that line is discarded
  // through its newline rather than silently truncated into a command that
  // looks valid -- the old 128-byte read had no way to tell the difference.
  bool overflow_ = false;
};
