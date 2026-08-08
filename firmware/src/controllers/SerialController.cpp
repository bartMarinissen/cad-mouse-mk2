#include "controllers/SerialController.h"

#include <string.h>

void SerialController::begin() {
  Serial.begin(115200);
  // Carried over from the setup() this replaced: gives USB CDC a moment to
  // enumerate so early boot diagnostics -- including which calibration
  // resolveCalibration() settled on -- have somewhere to land.
  delay(1000);
}

void SerialController::update() {
  size_t budget = kDrainBudget;

  while (budget-- > 0 && Serial.available() > 0) {
    const int next = Serial.read();
    if (next < 0) {
      break;
    }
    const char c = static_cast<char>(next);

    if (c != '\n') {
      if (overflow_) {
        continue;  // still discarding the remains of an oversized line
      }
      if (pendingLen_ + 1 >= kMaxLine) {
        overflow_ = true;
        pendingLen_ = 0;
        continue;
      }
      pending_[pendingLen_++] = c;
      continue;
    }

    if (overflow_) {
      Serial.println("STATUS SERIAL_OVERFLOW");
      overflow_ = false;
      pendingLen_ = 0;
      continue;
    }

    // Tolerate CRLF from hosts and terminals that send it.
    while (pendingLen_ > 0 && pending_[pendingLen_ - 1] == '\r') {
      pendingLen_--;
    }
    pending_[pendingLen_] = '\0';

    if (pendingLen_ > 0) {
      memcpy(line_, pending_, pendingLen_ + 1);
      // One slot, most recent wins. If a line completes while an earlier one
      // is still unconsumed the older is dropped, which is fine for a
      // request/response protocol at human pace -- and better than the
      // alternative of pausing the drain, since states that ignore serial
      // (IdleState) would then let the receive buffer back up.
      ready_ = true;
    }
    pendingLen_ = 0;
  }
}

const char* SerialController::takeLine() {
  if (!ready_) {
    return nullptr;
  }
  ready_ = false;
  return line_;
}
