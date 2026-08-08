#include "states/BundleState.h"

#include <Arduino.h>
#include <string.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void BundleState::enter() {
    InputController& input = inputController();
    ledController().startSpinner(0xFDFDFF);
    input.takeActivity();
    input.takeCalibrationRequest();

    // CalibratingState only gets us here by consuming a CAL_START, so the
    // command itself is already gone. Kick the run off directly instead of
    // waiting for a second one -- this also resets current_step, so a repeat
    // calibration works without a reboot.
    bundleCalibrationController().start();
}

void BundleState::update() {
    InputController& input = inputController();
    BundleCalibrationController& bundleCalibration = bundleCalibrationController();

    ledController().updateSpinner();
    input.update();

    // Non-blocking, and only dispatched when a line actually arrived -- the
    // old readBytesUntil() could stall the loop for up to Stream's 1000ms
    // timeout, and handed the controller a zeroed buffer every idle tick.
    const char* command = serialController().takeLine();
    if (command != nullptr) {
        // The way out that does not store anything. Capture and fit never
        // touch flash on their own -- only an explicit CAL_UPLOAD does -- so
        // this is what lets a session be run, looked at, and walked away from.
        // It is also the only escape from a run whose host went away, since
        // WAIT_FOR_ACK just times out back to WAIT_FOR_START_BTN forever.
        if (strcmp(command, "CAL_ABORT") == 0) {
            bundleCalibration.abort();
            stateMachine.changeState(&StateMachine::idleState);
            return;
        }
        bundleCalibration.handle_serial_command(command);
    }

    uint16_t button_bits = input.takeActivity();
    // We pass the sensorController so the callibrator can be selective in when it wants to read the sensor.
    bundleCalibration.update(button_bits, sensorController());

   if (bundleCalibration.is_done()){
       stateMachine.changeState(&StateMachine::idleState);
   }
}

void BundleState::exit() {}
