#include "states/BundleState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void BundleState::enter() {
    InputController& input = inputController();
    ledController().startSpinner(0xFDFDFF);
    input.takeActivity();
    input.takeCalibrationRequest();

}

void BundleState::update() {
    InputController& input = inputController();
    BundleCalibrationController& bundleCalibration = bundleCalibrationController();

    ledController().updateSpinner();
    input.update();

    char command[128]{};
    if (Serial.available())
        Serial.readBytesUntil('\n', command, sizeof(command));
    bundleCalibration.handle_serial_command(command);

    uint16_t button_bits = input.takeActivity();
    // We pass the sensorController so the callibrator can be selective in when it wants to read the sensor.
    bundleCalibration.update(button_bits, sensorController());

   if (bundleCalibration.is_done()){
       stateMachine.changeState(&StateMachine::idleState);
   }
}

void BundleState::exit() {}
