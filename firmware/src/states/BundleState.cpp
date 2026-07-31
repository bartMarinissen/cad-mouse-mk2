#include "states/BundleState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void BundleState::enter() {
    ledController.startSpinner(0xFDFDFF);
    inputController.takeActivity();
    inputController.takeCalibrationRequest();

}

void BundleState::update() {
    ledController.updateSpinner();
    inputController.update();

    char command[128]{};
    if (Serial.available())
        Serial.readBytesUntil('\n', command, sizeof(command));
    bundleCalibrationController.handle_serial_command(command);

    uint16_t button_bits = inputController.takeActivity();
    // We pass the sensorController so the callibrator can be selective in when it wants to read the sensor.
    bundleCalibrationController.update(button_bits, sensorController);

   if (bundleCalibrationController.is_done()){
       stateMachine.changeState(&StateMachine::idleState);
   }
}

void BundleState::exit() {}
