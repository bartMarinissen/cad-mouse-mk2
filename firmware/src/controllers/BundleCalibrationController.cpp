#include "controllers/BundleCalibrationController.h"

#include <string.h>

#include "CalibrationStorage.h"
#include "Config.h"
#include "Controllers.h"
#include "animations/Animations.h"

// Constants
constexpr uint32_t COUNTDOWN_MS = 1000;
constexpr uint32_t FRAME_INTERVAL_MS = 50; // 20Hz

// How long a phase ignores the buttons after it starts. Entry into
// calibration is a three-second both-button hold, so the buttons are still
// down when the first WAIT_FOR_START_BTN begins looking at them and the step
// fires instantly. A grace window off phase_start_time fixes that without
// blocking, and covers every phase that reads buttons rather than just the
// first one.
constexpr uint32_t BUTTON_GRACE_MS = 400;

BundleCalibrationController::BundleCalibrationController() 
    : current_step(CalibStep::NONE), current_phase(CalibPhase::IDLE) {}

uint32_t BundleCalibrationController::get_duration_for_step(CalibStep step) {
    if (step == CalibStep::STATIONARY) return 1000; // 1 second
    return 3000; // 3 seconds for dynamic movements
}

void BundleCalibrationController::change_phase(CalibPhase new_phase) {
    current_phase = new_phase;
    phase_start_time = millis();

    // Placeholder: same spinner for every phase. The real per-phase/per-step
    // look (see the LED RING comments below and TODO/calibration-led-animations.md)
    // is still undesigned.
    ledController().set(SpinnerAnimation(ledController().ring(), Config::LED_CALIBRATING_COLOR));

    // Notify PC of state change
    Serial.printf("CAL_STATE %d %d\n", (int)current_step, (int)current_phase);
}

void BundleCalibrationController::start() {
    Serial.println("STARTING_CAL");
    current_step = CalibStep::STATIONARY;
    change_phase(CalibPhase::WAIT_FOR_START_BTN);
}

void BundleCalibrationController::abort() {
    // Straight back to the constructed state, so a later start() begins clean
    // rather than resuming a half-finished run.
    current_step = CalibStep::NONE;
    current_phase = CalibPhase::IDLE;
    Serial.println("STATUS CAL_ABORTED");
}

bool BundleCalibrationController::handle_serial_command(const char* cmd) {
    if (strcmp(cmd, "CAL_ABORT") == 0) {
        // The way out that stores nothing. Capture and fit never touch flash
        // on their own -- only CAL_UPLOAD does -- so this is what lets a
        // session be run, looked at, and walked away from. It is also the only
        // escape from a run whose host went away, since WAIT_FOR_ACK just
        // times out back to WAIT_FOR_START_BTN forever.
        abort();
        return true;
    }

    if (CalibrationStorage::handleUploadCommand(cmd)) {
        // Reboots on success and does not return. If it did return the upload
        // was rejected, and the host gets to retry without losing the capture.
        return false;
    }

    if (strncmp(cmd, "CAL_START", 9) == 0) {
        start();
    }
    else if (strncmp(cmd, "CAL_ACK", 7) == 0) {
        if (current_phase == CalibPhase::WAIT_FOR_ACK) {
            acked_samples = atoi(cmd + 8);
            // Check if PC got enough samples (allow 10% drop rate tolerance)
            if (acked_samples == expected_samples) {
                // Straight on to the next step (or AWAITING_UPLOAD if that was
                // the last one) -- no button press in between. There is no
                // way back to redo a step, so there was nothing for a review
                // pause to do except make the user press again.
                int next_step = (int)current_step + 1;
                if (next_step > (int)CalibStep::RANDOM) {
                    current_step = CalibStep::COMPLETE;
                    change_phase(CalibPhase::AWAITING_UPLOAD);
                    Serial.println("STATUS ALL_STEPS_COMPLETE");
                    Serial.println("STATUS AWAITING_UPLOAD");
                } else {
                    current_step = (CalibStep)next_step;
                    change_phase(CalibPhase::WAIT_FOR_START_BTN);
                }
            } else {
                // PC rejected or lost data, force a retry
                Serial.printf("STATUS ACK_FAILED_INSUFFICIENT_DATA got %d expected %d\n", acked_samples, expected_samples);
                change_phase(CalibPhase::WAIT_FOR_START_BTN);
            }
        }
    }
    return false;
}

void BundleCalibrationController::update(uint16_t button_bits, SensorController &sensorController) {

    bool btn_left_pressed = button_bits & 1;
    bool btn_right_pressed = button_bits & 2;

    float raw_field[9];
    
    // Waiting for serial start. AWAITING_UPLOAD is deliberately not short-cut
    // here -- it is a real phase now, handled in the switch below.
    if (current_phase == CalibPhase::IDLE) {
        return;
    }

    uint32_t now = millis();
    uint32_t elapsed = now - phase_start_time;

    switch (current_phase) {
        case CalibPhase::WAIT_FOR_START_BTN:
            // LED RING: placeholder spinner set in change_phase(); intended
            // look is a per-step identifying animation, still undesigned.
            if (elapsed >= BUTTON_GRACE_MS && btn_left_pressed) {
                change_phase(CalibPhase::COUNTDOWN);
            }
            break;

        case CalibPhase::COUNTDOWN:
            // LED RING: placeholder spinner set in change_phase(); intended
            // look is a clockwise fill, still undesigned.
            if (elapsed >= COUNTDOWN_MS) {
                expected_samples = 0;
                change_phase(CalibPhase::RECORDING);
            }
            break;

        case CalibPhase::RECORDING:
            // LED RING: placeholder spinner set in change_phase(); intended
            // look is a per-step identifying animation, still undesigned.
            if (now - last_frame_time >= FRAME_INTERVAL_MS) {
                // TODO consider 2 quick reads in succession for smoothing

                // Only read the sensors if we need them, keeps things fast.
                sensorController.readUncorrected(raw_field);
                last_frame_time = now;
                send_frame(raw_field);
                expected_samples++;
            }
            
            if (elapsed >= get_duration_for_step(current_step)) {
                change_phase(CalibPhase::WAIT_FOR_ACK);
            }
            break;

        case CalibPhase::WAIT_FOR_ACK:
            // LED RING: placeholder spinner set in change_phase(); intended
            // look is "dead" (off), still undesigned.
            // Waiting for handle_serial_command to process "CAL_ACK"
            // Add a timeout just in case the PC script crashed

            if (elapsed > 20000) { 
                change_phase(CalibPhase::WAIT_FOR_START_BTN); // Timeout, retry
            }
            break;

        case CalibPhase::AWAITING_UPLOAD:
            // Every step captured. Stay put rather than dropping back to the
            // mouse: the host is about to solve and hand back a calibration,
            // and making the user walk the knob back into calibration mode to
            // receive it is the wrong shape. Leaves on CAL_UPLOAD (which
            // reboots) or CAL_ABORT.
            break;

        default:
            break;
    }
}

void BundleCalibrationController::send_frame(const float raw_field[9]) {
    Serial.printf("CAL_FRAME %d %lu ", (int)current_step, millis());
    for(int i=0; i<9; i++) {
        Serial.printf("%.8f ", raw_field[i]);
    }
    Serial.printf("\n");
}
