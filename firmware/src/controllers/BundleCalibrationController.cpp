#include "controllers/BundleCalibrationController.h"



// Constants
constexpr uint32_t COUNTDOWN_MS = 1000;
constexpr uint32_t FRAME_INTERVAL_MS = 50; // 20Hz

BundleCalibrationController::BundleCalibrationController() 
    : current_step(CalibStep::NONE), current_phase(CalibPhase::IDLE) {}

uint32_t BundleCalibrationController::get_duration_for_step(CalibStep step) {
    if (step == CalibStep::STATIONARY) return 1000; // 1 second
    return 3000; // 3 seconds for dynamic movements
}

void BundleCalibrationController::change_phase(CalibPhase new_phase) {
    current_phase = new_phase;
    phase_start_time = millis();
    
    // Notify PC of state change (and theoretically update LED ring here)
    Serial.printf("CAL_STATE %d %d\n", (int)current_step, (int)current_phase);
}

void BundleCalibrationController::handle_serial_command(const char* cmd) {
    if (strncmp(cmd, "CAL_START", 9) == 0) {
        Serial.println("STARTING_CAL");
        current_step = CalibStep::STATIONARY;
        change_phase(CalibPhase::WAIT_FOR_START_BTN);
    } 
    else if (strncmp(cmd, "CAL_ACK", 7) == 0) {
        if (current_phase == CalibPhase::WAIT_FOR_ACK) {
            acked_samples = atoi(cmd + 8);
            // Check if PC got enough samples (allow 10% drop rate tolerance)
            if (acked_samples == expected_samples) {
                change_phase(CalibPhase::REVIEW);
            } else {
                // PC rejected or lost data, force a retry
                Serial.printf("STATUS ACK_FAILED_INSUFFICIENT_DATA got %d expected %d\n", acked_samples, expected_samples);
                change_phase(CalibPhase::WAIT_FOR_START_BTN); 
            }
        }
    }
}

void BundleCalibrationController::update(uint16_t button_bits, SensorController &sensorController) {

    bool btn_left_pressed = button_bits & 1;
    bool btn_right_pressed = button_bits & 2;

    float raw_field[9];
    
    // When done or waiting for serial start, do nothing
    if (current_phase == CalibPhase::IDLE || current_step == CalibStep::COMPLETE) {
        return;
    }

    uint32_t now = millis();
    uint32_t elapsed = now - phase_start_time;

    switch (current_phase) {
        case CalibPhase::WAIT_FOR_START_BTN:
            // LED RING: animating the step with its specific animation
            if (btn_left_pressed) {
                change_phase(CalibPhase::COUNTDOWN);
                
            }
            break;

        case CalibPhase::COUNTDOWN:
            // LED RING: filling up clockwise
            if (elapsed >= COUNTDOWN_MS) {
                expected_samples = 0;
                change_phase(CalibPhase::RECORDING);
            }
            break;

        case CalibPhase::RECORDING:
            // LED RING: animating the step with its specific animation
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
            // LED RING: dead
            // Waiting for handle_serial_command to process "CAL_ACK"
            // Add a timeout just in case the PC script crashed

            if (elapsed > 20000) { 
                change_phase(CalibPhase::WAIT_FOR_START_BTN); // Timeout, retry
            }
            break;

        case CalibPhase::REVIEW:
            // LED RING: repeating animation of completed step
            if (button_bits) {
                // Next step!
                int next_step = (int)current_step + 1;
                if (next_step > (int)CalibStep::RANDOM) {
                    current_step = CalibStep::COMPLETE;
                    change_phase(CalibPhase::IDLE);
                    Serial.println("STATUS ALL_STEPS_COMPLETE");
                } else {
                    current_step = (CalibStep)next_step;
                    change_phase(CalibPhase::WAIT_FOR_START_BTN);
                }
            }
            // TODO: handle retries
            // } else if (btn_left_pressed) {
            //     // Retry same step
            //     change_phase(CalibPhase::WAIT_FOR_START_BTN);
            // }
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
