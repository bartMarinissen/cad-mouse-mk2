#pragma once
#include <Arduino.h>
#include "math3D.h" // Assuming your Vec3/Mat3 are here
// We need this to contditionally read the sensor
#include "controllers/SensorController.h"

enum class CalibStep {
    NONE = -1,
    STATIONARY = 0,
    FLAT_CIRCLE = 1,
    PITCH = 2,
    ROLL = 3,
    TWIST = 4,
    HEAVE = 5,
    RANDOM = 6,
    COMPLETE = 7
};

enum class CalibPhase {
    IDLE,               // Waiting for CAL_START
    WAIT_FOR_START_BTN, // Waiting for user to press LEFT
    COUNTDOWN,          // 1s countdown active
    RECORDING,          // Streaming data to PC
    WAIT_FOR_ACK,       // Waiting for PC to say "CAL_ACK 60"
    REVIEW              // Waiting for RIGHT (Next) or LEFT (Retry)
};

class BundleCalibrationController {
public:
    BundleCalibrationController();
    
    // Call this every cycle in your main loop()
    void update(uint16_t button_bits, SensorController &sensorController);
    
    // Call this when a complete line is received over Serial
    void handle_serial_command(const char* cmd);

    bool is_active() const { return current_phase != CalibPhase::IDLE; }
    bool is_done() const { return current_step == CalibStep::COMPLETE; }

private:
    CalibStep current_step;
    CalibPhase current_phase;
    
    uint32_t phase_start_time;
    uint32_t last_frame_time;
    
    int expected_samples;
    int acked_samples;

    void change_phase(CalibPhase new_phase);
    uint32_t get_duration_for_step(CalibStep step);
    void send_frame(const float raw_field[9]);
};
