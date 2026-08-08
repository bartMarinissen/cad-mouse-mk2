#include "controllers/TelemetryController.h"
#include <Arduino.h>
#include "Config.h"

namespace {
const int kPrintEvery = 20;
}

void TelemetryController::begin() { tick_ = 0; }

bool TelemetryController::enabled() const { return Config::ENABLE_TELEMETRY; }


void TelemetryController::publish(const float motion[6], float residual_percent, int buttonBits,
                                  bool hidReportSent, Statistics const &stats,
                                  const float raw_field[9], const Vec3 &last_pos, const Vec3 &last_rot, 
                                  float rcond) {
  if (!enabled()) {
    return;
  }

  tick_++;
  if ((tick_ % kPrintEvery) != 0) {
    return;
  }

  // --- UI Configuration Constants ---
  constexpr int kCols = 51; 
  constexpr int kRows = 24; // Increased to 23 to fit the split footer
  
  char buffer[kRows][kCols];
  
  // 1. Initialize all visible rows completely to spaces
  // We only memset (kRows - 1) because the final row is just for the \0
  memset(buffer, ' ', (kRows - 1) * kCols);
  
  // 2. Set up the vertical borders and newlines for all visible rows
  for (int i = 0; i < kRows - 1; ++i) {
    buffer[i][0] = '|';
    buffer[i][kCols - 2] = '|';
    buffer[i][kCols - 1] = '\n';
  }

  // 3. Set top and bottom horizontal borders
  memset(&buffer[0][1], '-', kCols - 3); 
  buffer[0][0] = '+'; 
  buffer[0][kCols - 2] = '+';

  memset(&buffer[kRows - 2][1], '-', kCols - 3); 
  buffer[kRows - 2][0] = '+'; 
  buffer[kRows - 2][kCols - 2] = '+';

  // 4. Set the final zero terminator on the very last line
  buffer[kRows - 1][0] = '\0';

  Vector9f residual_stddev = stats.get_residual_stddev();
  
  char temp[64];
  int len;
  int row = 1;  // Row 0 is the top border; advance this as rows are filled
                // so inserting/removing rows doesn't require renumbering.

  // Field labels for the Pose/USB rows below. Both value rows use an
  // 8-byte prefix ("| Pose: " / "| USB:  ") and the same field widths, so
  // this header lines up with both.
  // No degree unit here: a real "°" is 2 UTF-8 bytes but 1 terminal
  // column, which throws off the fixed-width %s/%f padding and shifts the
  // right-hand border on any row that uses it (verified: the single-byte
  // Latin-1 0xB0 renders wrong in this terminal). Fixing that properly
  // means giving each row its own border byte-offset instead of the
  // shared one set up above, which isn't worth it for a debug printout.
  len = snprintf(temp, sizeof(temp), "        X mm   Y mm   Z mm    Pitch  Roll Twist");
  memcpy(buffer[row++], temp, len);

  // Actual Measured Pose (Raw output from the solver)
  len = snprintf(temp, sizeof(temp), "| Pose: %5.3f %5.3f %5.3f %6.3f %5.1f %5.3f",
                 last_pos(0), last_pos(1), last_pos(2),
                 last_rot(0), last_rot(1), last_rot(2));
  memcpy(buffer[row++], temp, len);

  // USB Report (The filtered/mapped HID output). Prefix padded to match
  // "| Pose: "'s width, and field widths matched to Pose's, so the two
  // rows (and the header) line up column-for-column.
  len = snprintf(temp, sizeof(temp), "| USB:  %5.0f %5.0f %5.0f %6.0f %5.0f %5.0f",
                 motion[0], motion[1], motion[2], motion[3], motion[4], motion[5]);
  memcpy(buffer[row++], temp, len);

  row++;  // blank

  // Raw Field header
  len = snprintf(temp, sizeof(temp), "| Field (mT):    Sensor1  Sensor2  Sensor3");
  memcpy(buffer[row++], temp, len);

  // Raw Field rows. raw_field is sensor-major (RAW_MAG1_X/Y/Z, then
  // RAW_MAG2_X/Y/Z, ...), so the axis is the inner index and the sensor
  // is the outer one: raw_field[sensor * 3 + axis].
  for (int i = 0; i < 3; ++i) {
    char axis[] = {'X', 'Y', 'Z'};
    len = snprintf(temp, sizeof(temp), "|   %c:         %7.3f  %7.3f  %7.3f",
                   axis[i],
                   raw_field[0 * 3 + i],
                   raw_field[1 * 3 + i],
                   raw_field[2 * 3 + i]);
    memcpy(buffer[row++], temp, len);
  }

  row++;  // blank

  // Residual header
  len = snprintf(temp, sizeof(temp), "| Residual (mT): Sensor1  Sensor2  Sensor3");
  memcpy(buffer[row++], temp, len);

  // Residual rows. Same sensor-major layout as raw_field (see solve_pose.cpp,
  // where residual.block<3,1>(sensor*3, 0) is filled per sensor).
  for (int i = 0; i < 3; ++i) {
    char axis[] = {'X', 'Y', 'Z'};
    len = snprintf(temp, sizeof(temp), "|   %c:         %7.3f  %7.3f  %7.3f",
                   axis[i],
                   stats.avg_residual(0 * 3 + i),
                   stats.avg_residual(1 * 3 + i),
                   stats.avg_residual(2 * 3 + i));
    memcpy(buffer[row++], temp, len);
  }

  row++;  // blank

  // Std Dev header
  len = snprintf(temp, sizeof(temp), "| Std Dev (mT):  Sensor1  Sensor2  Sensor3");
  memcpy(buffer[row++], temp, len);

  // Std Dev rows. Same sensor-major layout as raw_field.
  for (int i = 0; i < 3; ++i) {
    char axis[] = {'X', 'Y', 'Z'};
    len = snprintf(temp, sizeof(temp), "|   %c:         %7.3f  %7.3f  %7.3f",
                   axis[i],
                   residual_stddev(0 * 3 + i),
                   residual_stddev(1 * 3 + i),
                   residual_stddev(2 * 3 + i));
    memcpy(buffer[row++], temp, len);
  }

  row++;  // blank

  // Footer 1 (Residual & Condition Number)
  len = snprintf(temp, sizeof(temp), "| Res: %5.2f%% | Rcond: %5.0f",
                 residual_percent,
                 rcond);
  memcpy(buffer[row++], temp, len);

  // Footer 2 (Update Rate, Buttons, HID status)
  float update_rate = (1000 * kPrintEvery) / (millis() - last_update_ms);
  // stats.time_tot accumulates microseconds now, not milliseconds.
  float free_flow_rate = stats.time_tot ? (stats.n_time * 1e6f) / stats.time_tot : 0.0f;

  len = snprintf(temp, sizeof(temp), "| Rate: %4.1f / %4.1f Hz | Btn: 0x%02x | HID: %-3s",
                 update_rate,
                 free_flow_rate,
                 buttonBits & 0x0003,
                 hidReportSent ? "Yes" : "No");
  memcpy(buffer[row++], temp, len);

  // Send the entire contiguous block to serial. 
  Serial.print((char*)buffer);
  Serial.flush();
  last_update_ms = millis();
}
