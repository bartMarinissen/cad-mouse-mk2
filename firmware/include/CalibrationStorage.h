#pragma once

#include <Arduino.h>

#include "CalibrationParams.h"

// Persistence for one unit's fitted CalibrationParams, stored on LittleFS as
// /calibration.bin. This is the flash loader resolveCalibration()
// (Controllers.cpp) plugs into, and the write side the host tooling reaches
// over serial.
//
// VALIDATION IS ALL-OR-NOTHING, and that is not a style preference. Per
// CalibrationParams.h's gain convention, sensor_gain and magnet_strength_mT
// are the same degree of freedom under the fit's det(G)=1 gauge -- "do not
// take a sensor_gain from one calibration and a magnet_strength_mT from
// another". So a stored blob is either accepted whole or rejected whole and
// the caller falls back to Config::defaultCalibration(). Nothing here ever
// repairs, clamps, or defaults an individual field while keeping the rest:
// that would silently manufacture a mixed-gauge calibration, which is worse
// than either input it was stitched from.
//
// The same bytes travel three routes -- read at boot, flashed as a
// prebuilt filesystem image via `pio run -t uploadfs`, or pushed over serial
// -- and all three land in deserialize(), so they cannot drift apart.
namespace CalibrationStorage {

// --- On-disk / on-wire layout. Byte-for-byte identical on all three routes. ---
//
//   offset  size  field
//   0       4     magic, ASCII 'C' 'M' 'K' '2'
//   4       4     version, uint32 LE
//   8       300   payload: a CalibrationParams, verbatim
//   308     4     crc32, uint32 LE over bytes [0, 308)
//
// Little-endian throughout: native for both the RP2040 and any host running
// the Python tooling, so neither side ever byte-swaps.
//
// The payload is the struct's own bytes. CalibrationParams is plain arrays
// precisely so that it can be -- see the note there -- which leaves the host
// as the only place that has to know the field order, and leaves this side
// with a memcpy.
constexpr size_t kBlobSize = 312;
constexpr size_t kPayloadSize = 300;
constexpr size_t kHexChars = kBlobSize * 2;

// Carried so a future format change has somewhere to announce itself. There
// is deliberately no migration logic -- a blob whose version we do not know
// is rejected rather than guessed at, which is what keeps a v2 layout from
// being silently misread as v1.
constexpr uint32_t kVersion = 1;

// Why a blob was turned away. Boot diagnostics and the serial upload's error
// responses both need to say which check failed, not just that one did.
enum class Result {
  Ok,
  BadMagic,
  BadVersion,
  BadCrc,
  NotFinite,
  Implausible,
};

const char* resultText(Result result);

// Standard reflected IEEE 802.3 CRC-32 (poly 0xEDB88320, init/final
// 0xFFFFFFFF) -- bit-identical to Python's zlib.crc32, which is the whole
// point: the host tooling computes the stored checksum, so the two
// implementations have to agree exactly. Written out here rather than taken
// from the core so there is one definition to reason about.
uint32_t crc32(const uint8_t* data, size_t len);

Result deserialize(const uint8_t in[kBlobSize], CalibrationParams& out);

// Range checks applied to an already-CRC-clean blob. These are corruption
// tripwires, not quality gates: bounds are orders of magnitude loose so a
// badly-fitted-but-real calibration still loads, and so both gauges pass --
// the fit's det(G)=1 one and Config::defaultCalibration()'s
// gain-carries-scale one.
bool isPlausible(const CalibrationParams& params);

// Mount (formatting if the filesystem is unreadable), read /calibration.bin,
// validate. False means "use the default", and the reason is printed.
bool load(CalibrationParams& out);

// Validate first, write only if it passes, and write via a temp file plus
// rename so a power loss mid-write leaves the previous good calibration
// intact.
bool saveBlob(const uint8_t* blob, size_t len);

// Handle a "CAL_UPLOAD <hex>" line. Returns true if the line was a
// CAL_UPLOAD command -- i.e. consumed -- whether or not it was accepted, so
// the caller can tell "handled" from "not mine". On success this reboots and
// does not return: SensorController and MotionController hold their
// calibration const from boot by design, so a fresh upload cannot take
// effect in place.
bool handleUploadCommand(const char* line);

}  // namespace CalibrationStorage
