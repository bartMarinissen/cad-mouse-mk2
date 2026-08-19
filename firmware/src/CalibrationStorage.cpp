#include "CalibrationStorage.h"

#include <LittleFS.h>
#include <string.h>

#include <type_traits>

#include "Config.h"

namespace CalibrationStorage {
namespace {

const char kPath[] = "/calibration.bin";
// Written here first, then renamed over kPath. lfs_rename replaces the
// destination atomically, so an interrupted write can only ever cost the
// half-written temp file, never the calibration already on the device.
const char kTmpPath[] = "/calibration.tmp";

const uint8_t kMagic[4] = {'C', 'M', 'K', '2'};

constexpr size_t kMagicOffset = 0;
constexpr size_t kVersionOffset = 4;
constexpr size_t kPayloadOffset = 8;
constexpr size_t kCrcOffset = kBlobSize - 4;

// This one IS load-bearing: the payload is a raw copy of the struct, so
// memcpy'ing into it is only defined behaviour while it stays trivially
// copyable. Keeping CalibrationParams plain arrays rather than matrix types is
// what buys that -- see the note in CalibrationParams.h.
static_assert(std::is_trivially_copyable<CalibrationParams>::value,
              "the stored format is a raw copy of CalibrationParams");

static_assert(sizeof(CalibrationParams) == kPayloadSize,
              "CalibrationParams is the payload; changing its size changes the "
              "format, which needs a kVersion bump");

static_assert(kPayloadOffset + kPayloadSize == kCrcOffset,
              "payload does not fill the space between the header and the CRC");

uint32_t readU32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Mount once per boot. resolveCalibration() runs twice (sensor and motion
// resolve independently, see Controllers.h), and the upload path mounts again
// later, so this has to be idempotent -- repeat LittleFS.begin() behaviour is
// not something worth depending on.
//
// A filesystem that will not mount gets formatted and retried rather than
// left broken: the device then boots on Config::defaultCalibration() for that
// cycle, but the store is writable again, so a later upload can fix it. The
// alternative -- refusing to mount -- would make a corrupt filesystem
// permanent.
bool mount() {
  static bool attempted = false;
  static bool mounted = false;
  if (attempted) {
    return mounted;
  }
  attempted = true;

  mounted = LittleFS.begin();
  if (!mounted) {
    Serial.println("CAL_FS mount failed, formatting");
    LittleFS.format();
    mounted = LittleFS.begin();
    if (!mounted) {
      Serial.println("CAL_FS format failed, storage unavailable");
    }
  }
  return mounted;
}

}  // namespace

const char* resultText(Result result) {
  switch (result) {
    case Result::Ok: return "ok";
    case Result::BadMagic: return "bad magic";
    case Result::BadVersion: return "unknown version";
    case Result::BadCrc: return "crc mismatch";
    case Result::NotFinite: return "non-finite float";
    case Result::Implausible: return "failed plausibility";
  }
  return "unknown";
}

uint32_t crc32(const uint8_t* data, size_t len) {
  // Bitwise rather than table-driven: this runs twice at boot over 312 bytes,
  // so the ~2.5k iterations cost nothing measurable and it saves a 1KB table
  // in flash.
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      const uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

Result deserialize(const uint8_t in[kBlobSize], CalibrationParams& out) {
  if (memcmp(in + kMagicOffset, kMagic, sizeof(kMagic)) != 0) {
    return Result::BadMagic;
  }
  if (readU32(in + kVersionOffset) != kVersion) {
    return Result::BadVersion;
  }
  if (crc32(in, kCrcOffset) != readU32(in + kCrcOffset)) {
    return Result::BadCrc;
  }

  // The payload is the struct, byte for byte. Which float means what is the
  // host's problem, decided once in format_binary(); this side only has to
  // agree on the length, which the static_asserts above pin down.
  memcpy(&out, in + kPayloadOffset, kPayloadSize);

  return isPlausible(out) ? Result::Ok : Result::Implausible;
}

bool isPlausible(const CalibrationParams& params) {
  // Finiteness first, over the whole struct at once, as a raw bit test
  // rather than a plain isfinite()-style check: this build sets
  // -ffinite-math-only project-wide (platformio.ini), which licenses GCC to
  // fold such a check into a compile-time constant, since the compiler
  // is allowed to assume the "not finite" branch is unreachable.
  // is_finite_bits() (math3D.h) reads the raw IEEE-754 bit pattern instead,
  // which isn't a floating-point operation that flag governs, so it can't be
  // folded away the same way. It is also what rejects erased flash, where
  // every byte reads 0xFF and every float comes out NaN.
  const float* values = &params.sensor_gain[0][0][0];
  for (size_t i = 0; i < kPayloadSize / sizeof(float); i++) {
    if (!is_finite_bits(values[i])) {
      return false;
    }
  }

  for (int i = 0; i < 3; i++) {
    // Magnitude only. A magnet installed with reversed polarity fits to a
    // negative Br, and that is correct rather than corrupt: MagnetModel
    // scales by magnet_strength_mT / BICUBIC_FIELD_REFERENCE_MT, so the
    // negative flips the modelled field to match. A sign test here would
    // reject a good calibration from a unit with a flipped magnet.
    const float strength = fabsf(params.magnet_strength_mT[i]);
    if (strength < 1e-3f || strength > 5000.0f) {
      return false;
    }

    // Same reasoning one level up: G_fw = inv(MAGNET_POLARITY * G_fit) folds
    // in polarity, so the determinant is legitimately either sign. What is
    // not legitimate is a singular gain (uninvertible, no usable correction)
    // or an absurd one. Fitted lands near 1 under the det(G)=1 gauge; the
    // default's scalar gains land near 0.885. Two orders either way clears
    // both without encoding either gauge.
    const float gainDet = fabsf(BLA::Determinant(toMat3(params.sensor_gain[i])));
    if (gainDet < 0.01f || gainDet > 100.0f) {
      return false;
    }

    for (int axis = 0; axis < 3; axis++) {
      // Real magnet spacing is ~28.58mm, so this only catches garbage that
      // happened to survive the CRC.
      if (fabsf(params.magnet_pos_knob[i][axis]) > 200.0f) {
        return false;
      }
      if (fabsf(params.sensor_offset_mT[i][axis]) > 1000.0f) {
        return false;
      }
    }
  }
  return true;
}

bool load(CalibrationParams& out) {
  if (!mount()) {
    return false;
  }

  File file = LittleFS.open(kPath, "r");
  if (!file) {
    // Not an error: this is every device that has never been calibrated. The
    // failures below are different, and stay unconditional.
    if (Config::ENABLE_TELEMETRY) {
      Serial.println("CAL_LOAD using default (no stored calibration)");
    }
    return false;
  }

  const size_t stored = file.size();
  uint8_t blob[kBlobSize];
  const size_t read = file.readBytes(reinterpret_cast<char*>(blob), kBlobSize);
  file.close();

  if (stored != kBlobSize || read != kBlobSize) {
    Serial.printf("CAL_LOAD using default (size %u, expected %u)\n",
                  static_cast<unsigned>(stored), static_cast<unsigned>(kBlobSize));
    return false;
  }

  const Result result = deserialize(blob, out);
  if (result != Result::Ok) {
    Serial.printf("CAL_LOAD using default (%s)\n", resultText(result));
    return false;
  }

  if (Config::ENABLE_TELEMETRY) {
    Serial.println("CAL_LOAD stored calibration");
  }
  return true;
}

bool saveBlob(const uint8_t* blob, size_t len) {
  if (len != kBlobSize) {
    return false;
  }

  // Validate before touching flash, through the same path the boot read uses.
  // A blob that would not survive the next boot never gets written.
  CalibrationParams probe;
  if (deserialize(blob, probe) != Result::Ok) {
    return false;
  }

  if (!mount()) {
    return false;
  }

  File file = LittleFS.open(kTmpPath, "w");
  if (!file) {
    return false;
  }
  const size_t written = file.write(blob, kBlobSize);
  file.close();

  if (written != kBlobSize) {
    LittleFS.remove(kTmpPath);
    return false;
  }

  if (!LittleFS.rename(kTmpPath, kPath)) {
    LittleFS.remove(kTmpPath);
    return false;
  }
  return true;
}

bool handleUploadCommand(const char* line) {
  static const char kPrefix[] = "CAL_UPLOAD ";
  const size_t prefixLen = sizeof(kPrefix) - 1;

  if (strncmp(line, kPrefix, prefixLen) != 0) {
    return false;
  }

  const char* hex = line + prefixLen;
  if (strlen(hex) != kHexChars) {
    Serial.println("CAL_UPLOAD_ERR LENGTH");
    return true;
  }

  uint8_t blob[kBlobSize];
  for (size_t i = 0; i < kBlobSize; i++) {
    const int high = hexNibble(hex[i * 2]);
    const int low = hexNibble(hex[i * 2 + 1]);
    if (high < 0 || low < 0) {
      Serial.println("CAL_UPLOAD_ERR HEX");
      return true;
    }
    blob[i] = static_cast<uint8_t>((high << 4) | low);
  }

  CalibrationParams probe;
  const Result result = deserialize(blob, probe);
  if (result != Result::Ok) {
    switch (result) {
      case Result::BadMagic: Serial.println("CAL_UPLOAD_ERR MAGIC"); break;
      case Result::BadVersion: Serial.println("CAL_UPLOAD_ERR VERSION"); break;
      case Result::BadCrc: Serial.println("CAL_UPLOAD_ERR CRC"); break;
      case Result::NotFinite:
      case Result::Implausible: Serial.println("CAL_UPLOAD_ERR IMPLAUSIBLE"); break;
      case Result::Ok: break;
    }
    return true;
  }

  if (!saveBlob(blob, kBlobSize)) {
    Serial.println("CAL_UPLOAD_ERR WRITE");
    return true;
  }

  // SensorController and MotionController hold their calibration const from
  // boot -- it lets the compiler fold the values in, and it stops anything
  // reassigning them by mistake. A reboot is therefore the only way an upload
  // takes effect; reloading in place would mean giving both of those up.
  Serial.println("CAL_UPLOAD_OK");
  Serial.println("STATUS REBOOTING");
  Serial.flush();
  delay(100);
  rp2040.reboot();
  return true;
}

}  // namespace CalibrationStorage
