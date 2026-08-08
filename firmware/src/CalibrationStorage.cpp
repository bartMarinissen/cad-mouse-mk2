#include "CalibrationStorage.h"

#include <LittleFS.h>
#include <string.h>

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

// Not load-bearing: the wire format is built from coefficient accessors, so
// it stays correct whatever Eigen does with padding. This only catches the
// struct quietly changing shape. If it ever fires, update the number -- do
// not change the format to match.
static_assert(sizeof(CalibrationParams) == 300,
              "CalibrationParams changed size; the wire format is unaffected "
              "but the change was probably not intentional");

static_assert(kPayloadOffset + kPayloadFloats * 4 == kCrcOffset,
              "payload does not fill the space between the header and the CRC");

uint32_t readU32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

void writeU32(uint8_t* p, uint32_t value) {
  p[0] = static_cast<uint8_t>(value);
  p[1] = static_cast<uint8_t>(value >> 8);
  p[2] = static_cast<uint8_t>(value >> 16);
  p[3] = static_cast<uint8_t>(value >> 24);
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

void serialize(const CalibrationParams& in, uint8_t out[kBlobSize]) {
  memcpy(out + kMagicOffset, kMagic, sizeof(kMagic));
  writeU32(out + kVersionOffset, kVersion);

  float payload[kPayloadFloats];
  size_t k = 0;

  // Row-major, matching numpy's default flatten. Keep this order in step with
  // deserialize() and with format_binary() in
  // magnet_field_model/calibration/export.py.
  for (int i = 0; i < 3; i++) {
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        payload[k++] = in.sensor_gain[i](r, c);
      }
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int r = 0; r < 3; r++) {
      payload[k++] = in.sensor_offset_mT[i](r);
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int r = 0; r < 3; r++) {
      payload[k++] = in.magnet_pos_knob[i](r);
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        payload[k++] = in.magnet_rotation[i](r, c);
      }
    }
  }
  for (int i = 0; i < 3; i++) {
    payload[k++] = in.magnet_strength_mT[i];
  }

  for (size_t i = 0; i < kPayloadFloats; i++) {
    uint32_t bits;
    memcpy(&bits, &payload[i], sizeof(bits));
    writeU32(out + kPayloadOffset + i * 4, bits);
  }

  writeU32(out + kCrcOffset, crc32(out, kCrcOffset));
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

  float payload[kPayloadFloats];
  for (size_t i = 0; i < kPayloadFloats; i++) {
    const uint32_t bits = readU32(in + kPayloadOffset + i * 4);
    // Tested on the bit pattern, not with isfinite(): the build enables
    // several -ffast-math-adjacent flags, and an exponent of all ones is
    // NaN-or-Inf regardless of what the optimizer believes about float
    // comparisons. Also the cheapest way to reject erased flash, where every
    // byte reads 0xFF.
    if ((bits & 0x7F800000u) == 0x7F800000u) {
      return Result::NotFinite;
    }
    memcpy(&payload[i], &bits, sizeof(float));
  }

  size_t k = 0;
  for (int i = 0; i < 3; i++) {
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        out.sensor_gain[i](r, c) = payload[k++];
      }
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int r = 0; r < 3; r++) {
      out.sensor_offset_mT[i](r) = payload[k++];
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int r = 0; r < 3; r++) {
      out.magnet_pos_knob[i](r) = payload[k++];
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        out.magnet_rotation[i](r, c) = payload[k++];
      }
    }
  }
  for (int i = 0; i < 3; i++) {
    out.magnet_strength_mT[i] = payload[k++];
  }

  return isPlausible(out) ? Result::Ok : Result::Implausible;
}

bool isPlausible(const CalibrationParams& params) {
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
    const float gainDet = fabsf(params.sensor_gain[i].determinant());
    if (gainDet < 0.01f || gainDet > 100.0f) {
      return false;
    }

    for (int axis = 0; axis < 3; axis++) {
      // Real magnet spacing is ~28.58mm, so this only catches garbage that
      // happened to survive the CRC.
      if (fabsf(params.magnet_pos_knob[i](axis)) > 200.0f) {
        return false;
      }
      if (fabsf(params.sensor_offset_mT[i](axis)) > 1000.0f) {
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
    Serial.println("CAL_LOAD using default (no stored calibration)");
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

  Serial.println("CAL_LOAD stored calibration");
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
  // boot, deliberately, "so nothing can drift onto a different calibration
  // mid-flight" (CalibrationParams.h). A reboot is therefore the only way the
  // upload takes effect -- reloading in place would mean unpicking that.
  Serial.println("CAL_UPLOAD_OK");
  Serial.println("STATUS REBOOTING");
  Serial.flush();
  delay(100);
  rp2040.reboot();
  return true;
}

}  // namespace CalibrationStorage
