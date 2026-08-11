# `firmware/data/` — LittleFS image source

PlatformIO's `data_dir` (see `platformio.ini`). Everything in here gets built
into the LittleFS image that `pio run -t uploadfs` flashes onto the device.

The directory is tracked but its one real payload, `calibration.bin`, is not:
it is fitted per physical unit, so committing one would ship a single device's
calibration to every checkout. `.gitignore` in this directory keeps it out.

## Flashing a calibration this way

```bash
cd magnet_field_model
uv run python bundle_callibration.py --replay calibration_runs/raw_<run>.json --emit-bin
cd ..
pio run -t uploadfs
```

`--emit-bin` defaults to writing `firmware/data/calibration.bin`. It is
byte-for-byte the same blob the serial upload path sends, so both routes
produce a file the firmware reads identically — see
`firmware/include/CalibrationStorage.h` for the size and layout, which are
declared there (`kBlobSize`/`kPayloadSize`) rather than restated here.

## The other route

`uploadfs` rewrites the whole filesystem and needs a rebuild-and-flash cycle.
To push a calibration to a device that is already running, use
`bundle_callibration.py --write-serial` instead: trigger a tare on the knob
(hold both buttons ~3s), and the script sends the same bytes over the wire
when the device announces `STATUS TARE_BEGIN`. The device validates, stores,
and reboots to pick it up.
