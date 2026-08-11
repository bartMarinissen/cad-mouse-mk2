# Design how a user actually enters bundle-calibration mode from the knob

Split out from `TODO/tare-and-calibration.md` — that file is about the
boot-time tare step; this one is specifically about the UX for entering the
guided PC-assisted bundle calibration (`BundleState` /
`BundleCalibrationController`). Needs real UX thinking, not just a bug fix.

## Current state (as of this writing)

The practical experience is that one starts the bundle-callibration PC side.
Then the next time you enter tare mode it goes into bundle-callibration.

The underlying mechanism is awkward. It relies on BUNDLE_START still being in
the serial input buffer when the tare starts. But it works. We might just stick
with this as a permanent temporary solution. At least until we might want to write
anything else to the knob over serial.

## Potential design

We might, if we want to make this nicer, issue a TARE_START from the knob. So
that the callibration software can use that to better time a CAL_START in tare mode.
