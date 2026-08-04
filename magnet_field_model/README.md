# magnet_field_model

Offline Python for the magnetic side of the project: the field simulation that
generates the firmware's interpolation table, and the **bundle calibration**
that measures a specific unit's real geometry.

```
field_approximation.ipynb   magpylib simulation -> firmware bicubic table (manual codegen)
bundle_callibration.py      entry point: capture a run, fit it, report
calibration/                the calibration package (see below)
calibration_runs/           raw captured frames, as JSON
tests/                      pytest suite - run this after touching any math
```

## Running it

```bash
uv sync
uv run python bundle_callibration.py                       # live capture over serial
uv run python bundle_callibration.py --replay calibration_runs/raw_*.json
uv run python bundle_callibration.py --replay <run> --emit-cpp   # + firmware constants
uv run --group dev python -m pytest tests/ -q
```

A fit over a captured run takes a few seconds.

## What bundle calibration is for

The knob has 3 magnets over 3 Hall sensors. The firmware infers pose by
inverting a magnetic forward model that assumes *nominal* geometry. Real
hardware misses nominal: a magnet sitting 0.2mm off, or a sensor 5% more
sensitive, looks to the solver like the knob moved. That is the "Phantom
Tilt" problem this fork exists to fix (see `../ARCHITECTURE.md`).

Calibration recovers, per unit:

| Group | Count | Meaning |
|---|---|---|
| `magnet_pos` | 3x3 | knob-frame magnet position offset (mm) |
| `magnet_tilt` | 3x2 | magnet axis tilt (rad) |
| `gain_iso` | 3x1 | isotropic sensor gain error |
| `gain_aniso` | 3x2 | per-axis sensitivity spread |
| `gain_sym` | 3x3 | cross-axis skew |
| `gain_rot` | 3x3 | sensor frame misalignment |

42 shared parameters, plus one free 6-DOF pose per captured frame — a
bundle adjustment, in the photogrammetry sense.

## Package layout

```
local_field.py        one magnet's axisymmetric field + its 3x3 gradient
bundle_geometry.py    nominal geometry, the parameter vector, forward model + analytic Jacobian
bundle_params.py      priors, residual weighting, solve stages, Schur-complement covariance
pose_solver.py        batched Levenberg-Marquardt: every frame's pose at once
frame_selection.py    farthest-point subset selection over pose space
calibration_algorithm.py  the staged fit that drives all of the above
export.py             fitted geometry -> firmware constants
protocol.py / serial_link.py / session.py / collector.py / tui.py   capture side
```

## Design notes

**Analytic Jacobian.** Every derivative in `bundle_geometry.py` is the same
closed-form chain rule the firmware uses in
`firmware/src/magnet_model/sensor.cpp`, layered on the local field gradient
from `local_field.py`. The previous version finite-differenced the entire
shared vector per frame — tens of thousands of magpylib calls per solver
iteration. `tests/test_jacobian.py` checks every derivative against central
differences, mirroring `firmware/test/test_jacobian.cpp`.

**Weighting.** Sensors span ~9-70 mT over a run, and the error budget is part
absolute (~0.1 mT read noise, measured on a stationary knob) and part
relative (~2% unmodelled field-shape error). A flat sigma therefore lets the
close-to-the-magnet frames dominate. `ResidualWeights` uses
`hypot(absolute, relative * |B|)` per sensor per frame, keyed off the
**measured** magnitude — a weight that depended on the prediction would bias
the fit toward shrinking |B|.

**Staging.** Parameters are freed a group at a time (gain scale → magnet
geometry → the weak cross-axis gain terms), each stage warm-starting from the
last, rather than throwing all 42 at a cold start.

**Reported uncertainty.** Each frame's 6 pose parameters touch only that
frame's 9 residuals, so the pose block of the normal equations is
block-diagonal and can be eliminated frame by frame (the Schur complement, in
`bundle_params.covariance_shared`). That yields a posterior sd per parameter,
reported as *information gain* = `1 - posterior/prior`. Low values are not a
bug — they mark parameters the data barely constrained, which is the
difference between measured and assumed.

## Assumptions

Recorded explicitly, because several are load-bearing:

1. **Magnet position reference is the BOTTOM FACE, not the centre.**
   `field_approximation.ipynb` builds the firmware's table with the cylinder at
   `z=+3` so its bottom face lands on `z=0`, so that is what `positions.h`'s
   magnet positions mean. magpylib positions a cylinder by its centre, so
   `local_field.py` adds the half-height. Getting this wrong shifts the model
   3mm and inflates the predicted field roughly 3x at rest.
2. **Nominal sensor gain is `-I`.** The capture path streams
   `readUncorrected()`, and the raw sensors read the opposite sign to the
   modelled field. The firmware carries this as `Config::magnet_gains`
   (`{-0.96, -1.2, -0.98}`). Whether the physical cause is magnet orientation
   or sensor axis convention does not matter to the fit — a global sign is
   equivalent either way — but the *nominal* value must be reachable, or the
   priors fight the data.
3. **Sensor positions are fixed, not calibrated.** They define the world
   frame. Real sensor placement error is absorbed by the magnet position
   offsets, which are related to it by a per-frame pose anyway.
4. **Per-magnet strength is not a free parameter.** It is very nearly
   degenerate with per-sensor gain: each sensor is dominated by its own
   magnet, and the cross-talk that would separate them is only about 1% of
   the signal, comparable to the model error. The firmware has no per-magnet
   strength multiplier either. Sensor gain owns scale.
5. **Magnet spin about its own axis is not a parameter.** A uniformly
   axially-polarized cylinder is a solid of revolution, so that rotation is
   unobservable for *any* dataset — a structurally dead direction rather than
   a poorly-measured one. Tilt carries 2 DOF, not 3.
6. **Gauge fixing is done softly, by the priors.** A global translation or
   rotation of all magnets is exactly degenerate with a compensating
   per-frame pose change (6 dead directions). Ridge priors on the magnet
   offsets anchor them. An explicit mean-zero constraint would be tidier.
7. **Gain is fitted on the model side, exported inverted.** See
   `export.py` — the firmware applies gain to the raw measurement, this fit
   applies it to the model, so the exported matrix is the inverse.

## Measured behaviour

Against the three captured runs in `calibration_runs/` (same hardware):

- Field residual: **1.6-1.9% at nominal geometry → 0.34-0.37% fitted**.
- Runtime: **~1.3s** at the default 60 frames, all three stages converging on
  `ftol`.
- Cross-run agreement: magnet positions reproduce to ~0.01mm, and every
  parameter group's run-to-run spread is well inside its own prior.
- Frame count barely matters: 30, 60, 120, 200 and all 387 frames land within
  0.03 percentage points of each other. The limit is systematic model error,
  not sample noise — so capturing more frames buys nothing, and the value is
  in pose *diversity*, which is what `frame_selection.py` selects for.

Note that reproducibility is not the same as identifiability: the weakly
determined directions (magnet z-offset against gain scale, in particular)
reproduce consistently because the *prior* resolves them the same way every
time, not because the data measured them. The information-gain column is
what distinguishes the two.

## Still open

- No persistence path into the firmware. There is no flash/EEPROM storage
  anywhere in this firmware yet, so `--emit-cpp` printing a pasteable snippet
  is as far as a result can travel.
- Running the fit on the knob itself. The reduction that makes this plausible
  is already here — analytic Jacobians and a per-frame Schur elimination with
  a fixed-size accumulator — and the firmware already has the same chain rule
  in `sensor.cpp`. What it would need is that reduction driving the solve
  rather than only the covariance, plus the persistence layer above.
- The `magnet_pos` z-offsets sit at ~2% information gain. Separating them
  from gain scale needs more Z travel than the current HEAVE step provides.
