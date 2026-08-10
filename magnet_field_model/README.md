# magnet_field_model

Offline Python for the magnetic side of the project: the field simulation that
generates the firmware's interpolation table, and the **bundle calibration**
that measures a specific unit's real geometry.

```
bicubic_table.py             the magnet model + grid the firmware's table is built from
generate_bicubic_table.py    entry point: (re)writes the firmware's table from bicubic_table.py
field_approximation.ipynb    inspection/visualization of that table - does not write it
bundle_callibration.py       entry point: capture a run, fit it, report
calibration/                 the calibration package (see below)
calibration_runs/            raw captured frames, as JSON
tests/                       pytest suite, one file per module - run after touching any math
```

## Running it

```bash
uv sync
uv run python generate_bicubic_table.py                    # (re)write the firmware's field table
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
| `magnet_pos` | 3 | in-plane shape of the magnet triangle (mm) |
| `magnet_tilt` | 3x2 | magnet axis tilt (rad) |
| `magnet_strength_mean` | 1 | common-mode polarization multiplier |
| `magnet_strength_diff` | 2 | how much individual magnets differ from it |
| `sensor_offset` | 3x3 | DC offset on the raw reading (mT) |
| `gain_aniso` | 3x2 | per-axis sensitivity spread |
| `gain_sym` | 3x3 | cross-axis skew |
| `gain_rot` | 3x3 | sensor frame misalignment |

45 shared parameters, plus one free 6-DOF pose per captured frame — a
bundle adjustment, in the photogrammetry sense. There is deliberately no
isotropic/scale gain parameter; see the gauge note below for why sensor gain
and magnet strength don't compete for the same degree of freedom.

## Package layout

```
local_field.py         one magnet's axisymmetric field + its 3x3 gradient
nominal_geometry.py    pure nominal/CAD constants (no other package dependencies)
parameterization.py    the calibration parameter layout: how the flat vector scipy
                        sees maps to physical offsets (ParamBlock, GROUP_SLICES, assemble())
bundle_geometry.py     BundleGeometry: physical geometry + forward model + analytic Jacobian
bundle_params.py       priors, residual weighting, BundleCalibrationProblem, covariance
pose_solver.py         batched Levenberg-Marquardt: every frame's pose at once
calibration_algorithm.py  the fit that drives all of the above (single joint solve)
export.py              fitted geometry -> firmware constants
report.py              fitted result -> the Rich content the TUI and CLI show
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

**Single joint solve, not staging.** Every one of the 45 shared parameters is
freed at once, in one `least_squares` call. An earlier version froze
parameters in a warm-started ladder (gain scale and DC offset → magnet
geometry → the weak cross-axis gain terms) on the theory that a cold start
needed the help. Ablated against real and synthetic data — checking recovered
parameters, not just residual level, since a bad local optimum can still
converge to a low residual — a single unstaged solve reaches the same optimum
every time. `SolveStage`/`BundleCalibrationProblem.build_problem` still exist,
purely as a way to isolate a parameter subset for testing (e.g. "fit only
magnet strength, holding everything else at nominal").

**No isotropic gain term, ever.** Sensor gain scale and magnet strength
describe the same thing from opposite ends: a sensor reading 5% high and its
magnet being 5% strong differ only through cross-talk, which is ~1% of the
signal here and so comparable to the model error. An earlier version fixed
this with a runtime pass — freeing an isotropic `gain_iso` parameter and then
transferring it into strength via a final `det(G)=1` renormalization stage.
That machinery is gone: `GAIN_BASIS` (`parameterization.py`) is now built from
only the 8 basis matrices that are exactly traceless (asserted at import
time), so `det(I + A) = 1 - tr(A²)/2 + det(A) ≈ 1` for *any* parameter value,
automatically, to second order — good to the same precision the old runtime
transfer already tolerated. There is no competing scale parameter for
`magnet_strength` to wait behind, so it is free from the start of the (single)
solve, not a final phase.

**Pose solving.** Every frame's pose is solved in one batched
Levenberg-Marquardt rather than a Python loop, since the frames are
independent. A cold start occasionally drops a single frame into the wrong
basin — near a magnet and well tilted, the residual surface has a second
minimum a few degrees away — so any frame ending far worse than its peers is
retried from a spread of starting orientations. `converged` reflects the final
residual, not the damping factor: `calibration_algorithm.py` only fits frames
that passed this gate (evenly decimated down to a runtime cap — see
`DEFAULT_N_FRAMES` — not selected for diversity; a farthest-point frame
selector used to run here, but frame choice barely affects the result, so it
was deleted rather than kept for a benefit that wasn't there).

**Reported uncertainty.** Each frame's 6 pose parameters touch only that
frame's 9 residuals, so the pose block of the normal equations is
block-diagonal, and its contribution to the shared-parameter covariance can be
eliminated (the Schur complement). `BundleCalibrationProblem.covariance_shared`
computes this as `inv(J.T @ J)[:p, :p]` — a plain dense inverse sliced to the
shared block, reusing the same Jacobian the solver itself trusts, mathematically
identical to a hand-rolled per-frame elimination but far simpler at this
problem's actual size (a few hundred to a couple thousand unknowns, well under
a second). Reported as *information gain* = `1 - posterior/prior`. Low values
are not a bug — they mark parameters the data barely constrained, which is the
difference between measured and assumed.

## Assumptions

Recorded explicitly, because several are load-bearing:

1. **Magnet position reference is the BOTTOM FACE, not the centre.**
   `bicubic_table.py`'s `build_magnet()` places the cylinder at `z=+3` so its
   bottom face lands on `z=0`, so that is what `positions.h`'s magnet
   positions mean. magpylib positions a cylinder by its centre, so
   `local_field.py` adds the half-height. Getting this wrong shifts the model
   3mm and inflates the predicted field roughly 3x at rest.
2. **The raw sensors' sign flip is attributed to magnet polarity, not gain.**
   The capture path streams `readUncorrected()`, and the raw sensors read the
   opposite sign to the field `local_field.py` models. The firmware carries
   that in `Config::magnet_gains` (`{-0.96, -1.2, -0.98}`); here
   `MAGNET_POLARITY = -1` carries it instead, so `local_field.py` stays a
   direct counterpart of the firmware's table and both fitted quantities read
   naturally (gain near `+I`, strength near `1`). The exported firmware gain
   still lands near `-I`. Which physical cause is the real one — magnets
   installed north-down, or an inverted sensor axis convention — does not
   matter to the fit, since a global sign is equivalent either way.
3. **Sensor positions are fixed, not calibrated.** They define the world
   frame. Real sensor placement error is absorbed by the magnet position
   offsets, which are related to it by a per-frame pose anyway.
4. **Magnet strength is split into common mode and differential, and the
   common mode carries no prior at all.** A prior would have to be centred on
   `local_field.py`'s 600mT polarization, which is a round guess rather than a
   measurement of these magnets — asserting a belief nobody holds, and
   dragging the fitted field scale toward an arbitrary number. So it is left
   free and reported with its own posterior sd.

   The result is worth knowing: **the data does not determine the absolute
   field scale.** Unregularized, the fit settles around 1.29–1.39 across the
   three runs — but with a posterior sd of ±0.43, i.e. under 1σ from nominal,
   and the residual only improves from 0.315% to 0.310% across that entire
   39% swing. The direction is very nearly flat, because a uniform scale
   change is largely absorbable by every frame's pose moving further away, and
   only the shape of |B| versus distance breaks it — over ~3mm of heave, only
   barely. Read the fitted strength as "unconstrained", not as "the magnets
   are 39% stronger than nominal". Dropping the prior did not reveal a better
   value; it revealed that there was never one to reveal.

   The *differential* part keeps its prior (1%), because that justification is
   independent of the 600mT figure: it says magnets cut from one batch are
   graded to about that of each other, which is a real belief about the parts.
   Empirically the constraint is free — sweeping it from 0.2 down to 0.0002
   moves the residual by 0.001 percentage points, so per-magnet differences
   were never explaining anything. The apparent per-sensor spread is accounted
   for by magnet position and DC offset instead.

5. **DC offset is applied on the raw side, after gain.** It is a property of
   the raw reading (Hall zero-point plus ambient field), not of the modelled
   field, so `pred = G @ B_model + offset`. The firmware subtracts its offset
   *after* gain instead, so the export maps `o_fw = G_fw @ o_fit`.
6. **Magnet spin about its own axis is not a parameter.** A uniformly
   axially-polarized cylinder is a solid of revolution, so that rotation is
   unobservable for *any* dataset — a structurally dead direction rather than
   a poorly-measured one. Tilt carries 2 DOF, not 3.
7. **The pose gauge is fixed explicitly, not by priors.** A global
   translation or rotation of the magnet trio is exactly cancelled by a
   compensating per-frame pose change — `mⱼ → Q mⱼ` with `R_n → R_n Qᵀ`
   leaves `R_n Qᵀ Q mⱼ = R_n mⱼ` untouched — so 6 of the 9 raw magnet
   coordinates carry no information whatsoever. Measured: 6 eigenvalues at
   machine zero in the reduced Hessian, identically at 60, 120 and 387
   frames. No dataset fixes this; each new frame brings 9 equations but also
   6 new pose unknowns.

   `MAGNET_POS_BASIS` removes them by construction, via three constraints:
   zero mean offset (kills translation), all z offsets equal (makes the
   triangle's plane horizontal, killing pitch and roll), and zero net yaw
   moment (kills twist, symmetrically rather than by pinning an edge). The
   flat constraint costs nothing — three points are always coplanar, so any
   arrangement can be rotated flat and the plane's tilt is pure gauge.

   What survives is 3 parameters: the triangle's in-plane shape, i.e. its
   three side lengths, which is the only part that can cause Phantom Tilt.
   Magnet tilts keep all 6 DOF, measured against the frame the flat triangle
   defines. (The rotation gauge is *shared* between positions and tilts, so
   it can be spent on either — fixing roll/pitch via mean tilt instead would
   give 5 position + 4 tilt. Total observable is 9 either way; only the
   presentation differs.)

   Two things this bought. Conditioning: the reduced Hessian's condition
   number drops from 2e305 (numerically singular) to 2e6. And frame count
   starts to matter — with all priors off, cross-run spread now improves
   2.1x going from 60 to 387 frames, against the √6.45 = 2.5 that pure
   averaging predicts, where before the gauge fix it was flat. Null
   directions cannot be out-voted by data; merely weak ones can.

8. **Gain is fitted on the model side, exported inverted.** See
   `export.py` — the firmware applies gain to the raw measurement, this fit
   applies it to the model, so the exported matrix is the inverse (with
   `MAGNET_POLARITY` folded back in, landing near `-I`).

## Measured behaviour

Against the three captured runs in `calibration_runs/` (same hardware):

- Field residual: **1.6-1.9% at nominal geometry → 0.26-0.34% fitted**, in a
  single unstaged solve.
- Runtime: well under a second at the default 60 frames. Solving all ~387
  captured frames unstaged instead costs 40+ seconds (more `least_squares`
  outer iterations at ~6x the problem dimension) for no measurable accuracy
  gain — see the next point — which is the actual reason `DEFAULT_N_FRAMES`
  still exists.
- Cross-run agreement: magnet positions reproduce to ~0.007mm, and every
  parameter group's run-to-run spread is a fraction of its posterior sd.
- Sensor DC offsets are real and worth fitting: adding them took the
  nominal-geometry residual from 1.53% to 0.97% before any other parameter is
  freed, and the final fitted result from 0.37% to 0.33%. They are also the
  best-determined group in the whole fit (~65% information gain), which is
  what you would expect of a constant that does not move with pose. The fitted
  values reach ~1 mT, mostly on z.
- Frame count barely matters: 30, 60, 120, 200 and all 387 frames land within
  0.03 percentage points of each other. The limit is systematic model error,
  not sample noise, and *which* frames are used barely matters either — an
  earlier version picked a diverse subset by farthest-point sampling over pose
  space; that machinery was deleted once ablation showed it wasn't earning its
  complexity. Frames are now just evenly decimated down to the runtime cap.

Note that reproducibility is not the same as identifiability: the weakly
determined directions (magnet z-offset against gain scale, in particular)
reproduce consistently because the *prior* resolves them the same way every
time, not because the data measured them. The information-gain column is
what distinguishes the two.

## Still open

- The `/calibration.bin` layout is no longer duplicated:
  `calibration/firmware_struct.py` extracts the `CalibrationParams`
  declaration straight out of `firmware/include/CalibrationParams.h` and hands
  it to cffi, so `format_binary()` fills the struct by field *name* and the
  order is only ever stated in the header. `tests/test_export.py` pins the
  resulting offsets and compiles the same extracted declaration with g++ to
  check cffi's ABI model against a real compiler. Both still only describe the
  *host* — the firmware's own `static_assert`s are what pin the target, and
  nothing here has run on hardware yet.
- Running the fit on the knob itself. The analytic Jacobian this needs is
  already here, and the firmware has the same chain rule in `sensor.cpp`. An
  on-device solve would need an O(n_frames) per-frame Schur elimination
  driving the actual solve (not just the reporting-only covariance
  `covariance_shared` computes today via a plain dense inverse, which is
  simpler at this problem's desktop-scale size but not embedded-friendly) —
  see git history for a prior implementation of that reduction. The storage
  half of that is done: a result fitted on the knob would have somewhere to
  land, via `CalibrationStorage`.
- Magnet strength stays weakly determined (~7% information gain). Scaling a
  magnet and moving it closer both scale |B|; only the shape of |B| versus
  distance separates them, and HEAVE supplies ~3mm of travel to do it with.
  More Z range would help, but the mechanism limits how much is available
  before the magnet leaves the modelled region.
- The `magnet_pos` prior is now optional rather than load-bearing. With the
  gauge fixed explicitly, dropping it entirely costs almost nothing: cross-run
  spread goes from 0.0041mm to 0.0053mm and the residual is unchanged. It is
  kept at 0.3mm because the knob is 3D printed and a few tenths of a
  millimetre is a well-founded statement about that process - unlike the 600mT
  figure. It is now a belief the fit could do without, which is the point of
  fixing the gauge properly, but it earns its place.
- The absolute field scale is not measurable from this capture (±0.43 on a
  multiplier of 1). If it is worth knowing — and it would tighten the z
  sensitivity of the whole pose solve — it needs either much more heave travel
  or an independent measurement of one magnet's remanence. Note the boot tare
  cancels a systematic z bias, so the practical cost of getting it wrong is
  mostly a slightly mis-scaled z axis, not an offset.
