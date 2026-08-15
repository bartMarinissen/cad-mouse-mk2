# Model cross-magnet interference

**Measured: 1.7–4.5% of the field, growing with knob-to-sensor distance.**
The "quantify it first" step below has been done, against all three captured
runs in `magnet_field_model/calibration_runs/`. The Python fit already models
all three magnets per sensor, so the term is directly extractable by comparing
its full prediction against just the paired-magnet contribution:

| \|t\| (mm) | cross-magnet share |
|---|---|
| 18.73 | 1.82% |
| 19.95 | 2.93% |
| 21.12 | 4.03% |

It grows with distance because the paired magnet's own field falls off fast
while the other two, ~28.58mm away, barely change — so their relative share
rises as the knob lifts.

**Currently worked around, not fixed.** Feeding parameters fitted under the
physically complete model into the firmware's single-magnet model leaves
exactly this term uncompensated — measured at **3.6%** field error on captured
data, and observed on hardware as a pose-residual regression from ~1% to ~3%
after applying a calibration. The workaround is
`calibration/bundle_geometry.py`'s `SENSOR_MAGNET_COUPLING`, set to
`PAIRED_ONLY` so the fit matches the firmware exactly; that brings the same
comparison to **0.49%**.

Two things worth knowing before picking this up:

- The workaround costs **absolute field scale**. Cross-talk is what separates
  magnet *strength* from magnet *distance* (a strength change scales the near
  and far contributions equally; a z-shift changes them at very different
  rates). Without it the two are near-degenerate over the ~3mm of heave the
  hardware gives, and `magnet_strength_mean`'s posterior sd roughly triples.
  The fit reports this honestly rather than hiding it — see
  `test_absolute_strength_needs_cross_magnet_coupling`.
- It does **not** cost fit quality. The single-magnet fit reaches 0.336%
  residual against 0.339% for the complete model, so at these pose ranges
  cross-talk is almost entirely absorbable into effective gain/offset/strength.
  What is lost is the physical meaning of the fitted numbers, not the fit.

So the case for doing this work is now "recover physically meaningful
parameters and absolute field scale", not "reduce pose residual" — the
workaround already handles the latter. Implementing it is a one-constant
switch back to `ALL_MAGNETS` on the Python side, plus the firmware work below.

## The problem

`ForwardModel::evaluate()` (`firmware/src/magnet_model/forward_model.cpp`)
pairs each sensor with exactly one magnet — `sensors_[i]` is only ever
evaluated against `magnets_[i]` — via `MagnetModel`'s near-field bicubic
lookup (`BicubicField`, built from a `magpylib` simulation of a single
isolated 6x6mm cylinder magnet, valid over the grid's `(r, z)` domain relative
to *that* magnet). The field each sensor actually sees from the *other* two
magnets — real physical magnets ~28.58mm away (the knob's triangle side
length, `Positions::triangle_sidelength_mm`) — is entirely unmodeled. That's
the cross-magnet interference this TODO is about: not a bug, a deliberately
missing higher-order term.

## Proposed approach, if it turns out to matter

Keep the implementation cost low by not extending the expensive part (the
bicubic near-field table) to cover cross terms. Instead:

- Add a dipole model to the magnet class (`MagnetModel`,
  `firmware/{include,src}/magnet_model/magnet_local_model.*`) — the standard
  closed-form ideal magnetic dipole field, valid in the far field where these
  cross terms live (28mm separation is comfortably far-field relative to a
  6mm magnet).
- Use that direct dipole formula (and its closed-form Jacobian — the solver
  needs analytic Jacobians everywhere, same as every other `evaluate()` in
  this chain, see `firmware/test/test_jacobian.cpp`) to estimate each sensor's
  field contribution *from the two non-paired magnets*, and add it as a
  correction on top of the existing near-field bicubic evaluation for the
  paired magnet.
- This keeps the added cost cheap on purpose: a closed-form dipole evaluation
  is far cheaper than a bicubic interpolation, so adding ~2 extra
  dipole evaluations per sensor per solver iteration should be a minor
  addition next to the existing 3 bicubic lookups, not a multiplier on the
  expensive part of the pipeline.

## Effect on on-device bundle calibration

`on-device-calibration.md` depends on the sparsity structure of the
calibration Hessian, so it is worth recording what switching to
`ALL_MAGNETS` does to it: less than expected.

- **The per-frame arrowhead is unaffected.** That structure needs only that
  one frame's residual depends on the shared parameters and its own pose.
  Cross-magnet coupling acts within a frame and never links two frames.
- **The Hessian's own block structure survives**, because what keeps the
  per-sensor blocks separate is that `sensor_offset` and `gain` belong to a
  physical sensor — no field cross-talk makes sensor *j*'s reading depend on
  sensor *i*'s gain. Only `magnet_tilt` moves from the per-sensor blocks into
  the shared border, taking the available sparsity win from ~5.6x to ~3.8x.

That is already accounted for: the on-device column ordering deliberately
places `magnet_tilt` in the border now, so the block geometry is invariant
across this switch and the accumulator will not need restructuring when it
happens.

## Open questions

- Where the dipole moment magnitude comes from, and whether it needs to match
  (or be separately calibrated from) whatever moment the near-field
  `magpylib` simulation used — inconsistency between the two would show up as
  a discontinuity/bias right at the boundary where the near-field model
  hands off to the dipole approximation.
- Whether per-magnet moment strength here should tie into the same
  per-magnet strength correction bundle calibration is meant to produce (the
  "Phantom Tilt" problem in `design documentation/context.md`) rather than
  being a separate constant.
- Whether this belongs on `MagnetModel` itself (a mode/flag) or as a
  separate lightweight class that `ForwardModel` composes in for the
  non-paired magnet terms.
