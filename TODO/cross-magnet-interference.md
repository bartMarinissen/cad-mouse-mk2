# Model cross-magnet interference

**Hopefully not needed.** Before implementing anything here, it's worth
actually quantifying the error this introduces — e.g. use `magpylib` in
`magnet_field_model/field_approximation.ipynb` to simulate all 3 real magnets
present simultaneously and compare each sensor's field against the
single-magnet approximation the firmware currently uses, across the working
range of poses. If the discrepancy is small relative to sensor noise / the
residual thresholds tare will be gating on (see `TODO/tare-and-calibration.md`),
this whole effort can stay shelved.

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
