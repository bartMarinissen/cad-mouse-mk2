# TODO

This directory tracks non-trivial follow-up work: real architectural
decisions or issues that don't belong in a code comment and don't fit in a
single commit. Some files originated from the architecture review in
`../ARCHITECTURE.md`; others get added independently as issues come up
during development. Each file covers one self-contained issue, or a small
cluster of tightly related issues.

File names are descriptive, and each file's own content is the source of
truth for the detail — skim the directory listing to see what's open. This
file also carries a 2-line summary per TODO, below, so the state of
everything open is skimmable from one place without opening each file. Keep
it in sync: it's a second copy of information the file itself owns, so it
only stays trustworthy if every commit that changes what a TODO is actually
about also updates its summary line here in the same commit.

## Writing a TODO file

- State the problem concretely: what's wrong, where in the code, and why it
  matters — not just "this is inconsistent," but what actually breaks or
  gets harder because of it.
- Include a "likely fix shape" if you have one, but it doesn't need to be a
  committed plan — just enough direction that whoever picks it up isn't
  starting from zero.
- Update a file in place as understanding deepens, rather than leaving it
  stale (see `controller-ownership.md`'s "Problem 3" for an example — a
  later investigation surfaced a related issue and it was added as a new
  section instead of a separate file).
- If a new problem turns out to be tightly coupled with an existing file's
  topic, add a section there instead of creating a duplicate.
- Add a 2-line summary entry below, in the index, in the same commit.

## Resolving a TODO file

- Delete the file once the work lands, and delete its entry below.
- If the issue was also tracked in `../ARCHITECTURE.md`'s "Issues /
  architecture drift" list, update that entry to say it's fixed instead of
  leaving it pointing at a deleted file.
- Trivial issues resolved without ever getting a TODO file don't need one
  written retroactively just to close it out.

## Index

Two lines per file, kept current with the file's actual content — if a TODO
gains a new open question or loses a resolved one, update its summary here
too, not just the file.

- **[`Performance.md`](Performance.md)** — Assembly/instruction-level
  profiling of `solve_knob_pose`; three completed optimization passes
  (NDEBUG/-O2 fix, `BicubicField` rewrite, solver algebra) took the loop from
  ~50Hz to ~80Hz. Iteration-count telemetry and an on-device wall-clock
  re-measurement of the combined changes are still open.
- **[`calibration-led-animations.md`](calibration-led-animations.md)** — The
  LED animation architecture (`AnimationBase`/`std::variant`) is in place,
  but bundle calibration only shows one placeholder spinner. The actual
  per-phase/per-step visuals are undesigned.
- **[`calibration-mode-entry.md`](calibration-mode-entry.md)** — Entry into
  bundle calibration is now PC-initiated via `CAL_START`, closing the old
  accidental-entry bug. A knob-side exit gesture and `BundleState.cpp`'s
  `takeActivity()`-instead-of-`buttonBits()` bug are still open.
- **[`controller-ownership.md`](controller-ownership.md)** — Of the three
  controller-coupling problems flagged in the architecture review, two are
  resolved. `SensorController` calling directly into `MotionController`
  during calibration is still open, tied to the tare redesign.
- **[`cross-magnet-interference.md`](cross-magnet-interference.md)** —
  Cross-magnet field coupling (measured 1.7–4.5% of the signal) is
  quantified but not modelled in firmware; a Python-side fitting workaround
  avoids the pose-residual cost at the price of physically meaningful fitted
  parameters. A cheap dipole-correction fix is proposed, not implemented.
- **[`multicore.md`](multicore.md)** — The RP2040's second core is entirely
  unused; sensor reads, the solve, HID, and telemetry all serialize on core0.
  No design work has started beyond identifying the gap.
- **[`readme-refresh.md`](readme-refresh.md)** — `firmware/README.md` still
  documents the old per-axis-averaging motion heuristic this fork replaced
  with the Gauss-Newton solver. Needs its "current implementation" section
  rewritten to match `ARCHITECTURE.md`.
- **[`sensor-gain-calibration.md`](sensor-gain-calibration.md)** — Sensor
  gain/skew correction now lives entirely in `SensorController`, resolving
  the original architecture-review issue. One follow-up gap remains:
  `calibration/export.py` still emits a dimensionless multiplier instead of
  a real mT `magnet_strength_mT` value.
- **[`sensor-read-speed.md`](sensor-read-speed.md)** — Decided: move all
  three Hall sensors to Master-Controlled Mode with per-read trigger bits,
  cutting staleness from ~6.25ms to under 1ms. Not yet implemented; open
  questions on library/hardware support for the trigger bits remain.
- **[`tare-and-calibration.md`](tare-and-calibration.md)** — `CalibratingState`'s
  boot-time averaging needs to become a proper "tare" step with sanity
  checks (rest position, polarization, residual) and a reject/retry path.
  Still a design sketch — no implementation decisions made yet.
- **[`telemetry-rework.md`](telemetry-rework.md)** — Four bundled issues:
  drop the hardcoded `rcond` stub, flatten `TelemetryController::publish()`'s
  growing parameter list, add solver convergence telemetry, and add a
  lightweight profiling API. None implemented yet.
