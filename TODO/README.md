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

`resolved/` holds finished TODOs rather than deleting them, so the reasoning
behind a decision outlives the decision. **Everything in there is history:
it is written in the present tense but describes code as it was.** Each
archived file carries a header saying what closed it and where the current
description lives. Don't cite `resolved/` as a statement about the tree.

## Writing a TODO file

- State the problem concretely: what's wrong, where in the code, and why it
  matters — not just "this is inconsistent," but what actually breaks or
  gets harder because of it.
- Include a "likely fix shape" if you have one, but it doesn't need to be a
  committed plan — just enough direction that whoever picks it up isn't
  starting from zero.
- **Head that section "Design", not "Decision".** A design is what you intend
  to build; a decision is a question that is closed, and calling a sketch a
  decision hides the fact that it can still change. Reserve "Decided" for
  parts that are actually settled.
- Update a file in place as understanding deepens, rather than leaving it
  stale. `Performance.md` is the model here: each optimization pass appended
  its own section and amended the earlier ones it invalidated, so the file
  reads as a running investigation rather than a stale first draft.
- **When you implement part of a TODO, update that TODO in the same commit.**
  The two worst drifts found so far both came from skipping this:
  `sensor-read-speed.md` was implemented 11 minutes after it was written and
  still claimed nothing had been done, and `Performance.md` described a
  constants bug as open that a parallel branch had fixed a minute earlier.
  Neither conflicted, so git had nothing to complain about.
- If a new problem turns out to be tightly coupled with an existing file's
  topic, add a section there instead of creating a duplicate.
- Add a 2-line summary entry below, in the index, in the same commit.

## Resolving a TODO file

- **Move the file to `resolved/`, don't delete it.** The investigation that
  produced a decision is worth more than the decision alone — why an approach
  was rejected, what was measured, which datasheet paragraph it turned on.
  That context is what stops the same ground being re-covered later, and it
  is not reconstructible from the diff that implemented it.
- Add a short header at the top of the moved file saying what resolved it
  (commit hashes where they're known), and where the *live* description now
  lives — usually a section of `../ARCHITECTURE.md`. A reader who lands in
  `resolved/` must be able to tell immediately that they're reading history,
  because the body will still be written in the present tense and will
  describe code that no longer looks like that.
- Move its index entry down to the "Resolved" list below.
- If the issue was also tracked in `../ARCHITECTURE.md`'s "Issues /
  architecture drift" list, update that entry to say it's fixed and point at
  the archived file.
- Prefer pointing live code comments at `ARCHITECTURE.md` rather than into
  `resolved/` — the archive is for someone digging into history, not a
  reference the code should depend on.
- Trivial issues resolved without ever getting a TODO file don't need one
  written retroactively just to close it out.

## Index — open

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
- **[`cross-magnet-interference.md`](cross-magnet-interference.md)** —
  Cross-magnet field coupling (measured 1.7–4.5% of the signal) is
  quantified but not modelled in firmware; a Python-side fitting workaround
  avoids the pose-residual cost at the price of physically meaningful fitted
  parameters. A cheap dipole-correction fix is proposed, not implemented.
- **[`multicore.md`](multicore.md)** — Proposed design: core1 runs the pose
  solve continuously, with sensor readings and poses crossing the core
  boundary as double buffers gated by a spinlocked counter held across the
  copy. Telemetry is a third channel that doesn't fit that shape and is
  undesigned; nothing is implemented.
- **[`on-device-calibration.md`](on-device-calibration.md)** — Feasibility of
  running the whole bundle calibration on the knob instead of the PC. Both
  feasibility questions have answers (the Jacobian wraps the existing forward
  model; Schur elimination makes the normal equations fit in RAM), verified by
  a prototype outside the build; tilt parameterization and column ordering are
  decided. Whether to build it is open.
- **[`tare-and-calibration.md`](tare-and-calibration.md)** — `CalibratingState`'s
  boot-time averaging needs to become a proper "tare" step with sanity
  checks (rest position, polarization, residual) and a reject/retry path.
  Still a design sketch — no implementation decisions made yet.
- **[`telemetry-rework.md`](telemetry-rework.md)** — Four bundled issues:
  drop the hardcoded `rcond` stub, flatten `TelemetryController::publish()`'s
  growing parameter list, add solver convergence telemetry, and add a
  lightweight profiling API. None implemented yet.

## Index — resolved (`resolved/`)

History, not current state. One line on what it was and what closed it; the
archived file's own header has the detail.

- **[`resolved/controller-ownership.md`](resolved/controller-ownership.md)** —
  Controller coupling around the pose pipeline. The forward model got a real
  owner (`1883bc2`/`20f5f4d`); the remaining "circular coupling" was settled
  by deciding the accessor pattern *is* the rule — see `ARCHITECTURE.md` →
  "Controllers".
- **[`resolved/sensor-gain-calibration.md`](resolved/sensor-gain-calibration.md)** —
  Moving sensor gain/skew out of the solver. Now applied in
  `SensorController::read_mT()` from `CalibrationParams`; the last gap
  (`export.py` emitting a bare multiplier) closed with
  `firmware_magnet_strength_mT()`.
- **[`resolved/sensor-read-speed.md`](resolved/sensor-read-speed.md)** —
  Master-Controlled Mode + trigger-on-read, cutting staleness from ~6.25ms to
  about one I2C round-trip. Implemented in `7d1fb42`/`26e604d`; live
  description is `ARCHITECTURE.md` → "Sensor read timing". One caveat is
  still unverified on hardware (RP2040 `Wire` clock stretching).
