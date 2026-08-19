# Working in this repo

**Routing and conventions only — this file must not restate
`ARCHITECTURE.md`.** No parameter counts, no Hz figures, no file inventories,
no pipeline diagrams. Keep it to things that change slowly and can't be
learned by reading the source; an auto-loaded file drifts invisibly.

## Read order

1. **`ARCHITECTURE.md`** — the *where/how*, written from reading the code.
   Start here. Re-verify function names and line numbers before relying on
   them.
2. **`design documentation/context.md`** — the *why*: hardware, intent, what
   this fork replaced.
3. **`design documentation/Math.md`** — the forward model and Jacobian
   derivation. Read before touching any `evaluate()`.
4. **`TODO/`** — open work. `TODO/README.md` has the conventions and a
   per-file index; read it before adding or resolving a TODO.

## One owner per fact

Every fact has one authoritative home, and everywhere else points at it
instead of restating it. `magnet_model_table.h` owns the interpolation grid's
bounds; `CalibrationStorage.h` owns the stored-blob layout; `ARCHITECTURE.md`
owns how the pipeline fits together; `TODO/` owns what is and isn't done. A
second copy isn't redundancy, it's a future contradiction.

Prefer "the grid bounds live in `magnet_model_table.h`" over quoting them.

**READMEs are where this goes wrong most, so hold them to it hardest.** 
One README is not an exception to the above but worth naming:
**`TODO/README.md` does own implementation status.** Being an index of open
work is its whole job, so it states plainly what's done and what isn't.

## Code comments describe the present, not the past

Write every comment as if the code had always looked this way. State
what's true now — an invariant, a constraint, a non-obvious consequence
— never what changed or why it changed; that's the commit message's job.

Bad: `// Made constexpr so this can be static constexpr; used to go
through FillRowMajor's loop.`
Good: `// constexpr: usable in a static_assert / constant expression.`

Catching yourself writing "used to", "no longer", "previously", "old X", or
"this replaces" in a code comment is the signal: that sentence belongs in
the commit message, not next to the code.

## Docs that will actively mislead you

- **`TODO/resolved/` is history.** Those files are written in the present
  tense and describe code as it was. Each has a header saying so. Never cite
  them as a statement about the tree.
- **The root `README.md` is the maintainer's personal file. Do not edit it.**
  Its "Current state" numbers are expected to lag. Do not "fix" it to match
  the other docs — that is the maintainer's call, not a doc-sync task.

## Docs can lag the code

Docs here are usually right, but they lag, and they lag most on whatever was
worked on recently. So when a claim actually matters to what you're about to
do — and especially when it's about whether something is already implemented —
check it against the source rather than against another doc. Two TODOs once
spent weeks describing work that had already shipped, and both read as
perfectly consistent until someone opened the `.cpp`.

The cheapest way to keep this from getting worse: **when you implement part of
a TODO, update that TODO in the same commit.** Neither drift above produced a
merge conflict, so nothing flagged them.

## Tests, and when they are not optional

- **After changing any `evaluate()` in `firmware/src/magnet_model/`** (or any
  Jacobian anywhere): `pio test -e seeed_xiao_rp2040_test`.
  `firmware/test/test_jacobian.cpp` checks every analytic Jacobian in the
  chain against central finite differences. It is the safety net for the whole
  solver, and the chain is hand-derived — nothing else will catch a wrong
  derivative.
- **Firmware build:** `pio run -e seeed_xiao_rp2040`. This works from a
  sandbox — `pip install platformio` then `pio run` fetches the ARM toolchain
  and builds end to end. A previous session concluded otherwise from a 403 on
  `api.github.com`, but the toolchain ships from
  `github.com/.../releases/download/`, a different host. Check with
  `pio pkg install` before believing you can't build.
- **After touching `magnet_field_model/`:**
  ```bash
  cd magnet_field_model
  uv run --group dev python -m pytest tests/ -q
  uv run --group dev ruff check .
  uv run --group dev mypy .
  ```
  All three are expected to be clean — if ruff or mypy reports anything, it is
  new. **Budget several minutes for the suite**, and don't kill it at two:
  several tests run real calibration fits, so it is slow by design rather than
  hung.
  `tests/test_jacobian.py` mirrors the firmware's Jacobian test on the Python
  side.
- **`firmware/src/magnet_model/magnet_model_table.cpp` is generated** — don't
  hand-edit. Regenerate with
  `uv run python generate_bicubic_table.py`.

## Writing TODOs

Full conventions in `TODO/README.md`. The two most often got wrong:

- Head a proposed approach **"Design", not "Decision"**. A decision is a
  closed question, which in practice means implemented. A sketch with
  load-bearing open questions is a design.
- **Resolving a TODO moves it to `TODO/resolved/` with a header** saying what
  closed it and where the live description now lives. Do not delete it — the
  investigation outlives the decision and is not reconstructible from the diff
  that implemented it.

## When you are blocked on a question

A question you have asked but that has not been answered is **not** a prompt
to keep working. Waiting is the correct state, and a short turn is the correct
output.

- **Ask once, then end the turn.** Do not re-propose, re-summarize, or
  re-plan while waiting. Restating a plan in new words is not progress; it
  buries the question under text and makes it harder to answer.
- **Do not poll.** No wakeups, loops, or timers to check for an answer —
  there is nothing to poll, the answer arrives as a message. Do not
  re-ask the same question on a later turn.
- **Do the answer-independent work first.** Everything that doesn't depend on
  the decision should already be finished and committed before you ask, so
  the wait blocks as little as possible.
- **A long wait is fine. Guessing to avoid one is not.** If the answer
  genuinely determines what to build, wait for it however long that takes —
  do not substitute an assumption to keep moving.

## Commits

- Explain *why*, not just what; these messages are load-bearing here and are
  cited from the docs by hash.
- Report outcomes honestly. If something is unverified on hardware, say so —
  several docs depend on that distinction being kept.
