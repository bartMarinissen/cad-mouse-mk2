# Working in this repo

**This file is routing and conventions only. It must not restate
`ARCHITECTURE.md`.** No parameter counts, no Hz figures, no file inventories,
no pipeline diagrams — those live in `ARCHITECTURE.md` and duplicating them
here just creates a second copy that drifts. This repo's dominant failure mode
is documentation disagreeing with code; a file that is auto-loaded drifts
invisibly, so keep it to things that change slowly and can't be learned by
reading the source.

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

## READMEs must not describe the current state of the code

A README says what a thing is *for*, how to run it, and what you need to know
before touching it. It does **not** say what the code currently does, how fast
it currently is, how many parameters it currently fits, or what currently
passes. Those claims rot, and a README is the worst possible place to put
them: it's the most-read and least-reviewed file in any directory, so nobody
notices when it stops being true, and everyone believes it.

Current state belongs in `ARCHITECTURE.md`, which exists to be re-derived from
the code, and in `TODO/`, which is explicitly about work in flight.
`TODO/README.md` is an index of open work, so saying what is and isn't
implemented is its entire job — this rule does not apply to it.

So: no benchmark numbers, no residuals, no "N tests pass", no "this is
implemented / not implemented yet", no restating a struct's layout or a
constant's value. Point at the header or the source instead. Prefer "the grid
bounds live in `magnet_model_table.h`" over quoting the bounds.

Two known exceptions:

- **The root `README.md` is the maintainer's personal file** — it has a
  "Current state of the project" section by design. Not yours to edit or to
  bring into line with this rule. See below.
- **`magnet_field_model/README.md` currently violates this heavily** — it
  carries parameter counts, fitted residuals, and measured cross-run spreads,
  i.e. it is doing `ARCHITECTURE.md`'s job for the Python side. Left alone
  rather than gutted, because deciding where that content should live is a
  real call, not a cleanup. Don't add more of it; don't cite it as current.

## Docs that will actively mislead you

- **`firmware/README.md` is knowingly stale.** It documents the old
  per-axis-averaging motion heuristic this fork *replaced*. It is not a
  description of the code. Tracked in `TODO/readme-refresh.md`.
- **`TODO/resolved/` is history.** Those files are written in the present
  tense and describe code as it was. Each has a header saying so. Never cite
  them as a statement about the tree.
- **The root `README.md` is the maintainer's personal file. Do not edit it.**
  Its "Current state" numbers are expected to lag. Do not "fix" it to match
  the other docs — that is the maintainer's call, not a doc-sync task.

## Verify against code, not against docs

When checking whether something is true, read the source. Do not confirm a
doc's claim by re-reading the doc. Two TODOs in this repo spent weeks
describing work that had already shipped, and one described a bug that a
parallel branch had fixed a minute earlier — all of which read as perfectly
consistent until someone opened the `.cpp`.

Corollary: **when you implement part of a TODO, update that TODO in the same
commit.** Both drifts above came from skipping this. Neither produced a merge
conflict, so nothing flagged them.

## Tests, and when they are not optional

- **After changing any `evaluate()` in `firmware/src/magnet_model/`** (or any
  Jacobian anywhere): `pio test -e seeed_xiao_rp2040_test`.
  `firmware/test/test_jacobian.cpp` checks every analytic Jacobian in the
  chain against central finite differences. It is the safety net for the whole
  solver, and the chain is hand-derived — nothing else will catch a wrong
  derivative.
- **Firmware build:** `pio run -e seeed_xiao_rp2040`.
- **After touching `magnet_field_model/`:**
  ```bash
  cd magnet_field_model
  uv run --group dev python -m pytest tests/ -q    # 57 tests, ~2m15s
  uv run --group dev ruff check .
  uv run --group dev mypy .
  ```
  All three are expected to be clean — if ruff or mypy reports anything, it is
  new. The suite is slow because several tests run real calibration fits;
  budget for it rather than killing it at 2 minutes.
  `tests/test_jacobian.py` mirrors the firmware's Jacobian test on the Python
  side.
- **`firmware/src/magnet_model/magnet_model_table.cpp` is generated** — don't
  hand-edit. Regenerate with
  `uv run python generate_bicubic_table.py`.

## Measurement traps (each one already cost a session)

- **The ARM toolchain *is* reachable from a sandbox.** `pip install platformio`
  then `pio run` works end to end. A previous session concluded otherwise from
  a 403 on `api.github.com` — but the toolchain ships from
  `github.com/.../releases/download/`, a different host. Check with
  `pio pkg install` before believing you can't build.
- **Static `bl`-counting can give the wrong sign.** Tallying call targets in a
  disassembly counts work inside a loop kernel once regardless of iterations,
  so replacing a library loop with straight-line code looks like a
  regression when it is a 20% win. Cross-check with
  `valgrind --tool=callgrind` on a host build. See `TODO/Performance.md`.
- **Prefer measuring to estimating.** Hand-counted flop estimates in
  `TODO/Performance.md` were wrong enough to need replacing wholesale. If a
  number goes in a doc, say how it was obtained.

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
