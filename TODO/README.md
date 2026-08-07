# TODO

This directory tracks non-trivial follow-up work: real architectural
decisions or issues that don't belong in a code comment and don't fit in a
single commit. Some files originated from the architecture review in
`../ARCHITECTURE.md`; others get added independently as issues come up
during development. Each file covers one self-contained issue, or a small
cluster of tightly related issues.

This file intentionally isn't an index. File names are descriptive, and
each file's own content is the source of truth — skim the directory listing
to see what's open. A manually-synced one-line summary per file here would
just be a second place that drifts out of date with no way to enforce it.

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

## Resolving a TODO file

- Delete the file once the work lands.
- If the issue was also tracked in `../ARCHITECTURE.md`'s "Issues /
  architecture drift" list, update that entry to say it's fixed instead of
  leaving it pointing at a deleted file.
- Trivial issues resolved without ever getting a TODO file don't need one
  written retroactively just to close it out.
