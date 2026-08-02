# magnet_field_model

The Python half of the project (`uv`-managed — see `pyproject.toml`/`uv.lock`).
Two independent pieces live here: the field-table codegen notebook, and the
PC side of the bundle-calibration capture tool.

## `field_approximation.ipynb`

Simulates the real magnet's field with `magpylib` and generates
`firmware/{include,src}/magnet_model/magnet_model_table.{h,cpp}` — the
precomputed bicubic interpolation table the firmware's pose solver queries at
runtime. Cell 17 writes both files **directly** (no copy-paste step); those
two files carry an "auto-generated, don't hand-edit" banner as a result. If
the magnet spec or grid resolution/bounds change, they need to change in
both this notebook and `magnet_model_table.h` — re-running the notebook is
what keeps them in sync, since nothing does that automatically. See
`ARCHITECTURE.md` at the repo root ("Codegen: Python → firmware table") for
the full pipeline, and `firmware/src/magnet_model/README.md` for what the
generated table feeds into.

## `bundle_callibration.py` + `calibration/`

The PC side of the in-progress guided hardware calibration routine (pairs
with `BundleState`/`BundleCalibrationController` in the firmware). Currently
data-capture only — it collects the per-step datasets but doesn't yet run
the actual bundle-adjustment fit. `bundle_callibration.py`'s own module
docstring maps the `calibration/` package (`protocol.py`, `serial_link.py`,
`session.py`, `collector.py`, `tui.py`, `bundle_geometry.py`/
`bundle_params.py`/`calibration_algorithm.py`); see `ARCHITECTURE.md`'s
"Calibration subsystem" section for its current status. This code is under
active development — check there and in `TODO/` before relying on specifics
beyond that map.
