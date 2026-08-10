"""The firmware's CalibrationParams layout, read from the firmware's own header.

/calibration.bin is a raw copy of the C struct: the device validates the CRC
and then memcpy's the payload straight into a CalibrationParams. That only
works if this side lays the floats out in exactly the order the C struct
declares them, and a wrong order is not a loud failure - it produces a
well-formed blob with a valid CRC that loads into plausible-looking nonsense.

So the layout is not restated here. This module extracts the struct
declaration out of firmware/include/CalibrationParams.h and hands it to cffi,
which works out sizes and offsets itself. Field order lives in exactly one
place, the header, and everything downstream addresses fields by name.

cffi is used in ABI mode - cdef() only, no set_source()/compile() - so nothing
here needs a C compiler at runtime. It computes the layout from its own model
of the platform ABI. tests/test_export.py cross-checks that model against a
real g++ build of the same declaration, which is worth keeping precisely
because both of them are only ever describing the *host*; the firmware's own
static_asserts are what pin the target.
"""

from __future__ import annotations

import re
from pathlib import Path

from cffi import FFI

# magnet_field_model/calibration/ -> magnet_field_model/ -> repo root
CALIBRATION_PARAMS_H = (
    Path(__file__).resolve().parent.parent.parent
    / "firmware"
    / "include"
    / "CalibrationParams.h"
)

STRUCT_NAME = "CalibrationParams"

# Non-greedy to the first line that is exactly "};", which is how the struct
# closes. Nothing else in the header is indented that way.
_STRUCT_RE = re.compile(
    rf"^struct\s+{STRUCT_NAME}\s*\{{.*?^\}};", re.MULTILINE | re.DOTALL
)

# Safe on this header: the struct is nothing but float arrays, so there are no
# string literals for a "//" to be hiding inside.
_LINE_COMMENT_RE = re.compile(r"//.*$", re.MULTILINE)


def struct_source(path: Path = CALIBRATION_PARAMS_H) -> str:
    """The `struct CalibrationParams { ... };` block as plain C, comments out.

    Raises rather than falling back to a built-in copy. A copy is the exact
    failure this module exists to remove - if the header has been renamed,
    moved or reshaped into something unrecognisable, that needs to surface
    here and not be papered over with a stale duplicate.
    """
    if not path.is_file():
        raise FileNotFoundError(
            f"cannot read the firmware calibration header at {path} - "
            "firmware_struct.py needs it to know the on-disk layout"
        )

    match = _STRUCT_RE.search(path.read_text())
    if match is None:
        raise ValueError(
            f"no `struct {STRUCT_NAME} {{ ... }};` block found in {path}. "
            "The declaration is the single source of truth for the "
            "/calibration.bin layout, so this cannot be guessed."
        )

    body = _LINE_COMMENT_RE.sub("", match.group(0))
    # cffi does not mind blank lines, but stripping them keeps the text
    # readable when a test prints it on failure.
    return "\n".join(line.rstrip() for line in body.splitlines() if line.strip())


ffi = FFI()
ffi.cdef(struct_source())

CDECL = f"struct {STRUCT_NAME}"

# Derived, never written down: this is the whole point of the module.
PAYLOAD_SIZE = ffi.sizeof(CDECL)

# Declaration order, straight from the cdef. Anything that needs to walk the
# fields should use this rather than repeating the list.
FIELD_NAMES = tuple(name for name, _ in ffi.typeof(CDECL).fields)


def new_params(**fields: object) -> object:
    """An initialised `CalibrationParams` cdata, keyed by field name.

    Keyword-only on purpose. cffi would happily take a positional list in
    declaration order, which is exactly the coupling being removed - passing
    by name means Python never has an opinion about what comes first.
    """
    missing = set(FIELD_NAMES) - set(fields)
    if missing:
        raise ValueError(
            f"missing calibration fields: {sorted(missing)}. Every field has "
            "to be supplied; a partly-filled struct would still serialise to a "
            "valid-looking blob."
        )
    unexpected = set(fields) - set(FIELD_NAMES)
    if unexpected:
        raise ValueError(
            f"unknown calibration fields: {sorted(unexpected)}. Known fields "
            f"are {list(FIELD_NAMES)}, taken from {CALIBRATION_PARAMS_H.name}."
        )
    return ffi.new(f"{CDECL} *", dict(fields))


def to_bytes(params: object) -> bytes:
    """The struct's own bytes - what the firmware memcpy's back out."""
    return bytes(ffi.buffer(params, PAYLOAD_SIZE))
