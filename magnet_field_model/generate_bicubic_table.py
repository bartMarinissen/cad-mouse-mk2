"""Regenerates the firmware's bicubic field table.

Writes firmware/include/magnet_model/magnet_model_table.h and
firmware/src/magnet_model/magnet_model_table.cpp from the magnet model and
grid defined in bicubic_table.py -- that module is the single source of truth
for both this script and field_approximation.ipynb (which imports the same
functions to inspect/visualize the table this script writes).

Run after changing anything in bicubic_table.py (magnet dimensions, grid
bounds/resolution, reference polarization), then rebuild the firmware.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from bicubic_table import BICUBIC_FIELD_REFERENCE_MT, generate

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_HEADER = REPO_ROOT / "firmware/include/magnet_model/magnet_model_table.h"
DEFAULT_CPP = REPO_ROOT / "firmware/src/magnet_model/magnet_model_table.cpp"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--header", type=Path, default=DEFAULT_HEADER,
                         help=f"Output header path (default: {DEFAULT_HEADER})")
    parser.add_argument("--cpp", type=Path, default=DEFAULT_CPP,
                         help=f"Output source path (default: {DEFAULT_CPP})")
    parser.add_argument("--polarization-mt", type=float, default=BICUBIC_FIELD_REFERENCE_MT,
                         help="Polarization (Br) to generate the table at, in mT. "
                              "Defaults to bicubic_table.BICUBIC_FIELD_REFERENCE_MT. "
                              "The field scales exactly linearly with this, so changing "
                              "it only rescales the table -- it does not need to match "
                              "any real magnet. Only pass this deliberately: it changes "
                              "what BICUBIC_FIELD_REFERENCE_MT reports in the generated "
                              "header, which every MagnetModel::magnet_strength_mT is "
                              "measured against.")
    args = parser.parse_args()

    header, cpp, table = generate(args.polarization_mt)
    args.header.write_text(header)
    args.cpp.write_text(cpp)

    print(f"Wrote {args.header}")
    print(f"Wrote {args.cpp}")
    print(f"Grid: {len(table.r_line)} x {len(table.z_line)} points, "
          f"r in [{table.r_line[0]}, {table.r_line[-1]}], "
          f"z in [{table.z_line[0]}, {table.z_line[-1]}], "
          f"generated at {args.polarization_mt} mT")


if __name__ == "__main__":
    main()
