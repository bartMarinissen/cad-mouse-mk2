"""Turns a BundleCalibrationResult into the Rich content the TUI and CLI show.

Split out from bundle_callibration.py so calibration/ (session.py, tui.py) can
depend on the report's *shape* (FitReport) without depending on the CLI entry
point's argument parsing. FitReport is split into three pieces rather than one
blob because the live display puts each in a different, fixed panel: what got
fitted (parameters), how much to trust it (diagnostics), and the plain-English
headline that summarizes diagnostics for the terminal dump.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from rich.console import Group, RenderableType
from rich.table import Table

from .bundle_geometry import MAGNET_POS_NOMINAL_KNOB, N_MAGNETS, N_SENSORS
from .calibration_algorithm import BundleCalibrationResult
from .parameterization import BLOCKS, GROUP_SLICES


@dataclass(frozen=True)
class FitReport:
    """A fit result, pre-split for where the live display puts each part."""

    converged: bool
    headline: RenderableType  # residual + converged/not, one line
    parameters: RenderableType  # fitted magnet + sensor tables
    diagnostics: RenderableType  # information-gain table + explanation

    def full(self) -> Group:
        """Everything together, for the plain-terminal dump after the run."""
        return Group(self.headline, self.parameters, self.diagnostics)


def build_fit_report(result: BundleCalibrationResult) -> FitReport:
    converged = all(s.success for s in result.stages)
    status = "[green]converged[/]" if converged else "[yellow]did NOT fully converge[/]"
    headline: RenderableType = (
        f"Bundle calibration {status}. Field residual "
        f"{result.initial_rms_mT:.3f} mT ({result.initial_rel_pct:.2f}%) at nominal "
        f"-> [bold]{result.field_rms_mT:.3f} mT ({result.field_rel_pct:.2f}%)[/] fitted, "
        f"over {len(result.selected_indices)} frames."
    )

    param_items: list[RenderableType] = []

    # Solving is a single joint fit by default (see calibration_algorithm.py's
    # module docstring) - a "stages" breakdown is only interesting when the
    # caller explicitly passed more than one, e.g. to isolate a parameter
    # subset for debugging.
    if len(result.stages) > 1:
        stages = Table(title="Solve stages")
        stages.add_column("Stage")
        stages.add_column("Free params", justify="right")
        stages.add_column("Residual (mT)", justify="right")
        stages.add_column("Residual (%)", justify="right")
        stages.add_column("Time (s)", justify="right")
        for s in result.stages:
            stages.add_row(s.name, str(s.n_free_shared), f"{s.field_rms_mT:.3f}",
                           f"{s.field_rel_pct:.2f}", f"{s.seconds:.1f}")
        param_items.append(stages)

    geometry = result.geometry

    # Magnets and sensors get separate tables on purpose: they are indexed by
    # different things. Sensor i is not exclusively paired with magnet i -
    # every sensor sees all three magnets, which is the cross-talk the model
    # accounts for - so listing them side by side would imply a pairing that
    # isn't there.
    magnets = Table(title="Fitted magnet geometry (offsets from nominal)")
    magnets.add_column("Magnet")
    magnets.add_column("dx (mm)", justify="right")
    magnets.add_column("dy (mm)", justify="right")
    magnets.add_column("dz (mm)", justify="right")
    magnets.add_column("tilt x (deg)", justify="right")
    magnets.add_column("tilt y (deg)", justify="right")
    magnets.add_column("strength", justify="right")
    for i in range(N_MAGNETS):
        d = geometry.magnet_pos_knob[i] - MAGNET_POS_NOMINAL_KNOB[i]
        tilt = np.degrees(geometry.magnet_tilt[i][:2])
        magnets.add_row(str(i + 1), f"{d[0]:+.3f}", f"{d[1]:+.3f}", f"{d[2]:+.3f}",
                        f"{tilt[0]:+.3f}", f"{tilt[1]:+.3f}",
                        f"{geometry.magnet_strength[i]:.4f}")
    param_items.append(magnets)

    # Absolute field scale deserves its error bar right next to it: with no
    # prior on the common mode, the fitted value is whatever the data prefers,
    # and the data has very little to say (a uniform scale change is largely
    # absorbable by every frame's pose moving further away).
    mean_sd = result.posterior_sigma[GROUP_SLICES["magnet_strength_mean"]][0]
    mean_strength = result.geometry.magnet_strength.mean()
    if np.isfinite(mean_sd):
        sigmas_from_nominal = abs(mean_strength - 1.0) / mean_sd if mean_sd > 0 else 0.0
        verdict = ("[green]consistent with nominal[/]" if sigmas_from_nominal < 2
                   else "[yellow]notably above nominal[/]")
        param_items.append(
            f"Common-mode strength [bold]{mean_strength:.3f} ± {mean_sd:.3f}[/] "
            f"({sigmas_from_nominal:.1f}σ from nominal) — {verdict}"
        )

    sensors = Table(title="Fitted sensor gain and DC offset (model side; nominal gain is +I)")
    sensors.add_column("Sensor")
    sensors.add_column("Gain matrix", justify="left")
    sensors.add_column("det", justify="right")
    sensors.add_column("DC offset (mT)", justify="right")
    for i in range(N_SENSORS):
        g = geometry.gain[i]
        rows = "\n".join("  ".join(f"{v:+.4f}" for v in row) for row in g)
        off = "\n".join(f"{v:+.3f}" for v in geometry.sensor_offset[i])
        sensors.add_row(str(i + 1), rows, f"{np.linalg.det(g):.4f}", off)
    param_items.append(sensors)

    # How much each group was actually pinned down by the data, as opposed to
    # left sitting at its prior. Low information gain is not a bug - some of
    # these directions are only weakly identifiable from any dataset (magnet
    # z-offset trades against gain scale, for instance) - but it is the
    # difference between "measured" and "assumed", so it gets shown.
    info = Table(title="What the data actually determined")
    info.add_column("Parameter group")
    info.add_column("Prior sd", justify="right")
    info.add_column("Posterior sd", justify="right")
    info.add_column("Information gain", justify="right")
    for block in BLOCKS:
        name = block.name
        sl = GROUP_SLICES[name]
        prior = result.prior_sigma[sl].mean()
        post = result.posterior_sigma[sl]
        gain = result.information_gain[sl]
        if not np.all(np.isfinite(post)):
            info.add_row(name, f"{prior:.3f}", "-", "[grey50]not fitted[/]")
            continue
        if not np.isfinite(prior):
            # No prior at all: the posterior sd stands on its own, and asking
            # how much the data added over the prior is meaningless.
            info.add_row(name, "[cyan]none[/]", f"{post.mean():.3f}",
                         "[cyan]data only[/]")
            continue
        colour = "green" if gain.mean() > 0.4 else ("yellow" if gain.mean() > 0.15 else "red")
        info.add_row(name, f"{prior:.3f}", f"{post.mean():.3f}",
                     f"[{colour}]{gain.mean():.0%}[/]")
    diagnostics: RenderableType = Group(
        info,
        "[grey50]Information gain = 1 - posterior/prior sd. Low means the fit "
        "mostly kept the prior, so treat that group as assumed, not measured. "
        "'data only' means the group carries no prior - read its posterior sd "
        "as the real uncertainty.[/]",
    )

    return FitReport(
        converged=converged,
        headline=headline,
        parameters=Group(*param_items),
        diagnostics=diagnostics,
    )
