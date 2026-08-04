"""Choosing which captured frames to actually fit.

A capture run streams ~390 frames at 20Hz, but consecutive frames are 50ms
apart and therefore nearly the same pose - they add very little beyond
averaging down noise (uncertainty falls only as 1/sqrt(N)), while every extra
frame costs 6 more unknowns in the fit. What the calibration actually needs
is *geometric diversity*: poses that excite the parameters differently.

Plain decimation (the old `frames[::8]`) throws away frames blind to that -
it keeps a fixed fraction of every step whether that step was moving or
sitting still. Instead, this module solves each frame's approximate pose
first and then picks a subset by greedy farthest-point sampling, so the
selected frames spread out over the pose volume the run actually covered.

The distance is measured in *magnet-position space*: the 9-vector of where
the three magnets end up in the world frame. That sidesteps the arbitrary
choice of how to weigh millimetres against radians - a rotation matters
exactly as much as it moves the magnets, which is the thing the sensors see.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from .bundle_geometry import MAGNET_POS_NOMINAL_KNOB


def magnet_world_positions(poses: NDArray[np.float64]) -> NDArray[np.float64]:
    """(n_frames, 9): the three magnets' world positions for each pose."""
    poses = np.asarray(poses, dtype=float)
    r = Rotation.from_rotvec(poses[:, 3:])
    pts = poses[:, None, :3] + np.einsum(
        "nab,jb->nja", r.as_matrix(), MAGNET_POS_NOMINAL_KNOB
    )
    return pts.reshape(len(poses), -1)


def select_diverse_frames(
    poses: NDArray[np.float64],
    n_select: int,
    *,
    valid: NDArray[np.bool_] | None = None,
) -> NDArray[np.int_]:
    """Greedy farthest-point subset of `poses`, returned as sorted indices.

    Starts from the frame nearest the centroid (so the rest pose, which
    anchors the fit, is always in) and then repeatedly adds whichever
    remaining frame is furthest from everything already chosen.
    """
    features = magnet_world_positions(poses)
    n = len(features)
    candidates = np.arange(n) if valid is None else np.flatnonzero(valid)
    if n_select >= len(candidates):
        return np.sort(candidates)

    feats = features[candidates]
    centroid = feats.mean(axis=0)
    first = int(np.argmin(np.linalg.norm(feats - centroid, axis=1)))

    chosen = [first]
    min_dist = np.linalg.norm(feats - feats[first], axis=1)
    for _ in range(n_select - 1):
        nxt = int(np.argmax(min_dist))
        chosen.append(nxt)
        min_dist = np.minimum(min_dist, np.linalg.norm(feats - feats[nxt], axis=1))

    return np.sort(candidates[np.array(chosen)])


def diversity_report(poses: NDArray[np.float64], indices: NDArray[np.int_]) -> dict[str, float]:
    """A few numbers describing how much pose volume a selection covers."""
    sel = np.asarray(poses)[indices]
    rot_deg = np.degrees(np.linalg.norm(sel[:, 3:], axis=1))
    return {
        "n": float(len(indices)),
        "t_span_x_mm": float(np.ptp(sel[:, 0])),
        "t_span_y_mm": float(np.ptp(sel[:, 1])),
        "t_span_z_mm": float(np.ptp(sel[:, 2])),
        "rot_span_deg": float(np.ptp(rot_deg)),
        "rot_max_deg": float(rot_deg.max()),
    }
