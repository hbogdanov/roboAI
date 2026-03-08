from __future__ import annotations

from typing import List, Tuple


def frontier_points_world(
    occ_grid,
    free_prob_thresh: float = 0.45,
    unknown_band: float = 0.06,
    stride: int = 3,
) -> List[Tuple[float, float]]:
    """
    Return world-space frontier points where free cells border unknown cells.
    """
    prob = occ_grid.prob()
    h, w = prob.shape
    out: List[Tuple[float, float]] = []

    for gy in range(1, h - 1, max(1, stride)):
        for gx in range(1, w - 1, max(1, stride)):
            p = float(prob[gy, gx])
            is_free = p < free_prob_thresh
            if not is_free:
                continue

            neigh = [
                float(prob[gy - 1, gx]),
                float(prob[gy + 1, gx]),
                float(prob[gy, gx - 1]),
                float(prob[gy, gx + 1]),
            ]
            has_unknown_neighbor = any(abs(n - 0.5) <= unknown_band for n in neigh)
            if has_unknown_neighbor:
                out.append(occ_grid.grid_to_world(gx, gy))

    return out
