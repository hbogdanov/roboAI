from __future__ import annotations

from typing import List, Tuple


def _has_occupied_neighbor(prob, gx: int, gy: int, clearance_cells: int, occ_thresh: float) -> bool:
    h, w = prob.shape
    for dy in range(-clearance_cells, clearance_cells + 1):
        for dx in range(-clearance_cells, clearance_cells + 1):
            nx = gx + dx
            ny = gy + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            if float(prob[ny, nx]) >= occ_thresh:
                return True
    return False


def frontier_points_world(
    occ_grid,
    free_prob_thresh: float = 0.45,
    unknown_band: float = 0.06,
    stride: int = 3,
    obstacle_clearance_cells: int = 0,
    occ_prob_thresh: float = 0.65,
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
                if obstacle_clearance_cells > 0 and _has_occupied_neighbor(
                    prob, gx, gy, obstacle_clearance_cells, occ_prob_thresh
                ):
                    continue
                out.append(occ_grid.grid_to_world(gx, gy))

    return out
