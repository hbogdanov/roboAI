from __future__ import annotations

from typing import List, Tuple, Dict, Any, Optional


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


def frontier_regions(
    occ_grid,
    free_prob_thresh: float = 0.45,
    unknown_band: float = 0.06,
    stride: int = 3,
    obstacle_clearance_cells: int = 0,
    occ_prob_thresh: float = 0.65,
    robot_xy: Optional[Tuple[float, float]] = None,
    min_distance_m: float = 0.0,
    min_cluster_size: int = 2,
) -> List[Dict[str, Any]]:
    prob = occ_grid.prob()
    h, w = prob.shape
    frontier_cells = []
    frontier_set = set()

    for gy in range(1, h - 1, max(1, stride)):
        for gx in range(1, w - 1, max(1, stride)):
            p = float(prob[gy, gx])
            if p >= free_prob_thresh:
                continue
            neigh = [
                float(prob[gy - 1, gx]),
                float(prob[gy + 1, gx]),
                float(prob[gy, gx - 1]),
                float(prob[gy, gx + 1]),
            ]
            if not any(abs(n - 0.5) <= unknown_band for n in neigh):
                continue
            if obstacle_clearance_cells > 0 and _has_occupied_neighbor(
                prob, gx, gy, obstacle_clearance_cells, occ_prob_thresh
            ):
                continue
            frontier_cells.append((gx, gy))
            frontier_set.add((gx, gy))

    regions: List[Dict[str, Any]] = []
    visited = set()
    neighbor_step = max(1, stride)
    for seed in frontier_cells:
        if seed in visited:
            continue
        queue = [seed]
        visited.add(seed)
        cluster = []
        while queue:
            cx, cy = queue.pop()
            cluster.append((cx, cy))
            for dy in (-neighbor_step, 0, neighbor_step):
                for dx in (-neighbor_step, 0, neighbor_step):
                    if dx == 0 and dy == 0:
                        continue
                    nxt = (cx + dx, cy + dy)
                    if nxt in frontier_set and nxt not in visited:
                        visited.add(nxt)
                        queue.append(nxt)

        if len(cluster) < max(1, min_cluster_size):
            continue
        mean_x = sum(pt[0] for pt in cluster) / float(len(cluster))
        mean_y = sum(pt[1] for pt in cluster) / float(len(cluster))
        centroid_grid = (int(round(mean_x)), int(round(mean_y)))
        centroid_world = occ_grid.grid_to_world(centroid_grid[0], centroid_grid[1])
        distance_m = None
        if robot_xy is not None:
            distance_m = ((centroid_world[0] - robot_xy[0]) ** 2 + (centroid_world[1] - robot_xy[1]) ** 2) ** 0.5
            if distance_m < min_distance_m:
                continue
        regions.append(
            {
                "centroid_world": centroid_world,
                "centroid_grid": centroid_grid,
                "size": len(cluster),
                "distance_m": distance_m,
            }
        )

    regions.sort(
        key=lambda region: (
            float("inf") if region["distance_m"] is None else region["distance_m"] - (0.03 * float(region["size"]))
        )
    )
    return regions
