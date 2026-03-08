from __future__ import annotations

from typing import List, Tuple, Dict, Optional, Any
import heapq
import math


GridPt = Tuple[int, int]


def _neighbors8(x: int, y: int) -> List[Tuple[int, int, float]]:
    return [
        (x + 1, y, 1.0),
        (x - 1, y, 1.0),
        (x, y + 1, 1.0),
        (x, y - 1, 1.0),
        (x + 1, y + 1, math.sqrt(2.0)),
        (x + 1, y - 1, math.sqrt(2.0)),
        (x - 1, y + 1, math.sqrt(2.0)),
        (x - 1, y - 1, math.sqrt(2.0)),
    ]


def _heur(a: GridPt, b: GridPt) -> float:
    return math.hypot(float(a[0] - b[0]), float(a[1] - b[1]))


def _is_inside(gx: int, gy: int, w: int, h: int) -> bool:
    return 0 <= gx < w and 0 <= gy < h


def _make_occupancy_mask(
    prob_grid,
    occ_thresh: float = 0.65,
    free_thresh: float = 0.45,
    unknown_band: float = 0.06,
    block_unknown: bool = True,
    inflate_cells: int = 2,
):
    """
    Build a non-traversable mask from occupancy probabilities.

    Conservative default:
      - free if p <= free_thresh
      - occupied if p >= occ_thresh
      - unknown near 0.5 is blocked when block_unknown=True
      - mid-confidence cells are also treated as blocked
    """
    occupied = prob_grid >= occ_thresh
    unknown = abs(prob_grid - 0.5) <= unknown_band
    free = prob_grid <= free_thresh

    traversable = free.copy()
    if block_unknown:
        traversable &= ~unknown

    # Non-traversable includes occupied/unknown/mid-confidence cells.
    occ_mask = ~traversable

    if inflate_cells <= 0:
        return occ_mask

    # Inflate only hard occupied cells for safety margin.
    h, w = occupied.shape
    out = occ_mask.copy()
    for gy in range(h):
        for gx in range(w):
            if not occupied[gy, gx]:
                continue
            for dy in range(-inflate_cells, inflate_cells + 1):
                for dx in range(-inflate_cells, inflate_cells + 1):
                    ny, nx = gy + dy, gx + dx
                    if 0 <= ny < h and 0 <= nx < w:
                        out[ny, nx] = True
    return out


def _nearest_free(start: GridPt, occ_mask) -> GridPt:
    h, w = occ_mask.shape
    sx, sy = start
    if _is_inside(sx, sy, w, h) and not occ_mask[sy, sx]:
        return start

    best = start
    best_d = float("inf")
    max_r = max(w, h)
    for r in range(1, max_r):
        x0, x1 = sx - r, sx + r
        y0, y1 = sy - r, sy + r
        for x in range(x0, x1 + 1):
            for y in (y0, y1):
                if _is_inside(x, y, w, h) and not occ_mask[y, x]:
                    d = _heur((sx, sy), (x, y))
                    if d < best_d:
                        best_d = d
                        best = (x, y)
        for y in range(y0, y1 + 1):
            for x in (x0, x1):
                if _is_inside(x, y, w, h) and not occ_mask[y, x]:
                    d = _heur((sx, sy), (x, y))
                    if d < best_d:
                        best_d = d
                        best = (x, y)
        if best_d < float("inf"):
            return best
    return best


def astar_grid(start: GridPt, goal: GridPt, occ_mask) -> List[GridPt]:
    h, w = occ_mask.shape
    sx, sy = start
    gx, gy = goal
    if not _is_inside(sx, sy, w, h) or not _is_inside(gx, gy, w, h):
        return []

    frontier: List[Tuple[float, GridPt]] = []
    heapq.heappush(frontier, (0.0, start))
    came_from: Dict[GridPt, Optional[GridPt]] = {start: None}
    g_cost: Dict[GridPt, float] = {start: 0.0}

    while frontier:
        _, cur = heapq.heappop(frontier)
        if cur == goal:
            break
        cx, cy = cur
        for nx, ny, step_cost in _neighbors8(cx, cy):
            if not _is_inside(nx, ny, w, h):
                continue
            if occ_mask[ny, nx]:
                continue
            nxt = (nx, ny)
            new_cost = g_cost[cur] + step_cost
            if nxt not in g_cost or new_cost < g_cost[nxt]:
                g_cost[nxt] = new_cost
                f = new_cost + _heur(nxt, goal)
                heapq.heappush(frontier, (f, nxt))
                came_from[nxt] = cur

    if goal not in came_from:
        return []

    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = came_from[cur]
    path.reverse()
    return path


def plan_world_path(
    occ_grid,
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    block_unknown: bool = True,
    inflate_cells: int = 2,
    goal_clearance_cells: int = 0,
    return_meta: bool = False,
) -> Any:
    """
    Compute an A* path in occupancy grid and return world-space waypoints.
    Returns [] when planning fails.
    When return_meta=True, returns (path_world, meta_dict).
    """
    sx, sy = occ_grid.world_to_grid(start_xy[0], start_xy[1])
    gx, gy = occ_grid.world_to_grid(goal_xy[0], goal_xy[1])

    prob = occ_grid.prob()
    occ_mask = _make_occupancy_mask(
        prob_grid=prob,
        occ_thresh=0.65,
        free_thresh=0.45,
        unknown_band=0.06,
        block_unknown=block_unknown,
        inflate_cells=inflate_cells,
    )
    s_free = _nearest_free((sx, sy), occ_mask)

    # For goal safety, optionally require extra clearance from occupied cells.
    if goal_clearance_cells > 0:
        goal_mask = _make_occupancy_mask(
            prob_grid=prob,
            occ_thresh=0.65,
            free_thresh=0.45,
            unknown_band=0.06,
            block_unknown=block_unknown,
            inflate_cells=inflate_cells + goal_clearance_cells,
        )
        g_free = _nearest_free((gx, gy), goal_mask)
    else:
        g_free = _nearest_free((gx, gy), occ_mask)

    snapped = (g_free != (gx, gy))
    path_grid = astar_grid(s_free, g_free, occ_mask)
    if not path_grid:
        if return_meta:
            return [], {
                "snapped_goal": snapped,
                "goal_grid_raw": (gx, gy),
                "goal_grid_used": g_free,
            }
        return []

    path_world = [occ_grid.grid_to_world(px, py) for (px, py) in path_grid]
    if return_meta:
        return path_world, {
            "snapped_goal": snapped,
            "goal_grid_raw": (gx, gy),
            "goal_grid_used": g_free,
        }
    return path_world
