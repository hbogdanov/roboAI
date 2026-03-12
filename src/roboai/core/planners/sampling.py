from __future__ import annotations

import math

import numpy as np

from roboai.core.occupancy_grid import OCCUPIED, UNKNOWN, OccupancyGrid


def build_blocked_mask(grid: OccupancyGrid, robot_radius: float, allow_unknown: bool) -> np.ndarray:
    radius_cells = int(math.ceil(robot_radius / grid.resolution))
    blocked = grid.inflated_obstacles(radius_cells)
    if not allow_unknown:
        blocked = blocked | (grid.grid == UNKNOWN)
    return blocked


def is_world_free(grid: OccupancyGrid, blocked: np.ndarray, point_xy: tuple[float, float]) -> bool:
    gx, gy = grid.world_to_grid(*point_xy)
    if not grid.in_bounds(gx, gy):
        return False
    if blocked[gy, gx]:
        return False
    return grid.get_cell(gx, gy) != OCCUPIED


def line_is_free(
    grid: OccupancyGrid,
    blocked: np.ndarray,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    step_scale: float = 0.5,
) -> bool:
    distance = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
    steps = max(2, int(math.ceil(distance / max(grid.resolution * step_scale, 1e-6))))
    for idx in range(steps + 1):
        alpha = idx / steps
        x = start_xy[0] + alpha * (end_xy[0] - start_xy[0])
        y = start_xy[1] + alpha * (end_xy[1] - start_xy[1])
        if not is_world_free(grid, blocked, (x, y)):
            return False
    return True


def path_cost(path: list[tuple[float, float]]) -> float:
    if len(path) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(path, path[1:]):
        total += float(math.hypot(b[0] - a[0], b[1] - a[1]))
    return total
