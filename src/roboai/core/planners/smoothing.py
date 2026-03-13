from __future__ import annotations

import math

from roboai.core.occupancy_grid import OccupancyGrid
from roboai.core.planners.sampling import build_blocked_mask, line_is_free, path_cost


def shortcut_smooth_path(
    grid: OccupancyGrid,
    path: list[tuple[float, float]],
    robot_radius: float,
    allow_unknown: bool = False,
) -> list[tuple[float, float]]:
    if len(path) <= 2:
        return list(path)

    blocked = build_blocked_mask(grid, robot_radius, allow_unknown)
    smoothed = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        next_idx = len(path) - 1
        while next_idx > anchor + 1:
            if line_is_free(grid, blocked, path[anchor], path[next_idx]):
                break
            next_idx -= 1
        smoothed.append(path[next_idx])
        anchor = next_idx
    return smoothed


def path_turn_count(path: list[tuple[float, float]], angle_threshold_deg: float = 20.0) -> int:
    if len(path) < 3:
        return 0
    threshold = math.radians(angle_threshold_deg)
    turns = 0
    for a, b, c in zip(path, path[1:], path[2:]):
        heading_ab = math.atan2(b[1] - a[1], b[0] - a[0])
        heading_bc = math.atan2(c[1] - b[1], c[0] - b[0])
        delta = _wrap_angle(heading_bc - heading_ab)
        if abs(delta) >= threshold:
            turns += 1
    return turns


def path_quality(path: list[tuple[float, float]]) -> tuple[float, int]:
    return path_cost(path), path_turn_count(path)


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
