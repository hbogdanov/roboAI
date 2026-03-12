from __future__ import annotations

from collections import deque

import numpy as np

from roboai.core.occupancy_grid import FREE, UNKNOWN, OccupancyGrid


def frontier_cells(grid: OccupancyGrid) -> list[tuple[int, int]]:
    frontiers: list[tuple[int, int]] = []
    for gy in range(grid.height):
        for gx in range(grid.width):
            if grid.get_cell(gx, gy) != FREE:
                continue
            if any(grid.get_cell(nx, ny) == UNKNOWN for nx, ny in _neighbors4(gx, gy)):
                frontiers.append((gx, gy))
    return frontiers


def frontier_regions(grid: OccupancyGrid) -> list[list[tuple[int, int]]]:
    cells = set(frontier_cells(grid))
    regions: list[list[tuple[int, int]]] = []
    while cells:
        start = cells.pop()
        queue = deque([start])
        region = [start]
        while queue:
            gx, gy = queue.popleft()
            for nx, ny in _neighbors8(gx, gy):
                node = (nx, ny)
                if node in cells:
                    cells.remove(node)
                    queue.append(node)
                    region.append(node)
        regions.append(region)
    return regions


def select_frontier_target(grid: OccupancyGrid, robot_xy: tuple[float, float], robot_theta: float | None = None) -> tuple[float, float] | None:
    ranked = rank_frontier_targets(grid, robot_xy, robot_theta=robot_theta)
    return ranked[0] if ranked else None


def rank_frontier_targets(
    grid: OccupancyGrid,
    robot_xy: tuple[float, float],
    robot_theta: float | None = None,
) -> list[tuple[float, float]]:
    regions = frontier_regions(grid)
    if not regions:
        return []

    rx, ry = robot_xy
    scored_targets: list[tuple[float, tuple[float, float]]] = []
    for region in regions:
        points = np.asarray([grid.grid_to_world(gx, gy) for gx, gy in region], dtype=float)
        centroid = points.mean(axis=0)
        representative = min(
            points,
            key=lambda point: float(np.hypot(point[0] - centroid[0], point[1] - centroid[1])),
        )
        distance = float(np.hypot(representative[0] - rx, representative[1] - ry))
        region_size = float(len(region))
        heading_penalty = 0.0
        if robot_theta is not None:
            target_heading = float(np.arctan2(representative[1] - ry, representative[0] - rx))
            heading_error = _wrap_angle(target_heading - float(robot_theta))
            heading_penalty = abs(heading_error) * 0.75
        score = distance + heading_penalty - 0.15 * region_size
        scored_targets.append((score, (float(representative[0]), float(representative[1]))))
    scored_targets.sort(key=lambda item: item[0])
    return [target for _, target in scored_targets]


def _neighbors4(gx: int, gy: int):
    yield gx + 1, gy
    yield gx - 1, gy
    yield gx, gy + 1
    yield gx, gy - 1


def _neighbors8(gx: int, gy: int):
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            yield gx + dx, gy + dy


def _wrap_angle(angle: float) -> float:
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return float(angle)
