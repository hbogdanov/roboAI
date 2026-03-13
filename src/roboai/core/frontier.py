from __future__ import annotations

from collections import deque

import numpy as np

from roboai.core.frontier_model import score_frontier
from roboai.core.occupancy_grid import FREE, UNKNOWN, OccupancyGrid
from roboai.sim.grid2d.maps import SEMANTIC_BEACON, SEMANTIC_DESK, SEMANTIC_DOOR, SEMANTIC_EXIT, SEMANTIC_PERSON


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
    policy: str = "naive",
    blocked_targets: list[tuple[float, float]] | None = None,
    revisit_counts: dict[tuple[int, int], int] | None = None,
    blocked_radius: float = 0.75,
    semantic_grid: np.ndarray | None = None,
    localization_uncertainty: float = 0.0,
    learned_weights: dict[str, float] | None = None,
) -> list[tuple[float, float]]:
    regions = frontier_regions(grid)
    if not regions:
        return []

    rx, ry = robot_xy
    scored_targets: list[tuple[float, tuple[float, float]]] = []
    blocked_targets = blocked_targets or []
    revisit_counts = revisit_counts or {}
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
        blocked_penalty = _blocked_penalty(representative, blocked_targets, radius=blocked_radius)
        revisit_penalty = float(revisit_counts.get(grid.world_to_grid(*representative), 0)) * 0.6
        features = _frontier_features(
            grid=grid,
            representative=(float(representative[0]), float(representative[1])),
            distance=distance,
            heading_penalty=heading_penalty,
            region_size=region_size,
            blocked_penalty=blocked_penalty,
            revisit_penalty=revisit_penalty,
            semantic_grid=semantic_grid,
            localization_uncertainty=localization_uncertainty,
        )
        if policy == "learned_linear":
            score = -score_frontier(features, learned_weights)
        elif policy in {"information_gain", "semantic_information_gain"}:
            score = _information_gain_score(features)
        else:
            score = distance + heading_penalty - 0.15 * region_size + blocked_penalty + revisit_penalty
        scored_targets.append((score, (float(representative[0]), float(representative[1]))))
    scored_targets.sort(key=lambda item: item[0])
    return [target for _, target in scored_targets]


def _frontier_features(
    grid: OccupancyGrid,
    representative: tuple[float, float],
    distance: float,
    heading_penalty: float,
    region_size: float,
    blocked_penalty: float,
    revisit_penalty: float,
    semantic_grid: np.ndarray | None,
    localization_uncertainty: float,
) -> dict[str, float]:
    gx, gy = grid.world_to_grid(*representative)
    semantic_value = _semantic_value(semantic_grid, gx, gy)
    beacon_bonus = _beacon_bonus(semantic_grid, gx, gy)
    uncertainty_penalty = localization_uncertainty * max(distance - beacon_bonus, 0.0)
    return {
        "bias": 1.0,
        "distance": distance,
        "heading_penalty": heading_penalty,
        "region_size": region_size,
        "info_gain": float(_unknown_neighbors_within_radius(grid, gx, gy, radius_cells=6)),
        "semantic_value": semantic_value,
        "blocked_penalty": blocked_penalty,
        "revisit_penalty": revisit_penalty,
        "uncertainty_penalty": uncertainty_penalty,
        "beacon_bonus": beacon_bonus,
    }


def _information_gain_score(features: dict[str, float]) -> float:
    return (
        features["distance"]
        + features["heading_penalty"]
        + features["blocked_penalty"]
        + features["revisit_penalty"]
        + 0.55 * features["uncertainty_penalty"]
        - 0.12 * features["region_size"]
        - 0.42 * features["info_gain"]
        - 0.35 * features["semantic_value"]
        - 0.25 * features["beacon_bonus"]
    )


def _unknown_neighbors_within_radius(grid: OccupancyGrid, gx: int, gy: int, radius_cells: int) -> int:
    total = 0
    for ny in range(max(0, gy - radius_cells), min(grid.height, gy + radius_cells + 1)):
        for nx in range(max(0, gx - radius_cells), min(grid.width, gx + radius_cells + 1)):
            if grid.get_cell(nx, ny) == UNKNOWN:
                total += 1
    return total


def _blocked_penalty(representative: np.ndarray, blocked_targets: list[tuple[float, float]], radius: float) -> float:
    penalty = 0.0
    for bx, by in blocked_targets:
        distance = float(np.hypot(float(representative[0]) - bx, float(representative[1]) - by))
        if distance <= radius:
            penalty += 4.0
    return penalty


def _semantic_value(semantic_grid: np.ndarray | None, gx: int, gy: int) -> float:
    if semantic_grid is None:
        return 0.0
    x0 = max(0, gx - 4)
    x1 = min(semantic_grid.shape[1], gx + 5)
    y0 = max(0, gy - 4)
    y1 = min(semantic_grid.shape[0], gy + 5)
    local = semantic_grid[y0:y1, x0:x1]
    weights = {
        SEMANTIC_DOOR: 1.0,
        SEMANTIC_DESK: 0.7,
        SEMANTIC_EXIT: 1.2,
        SEMANTIC_PERSON: 0.8,
    }
    total = 0.0
    for semantic_id, weight in weights.items():
        total += weight * float(np.count_nonzero(local == semantic_id))
    return total


def _beacon_bonus(semantic_grid: np.ndarray | None, gx: int, gy: int) -> float:
    if semantic_grid is None:
        return 0.0
    ys, xs = np.nonzero(semantic_grid == SEMANTIC_BEACON)
    if len(xs) == 0:
        return 0.0
    min_distance = min(float(np.hypot(gx - x, gy - y)) for x, y in zip(xs, ys))
    return max(0.0, 6.0 - min_distance) / 6.0


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
