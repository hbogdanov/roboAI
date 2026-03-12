from __future__ import annotations

import math

import numpy as np

from roboai.core.planners.sampling import build_blocked_mask, is_world_free, line_is_free, path_cost
from roboai.core.types import PlannerResult


def plan_rrt(
    grid,
    start_xy,
    goal_xy,
    robot_radius,
    allow_unknown=False,
    max_iters: int = 1200,
    step_size: float | None = None,
    goal_sample_rate: float = 0.15,
):
    blocked = build_blocked_mask(grid, robot_radius, allow_unknown)
    if not is_world_free(grid, blocked, start_xy) or not is_world_free(grid, blocked, goal_xy):
        return PlannerResult(success=False)

    step = float(step_size or max(grid.resolution * 2.0, 0.35))
    bounds = (
        grid.origin[0],
        grid.origin[0] + grid.width * grid.resolution,
        grid.origin[1],
        grid.origin[1] + grid.height * grid.resolution,
    )

    nodes: list[tuple[float, float]] = [start_xy]
    parents: list[int] = [-1]

    for _ in range(max_iters):
        sample = goal_xy if np.random.random() < goal_sample_rate else _sample_free(bounds, grid, blocked)
        if sample is None:
            continue
        nearest_idx = _nearest_index(nodes, sample)
        candidate = _steer(nodes[nearest_idx], sample, step)
        if not is_world_free(grid, blocked, candidate):
            continue
        if not line_is_free(grid, blocked, nodes[nearest_idx], candidate):
            continue

        nodes.append(candidate)
        parents.append(nearest_idx)
        new_idx = len(nodes) - 1

        if math.hypot(candidate[0] - goal_xy[0], candidate[1] - goal_xy[1]) <= step:
            if line_is_free(grid, blocked, candidate, goal_xy):
                nodes.append(goal_xy)
                parents.append(new_idx)
                path = _trace_path(nodes, parents, len(nodes) - 1)
                return PlannerResult(
                    path=path,
                    success=True,
                    cost=path_cost(path),
                    nodes_expanded=len(nodes),
                )

    return PlannerResult(success=False, nodes_expanded=len(nodes))


def _sample_free(bounds, grid, blocked):
    xmin, xmax, ymin, ymax = bounds
    for _ in range(100):
        point = (
            float(np.random.uniform(xmin, xmax)),
            float(np.random.uniform(ymin, ymax)),
        )
        if is_world_free(grid, blocked, point):
            return point
    return None


def _nearest_index(nodes, sample) -> int:
    distances = [(node[0] - sample[0]) ** 2 + (node[1] - sample[1]) ** 2 for node in nodes]
    return int(np.argmin(distances))


def _steer(start_xy, target_xy, step_size: float):
    dx = target_xy[0] - start_xy[0]
    dy = target_xy[1] - start_xy[1]
    distance = math.hypot(dx, dy)
    if distance <= step_size:
        return float(target_xy[0]), float(target_xy[1])
    scale = step_size / max(distance, 1e-9)
    return float(start_xy[0] + dx * scale), float(start_xy[1] + dy * scale)


def _trace_path(nodes, parents, idx: int):
    path = []
    while idx >= 0:
        path.append(nodes[idx])
        idx = parents[idx]
    path.reverse()
    return path
