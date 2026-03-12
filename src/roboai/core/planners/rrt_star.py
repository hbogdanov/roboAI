from __future__ import annotations

import math

import numpy as np

from roboai.core.planners.rrt import _nearest_index, _sample_free, _steer, _trace_path
from roboai.core.planners.sampling import build_blocked_mask, is_world_free, line_is_free, path_cost
from roboai.core.types import PlannerResult


def plan_rrt_star(
    grid,
    start_xy,
    goal_xy,
    robot_radius,
    allow_unknown=False,
    max_iters: int = 1500,
    step_size: float | None = None,
    goal_sample_rate: float = 0.15,
    rewire_radius: float | None = None,
):
    blocked = build_blocked_mask(grid, robot_radius, allow_unknown)
    if not is_world_free(grid, blocked, start_xy) or not is_world_free(grid, blocked, goal_xy):
        return PlannerResult(success=False)

    step = float(step_size or max(grid.resolution * 2.0, 0.35))
    radius = float(rewire_radius or max(step * 2.5, 0.8))
    bounds = (
        grid.origin[0],
        grid.origin[0] + grid.width * grid.resolution,
        grid.origin[1],
        grid.origin[1] + grid.height * grid.resolution,
    )

    nodes: list[tuple[float, float]] = [start_xy]
    parents: list[int] = [-1]
    costs: list[float] = [0.0]

    best_goal_idx: int | None = None
    best_goal_cost = float("inf")

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

        near_indices = _near_indices(nodes, candidate, radius)
        best_parent = nearest_idx
        best_cost = costs[nearest_idx] + _distance(nodes[nearest_idx], candidate)
        for idx in near_indices:
            if not line_is_free(grid, blocked, nodes[idx], candidate):
                continue
            cost = costs[idx] + _distance(nodes[idx], candidate)
            if cost < best_cost:
                best_parent = idx
                best_cost = cost

        nodes.append(candidate)
        parents.append(best_parent)
        costs.append(best_cost)
        new_idx = len(nodes) - 1

        for idx in near_indices:
            if idx == best_parent:
                continue
            if not line_is_free(grid, blocked, candidate, nodes[idx]):
                continue
            new_cost = costs[new_idx] + _distance(candidate, nodes[idx])
            if new_cost < costs[idx]:
                parents[idx] = new_idx
                costs[idx] = new_cost

        if _distance(candidate, goal_xy) <= step and line_is_free(grid, blocked, candidate, goal_xy):
            goal_cost = costs[new_idx] + _distance(candidate, goal_xy)
            if goal_cost < best_goal_cost:
                best_goal_cost = goal_cost
                best_goal_idx = new_idx

    if best_goal_idx is None:
        return PlannerResult(success=False, nodes_expanded=len(nodes))

    nodes.append(goal_xy)
    parents.append(best_goal_idx)
    path = _trace_path(nodes, parents, len(nodes) - 1)
    return PlannerResult(
        path=path,
        success=True,
        cost=path_cost(path),
        nodes_expanded=len(nodes),
    )


def _near_indices(nodes, candidate, radius: float) -> list[int]:
    radius_sq = radius * radius
    return [
        idx for idx, node in enumerate(nodes)
        if (node[0] - candidate[0]) ** 2 + (node[1] - candidate[1]) ** 2 <= radius_sq
    ]


def _distance(a, b) -> float:
    return float(math.hypot(a[0] - b[0], a[1] - b[1]))
