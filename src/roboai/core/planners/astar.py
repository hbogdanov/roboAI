from __future__ import annotations

import heapq
import math

from roboai.core.occupancy_grid import OCCUPIED, UNKNOWN, OccupancyGrid
from roboai.core.types import PlannerResult


def plan_astar(
    grid: OccupancyGrid,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    robot_radius: float,
    allow_unknown: bool = False,
) -> PlannerResult:
    start = grid.world_to_grid(*start_xy)
    goal = grid.world_to_grid(*goal_xy)
    if not grid.in_bounds(*start) or not grid.in_bounds(*goal):
        return PlannerResult(success=False)

    radius_cells = int(math.ceil(robot_radius / grid.resolution))
    blocked = grid.inflated_obstacles(radius_cells)
    if blocked[start[1], start[0]] or blocked[goal[1], goal[0]]:
        return PlannerResult(success=False)

    open_heap = [(0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start: 0.0}
    nodes_expanded = 0

    while open_heap:
        _, current = heapq.heappop(open_heap)
        nodes_expanded += 1
        if current == goal:
            path = _reconstruct_path(grid, came_from, current)
            return PlannerResult(
                path=path,
                success=True,
                cost=g_score[current],
                nodes_expanded=nodes_expanded,
            )

        for neighbor, step_cost in _neighbors8(current):
            nx, ny = neighbor
            if not grid.in_bounds(nx, ny):
                continue
            if blocked[ny, nx]:
                continue
            cell = grid.get_cell(nx, ny)
            if cell == OCCUPIED or (cell == UNKNOWN and not allow_unknown):
                continue
            tentative = g_score[current] + step_cost
            if tentative >= g_score.get(neighbor, float("inf")):
                continue
            came_from[neighbor] = current
            g_score[neighbor] = tentative
            heapq.heappush(open_heap, (tentative + _heuristic(neighbor, goal), neighbor))

    return PlannerResult(success=False, nodes_expanded=nodes_expanded)


def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
    return float(math.hypot(a[0] - b[0], a[1] - b[1]))


def _neighbors8(node: tuple[int, int]):
    x, y = node
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            yield (x + dx, y + dy), (math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0)


def _reconstruct_path(grid: OccupancyGrid, came_from: dict[tuple[int, int], tuple[int, int]], current: tuple[int, int]):
    cells = [current]
    while current in came_from:
        current = came_from[current]
        cells.append(current)
    cells.reverse()
    return [grid.grid_to_world(gx, gy) for gx, gy in cells]
