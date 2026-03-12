from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(slots=True)
class Pose2D:
    x: float
    y: float
    theta: float


@dataclass(slots=True)
class LaserScan2D:
    angles: np.ndarray
    ranges: np.ndarray
    max_range: float


@dataclass(slots=True)
class PlannerResult:
    path: list[tuple[float, float]] = field(default_factory=list)
    success: bool = False
    cost: float = 0.0
    nodes_expanded: int = 0


@dataclass(slots=True)
class RenderFrame:
    occupancy: np.ndarray
    trajectory: list[tuple[float, float]]
    planner_path: list[tuple[float, float]]
    frontier_points: list[tuple[float, float]]
    robot_pose: Pose2D
    coverage: float


@dataclass(slots=True)
class RunMetrics:
    map_name: str
    planner_name: str
    seed: int
    success: bool
    stop_reason: str
    steps: int
    runtime_seconds: float
    coverage: float
    path_length: float
    collisions: int
    replans: int
    explored_cells: int
    known_cells: int
    coverage_history: list[float] = field(default_factory=list)
