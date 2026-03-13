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
    semantic_overlay: np.ndarray | None = None
    robot_poses: list[Pose2D] = field(default_factory=list)
    localization_uncertainty: float = 0.0
    additional_trajectories: list[list[tuple[float, float]]] = field(default_factory=list)


@dataclass(slots=True)
class RunMetrics:
    map_name: str
    planner_name: str
    planner_policy: str
    frontier_policy: str
    seed: int
    success: bool
    stop_reason: str
    steps: int
    runtime_seconds: float
    coverage: float
    path_length: float
    raw_path_length: float
    smoothed_path_length: float
    path_turn_count: int
    collisions: int
    replans: int
    replan_triggers: int
    recovery_events: int
    disturbance_name: str
    disturbance_events: int
    range_noise_std: float
    dropout_prob: float
    pose_noise_std: float
    semantic_mode: str
    final_localization_uncertainty: float
    robot_count: int
    time_to_coverage_step: int
    explored_cells: int
    known_cells: int
    explored_overlap_ratio: float = 0.0
    duplicate_frontier_assignments: int = 0
    near_conflicts: int = 0
    coverage_history: list[float] = field(default_factory=list)
