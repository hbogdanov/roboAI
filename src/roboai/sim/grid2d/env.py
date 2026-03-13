from __future__ import annotations

import math

import numpy as np

from roboai.core.types import Pose2D
from roboai.sim.grid2d.robot import step_unicycle


class Grid2DEnv:
    def __init__(
        self,
        obstacle_grid: np.ndarray,
        resolution: float = 0.2,
        robot_radius: float = 0.25,
        disturbance_name: str = "none",
    ):
        self.obstacle_grid = np.asarray(obstacle_grid, dtype=bool)
        self.resolution = float(resolution)
        self.robot_radius = float(robot_radius)
        self.height, self.width = self.obstacle_grid.shape
        self.disturbance_name = disturbance_name
        self.dynamic_obstacle_grid = np.zeros_like(self.obstacle_grid, dtype=bool)
        self.pose = Pose2D(x=self.resolution * 2.0, y=self.resolution * 2.0, theta=0.0)
        self.trajectory = [(self.pose.x, self.pose.y)]
        self.step_count = 0
        self.disturbance_events = 0
        self._disturbance_was_active = False
        self._update_dynamic_obstacles()

    def reset(self, pose: Pose2D) -> Pose2D:
        if self.collides(pose):
            raise ValueError("Initial pose is in collision.")
        self.pose = pose
        self.trajectory = [(pose.x, pose.y)]
        self.step_count = 0
        self.disturbance_events = 0
        self._disturbance_was_active = False
        self._update_dynamic_obstacles()
        return self.pose

    def step(self, linear: float, angular: float, dt: float) -> tuple[Pose2D, bool]:
        self.step_count += 1
        self._update_dynamic_obstacles()
        proposal = step_unicycle(self.pose, linear, angular, dt)
        collision = self.collides(proposal)
        if not collision:
            self.pose = proposal
            self.trajectory.append((self.pose.x, self.pose.y))
        return self.pose, collision

    def collides(self, pose: Pose2D) -> bool:
        radius_cells = int(math.ceil(self.robot_radius / self.resolution))
        gx, gy = self.world_to_grid(pose.x, pose.y)
        for oy in range(-radius_cells, radius_cells + 1):
            for ox in range(-radius_cells, radius_cells + 1):
                nx, ny = gx + ox, gy + oy
                if not self.in_bounds(nx, ny):
                    return True
                if self.combined_obstacle_grid[ny, nx]:
                    return True
        return False

    def is_occupied_world(self, x: float, y: float) -> bool:
        gx, gy = self.world_to_grid(x, y)
        if not self.in_bounds(gx, gy):
            return True
        return bool(self.combined_obstacle_grid[gy, gx])

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        return int(x / self.resolution), int(y / self.resolution)

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        return (gx + 0.5) * self.resolution, (gy + 0.5) * self.resolution

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def free_cells(self) -> int:
        return int(np.count_nonzero(~self.combined_obstacle_grid))

    @property
    def combined_obstacle_grid(self) -> np.ndarray:
        return np.logical_or(self.obstacle_grid, self.dynamic_obstacle_grid)

    def _update_dynamic_obstacles(self) -> None:
        self.dynamic_obstacle_grid.fill(False)
        active = False
        if self.disturbance_name == "moving_obstacle":
            active = self._apply_moving_obstacle()
        elif self.disturbance_name == "temporary_block":
            active = self._apply_temporary_block()
        if active and not self._disturbance_was_active:
            self.disturbance_events += 1
        self._disturbance_was_active = active

    def _apply_moving_obstacle(self) -> bool:
        half = max(1, int(round(0.35 / self.resolution)))
        travel = max(half + 2, self.width - half - 3)
        t = self.step_count % max(1, 2 * travel)
        center_x = half + 2 + (t if t < travel else 2 * travel - t)
        center_y = max(half + 2, self.height // 2)
        self._paint_square(center_x, center_y, half)
        return True

    def _apply_temporary_block(self) -> bool:
        if not 30 <= self.step_count <= 110:
            return False
        band_x = self.width // 2
        gap_y = self.height // 2
        thickness = max(1, int(round(0.2 / self.resolution)))
        for gx in range(max(0, band_x - thickness), min(self.width, band_x + thickness + 1)):
            for gy in range(1, self.height - 1):
                if abs(gy - gap_y) <= 1:
                    continue
                self.dynamic_obstacle_grid[gy, gx] = True
        return True

    def _paint_square(self, cx: int, cy: int, half: int) -> None:
        x0 = max(0, cx - half)
        x1 = min(self.width, cx + half + 1)
        y0 = max(0, cy - half)
        y1 = min(self.height, cy + half + 1)
        self.dynamic_obstacle_grid[y0:y1, x0:x1] = True
