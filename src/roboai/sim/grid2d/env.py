from __future__ import annotations

import math

import numpy as np

from roboai.core.types import Pose2D
from roboai.sim.grid2d.robot import step_unicycle


class Grid2DEnv:
    def __init__(self, obstacle_grid: np.ndarray, resolution: float = 0.2, robot_radius: float = 0.25):
        self.obstacle_grid = np.asarray(obstacle_grid, dtype=bool)
        self.resolution = float(resolution)
        self.robot_radius = float(robot_radius)
        self.height, self.width = self.obstacle_grid.shape
        self.pose = Pose2D(x=self.resolution * 2.0, y=self.resolution * 2.0, theta=0.0)
        self.trajectory = [(self.pose.x, self.pose.y)]

    def reset(self, pose: Pose2D) -> Pose2D:
        if self.collides(pose):
            raise ValueError("Initial pose is in collision.")
        self.pose = pose
        self.trajectory = [(pose.x, pose.y)]
        return self.pose

    def step(self, linear: float, angular: float, dt: float) -> tuple[Pose2D, bool]:
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
                if self.obstacle_grid[ny, nx]:
                    return True
        return False

    def is_occupied_world(self, x: float, y: float) -> bool:
        gx, gy = self.world_to_grid(x, y)
        if not self.in_bounds(gx, gy):
            return True
        return bool(self.obstacle_grid[gy, gx])

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        return int(x / self.resolution), int(y / self.resolution)

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        return (gx + 0.5) * self.resolution, (gy + 0.5) * self.resolution

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def free_cells(self) -> int:
        return int(np.count_nonzero(~self.obstacle_grid))
