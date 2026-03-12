from __future__ import annotations

import math

import numpy as np

from roboai.core.types import LaserScan2D, Pose2D


class SimulatedLidar:
    def __init__(self, num_beams: int = 91, max_range: float = 6.0, step_size: float = 0.05):
        self.num_beams = int(num_beams)
        self.max_range = float(max_range)
        self.step_size = float(step_size)
        self.angles = np.linspace(-math.pi, math.pi, self.num_beams, endpoint=False)

    def scan(self, env, pose: Pose2D) -> LaserScan2D:
        ranges = np.asarray([self._cast_ray(env, pose, angle) for angle in self.angles], dtype=float)
        return LaserScan2D(angles=self.angles.copy(), ranges=ranges, max_range=self.max_range)

    def _cast_ray(self, env, pose: Pose2D, rel_angle: float) -> float:
        angle = pose.theta + rel_angle
        distance = 0.0
        while distance < self.max_range:
            x = pose.x + distance * math.cos(angle)
            y = pose.y + distance * math.sin(angle)
            if env.is_occupied_world(x, y):
                return distance
            distance += self.step_size
        return self.max_range
