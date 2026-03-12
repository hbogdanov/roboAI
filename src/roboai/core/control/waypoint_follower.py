from __future__ import annotations

import math

from roboai.core.types import Pose2D


class WaypointFollower:
    def __init__(self, linear_speed: float = 0.55, angular_gain: float = 2.8, waypoint_tolerance: float = 0.2):
        self.linear_speed = float(linear_speed)
        self.angular_gain = float(angular_gain)
        self.waypoint_tolerance = float(waypoint_tolerance)
        self.path: list[tuple[float, float]] = []
        self.index = 0

    def set_path(self, path: list[tuple[float, float]]) -> None:
        self.path = list(path)
        self.index = 0

    def is_done(self) -> bool:
        return self.index >= len(self.path)

    def command(self, pose: Pose2D) -> tuple[float, float]:
        if self.is_done():
            return 0.0, 0.0

        target = self.path[self.index]
        dx = target[0] - pose.x
        dy = target[1] - pose.y
        distance = math.hypot(dx, dy)
        if distance <= self.waypoint_tolerance:
            self.index += 1
            return self.command(pose)

        heading = math.atan2(dy, dx)
        error = _wrap_angle(heading - pose.theta)
        if abs(error) > 0.4:
            return 0.0, max(-2.0, min(2.0, self.angular_gain * error))

        linear = self.linear_speed * max(0.15, 1.0 - min(abs(error), math.pi) / math.pi)
        angular = max(-2.0, min(2.0, self.angular_gain * error))
        return linear, angular


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
