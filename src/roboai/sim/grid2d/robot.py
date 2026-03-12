from __future__ import annotations

import math

from roboai.core.types import Pose2D


def step_unicycle(pose: Pose2D, linear: float, angular: float, dt: float) -> Pose2D:
    theta = pose.theta + angular * dt
    x = pose.x + linear * math.cos(theta) * dt
    y = pose.y + linear * math.sin(theta) * dt
    return Pose2D(x=x, y=y, theta=theta)
