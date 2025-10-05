from dataclasses import dataclass
from typing import Optional
from config import WHEEL_RADIUS_M, AXLE_LENGTH_M
import math

@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vl: float = 0.0
    vr: float = 0.0

class StateEstimator:
    def __init__(self):
        self._last_left: Optional[float] = None
        self._last_right: Optional[float] = None
        self.state = RobotState()

    def reset_pose(self, x=0.0, y=0.0, theta=0.0):
        self.state.x, self.state.y, self.state.theta = x, y, theta

    def update(self, encoders: Optional[tuple], dt: float) -> RobotState:
        if encoders is None or dt <= 0:
            return self.state

        left, right = encoders
        if self._last_left is None or self._last_right is None:
            self._last_left, self._last_right = left, right
            return self.state

        d_left  = left  - self._last_left
        d_right = right - self._last_right
        self._last_left, self._last_right = left, right

        dl = d_left  * WHEEL_RADIUS_M
        dr = d_right * WHEEL_RADIUS_M
        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / AXLE_LENGTH_M

        th = self.state.theta + dtheta / 2.0
        self.state.x += dc * math.cos(th)
        self.state.y += dc * math.sin(th)
        self.state.theta += dtheta

        self.state.vl = d_left  / dt
        self.state.vr = d_right / dt
        return self.state
