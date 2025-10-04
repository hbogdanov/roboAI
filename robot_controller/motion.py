from typing import Tuple
from controller import Robot
from .config import TIME_STEP_MS, FORWARD_SPEED, TURN_SPEED, SECS_PER_DEG

class Drive:
    def __init__(self, robot: Robot, left_name: str, right_name: str):
        self.robot = robot
        self.left = robot.getDevice(left_name)
        self.right = robot.getDevice(right_name)
        self.left.setPosition(float("inf"))
        self.right.setPosition(float("inf"))
        self.stop()

    def step(self, ms: int) -> int:
        return self.robot.step(ms)

    def set_velocity(self, lv: float, rv: float):
        self.left.setVelocity(lv)
        self.right.setVelocity(rv)

    def stop(self):
        self.set_velocity(0.0, 0.0)

    def wait_seconds(self, seconds: float):
        steps = max(1, int((seconds * 1000) / TIME_STEP_MS))
        for _ in range(steps):
            if self.step(TIME_STEP_MS) == -1:
                break

    def forward(self, seconds: float, speed: float = FORWARD_SPEED):
        self.set_velocity(speed, speed)
        self.wait_seconds(seconds)
        self.stop()

    def turn_left_deg(self, deg: float, speed: float = TURN_SPEED):
        secs = max(0.0, SECS_PER_DEG * deg)
        self.set_velocity(-speed, speed)
        self.wait_seconds(secs)
        self.stop()

    def turn_right_deg(self, deg: float, speed: float = TURN_SPEED):
        secs = max(0.0, SECS_PER_DEG * deg)
        self.set_velocity(speed, -speed)
        self.wait_seconds(secs)
        self.stop()