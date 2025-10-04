from typing import Optional
from controller import Robot
from .config import TIME_STEP_MS, FRONT_DISTANCE_SENSOR_NAME

class Sensors:
    def __init__(self, robot: Robot):
        self.robot = robot
        self.front_ds = None
        if FRONT_DISTANCE_SENSOR_NAME:
            try:
                self.front_ds = robot.getDevice(FRONT_DISTANCE_SENSOR_NAME)
                self.front_ds.enable(TIME_STEP_MS)
            except Exception:
                self.front_ds = None

    def read_front_distance(self) -> Optional[float]:
        if self.front_ds is None:
            return None
        self.robot.step(0)
        return float(self.front_ds.getValue())