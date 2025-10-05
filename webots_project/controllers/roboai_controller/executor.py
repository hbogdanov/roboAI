from typing import List, Dict, Optional, TYPE_CHECKING, Tuple
from config import TIME_STEP_MS, SECS_PER_DEG, TURN_SPEED
from logger import RunLogger
from navigator import steer
from controller import Robot
from motion import Drive
from sensors import Sensors

class PlanExecutor:
    def __init__(self, robot: "Robot", drive: Drive, sensors: Sensors, log: RunLogger):
        self.robot = robot
        self.drive = drive
        self.sensors = sensors
        self.log = log
        self.plan: List[Dict] = []
        self.idx = 0
        self.op_timer = 0.0
        self.turn_target_secs: Optional[float] = None
        self.last_cmd: Tuple[float, float] = (0.0, 0.0)

    def load(self, plan: List[Dict]):
        self.plan = plan or [{"op": "stop"}]
        self.idx = 0
        self.op_timer = 0.0
        self.turn_target_secs = None

    def _wait_step(self) -> bool:
        return self.robot.step(TIME_STEP_MS) == -1

    def step(self, dt: float, ir: List[Optional[float]]) -> bool:
        """One SPA tick. Returns True when finished."""
        if self.idx >= len(self.plan):
            self.drive.stop()
            self.last_cmd = (0.0, 0.0)
            return True

        step = self.plan[self.idx]
        op = step.get("op")

        if op == "forward":
            secs = float(step.get("seconds", 1.0))
            l, r, front = steer(ir)
            self.drive.set_velocity(l, r)
            self.last_cmd = (l, r)
            self.log.event(op="spa_forward_tick", l=l, r=r, front=front)
            self.op_timer += dt
            if self.op_timer >= secs:
                self.drive.stop()
                self.log.event(op="forward_done", seconds=secs)
                self.idx += 1
                self.op_timer = 0.0

        elif op == "turn":
            if self.turn_target_secs is None:
                deg = float(step.get("deg", 90))
                self.turn_target_secs = max(0.0, SECS_PER_DEG * deg)
                direction = step.get("dir", "left")
                if direction == "left":
                    self.drive.set_velocity(-TURN_SPEED, TURN_SPEED)
                    self.last_cmd = (-TURN_SPEED, TURN_SPEED)
                else:
                    self.drive.set_velocity(TURN_SPEED, -TURN_SPEED)
                    self.last_cmd = (TURN_SPEED, -TURN_SPEED)
                self.log.event(op="turn_start", dir=direction, deg=deg, secs=self.turn_target_secs)

            self.op_timer += dt
            if self.op_timer >= (self.turn_target_secs or 0.0):
                self.drive.stop()
                self.log.event(op="turn_done")
                self.idx += 1
                self.op_timer = 0.0
                self.turn_target_secs = None

        elif op == "scan":
            dist = self.sensors.read_front_distance()
            self.log.event(op="scan", sensor=step.get("sensor", "ir"), value=dist)
            self.idx += 1  
            self.last_cmd = (0.0, 0.0)

        elif op == "return_base":
            self.drive.stop()
            self.log.event(op="return_base")
            self.idx += 1
            self.last_cmd = (0.0, 0.0)

        elif op == "stop":
            self.drive.stop()
            self.log.event(op="stop")
            self.idx = len(self.plan)
            self.last_cmd = (0.0, 0.0)
            return True

        else:
            self.log.event(op="unknown_step", step=step)
            self.idx += 1

        return self.idx >= len(self.plan)
