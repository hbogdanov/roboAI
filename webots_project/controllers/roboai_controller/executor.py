from __future__ import annotations
from typing import List, Dict, Optional, Tuple, Any
from config import TIME_STEP_MS, SECS_PER_DEG, TURN_SPEED
from logger import RunLogger
from navigator import steer
from controller import Robot
from motion import Drive
from sensors import Sensors

class PlanExecutor:
    """
    Executes a simple SPA-like plan consisting of primitive ops:
      - forward(seconds)
      - turn(dir, deg)
      - scan(sensor)
      - wait(seconds)
      - return_base
      - stop

    It keeps your original behavior but adds 'wait' support and
    ensures timers reset correctly across ops.
    """

    def __init__(self, robot: "Robot", drive: Drive, sensors: Sensors, log: RunLogger):
        self.robot = robot
        self.drive = drive
        self.sensors = sensors
        self.log = log

        self.plan: List[Dict[str, Any]] = []
        self.idx = 0
        self.op_timer = 0.0
        self.turn_target_secs: Optional[float] = None
        self.last_cmd: Tuple[float, float] = (0.0, 0.0)

    # lifecycle

    def load(self, plan: List[Dict[str, Any]]):
        """Load a new plan and reset state."""
        self.plan = plan or [{"op": "stop"}]
        self.idx = 0
        self.op_timer = 0.0
        self.turn_target_secs = None
        self.last_cmd = (0.0, 0.0)
        self.log.event(op="plan_loaded", steps=len(self.plan))

    def _wait_step(self) -> bool:
        """Advance Webots simulation by one controller tick. Returns True if simulation stopped."""
        return self.robot.step(TIME_STEP_MS) == -1

    # main tick

    def step(self, dt: float, ir: List[Optional[float]]) -> bool:
        """
        Execute one SPA tick. Returns True when the plan has finished or simulation ended.
        dt: timestep seconds
        ir: list of IR sensor readings (some may be None)
        """
        if self.idx >= len(self.plan):
            self.drive.stop()
            self.last_cmd = (0.0, 0.0)
            return True

        step = self.plan[self.idx]
        op = str(step.get("op", "")).lower()

        # FORWARD
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

        # TURN
        elif op == "turn":
            # Initialize on first tick of this op
            if self.turn_target_secs is None:
                deg = abs(float(step.get("deg", 90)))
                # Convert degrees to target time
                self.turn_target_secs = max(0.0, SECS_PER_DEG * deg)
                direction = str(step.get("dir", "left")).lower()
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

        # CAN 
        elif op == "scan":
            sensor_name = str(step.get("sensor", "ir")).lower()
            # currently only 'ir' is supported
            dist = self.sensors.read_front_distance()
            self.log.event(op="scan", sensor=sensor_name, value=dist)
            self.idx += 1
            self.last_cmd = (0.0, 0.0)

        # WAIT
        elif op == "wait":
            secs = float(step.get("seconds", 1.0))
            self.drive.stop()
            self.last_cmd = (0.0, 0.0)
            self.op_timer += dt
            if self.op_timer >= secs:
                self.log.event(op="wait_done", seconds=secs)
                self.idx += 1
                self.op_timer = 0.0

        # RETURN TO BASE (stubbed)
        elif op == "return_base":
            # Currently just logs + proceeds
            self.drive.stop()
            self.log.event(op="return_base")
            self.idx += 1
            self.last_cmd = (0.0, 0.0)

        # STOP
        elif op == "stop":
            self.drive.stop()
            self.log.event(op="stop")
            self.idx = len(self.plan)
            self.last_cmd = (0.0, 0.0)
            return True

        # UNKNOWN
        else:
            self.log.event(op="unknown_step", step=step)
            self.idx += 1

        # End condition if we've consumed plan
        return self.idx >= len(self.plan)
