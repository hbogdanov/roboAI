from typing import List, Optional, TYPE_CHECKING
from config import TIME_STEP_MS, EPUCK_IR_NAMES, LEFT_MOTOR_NAME, RIGHT_MOTOR_NAME
from controller import Robot

class Sensors:
    def __init__(self, robot: "Robot"):
        self.robot = robot

        # IR array
        self.ir = []
        for name in EPUCK_IR_NAMES:
            try:
                dev = robot.getDevice(name)
                dev.enable(TIME_STEP_MS)
                self.ir.append(dev)
            except Exception:
                self.ir.append(None)

        # Motors + encoders
        self.left_motor  = robot.getDevice(LEFT_MOTOR_NAME)
        self.right_motor = robot.getDevice(RIGHT_MOTOR_NAME)
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

        self.left_enc = None
        self.right_enc = None
        try:
            self.left_enc = self.left_motor.getPositionSensor()
            self.left_enc.enable(TIME_STEP_MS)
        except Exception:
            pass
        try:
            self.right_enc = self.right_motor.getPositionSensor()
            self.right_enc.enable(TIME_STEP_MS)
        except Exception:
            pass

    def read_ir(self) -> List[Optional[float]]:
        return [float(d.getValue()) if d is not None else None for d in self.ir]

    def read_encoders(self) -> Optional[tuple]:
        if self.left_enc is None or self.right_enc is None:
            return None
        return float(self.left_enc.getValue()), float(self.right_enc.getValue())

    def read_front_distance(self) -> Optional[float]:
        return float(self.ir[0].getValue()) if (self.ir and self.ir[0] is not None) else None
