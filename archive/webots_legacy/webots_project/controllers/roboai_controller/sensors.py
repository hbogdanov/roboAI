from typing import List, Optional, TYPE_CHECKING
import math
from config import TIME_STEP_MS, EPUCK_IR_NAMES, LEFT_MOTOR_NAME, RIGHT_MOTOR_NAME, GPS_NAME, COMPASS_NAME
from controller import Robot
from controller import Lidar as _WebotsLidar
from controller import Camera as _WebotsCamera
from controller import GPS as _WebotsGPS
from controller import Compass as _WebotsCompass

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

class LidarWrapper:
    """
    Simple 2D lidar reader for Webots.
    Returns (ranges:list[float], angle_min:float, angle_inc:float, range_max:float).
    """
    def __init__(self, robot, name="LDS-01", timestep=32, enable_pointcloud=False):
        self._lidar: _WebotsLidar = robot.getDevice(name)
        self._lidar.enable(timestep)
        if enable_pointcloud:
            self._lidar.enablePointCloud()
        self.fov = self._lidar.getFov()
        self.res = self._lidar.getHorizontalResolution()
        self.range_max = self._lidar.getMaxRange()

    def read_scan(self):
        ranges = list(self._lidar.getRangeImage())
        angle_min = -self.fov / 2.0
        angle_inc = self.fov / max(1, (self.res - 1))
        return ranges, angle_min, angle_inc, self.range_max


class CameraWrapper:
    """
    Optional Webots camera wrapper.
    Returns raw BGRA bytes plus image size.
    """
    def __init__(self, robot, name="camera", timestep=32):
        self._camera = None
        try:
            cam: _WebotsCamera = robot.getDevice(name)
            cam.enable(timestep)
            self._camera = cam
        except Exception:
            self._camera = None

    def available(self) -> bool:
        return self._camera is not None

    def read_image(self):
        if self._camera is None:
            return None, 0, 0
        return self._camera.getImage(), self._camera.getWidth(), self._camera.getHeight()


class PoseSensorWrapper:
    """
    Preferred Webots pose source for navigation.
    Uses GPS for planar position and Compass for heading.
    """

    def __init__(self, robot, gps_name=GPS_NAME, compass_name=COMPASS_NAME, timestep=32):
        self._gps = None
        self._compass = None
        try:
            gps: _WebotsGPS = robot.getDevice(gps_name)
            gps.enable(timestep)
            self._gps = gps
        except Exception:
            self._gps = None
        try:
            compass: _WebotsCompass = robot.getDevice(compass_name)
            compass.enable(timestep)
            self._compass = compass
        except Exception:
            self._compass = None

    def available(self) -> bool:
        return self._gps is not None and self._compass is not None

    @staticmethod
    def normalize_angle(theta: float) -> float:
        return (theta + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def heading_from_compass(values) -> Optional[float]:
        if values is None or len(values) < 2:
            return None
        try:
            cx = float(values[0])
            cy = float(values[1])
        except Exception:
            return None
        if not math.isfinite(cx) or not math.isfinite(cy):
            return None
        if abs(cx) < 1e-9 and abs(cy) < 1e-9:
            return None
        # Webots compass reports the north vector in robot coordinates.
        # In these worlds, heading 0 aligns with the robot facing +Y.
        return PoseSensorWrapper.normalize_angle(math.atan2(cx, cy))

    def read_pose(self):
        if not self.available():
            return None
        gps_values = self._gps.getValues()
        heading = self.heading_from_compass(self._compass.getValues())
        if gps_values is None or len(gps_values) < 2 or heading is None:
            return None
        return float(gps_values[0]), float(gps_values[1]), float(heading)
