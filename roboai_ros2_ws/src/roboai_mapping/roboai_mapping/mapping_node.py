from __future__ import annotations

from roboai.core.occupancy_grid import OccupancyGrid
from roboai.core.types import LaserScan2D, Pose2D


class MappingEngine:
    def __init__(self, width: int = 400, height: int = 400, resolution: float = 0.05) -> None:
        self.grid = OccupancyGrid(width=width, height=height, resolution=resolution, origin=(-10.0, -10.0))

    def update(self, pose: Pose2D, scan: LaserScan2D) -> OccupancyGrid:
        self.grid.update_from_scan(pose, scan)
        return self.grid


def main() -> None:
    raise RuntimeError(
        "roboai_mapping is a ROS2 Humble node scaffold. Run it on Ubuntu 22.04 inside a sourced ROS2 workspace."
    )
