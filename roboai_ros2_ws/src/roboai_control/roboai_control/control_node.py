from __future__ import annotations

from roboai.core.control.waypoint_follower import WaypointFollower


class ControlEngine:
    def __init__(self) -> None:
        self.follower = WaypointFollower()

    def set_path(self, path):
        self.follower.set_path(path)

    def command(self, pose):
        return self.follower.command(pose)


def main() -> None:
    raise RuntimeError(
        "roboai_control is a ROS2 Humble node scaffold. Run it on Ubuntu 22.04 inside a sourced ROS2 workspace."
    )
