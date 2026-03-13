from __future__ import annotations


class ExplorationManagerState:
    def __init__(self) -> None:
        self.current_goal = None
        self.coverage = 0.0
        self.replans = 0
        self.recovery_events = 0


def main() -> None:
    raise RuntimeError(
        "roboai_manager is a ROS2 Humble node scaffold. Run it on Ubuntu 22.04 inside a sourced ROS2 workspace."
    )
