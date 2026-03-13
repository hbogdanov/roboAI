from __future__ import annotations


class VisualizationState:
    def __init__(self) -> None:
        self.frontiers = []
        self.blocked_goals = []
        self.paths = []


def main() -> None:
    raise RuntimeError(
        "roboai_viz is a ROS2 Humble node scaffold. Run it on Ubuntu 22.04 inside a sourced ROS2 workspace."
    )
