from __future__ import annotations

from roboai.core.frontier import rank_frontier_targets


class FrontierEngine:
    def rank(self, **kwargs):
        return rank_frontier_targets(**kwargs)


def main() -> None:
    raise RuntimeError(
        "roboai_frontier is a ROS2 Humble node scaffold. Run it on Ubuntu 22.04 inside a sourced ROS2 workspace."
    )
