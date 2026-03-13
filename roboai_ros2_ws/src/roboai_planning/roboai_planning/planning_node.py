from __future__ import annotations

from roboai.core.planners.astar import plan_astar
from roboai.core.planners.rrt import plan_rrt
from roboai.core.planners.rrt_star import plan_rrt_star


PLANNERS = {
    "astar": plan_astar,
    "rrt": plan_rrt,
    "rrt_star": plan_rrt_star,
}


class PlanningEngine:
    def plan(self, planner_name: str, **kwargs):
        return PLANNERS[planner_name](**kwargs)


def main() -> None:
    raise RuntimeError(
        "roboai_planning is a ROS2 Humble node scaffold. Run it on Ubuntu 22.04 inside a sourced ROS2 workspace."
    )
