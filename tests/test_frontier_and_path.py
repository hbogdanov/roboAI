import pathlib
import sys

import numpy as np


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
CTRL_DIR = REPO_ROOT / "webots_project" / "controllers" / "roboai_controller"
sys.path.insert(0, str(CTRL_DIR))

from frontier import frontier_points_world  # noqa: E402
from path_planner import plan_world_path  # noqa: E402


class DummyGrid:
    def __init__(self, prob):
        self._prob = np.array(prob, dtype=np.float32)
        self.h, self.w = self._prob.shape
        self.origin_m = (0.0, 0.0)
        self.res = 1.0

    def prob(self):
        return self._prob

    def world_to_grid(self, x, y):
        return int(round(x)), int(round(y))

    def grid_to_world(self, gx, gy):
        return float(gx), float(gy)


def test_frontier_points_detected():
    # center free, some unknown around 0.5 -> should form frontier candidates
    prob = [
        [0.9, 0.9, 0.9, 0.9, 0.9],
        [0.9, 0.5, 0.5, 0.5, 0.9],
        [0.9, 0.5, 0.2, 0.5, 0.9],
        [0.9, 0.5, 0.5, 0.5, 0.9],
        [0.9, 0.9, 0.9, 0.9, 0.9],
    ]
    g = DummyGrid(prob)
    pts = frontier_points_world(g, stride=1)
    assert len(pts) > 0


def test_plan_world_path_empty_map():
    prob = np.full((10, 10), 0.2, dtype=np.float32)  # free
    g = DummyGrid(prob)
    path = plan_world_path(g, start_xy=(1.0, 1.0), goal_xy=(8.0, 8.0))
    assert len(path) >= 2
    assert path[-1] == (8.0, 8.0)
