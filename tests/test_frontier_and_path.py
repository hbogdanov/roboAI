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


def test_plan_blocks_unknown_band_by_default():
    # Unknown barrier (0.5) across map should block traversal.
    prob = np.full((10, 10), 0.2, dtype=np.float32)
    prob[5, :] = 0.5
    g = DummyGrid(prob)
    path = plan_world_path(g, start_xy=(1.0, 1.0), goal_xy=(8.0, 8.0))
    assert path == []


def test_plan_snaps_goal_away_from_occupied_with_clearance():
    prob = np.full((10, 10), 0.2, dtype=np.float32)  # mostly free
    # Make target cell occupied; planner should snap to a nearby safe cell.
    prob[8, 8] = 0.95
    g = DummyGrid(prob)

    path, meta = plan_world_path(
        g,
        start_xy=(1.0, 1.0),
        goal_xy=(8.0, 8.0),
        block_unknown=True,
        inflate_cells=1,
        goal_clearance_cells=1,
        return_meta=True,
    )
    assert len(path) >= 2
    assert meta["snapped_goal"] is True
    assert meta["goal_grid_raw"] == (8, 8)
    assert meta["goal_grid_used"] != (8, 8)


def test_plan_world_path_los_smoothing_reduces_waypoints():
    prob = np.full((12, 12), 0.2, dtype=np.float32)  # open/free map
    g = DummyGrid(prob)
    raw_path = plan_world_path(
        g,
        start_xy=(1.0, 1.0),
        goal_xy=(10.0, 9.0),
        smooth_path=False,
    )
    smooth_path = plan_world_path(
        g,
        start_xy=(1.0, 1.0),
        goal_xy=(10.0, 9.0),
        smooth_path=True,
    )
    assert len(raw_path) >= 2
    assert len(smooth_path) >= 2
    assert smooth_path[0] == raw_path[0]
    assert smooth_path[-1] == raw_path[-1]
    assert len(smooth_path) <= len(raw_path)
