import pathlib
import sys
import types


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
CTRL_DIR = REPO_ROOT / "webots_project" / "controllers" / "roboai_controller"
sys.path.insert(0, str(CTRL_DIR))


if "controller" not in sys.modules:
    sys.modules["controller"] = types.SimpleNamespace(
        Robot=object,
        Lidar=object,
        Camera=object,
    )

from executor import PlanExecutor  # noqa: E402
from frontier import frontier_points_world  # noqa: E402


class DummyDrive:
    def __init__(self):
        self.commands = []
        self.stopped = 0

    def set_velocity(self, left, right):
        self.commands.append((left, right))

    def stop(self):
        self.stopped += 1
        self.commands.append((0.0, 0.0))


class DummyLog:
    def __init__(self):
        self.events = []

    def event(self, **kwargs):
        self.events.append(kwargs)


class DummySensors:
    def read_front_distance(self):
        return 0.0


class DummyState:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta


class DummyOccGrid:
    def __init__(self, prob_grid, origin=(-0.2, -0.2), res=0.1):
        self._prob = prob_grid
        self.origin_m = origin
        self.res = res

    def prob(self):
        return self._prob

    def grid_to_world(self, gx, gy):
        return gx * self.res + self.origin_m[0], gy * self.res + self.origin_m[1]


def test_return_base_compiles_to_goto_face_stop():
    drive = DummyDrive()
    log = DummyLog()
    executor = PlanExecutor(robot=None, drive=drive, sensors=DummySensors(), log=log)
    executor.load([{"op": "return_base"}])

    state = DummyState(x=1.5, y=-0.25, theta=0.6)
    done = executor.step(dt=0.1, ir=[0.0] * 8, state=state, occ_grid=None)

    assert done is False
    assert [step["op"] for step in executor.plan] == ["goto", "face", "stop"]
    assert executor.plan[0]["x"] == 1.5
    assert executor.plan[0]["y"] == -0.25
    assert executor.plan[1]["theta_deg"] != 0.0
    assert any(evt["op"] == "return_base_compiled" for evt in log.events)


def test_goto_completes_immediately_when_already_at_goal():
    drive = DummyDrive()
    log = DummyLog()
    executor = PlanExecutor(robot=None, drive=drive, sensors=DummySensors(), log=log)
    executor.load([{"op": "goto", "x": 0.0, "y": 0.0, "accept_radius": 0.1}])

    state = DummyState(x=0.0, y=0.0, theta=0.0)
    done = executor.step(dt=0.1, ir=[0.0] * 8, state=state, occ_grid=None)

    assert done is True
    assert executor.idx == 1
    assert drive.stopped >= 1
    ops = [evt["op"] for evt in log.events]
    assert "goto_start" in ops
    assert "goto_done" in ops


def test_lidar_local_avoidance_turns_when_front_blocked():
    drive = DummyDrive()
    log = DummyLog()
    executor = PlanExecutor(robot=None, drive=drive, sensors=DummySensors(), log=log)

    ranges = [1.0] * 181
    for i in range(80, 101):
        ranges[i] = 0.09

    avoid = executor._lidar_local_avoidance(
        lidar_scan=(ranges, -1.57079632679, 0.01745329252, 3.5),
        base=0.5,
        heading_error=0.0,
    )

    assert avoid is not None
    left, right, meta = avoid
    assert meta["decision"].startswith("front_blocked_turn_")
    assert left != right


def test_frontier_points_skip_cells_tight_to_obstacles():
    import numpy as np

    prob = np.full((7, 7), 0.5, dtype=float)
    prob[3, 3] = 0.2
    prob[3, 4] = 0.2
    prob[3, 2] = 0.8

    occ = DummyOccGrid(prob)
    frontiers = frontier_points_world(occ, stride=1, obstacle_clearance_cells=1)

    assert (0.1, 0.1) not in frontiers
