import pathlib
import sys


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
CTRL_DIR = REPO_ROOT / "webots_project" / "controllers" / "roboai_controller"
sys.path.insert(0, str(CTRL_DIR))

import waypoint_planner  # noqa: E402


def test_station_a_semantic_lookup():
    plan = waypoint_planner._fallback_waypoint_plan("go to station A")
    goto = next(s for s in plan["steps"] if s["op"] == "goto")
    assert goto["x"] == waypoint_planner.GOAL_LIBRARY["station_a"][0]
    assert goto["y"] == waypoint_planner.GOAL_LIBRARY["station_a"][1]
    assert goto["goal"] == "station_a"


def test_charging_dock_semantic_lookup():
    plan = waypoint_planner._fallback_waypoint_plan("go to the charging dock")
    goto = next(s for s in plan["steps"] if s["op"] == "goto")
    assert goto["x"] == waypoint_planner.GOAL_LIBRARY["charging_dock"][0]
    assert goto["y"] == waypoint_planner.GOAL_LIBRARY["charging_dock"][1]


def test_door_and_face_parse():
    plan = waypoint_planner._fallback_waypoint_plan("go to the door and face 90 degrees")
    ops = [s["op"] for s in plan["steps"]]
    assert "goto" in ops
    face = next(s for s in plan["steps"] if s["op"] == "face")
    assert face["theta_deg"] == 90.0


def test_explore_and_map_parse():
    plan = waypoint_planner._fallback_waypoint_plan("explore the room and build a map for 12 seconds")
    explore = next(s for s in plan["steps"] if s["op"] == "explore")
    assert explore["seconds"] == 12.0
