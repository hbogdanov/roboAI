import pathlib
import sys


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
CTRL_DIR = REPO_ROOT / "webots_project" / "controllers" / "roboai_controller"
sys.path.insert(0, str(CTRL_DIR))

import waypoint_planner  # noqa: E402


def test_station_a_semantic_lookup():
    lib = dict(waypoint_planner.DEFAULT_GOAL_LIBRARY)
    plan = waypoint_planner._fallback_waypoint_plan("go to station A", goal_library=lib)
    goto = next(s for s in plan["steps"] if s["op"] == "goto")
    assert goto["x"] == lib["station_a"][0]
    assert goto["y"] == lib["station_a"][1]
    assert goto["goal"] == "station_a"


def test_charging_dock_semantic_lookup():
    lib = dict(waypoint_planner.DEFAULT_GOAL_LIBRARY)
    plan = waypoint_planner._fallback_waypoint_plan("go to the charging dock", goal_library=lib)
    goto = next(s for s in plan["steps"] if s["op"] == "goto")
    assert goto["x"] == lib["charging_dock"][0]
    assert goto["y"] == lib["charging_dock"][1]


def test_door_and_face_parse():
    lib = dict(waypoint_planner.DEFAULT_GOAL_LIBRARY)
    plan = waypoint_planner._fallback_waypoint_plan("go to the door and face 90 degrees", goal_library=lib)
    ops = [s["op"] for s in plan["steps"]]
    assert "goto" in ops
    face = next(s for s in plan["steps"] if s["op"] == "face")
    assert face["theta_deg"] == 90.0


def test_explore_and_map_parse():
    lib = dict(waypoint_planner.DEFAULT_GOAL_LIBRARY)
    plan = waypoint_planner._fallback_waypoint_plan("explore the room and build a map for 12 seconds", goal_library=lib)
    explore = next(s for s in plan["steps"] if s["op"] == "explore")
    assert explore["seconds"] == 12.0


def test_door_scan_return_parse():
    lib = dict(waypoint_planner.DEFAULT_GOAL_LIBRARY)
    plan = waypoint_planner._fallback_waypoint_plan("go to the door, scan, and return to base", goal_library=lib)
    ops = [s["op"] for s in plan["steps"]]
    assert "goto" in ops
    assert "scan" in ops
    assert "return_base" in ops


def test_world_specific_goal_file_load():
    lib, src = waypoint_planner.load_goal_library()
    assert isinstance(lib, dict)
    assert "door" in lib
    assert isinstance(src, str)
