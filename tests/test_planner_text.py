import pathlib
import sys


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
PLANNER_DIR = REPO_ROOT / "webots_project" / "controllers" / "roboai_controller"
sys.path.insert(0, str(PLANNER_DIR))

import planner_text  # noqa: E402


def test_go_forward_3_seconds():
    plan = planner_text.stub_plan("go forward 3 seconds")
    assert plan[0]["op"] == "forward"
    assert plan[0]["seconds"] == 3.0
    assert plan[-1]["op"] == "stop"


def test_turn_right_defaults_90():
    plan = planner_text.stub_plan("turn right")
    turn = next(s for s in plan if s["op"] == "turn")
    assert turn["dir"] == "right"
    assert turn["deg"] == 90.0
    assert plan[-1]["op"] == "stop"


def test_scan_and_stop():
    plan = planner_text.stub_plan("scan and stop")
    assert any(s["op"] == "scan" for s in plan)
    assert plan[-1]["op"] == "stop"


def test_malformed_llm_output_falls_back_to_stub(monkeypatch):
    monkeypatch.setattr(planner_text, "USE_LLM", True)

    def _bad_llm(_text):
        raise RuntimeError("Malformed model output")

    monkeypatch.setattr(planner_text, "_llm_plan", _bad_llm)
    plan = planner_text.get_plan("go forward 3 seconds")
    assert plan[0]["op"] == "forward"
    assert plan[0]["seconds"] == 3.0
    assert plan[-1]["op"] == "stop"


def test_task_abstraction_scan_room():
    plan = planner_text.stub_plan("scan the room")
    assert sum(1 for s in plan if s["op"] == "scan") >= 2
    assert plan[-1]["op"] == "stop"
