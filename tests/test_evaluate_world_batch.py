import json
import pathlib
import sys


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from scripts.evaluate_world_batch import aggregate, evaluate_log, to_markdown  # noqa: E402


def test_evaluate_log_extracts_world_metrics():
    payload = {
        "events": [
            {"op": "plan_built", "t": 0.0, "world_name": "world_office", "plan": [{"op": "goto", "x": 2.0, "y": 0.0}]},
            {"op": "goto_start", "t": 0.1},
            {"op": "path_planned", "t": 0.2},
            {"op": "spa_tick", "t": 1.0, "x": 0.0, "y": 0.0},
            {"op": "spa_tick", "t": 2.0, "x": 1.0, "y": 0.0},
            {"op": "spa_tick", "t": 3.0, "x": 2.0, "y": 0.0},
            {"op": "goto_done", "t": 3.1, "goal_error_m": 0.04},
            {"op": "collision_warning", "t": 3.2},
        ]
    }

    log_path = REPO_ROOT / "tests" / "_tmp_run_world_office.json"
    try:
        log_path.write_text(json.dumps(payload), encoding="utf-8")
        row = evaluate_log(str(log_path))
    finally:
        if log_path.exists():
            log_path.unlink()

    assert row["world"] == "world_office"
    assert row["success"] is True
    assert row["final_goal_error_m"] == 0.04
    assert row["runtime_s"] == 3.2
    assert row["collision_count"] == 1
    assert row["path_efficiency"] == 1.0


def test_aggregate_and_markdown_include_empty_expected_worlds():
    rows = [
        {
            "run": "run_world_office.json",
            "world": "world_office",
            "success": True,
            "final_goal_error_m": 0.04,
            "replans": 0,
            "runtime_s": 3.2,
            "collision_count": 1,
            "path_efficiency": 1.0,
        }
    ]

    summary = aggregate(rows, expected_worlds=["world_empty", "world_office"])

    assert summary["world_empty"]["trials"] == 0
    assert summary["world_office"]["success_rate"] == 100.0

    md = to_markdown(summary)
    assert "`world_empty`" in md
    assert "`world_office`" in md
    assert "100.0%" in md
