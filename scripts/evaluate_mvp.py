import argparse
import glob
import json
import os
from collections import Counter
from statistics import mean


ALLOWED_OPS = {"forward", "turn", "scan", "return_base", "wait", "stop", "goto", "face", "explore"}
STEP_TO_EVENT = {
    "forward": "forward_done",
    "turn": "turn_done",
    "scan": "scan",
    "return_base": "return_base",
    "wait": "wait_done",
    "goto": "goto_done",
    "face": "face_done",
    "explore": "explore_done",
    "stop": "stop",
}


def _safe_ratio(num: float, den: float) -> float:
    return (num / den) if den else 0.0


def _parse_success(plan) -> bool:
    if not isinstance(plan, list) or not plan:
        return False
    for step in plan:
        if not isinstance(step, dict):
            return False
        if str(step.get("op", "")).lower().strip() not in ALLOWED_OPS:
            return False
    return True


def _step_completion(plan, event_counter: Counter) -> float:
    if not isinstance(plan, list) or not plan:
        return 0.0
    remaining = event_counter.copy()
    completed = 0
    for step in plan:
        op = str(step.get("op", "")).lower().strip()
        event_name = STEP_TO_EVENT.get(op)
        if not event_name:
            continue
        if remaining[event_name] > 0:
            remaining[event_name] -= 1
            completed += 1
    return _safe_ratio(completed, len(plan))


def _distance_m(events) -> float:
    ticks = [e for e in events if e.get("op") == "spa_tick" and "x" in e and "y" in e]
    if len(ticks) < 2:
        return 0.0
    dist = 0.0
    for a, b in zip(ticks[:-1], ticks[1:]):
        dx = float(b["x"]) - float(a["x"])
        dy = float(b["y"]) - float(a["y"])
        dist += (dx * dx + dy * dy) ** 0.5
    return dist


def evaluate_one(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    events = data.get("events", [])
    event_counter = Counter(e.get("op") for e in events if "op" in e)

    plan_evt = next((e for e in events if e.get("op") == "plan_built"), {})
    plan = plan_evt.get("plan", [])
    command = plan_evt.get("command", "")

    planning_parse_success = _parse_success(plan)
    path_completion = _step_completion(plan, event_counter)
    command_success = planning_parse_success and path_completion >= 0.999

    forward_ticks = [e for e in events if e.get("op") == "spa_forward_tick"]
    if forward_ticks:
        max_front = max(float(e.get("front", 0.0)) for e in forward_ticks)
        obstacle_avoidance_success = max_front < 0.95
    else:
        max_front = 0.0
        obstacle_avoidance_success = False

    map_saved_evt = next((e for e in events if e.get("op") == "map_saved"), {})
    outputs = map_saved_evt.get("outputs", {}) if isinstance(map_saved_evt, dict) else {}
    has_map_output = bool(outputs.get("npy") or outputs.get("png"))

    return {
        "run": os.path.basename(path),
        "command": command,
        "planning_parse_success": planning_parse_success,
        "path_completion": round(path_completion, 3),
        "command_success": command_success,
        "obstacle_avoidance_success": obstacle_avoidance_success,
        "max_front_level": round(max_front, 3),
        "map_generation_output": has_map_output,
        "distance_m": round(_distance_m(events), 3),
        "events": len(events),
    }


def aggregate(rows):
    total = len(rows)
    with_forward = [r for r in rows if r["max_front_level"] > 0 or r["obstacle_avoidance_success"]]
    return {
        "runs": total,
        "command_success_rate": round(100.0 * _safe_ratio(sum(r["command_success"] for r in rows), total), 1),
        "path_completion_avg": round(100.0 * mean([r["path_completion"] for r in rows]), 1) if rows else 0.0,
        "obstacle_avoidance_success_rate": round(
            100.0 * _safe_ratio(sum(r["obstacle_avoidance_success"] for r in rows), len(with_forward)), 1
        ) if with_forward else 0.0,
        "planning_parse_success_rate": round(
            100.0 * _safe_ratio(sum(r["planning_parse_success"] for r in rows), total), 1
        ),
        "map_generation_output_rate": round(100.0 * _safe_ratio(sum(r["map_generation_output"] for r in rows), total), 1),
    }


def to_markdown(rows, agg, out_path: str):
    lines = [
        "# RoboAI MVP Benchmark",
        "",
        "## Project Boundary",
        (
            "RoboAI is a Webots-based mobile robot system that converts natural-language commands "
            "into executable motion plans, performs obstacle-aware navigation with onboard sensors, "
            "and logs trajectories / sensor data for analysis."
        ),
        "",
        "## KPI Summary",
        f"- Runs evaluated: **{agg['runs']}**",
        f"- Command success rate: **{agg['command_success_rate']}%**",
        f"- Path completion (avg): **{agg['path_completion_avg']}%**",
        f"- Obstacle avoidance success: **{agg['obstacle_avoidance_success_rate']}%**",
        f"- Planning parse success: **{agg['planning_parse_success_rate']}%**",
        f"- Map generation output: **{agg['map_generation_output_rate']}%**",
        "",
        "## Metric Definitions",
        "- `planning_parse_success`: `plan_built.plan` exists and contains only allowed MVP ops.",
        "- `path_completion`: fraction of planned steps with matching completion events in the log.",
        "- `command_success`: parse success and full path completion.",
        "- `obstacle_avoidance_success`: max `front` level during forward ticks < `0.95`.",
        "- `map_generation_output`: run includes `map_saved` event with `.npy` or `.png` output.",
        "",
        "## Per-Run Table",
        "| Run | Command Success | Path Completion | Obstacle Avoidance | Parse Success | Map Output | Distance (m) |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for r in rows:
        lines.append(
            f"| `{r['run']}` | {'Yes' if r['command_success'] else 'No'} | "
            f"{round(r['path_completion']*100,1)}% | "
            f"{'Yes' if r['obstacle_avoidance_success'] else 'No'} | "
            f"{'Yes' if r['planning_parse_success'] else 'No'} | "
            f"{'Yes' if r['map_generation_output'] else 'No'} | "
            f"{r['distance_m']} |"
        )

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", default="data/logs")
    parser.add_argument("--out-md", default="reports/benchmark_mvp.md")
    parser.add_argument("--out-json", default="reports/benchmark_mvp.json")
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(args.log_dir, "run_*.json")))
    rows = [evaluate_one(p) for p in files]
    agg = aggregate(rows)

    os.makedirs(os.path.dirname(args.out_md), exist_ok=True)
    os.makedirs(os.path.dirname(args.out_json), exist_ok=True)
    to_markdown(rows, agg, args.out_md)
    with open(args.out_json, "w", encoding="utf-8") as f:
        json.dump({"summary": agg, "runs": rows}, f, indent=2)

    print(f"Wrote {args.out_md}")
    print(f"Wrote {args.out_json}")


if __name__ == "__main__":
    main()
