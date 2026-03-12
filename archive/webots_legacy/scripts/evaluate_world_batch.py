import argparse
import glob
import json
import os
from collections import defaultdict
from statistics import mean


def _safe_ratio(num: float, den: float) -> float:
    return (num / den) if den else 0.0


def _runtime_s(events) -> float:
    ts = [float(e.get("t")) for e in events if "t" in e]
    if len(ts) < 2:
        return 0.0
    return max(ts) - min(ts)


def _path_length_m(events) -> float:
    ticks = [e for e in events if e.get("op") == "spa_tick" and "x" in e and "y" in e]
    if len(ticks) < 2:
        return 0.0
    d = 0.0
    for a, b in zip(ticks[:-1], ticks[1:]):
        dx = float(b["x"]) - float(a["x"])
        dy = float(b["y"]) - float(a["y"])
        d += (dx * dx + dy * dy) ** 0.5
    return d


def _start_xy(events):
    for e in events:
        if e.get("op") == "spa_tick" and "x" in e and "y" in e:
            return float(e["x"]), float(e["y"])
    return None


def _goto_goal(plan):
    if not isinstance(plan, list):
        return None
    for s in plan:
        if str(s.get("op", "")).lower() == "goto":
            try:
                return float(s.get("x")), float(s.get("y"))
            except Exception:
                return None
    return None


def _final_goal_error(events):
    done = [e for e in events if e.get("op") == "goto_done"]
    if done:
        try:
            return float(done[-1].get("goal_error_m", 0.0))
        except Exception:
            return 0.0
    prog = [e for e in events if e.get("op") == "goto_progress"]
    if prog:
        try:
            return float(prog[-1].get("goal_error_m", 0.0))
        except Exception:
            return 0.0
    return 0.0


def _world_name(plan_evt):
    wn = str(plan_evt.get("world_name", "")).strip()
    if wn:
        return wn
    return "unknown_world"


def evaluate_log(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    events = data.get("events", [])
    plan_evt = next((e for e in events if e.get("op") == "plan_built"), {})
    plan = plan_evt.get("plan", [])
    world = _world_name(plan_evt)

    goto_start = sum(1 for e in events if e.get("op") == "goto_start")
    goto_done = sum(1 for e in events if e.get("op") == "goto_done")
    success = goto_start > 0 and goto_done >= goto_start
    if goto_start == 0:
        success = any(e.get("op") == "stop" for e in events)

    goal_err = _final_goal_error(events)
    replans = max(0, sum(1 for e in events if e.get("op") == "path_planned") - goto_start)
    runtime_s = _runtime_s(events)
    collisions = sum(1 for e in events if e.get("op") in {"collision_warning", "collision_burst_escape"})
    path_len = _path_length_m(events)

    start = _start_xy(events)
    goal = _goto_goal(plan)
    straight = 0.0
    eff = 0.0
    if start is not None and goal is not None:
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        straight = (dx * dx + dy * dy) ** 0.5
        eff = _safe_ratio(straight, path_len) if path_len > 0.0 else 0.0

    return {
        "run": os.path.basename(path),
        "world": world,
        "success": bool(success),
        "final_goal_error_m": goal_err,
        "replans": replans,
        "runtime_s": runtime_s,
        "collision_count": collisions,
        "path_efficiency": eff,
    }


def aggregate(rows, expected_worlds=None):
    by_world = defaultdict(list)
    for r in rows:
        by_world[r["world"]].append(r)
    for w in expected_worlds or []:
        by_world.setdefault(w, [])
    out = {}
    for world, grp in by_world.items():
        if not grp:
            out[world] = {
                "trials": 0,
                "success_rate": 0.0,
                "avg_final_goal_error_m": 0.0,
                "avg_replans": 0.0,
                "avg_runtime_s": 0.0,
                "avg_collision_count": 0.0,
                "avg_path_efficiency": 0.0,
            }
            continue
        out[world] = {
            "trials": len(grp),
            "success_rate": round(100.0 * _safe_ratio(sum(1 for r in grp if r["success"]), len(grp)), 1),
            "avg_final_goal_error_m": round(mean([r["final_goal_error_m"] for r in grp]), 3),
            "avg_replans": round(mean([r["replans"] for r in grp]), 3),
            "avg_runtime_s": round(mean([r["runtime_s"] for r in grp]), 3),
            "avg_collision_count": round(mean([r["collision_count"] for r in grp]), 3),
            "avg_path_efficiency": round(mean([r["path_efficiency"] for r in grp]), 3),
        }
    return out


def to_markdown(summary: dict) -> str:
    lines = [
        "# Supervisor-Like World Batch Evaluation",
        "",
        "| World | Trials | Success Rate | Avg Final Goal Error (m) | Avg Replans | Avg Runtime (s) | Avg Collisions | Avg Path Efficiency |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for world in sorted(summary.keys()):
        s = summary[world]
        lines.append(
            f"| `{world}` | {s['trials']} | {s['success_rate']}% | {s['avg_final_goal_error_m']} | "
            f"{s['avg_replans']} | {s['avg_runtime_s']} | {s['avg_collision_count']} | {s['avg_path_efficiency']} |"
        )
    return "\n".join(lines) + "\n"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--log-dir", default="data/logs")
    p.add_argument("--glob", default="run_*.json")
    p.add_argument("--out-json", default="reports/world_batch_eval.json")
    p.add_argument("--out-md", default="reports/world_batch_eval.md")
    p.add_argument("--worlds", nargs="*", default=["world_empty", "world_obstacles", "world_office"])
    args = p.parse_args()

    files = sorted(glob.glob(os.path.join(args.log_dir, args.glob)))
    rows = [evaluate_log(f) for f in files]
    summary = aggregate(rows, expected_worlds=args.worlds)

    os.makedirs(os.path.dirname(args.out_json), exist_ok=True)
    os.makedirs(os.path.dirname(args.out_md), exist_ok=True)
    with open(args.out_json, "w", encoding="utf-8") as f:
        json.dump({"summary": summary, "runs": rows}, f, indent=2)
    with open(args.out_md, "w", encoding="utf-8") as f:
        f.write(to_markdown(summary))
    print(f"Wrote {args.out_json}")
    print(f"Wrote {args.out_md}")


if __name__ == "__main__":
    main()
