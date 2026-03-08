import argparse
import glob
import json
import os
from collections import defaultdict


def _safe_ratio(num: float, den: float) -> float:
    return (num / den) if den else 0.0


def _is_randomized_world(world_name: str) -> bool:
    w = (world_name or "").lower()
    return ("rand" in w) or ("generated" in w)


def _load_rows(paths):
    rows = []
    for p in paths:
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
        events = data.get("events", [])
        plan_evt = next((e for e in events if e.get("op") == "plan_built"), {})
        world = str(plan_evt.get("world_name", "unknown_world"))
        constraints = plan_evt.get("constraints", {}) if isinstance(plan_evt.get("constraints"), dict) else {}
        planner = constraints.get("planner", {}) if isinstance(constraints.get("planner"), dict) else {}
        inflate = int(planner.get("inflate_cells", 0))
        avoid_mode = str(planner.get("local_avoid_mode", "unknown")).lower()

        goto_start = sum(1 for e in events if e.get("op") == "goto_start")
        goto_done = sum(1 for e in events if e.get("op") == "goto_done")
        success = goto_start > 0 and goto_done >= goto_start
        if goto_start == 0:
            success = any(e.get("op") == "stop" for e in events)
        collisions = sum(1 for e in events if e.get("op") in {"collision_warning", "collision_burst_escape"})
        rows.append(
            {
                "run": os.path.basename(p),
                "world": world,
                "randomized": _is_randomized_world(world),
                "inflate_cells": inflate,
                "inflation_mode": "inflated" if inflate >= 4 else "low_inflation",
                "local_avoid_mode": avoid_mode if avoid_mode in {"lidar", "ir"} else "unknown",
                "success": bool(success),
                "collisions": int(collisions),
            }
        )
    return rows


def _aggregate(rows, key_fn):
    buckets = defaultdict(list)
    for r in rows:
        buckets[key_fn(r)].append(r)
    out = {}
    for k, grp in buckets.items():
        out[k] = {
            "runs": len(grp),
            "success_rate": round(100.0 * _safe_ratio(sum(1 for x in grp if x["success"]), len(grp)), 1),
            "avg_collisions": round(sum(x["collisions"] for x in grp) / max(1, len(grp)), 3),
        }
    return out


def to_markdown(rand_rows, by_inflation, by_avoid):
    lines = [
        "# Randomized-World Experiment",
        "",
        f"- Runs on randomized worlds: **{len(rand_rows)}**",
        "",
        "## Inflation Comparison",
        "| Inflation Mode | Runs | Success Rate | Avg Collisions |",
        "|---|---:|---:|---:|",
    ]
    for k in sorted(by_inflation.keys()):
        v = by_inflation[k]
        lines.append(f"| `{k}` | {v['runs']} | {v['success_rate']}% | {v['avg_collisions']} |")

    lines.extend(
        [
            "",
            "## Local Avoidance Comparison",
            "| Local Avoid Mode | Runs | Success Rate | Avg Collisions |",
            "|---|---:|---:|---:|",
        ]
    )
    for k in sorted(by_avoid.keys()):
        v = by_avoid[k]
        lines.append(f"| `{k}` | {v['runs']} | {v['success_rate']}% | {v['avg_collisions']} |")
    return "\n".join(lines) + "\n"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--log-dir", default="data/logs")
    p.add_argument("--glob", default="run_*.json")
    p.add_argument("--out-json", default="reports/randomization_experiment.json")
    p.add_argument("--out-md", default="reports/randomization_experiment.md")
    args = p.parse_args()

    files = sorted(glob.glob(os.path.join(args.log_dir, args.glob)))
    rows = _load_rows(files)
    rand_rows = [r for r in rows if r["randomized"]]
    by_inflation = _aggregate(rand_rows, key_fn=lambda r: r["inflation_mode"])
    by_avoid = _aggregate(rand_rows, key_fn=lambda r: r["local_avoid_mode"])

    payload = {
        "runs_total": len(rows),
        "runs_randomized": len(rand_rows),
        "inflation_comparison": by_inflation,
        "local_avoid_comparison": by_avoid,
        "runs": rand_rows,
    }

    os.makedirs(os.path.dirname(args.out_json), exist_ok=True)
    os.makedirs(os.path.dirname(args.out_md), exist_ok=True)
    with open(args.out_json, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    with open(args.out_md, "w", encoding="utf-8") as f:
        f.write(to_markdown(rand_rows, by_inflation, by_avoid))
    print(f"Wrote {args.out_json}")
    print(f"Wrote {args.out_md}")


if __name__ == "__main__":
    main()
