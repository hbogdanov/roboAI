import argparse
import glob
import json
import os
from collections import Counter


def _distance_m(spa_ticks):
    if len(spa_ticks) < 2:
        return 0.0
    dist = 0.0
    for a, b in zip(spa_ticks[:-1], spa_ticks[1:]):
        dx = float(b.get("x", 0.0)) - float(a.get("x", 0.0))
        dy = float(b.get("y", 0.0)) - float(a.get("y", 0.0))
        dist += (dx * dx + dy * dy) ** 0.5
    return dist


def summarize_run(path: str, collision_warn_thresh: float = 0.25):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    events = data.get("events", [])
    c = Counter(e.get("op") for e in events if "op" in e)

    t_values = [float(e.get("t")) for e in events if "t" in e]
    runtime_s = (max(t_values) - min(t_values)) if len(t_values) >= 2 else 0.0

    spa_ticks = [e for e in events if e.get("op") == "spa_tick" and "x" in e and "y" in e]
    path_length_m = _distance_m(spa_ticks)
    final_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
    if spa_ticks:
        last = spa_ticks[-1]
        final_pose = {
            "x": float(last.get("x", 0.0)),
            "y": float(last.get("y", 0.0)),
            "theta": float(last.get("theta", 0.0)),
        }

    turns = int(c.get("turn_start", 0))
    scans = int(c.get("scan", 0))

    fw_ticks = [e for e in events if e.get("op") == "spa_forward_tick"]
    collision_warnings = sum(float(e.get("front", 0.0)) >= collision_warn_thresh for e in fw_ticks)
    collision_warnings += int(c.get("collision_warning", 0))
    max_front = max([float(e.get("front", 0.0)) for e in fw_ticks], default=0.0)

    return {
        "run_file": os.path.basename(path),
        "runtime_s": round(runtime_s, 3),
        "path_length_m": round(path_length_m, 3),
        "number_of_turns": turns,
        "number_of_scan_steps": scans,
        "collision_warnings": int(collision_warnings),
        "max_front_level": round(max_front, 3),
        "final_pose": {
            "x": round(final_pose["x"], 3),
            "y": round(final_pose["y"], 3),
            "theta": round(final_pose["theta"], 3),
        },
        "events_total": len(events),
    }


def to_markdown(summary: dict) -> str:
    fp = summary["final_pose"]
    lines = [
        "# RoboAI Run Summary",
        "",
        f"- Run file: `{summary['run_file']}`",
        f"- Runtime: **{summary['runtime_s']} s**",
        f"- Path length: **{summary['path_length_m']} m**",
        f"- Number of turns: **{summary['number_of_turns']}**",
        f"- Number of scan steps: **{summary['number_of_scan_steps']}**",
        f"- Collision warnings: **{summary['collision_warnings']}**",
        f"- Max front level: **{summary['max_front_level']}**",
        f"- Final pose: **x={fp['x']}, y={fp['y']}, theta={fp['theta']}**",
        f"- Total events: **{summary['events_total']}**",
        "",
        "Collision warning definition:",
        "- A `spa_forward_tick.front` value >= threshold (default 0.25).",
    ]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-file", default="")
    parser.add_argument("--log-dir", default="data/logs")
    parser.add_argument("--out-md", default="reports/run_summary.md")
    parser.add_argument("--out-json", default="reports/run_summary.json")
    parser.add_argument("--collision-warn-thresh", type=float, default=0.25)
    args = parser.parse_args()

    log_path = args.log_file.strip()
    if not log_path:
        files = sorted(glob.glob(os.path.join(args.log_dir, "run_*.json")))
        if not files:
            raise RuntimeError(f"No logs found in {args.log_dir}")
        log_path = files[-1]

    summary = summarize_run(log_path, collision_warn_thresh=args.collision_warn_thresh)

    os.makedirs(os.path.dirname(args.out_md), exist_ok=True)
    os.makedirs(os.path.dirname(args.out_json), exist_ok=True)
    with open(args.out_md, "w", encoding="utf-8") as f:
        f.write(to_markdown(summary))
    with open(args.out_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print(f"Wrote {args.out_md}")
    print(f"Wrote {args.out_json}")


if __name__ == "__main__":
    main()
