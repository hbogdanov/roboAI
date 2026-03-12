import argparse
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEMO_DIR = REPO_ROOT / "demo"
LOG_ROOT = REPO_ROOT / "data" / "logs"
MANUAL_ROOT = LOG_ROOT / "manual_batches"

WORLD_FILES = {
    "world_empty": REPO_ROOT / "webots_project" / "worlds" / "world_empty.wbt",
    "world_obstacles": REPO_ROOT / "webots_project" / "worlds" / "world_obstacles.wbt",
    "world_office": REPO_ROOT / "webots_project" / "worlds" / "world_office.wbt",
}

COMMANDS = {
    "explore_map": "explore the room and build a map for 60 seconds",
    "goal_navigation": "go to station A and face 90 degrees",
    "scan_return": "go to the door, scan, and return to base",
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", choices=sorted(WORLD_FILES.keys()), required=True)
    parser.add_argument("--command", choices=sorted(COMMANDS.keys()), default="explore_map")
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--run-seconds", type=int, default=60)
    parser.add_argument("--batch-name", default="")
    args = parser.parse_args()

    batch_name = args.batch_name.strip() or f"{args.world}_{args.command}"
    batch_dir = MANUAL_ROOT / batch_name
    batch_dir.mkdir(parents=True, exist_ok=True)

    command_text = COMMANDS[args.command]
    (DEMO_DIR / "mvp_mode.txt").write_text("waypoint\n", encoding="utf-8")
    (DEMO_DIR / "mvp_command.txt").write_text(command_text + "\n", encoding="utf-8")
    (DEMO_DIR / "mvp_world.txt").write_text(args.world + "\n", encoding="utf-8")

    manifest = {
        "world": args.world,
        "world_file": str(WORLD_FILES[args.world]),
        "command_key": args.command,
        "command_text": command_text,
        "trials_target": args.trials,
        "run_seconds": args.run_seconds,
        "batch_dir": str(batch_dir),
    }
    (batch_dir / "batch_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    instructions = "\n".join(
        [
            "# Manual Batch Setup",
            "",
            f"- World: `{args.world}`",
            f"- World file: `{WORLD_FILES[args.world]}`",
            f"- Command: `{command_text}`",
            "- Plan mode: `waypoint`",
            f"- Trial target: `{args.trials}`",
            f"- Suggested run length: `{args.run_seconds}` seconds",
            "",
            "After each trial, archive the latest log with:",
            "",
            f"```bash\npython scripts/archive_latest_log.py --batch-dir \"{batch_dir}\"\n```",
            "",
            "After the batch is complete, aggregate the results with:",
            "",
            f"```bash\npython scripts/evaluate_world_batch.py --log-dir \"{batch_dir}\" --out-md reports/{batch_name}_eval.md --out-json reports/{batch_name}_eval.json --worlds {args.world}\n```",
            "",
        ]
    )
    (batch_dir / "README.md").write_text(instructions, encoding="utf-8")

    print(f"Prepared batch: {batch_name}")
    print(f"World file: {WORLD_FILES[args.world]}")
    print(f"Batch dir: {batch_dir}")
    print(f"Command: {command_text}")
    print(f"Suggested run length: {args.run_seconds} seconds")


if __name__ == "__main__":
    main()
