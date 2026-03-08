import argparse
import os


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEMO_DIR = os.path.join(REPO_ROOT, "demo")
MVP_COMMAND = os.path.join(DEMO_DIR, "mvp_command.txt")
MVP_MODE = os.path.join(DEMO_DIR, "mvp_mode.txt")
MVP_WORLD = os.path.join(DEMO_DIR, "mvp_world.txt")

DEMOS = {
    "demo1": {
        "command_file": os.path.join(DEMO_DIR, "demo1_goal_navigation.txt"),
        "mode": "waypoint",
        "world": "world_office",
        "name": "Goal navigation",
    },
    "demo2": {
        "command_file": os.path.join(DEMO_DIR, "demo2_explore_map.txt"),
        "mode": "waypoint",
        "world": "world_office",
        "name": "Explore and map",
    },
    "demo3": {
        "command_file": os.path.join(DEMO_DIR, "demo3_go_scan_return.txt"),
        "mode": "waypoint",
        "world": "world_office",
        "name": "Go, inspect, return",
    },
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", choices=sorted(DEMOS.keys()), required=True)
    args = parser.parse_args()

    d = DEMOS[args.demo]
    with open(d["command_file"], "r", encoding="utf-8") as f:
        cmd = f.read().strip()

    with open(MVP_COMMAND, "w", encoding="utf-8") as f:
        f.write(cmd + "\n")
    with open(MVP_MODE, "w", encoding="utf-8") as f:
        f.write(d["mode"] + "\n")
    with open(MVP_WORLD, "w", encoding="utf-8") as f:
        f.write(d["world"] + "\n")

    print(f"Selected {args.demo}: {d['name']}")
    print(f"Command: {cmd}")
    print(f"Mode: {d['mode']}")
    print(f"World: {d['world']}")


if __name__ == "__main__":
    main()
