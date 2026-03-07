import argparse
import json
import os
import sys
import time

from controller import Robot
from config import (
    TIME_STEP_MS,
    LEFT_MOTOR_NAME,
    RIGHT_MOTOR_NAME,
    DEFAULT_COMMAND_FILE,
    REPORTS_DIR,
)
from motion import Drive
from sensors import Sensors
from logger import RunLogger
from state import StateEstimator
from planner_text import get_plan
from waypoint_planner import get_waypoint_plan
from executor import PlanExecutor
from sensors import LidarWrapper
from occupancy_grid import OccupancyGrid

RUN_SECONDS = 40.0
DEFAULT_COMMAND = "Go forward for 3 seconds, turn left 90, scan, then stop."


def resolve_command() -> str:
    """
    Command precedence:
      1) --command "..."
      2) ROBOAI_COMMAND env var
      3) --command-file <path> (or default demo/mvp_command.txt)
      4) DEFAULT_COMMAND fallback
    """
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--command", type=str, default="")
    parser.add_argument("--command-file", type=str, default=DEFAULT_COMMAND_FILE)
    args, _ = parser.parse_known_args(sys.argv[1:])

    if args.command and args.command.strip():
        return args.command.strip()

    env_cmd = os.getenv("ROBOAI_COMMAND", "").strip()
    if env_cmd:
        return env_cmd

    file_path = (args.command_file or "").strip()
    if file_path and os.path.exists(file_path):
        with open(file_path, "r", encoding="utf-8") as f:
            file_cmd = f.read().strip()
        if file_cmd:
            return file_cmd

    return DEFAULT_COMMAND


def write_demo_artifact(command: str, plan, log_path: str):
    os.makedirs(REPORTS_DIR, exist_ok=True)
    out_path = os.path.join(REPORTS_DIR, "latest_demo_artifact.md")
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    lines = [
        "# RoboAI MVP Demo Artifact",
        "",
        f"- Timestamp: `{ts}`",
        f"- Command: `{command}`",
        f"- Log file: `{log_path}`",
        "",
        "## Resolved Plan",
        "```json",
        json.dumps(plan, indent=2),
        "```",
        "",
    ]
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def write_explanation_artifact(command: str, plan_type: str, plan_steps, events):
    os.makedirs(REPORTS_DIR, exist_ok=True)
    out_path = os.path.join(REPORTS_DIR, "latest_explanation.md")
    counts = {}
    for e in events:
        op = str(e.get("op", ""))
        counts[op] = counts.get(op, 0) + 1

    lines = [
        "# RoboAI Run Explanation",
        "",
        "## Intent",
        f"- Received command: `{command}`",
        f"- Planning mode: `{plan_type}`",
        "",
        "## Interpreted Plan",
        "```json",
        json.dumps(plan_steps, indent=2),
        "```",
        "",
        "## Execution Evidence",
        f"- `plan_built`: `{counts.get('plan_built', 0)}`",
        f"- `plan_loaded`: `{counts.get('plan_loaded', 0)}`",
        f"- `spa_tick`: `{counts.get('spa_tick', 0)}`",
        f"- `turn_done`: `{counts.get('turn_done', 0)}`",
        f"- `scan`: `{counts.get('scan', 0)}`",
        f"- `goto_done`: `{counts.get('goto_done', 0)}`",
        f"- `face_done`: `{counts.get('face_done', 0)}`",
        f"- `collision_warning`: `{counts.get('collision_warning', 0)}`",
        f"- `stop`: `{counts.get('stop', 0)}`",
        "",
        "## Summary",
        (
            "The system explains behavior by exposing the resolved plan and concrete execution events "
            "from runtime logs (rather than free-form text generation)."
        ),
        "",
    ]
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def save_occupancy_artifacts(occ_grid):
    """
    Save map outputs for MVP evaluation:
      - reports/occupancy_map.npy
      - reports/occupancy_map.png (heatmap, if matplotlib is available)
      - reports/occupancy_map_binary.png (thresholded view, if matplotlib is available)
    Returns dict with output paths.
    """
    os.makedirs(REPORTS_DIR, exist_ok=True)
    out = {}
    grid_prob = occ_grid.prob()

    npy_path = os.path.join(REPORTS_DIR, "occupancy_map.npy")
    try:
        import numpy as np
        np.save(npy_path, grid_prob)
        out["npy"] = npy_path
    except Exception:
        out["npy"] = ""

    png_path = os.path.join(REPORTS_DIR, "occupancy_map.png")
    try:
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111)
        im = ax.imshow(grid_prob, origin="lower", cmap="viridis")
        ax.set_title("Occupancy Map (Probability)")
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        fig.tight_layout()
        fig.savefig(png_path, dpi=140)
        plt.close(fig)
        out["png"] = png_path
    except Exception:
        out["png"] = ""

    binary_png_path = os.path.join(REPORTS_DIR, "occupancy_map_binary.png")
    try:
        import matplotlib.pyplot as plt
        import numpy as np

        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111)
        # Occupied = 1 when probability >= 0.55
        binary = (grid_prob >= 0.55).astype(np.uint8)
        ax.imshow(binary, origin="lower", cmap="gray")
        ax.set_title("Occupancy Map (Binary >= 0.55)")
        fig.tight_layout()
        fig.savefig(binary_png_path, dpi=140)
        plt.close(fig)
        out["png_binary"] = binary_png_path
    except Exception:
        out["png_binary"] = ""

    return out


def main():
    print("roboai_controller (MVP primitive planner) loaded")
    command = resolve_command()
    print("Command:", command)
    plan_mode = os.getenv("ROBOAI_PLAN_MODE", "primitive").strip().lower()
    if plan_mode not in {"primitive", "waypoint"}:
        plan_mode = "primitive"
    print("Plan mode:", plan_mode)

    robot = Robot()
    log = RunLogger()

    sensors = Sensors(robot)
    drive = Drive(robot, LEFT_MOTOR_NAME, RIGHT_MOTOR_NAME)
    est = StateEstimator()

    lidar = LidarWrapper(robot, name="LDS-01", timestep=TIME_STEP_MS)
    occ_grid = OccupancyGrid(width_m=20.0, height_m=20.0, resolution=0.05)

    # High-level plan
    constraints = {"speed_limit": 0.5, "avoid": []}
    if plan_mode == "waypoint":
        wp = get_waypoint_plan(command)
        plan = wp.get("steps", [{"op": "stop"}]) if isinstance(wp, dict) else [{"op": "stop"}]
        constraints = wp.get("constraints", constraints) if isinstance(wp, dict) else constraints
    else:
        plan = get_plan(command)
    print("Plan:", plan)
    log.event(op="plan_built", command=command, plan_mode=plan_mode, plan=plan, constraints=constraints)

    execu = PlanExecutor(robot, drive, sensors, log)
    execu.load(plan, constraints=constraints)

    dt = TIME_STEP_MS / 1000.0
    elapsed = 0.0
    while elapsed < RUN_SECONDS:
        if robot.step(TIME_STEP_MS) == -1:
            break

        # Sense
        ir = sensors.read_ir()
        enc = sensors.read_encoders()

        # State
        state = est.update(enc, dt)

        ranges, angle_min, angle_inc, range_max = lidar.read_scan()
        x, y, th = state.x, state.y, state.theta
        occ_grid.update_from_scan((x, y, th), ranges, angle_min, angle_inc, range_max)

        # Plan + Act
        done = execu.step(dt, ir, state=state)

        # Log tick state
        lcmd, rcmd = execu.last_cmd
        log.event(
            op="spa_tick",
            x=state.x, y=state.y, theta=state.theta,
            vl=state.vl, vr=state.vr,
            left_cmd=lcmd, right_cmd=rcmd
        )

        if done:
            break

        elapsed += dt

    drive.stop()
    log.event(op="stop")

    map_out = save_occupancy_artifacts(occ_grid)
    log.event(op="map_saved", outputs=map_out)

    log.close()
    write_demo_artifact(command=command, plan=plan, log_path=log.path)
    write_explanation_artifact(
        command=command,
        plan_type=plan_mode,
        plan_steps=plan,
        events=log.buffer.get("events", []),
    )
    print("roboai_controller finished")


if __name__ == "__main__":
    main()
