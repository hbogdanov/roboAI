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
    DEFAULT_MODE_FILE,
    REPORTS_DIR,
)
from motion import Drive
from sensors import Sensors
from sensors import CameraWrapper
from logger import RunLogger
from state import StateEstimator
from planner_text import get_plan
from waypoint_planner import get_waypoint_plan
from executor import PlanExecutor
from waypoint_planner import get_goal_xy, get_goals_source, resolve_world_name
from sensors import LidarWrapper
from occupancy_grid import OccupancyGrid
from pose_fusion import PoseFusion
from perception import detect_color_marker_bgra

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


def infer_plan_mode(command: str) -> str:
    t = command.lower()
    waypoint_hints = [
        "go to",
        "goto",
        "face",
        "station",
        "charging dock",
        "door",
        "explore",
        "build a map",
    ]
    if any(k in t for k in waypoint_hints):
        return "waypoint"
    return "primitive"


def planner_settings_for_world(world_name: str) -> dict:
    wn = (world_name or "").strip().lower()
    # Office is cluttered: wider safety margin and goal clearance.
    if wn == "world_office":
        cfg = {
            "block_unknown": True,
            "inflate_cells": 2,
            "goal_clearance_cells": 0,
            "max_goal_snap_cells": 12,
            "local_avoid_mode": "lidar",
            "replan_limit": 6,
            "path_stride": 2,
        }
    elif wn == "world_obstacles":
        cfg = {
            "block_unknown": True,
            "inflate_cells": 4,
            "goal_clearance_cells": 1,
            "max_goal_snap_cells": 8,
            "local_avoid_mode": "lidar",
            "replan_limit": 6,
            "path_stride": 3,
        }
    elif wn == "world_empty":
        cfg = {
            "block_unknown": True,
            "inflate_cells": 2,
            "goal_clearance_cells": 0,
            "max_goal_snap_cells": 12,
            "local_avoid_mode": "lidar",
            "replan_limit": 6,
            "path_stride": 4,
        }
    else:
        cfg = {
            "block_unknown": True,
            "inflate_cells": 3,
            "goal_clearance_cells": 1,
            "max_goal_snap_cells": 8,
            "local_avoid_mode": "lidar",
            "replan_limit": 6,
            "path_stride": 3,
        }

    env_inflate = os.getenv("ROBOAI_INFLATE_CELLS", "").strip()
    if env_inflate:
        try:
            cfg["inflate_cells"] = max(0, int(env_inflate))
        except Exception:
            pass
    env_clear = os.getenv("ROBOAI_GOAL_CLEARANCE_CELLS", "").strip()
    if env_clear:
        try:
            cfg["goal_clearance_cells"] = max(0, int(env_clear))
        except Exception:
            pass
    env_snap = os.getenv("ROBOAI_MAX_GOAL_SNAP_CELLS", "").strip()
    if env_snap:
        try:
            cfg["max_goal_snap_cells"] = max(1, int(env_snap))
        except Exception:
            pass
    env_stride = os.getenv("ROBOAI_PATH_STRIDE", "").strip()
    if env_stride:
        try:
            cfg["path_stride"] = max(1, int(env_stride))
        except Exception:
            pass
    env_avoid = os.getenv("ROBOAI_LOCAL_AVOID_MODE", "").strip().lower()
    if env_avoid in {"lidar", "ir"}:
        cfg["local_avoid_mode"] = env_avoid
    return cfg


def resolve_plan_mode(command: str) -> str:
    """
    Plan mode precedence:
      1) --plan-mode primitive|waypoint
      2) ROBOAI_PLAN_MODE env var
      3) --mode-file <path> (or default demo/mvp_mode.txt)
      4) infer from command text
    """
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--plan-mode", type=str, default="")
    parser.add_argument("--mode-file", type=str, default=DEFAULT_MODE_FILE)
    args, _ = parser.parse_known_args(sys.argv[1:])

    cli_mode = (args.plan_mode or "").strip().lower()
    if cli_mode in {"primitive", "waypoint"}:
        return cli_mode

    env_mode = os.getenv("ROBOAI_PLAN_MODE", "").strip().lower()
    if env_mode in {"primitive", "waypoint"}:
        return env_mode

    mode_file = (args.mode_file or "").strip()
    if mode_file and os.path.exists(mode_file):
        with open(mode_file, "r", encoding="utf-8") as f:
            file_mode = f.read().strip().lower()
        if file_mode in {"primitive", "waypoint"}:
            return file_mode

    return infer_plan_mode(command)


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
        f"- `state_transition`: `{counts.get('state_transition', 0)}`",
        f"- `spa_tick`: `{counts.get('spa_tick', 0)}`",
        f"- `turn_done`: `{counts.get('turn_done', 0)}`",
        f"- `scan`: `{counts.get('scan', 0)}`",
        f"- `goto_done`: `{counts.get('goto_done', 0)}`",
        f"- `goto_failed`: `{counts.get('goto_failed', 0)}`",
        f"- `goto_abort`: `{counts.get('goto_abort', 0)}`",
        f"- `goto_start`: `{counts.get('goto_start', 0)}`",
        f"- `goto_progress`: `{counts.get('goto_progress', 0)}`",
        f"- `goto_stuck`: `{counts.get('goto_stuck', 0)}`",
        f"- `goto_recovery_tick`: `{counts.get('goto_recovery_tick', 0)}`",
        f"- `path_planned`: `{counts.get('path_planned', 0)}`",
        f"- `path_plan_failed`: `{counts.get('path_plan_failed', 0)}`",
        f"- `goal_snapped`: `{counts.get('goal_snapped', 0)}`",
        f"- `goal_clearance_checked`: `{counts.get('goal_clearance_checked', 0)}`",
        f"- `lidar_avoid`: `{counts.get('lidar_avoid', 0)}`",
        f"- `frontier_detected`: `{counts.get('frontier_detected', 0)}`",
        f"- `frontier_selected`: `{counts.get('frontier_selected', 0)}`",
        f"- `frontier_reached`: `{counts.get('frontier_reached', 0)}`",
        f"- `frontier_failed`: `{counts.get('frontier_failed', 0)}`",
        f"- `explore_done`: `{counts.get('explore_done', 0)}`",
        f"- `face_done`: `{counts.get('face_done', 0)}`",
        f"- `pose_correction`: `{counts.get('pose_correction', 0)}`",
        f"- `camera_marker`: `{counts.get('camera_marker', 0)}`",
        f"- `collision_warning`: `{counts.get('collision_warning', 0)}`",
        f"- `collision_burst_escape`: `{counts.get('collision_burst_escape', 0)}`",
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
    plan_mode = resolve_plan_mode(command)
    print("Plan mode:", plan_mode)
    world_name = resolve_world_name()
    print("World name:", world_name)

    robot = Robot()
    log = RunLogger()

    sensors = Sensors(robot)
    drive = Drive(robot, LEFT_MOTOR_NAME, RIGHT_MOTOR_NAME)
    est = StateEstimator()
    fusion = PoseFusion()

    lidar = LidarWrapper(robot, name="LDS-01", timestep=TIME_STEP_MS)
    camera = CameraWrapper(robot, name="camera", timestep=TIME_STEP_MS)
    occ_grid = OccupancyGrid(width_m=20.0, height_m=20.0, resolution=0.05)

    # High-level plan
    constraints = {"speed_limit": 0.5, "avoid": [], "planner": planner_settings_for_world(world_name)}
    if plan_mode == "waypoint":
        wp = get_waypoint_plan(command)
        plan = wp.get("steps", [{"op": "stop"}]) if isinstance(wp, dict) else [{"op": "stop"}]
        constraints = wp.get("constraints", constraints) if isinstance(wp, dict) else constraints
        if not isinstance(constraints, dict):
            constraints = {"speed_limit": 0.5, "avoid": []}
        if "planner" not in constraints or not isinstance(constraints.get("planner"), dict):
            constraints["planner"] = planner_settings_for_world(world_name)
        else:
            merged = planner_settings_for_world(world_name)
            merged.update(constraints.get("planner", {}))
            constraints["planner"] = merged
        goals_source = get_goals_source()
    else:
        plan = get_plan(command)
        goals_source = ""
    print("Plan:", plan)
    log.event(
        op="plan_built",
        command=command,
        plan_mode=plan_mode,
        world_name=world_name,
        goals_source=goals_source,
        plan=plan,
        constraints=constraints,
    )

    execu = PlanExecutor(robot, drive, sensors, log)
    execu.load(plan, constraints=constraints)

    dt = TIME_STEP_MS / 1000.0
    elapsed = 0.0
    tick = 0
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

        # Optional camera-based marker detection.
        if camera.available() and (tick % 10 == 0):
            img, w, h = camera.read_image()
            det = detect_color_marker_bgra(img, w, h)
            log.event(op="camera_marker", **det)

        # Plan + Act
        done = execu.step(
            dt,
            ir,
            state=state,
            occ_grid=occ_grid,
            lidar_scan=(ranges, angle_min, angle_inc, range_max),
        )

        # Optional landmark correction when a named goal is reached.
        reached = execu.last_goal_reached
        if reached is not None:
            goal_name = reached.get("goal")
            landmark_xy = get_goal_xy(goal_name) if isinstance(goal_name, str) else None
            corr = fusion.maybe_correct_with_landmark(state=state, landmark_xy=landmark_xy)
            log.event(op="pose_correction", goal=goal_name, **corr)
            execu.last_goal_reached = None

        pose_conf = fusion.update_confidence(
            dt=dt,
            encoders_available=(enc is not None),
            lidar_ranges_count=len(ranges),
            wheel_speed_mag=abs(state.vl) + abs(state.vr),
        )

        # Log tick state
        lcmd, rcmd = execu.last_cmd
        log.event(
            op="spa_tick",
            x=state.x, y=state.y, theta=state.theta,
            vl=state.vl, vr=state.vr,
            left_cmd=lcmd, right_cmd=rcmd,
            pose_confidence=pose_conf,
            behavior_state=execu.behavior_state,
        )

        if done:
            break

        elapsed += dt
        tick += 1

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
