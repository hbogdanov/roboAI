from controller import Supervisor
import json
import os
import random
import time
import glob
import math


TIME_STEP = 64
RUN_SECONDS = float(os.getenv("ROBOAI_EVAL_RUN_SECONDS", "25"))
TRIALS = int(os.getenv("ROBOAI_EVAL_TRIALS", "3"))
SEED = int(os.getenv("ROBOAI_EVAL_SEED", "42"))
ENABLE_EVAL = os.getenv("ROBOAI_SUPERVISOR_ENABLE", "0").strip().lower() in {"1", "true", "yes", "on"}
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
OUT_DIR = os.path.join(REPO_ROOT, "reports")
OUT_PATH = os.path.join(OUT_DIR, "supervisor_eval.json")
LOG_DIR = os.path.join(REPO_ROOT, "data", "logs")


OBSTACLE_DEFS = ["OBS1", "OBS2", "DESK_A", "DESK_B", "WALL_SEG"]


def _node(supervisor: Supervisor, def_name: str):
    return supervisor.getFromDef(def_name)


def _set_robot_pose(supervisor: Supervisor, x: float, y: float, yaw: float):
    robot = _node(supervisor, "EPUCK")
    if robot is None:
        robot = supervisor.getFromDef("e-puck")
    if robot is None:
        return False
    trans = robot.getField("translation")
    rot = robot.getField("rotation")
    # Preserve current Z to avoid spawning under/inside the floor.
    cur = trans.getSFVec3f()
    z = cur[2] if len(cur) >= 3 else 0.02
    if z < 0.01:
        z = 0.02
    trans.setSFVec3f([x, y, z])
    rot.setSFRotation([0.0, 0.0, 1.0, yaw])
    robot.resetPhysics()
    return True


def _randomize_obstacles(supervisor: Supervisor):
    for def_name in OBSTACLE_DEFS:
        n = _node(supervisor, def_name)
        if n is None:
            continue
        t = n.getField("translation")
        # Keep obstacles inside the arena envelope.
        x = random.uniform(-0.75, 0.75)
        y = random.uniform(-0.75, 0.75)
        z = t.getSFVec3f()[2]
        t.setSFVec3f([x, y, z])


def _run_trial(supervisor: Supervisor, trial_id: int):
    before = set(glob.glob(os.path.join(LOG_DIR, "run_*.json")))
    start_x = random.uniform(-0.45, 0.45)
    start_y = random.uniform(-0.45, 0.45)
    start_yaw = random.uniform(-3.14, 3.14)
    _set_robot_pose(supervisor, start_x, start_y, start_yaw)
    _randomize_obstacles(supervisor)
    supervisor.simulationResetPhysics()

    steps = int((RUN_SECONDS * 1000.0) / TIME_STEP)
    start = time.time()
    for _ in range(steps):
        if supervisor.step(TIME_STEP) == -1:
            break

    duration = time.time() - start
    after = set(glob.glob(os.path.join(LOG_DIR, "run_*.json")))
    new_logs = sorted(list(after - before))
    metrics = _read_trial_metrics(new_logs[-1]) if new_logs else {}
    return {
        "trial": trial_id,
        "start_pose": {"x": start_x, "y": start_y, "yaw": start_yaw},
        "duration_s": round(duration, 3),
        "collision_detected": bool(metrics.get("collision_count", 0) > 0),
        "completed": bool(metrics.get("success", True)),
        "metrics": metrics,
    }


def _read_trial_metrics(path: str):
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return {}
    events = data.get("events", [])
    goto_start = sum(1 for e in events if e.get("op") == "goto_start")
    goto_done = sum(1 for e in events if e.get("op") == "goto_done")
    success = goto_start > 0 and goto_done >= goto_start
    if goto_start == 0:
        success = any(e.get("op") == "stop" for e in events)
    replans = max(0, sum(1 for e in events if e.get("op") == "path_planned") - goto_start)
    collisions = sum(1 for e in events if e.get("op") in {"collision_warning", "collision_burst_escape"})

    tvals = [float(e.get("t")) for e in events if "t" in e]
    runtime_s = (max(tvals) - min(tvals)) if len(tvals) >= 2 else 0.0

    ticks = [e for e in events if e.get("op") == "spa_tick" and "x" in e and "y" in e]
    path_len = 0.0
    for a, b in zip(ticks[:-1], ticks[1:]):
        dx = float(b["x"]) - float(a["x"])
        dy = float(b["y"]) - float(a["y"])
        path_len += math.hypot(dx, dy)
    start_xy = (float(ticks[0]["x"]), float(ticks[0]["y"])) if ticks else None

    plan_evt = next((e for e in events if e.get("op") == "plan_built"), {})
    goal_xy = None
    for s in plan_evt.get("plan", []) if isinstance(plan_evt.get("plan"), list) else []:
        if str(s.get("op", "")).lower() == "goto":
            try:
                goal_xy = (float(s.get("x")), float(s.get("y")))
            except Exception:
                goal_xy = None
            break
    straight = 0.0
    if start_xy is not None and goal_xy is not None:
        straight = math.hypot(goal_xy[0] - start_xy[0], goal_xy[1] - start_xy[1])
    path_eff = (straight / path_len) if path_len > 0.0 else 0.0

    goal_err = 0.0
    done = [e for e in events if e.get("op") == "goto_done"]
    prog = [e for e in events if e.get("op") == "goto_progress"]
    if done:
        goal_err = float(done[-1].get("goal_error_m", 0.0))
    elif prog:
        goal_err = float(prog[-1].get("goal_error_m", 0.0))

    return {
        "log_file": os.path.basename(path),
        "success": bool(success),
        "final_goal_error_m": round(goal_err, 3),
        "replans": int(replans),
        "runtime_s": round(runtime_s, 3),
        "collision_count": int(collisions),
        "path_efficiency": round(path_eff, 3),
    }


def main():
    random.seed(SEED)
    supervisor = Supervisor()
    os.makedirs(OUT_DIR, exist_ok=True)

    if not ENABLE_EVAL:
        print("roboai_supervisor: passive mode (set ROBOAI_SUPERVISOR_ENABLE=1 to run trials)")
        while supervisor.step(TIME_STEP) != -1:
            pass
        return

    trials = []
    for i in range(TRIALS):
        trials.append(_run_trial(supervisor, i + 1))

    result = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "trials": trials,
        "summary": {
            "trials": len(trials),
            "completion_rate": round(100.0 * sum(1 for t in trials if t["completed"]) / max(1, len(trials)), 1),
            "collision_rate": round(100.0 * sum(1 for t in trials if t["collision_detected"]) / max(1, len(trials)), 1),
            "success_rate": round(100.0 * sum(1 for t in trials if t.get("metrics", {}).get("success")) / max(1, len(trials)), 1),
            "avg_final_goal_error_m": round(
                sum(float(t.get("metrics", {}).get("final_goal_error_m", 0.0)) for t in trials) / max(1, len(trials)), 3
            ),
            "avg_replans": round(
                sum(float(t.get("metrics", {}).get("replans", 0.0)) for t in trials) / max(1, len(trials)), 3
            ),
            "avg_runtime_s": round(
                sum(float(t.get("metrics", {}).get("runtime_s", 0.0)) for t in trials) / max(1, len(trials)), 3
            ),
            "avg_collision_count": round(
                sum(float(t.get("metrics", {}).get("collision_count", 0.0)) for t in trials) / max(1, len(trials)), 3
            ),
            "avg_path_efficiency": round(
                sum(float(t.get("metrics", {}).get("path_efficiency", 0.0)) for t in trials) / max(1, len(trials)), 3
            ),
        },
    }
    with open(OUT_PATH, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    print(f"Saved supervisor eval to {OUT_PATH}")

    while supervisor.step(TIME_STEP) != -1:
        break


if __name__ == "__main__":
    main()
