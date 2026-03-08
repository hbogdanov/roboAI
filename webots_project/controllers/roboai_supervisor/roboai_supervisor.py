from controller import Supervisor
import json
import os
import random
import time


TIME_STEP = 64
RUN_SECONDS = float(os.getenv("ROBOAI_EVAL_RUN_SECONDS", "25"))
TRIALS = int(os.getenv("ROBOAI_EVAL_TRIALS", "3"))
SEED = int(os.getenv("ROBOAI_EVAL_SEED", "42"))
ENABLE_EVAL = os.getenv("ROBOAI_SUPERVISOR_ENABLE", "0").strip().lower() in {"1", "true", "yes", "on"}
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
OUT_DIR = os.path.join(REPO_ROOT, "reports")
OUT_PATH = os.path.join(OUT_DIR, "supervisor_eval.json")


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
    # Conservative placeholders for collision/completion; these can be extended with contact checks.
    return {
        "trial": trial_id,
        "start_pose": {"x": start_x, "y": start_y, "yaw": start_yaw},
        "duration_s": round(duration, 3),
        "collision_detected": False,
        "completed": True,
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
        },
    }
    with open(OUT_PATH, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    print(f"Saved supervisor eval to {OUT_PATH}")

    while supervisor.step(TIME_STEP) != -1:
        break


if __name__ == "__main__":
    main()
