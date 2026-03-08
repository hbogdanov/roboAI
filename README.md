# RoboAI

RoboAI is a Webots-based mobile robot system that converts natural-language commands into executable motion plans, performs obstacle-aware navigation with onboard sensors, and logs trajectories / sensor data for analysis.

## Project Boundary

This project focuses on:
- NL command interpretation for mobile robot actions.
- Execution in Webots with onboard sensor feedback.
- Reproducible logging, map export, and evaluation artifacts.

This is not positioned as a general-purpose autonomy stack.

## Runtime Plan Modes

RoboAI supports two explicit runtime modes:
- `primitive` (default): NL -> `forward` / `turn` / `scan` / `wait` / `return_base` / `stop`
- `waypoint`: NL -> `goto` / `face` / `wait` / `stop`

Select mode with:
- `ROBOAI_PLAN_MODE=primitive`
- `ROBOAI_PLAN_MODE=waypoint`

Goal-directed waypoint examples:
- `go to station A`
- `go to the charging dock`
- `go to the door and face 90 degrees`
- `explore the room and build a map`

## What The Robot Does

- Reads an NL command.
- Builds a plan according to runtime mode.
- Executes plan on a differential-drive robot in Webots.
- Uses IR for obstacle-aware behavior.
- Uses encoders for odometry state (`x`, `y`, `theta`).
- Updates an occupancy grid from lidar scans.
- Writes JSON logs plus markdown artifacts.

## Behavior State Machine

Runtime behavior states are explicitly logged through transitions:
- `IDLE`
- `PLAN`
- `NAVIGATE`
- `AVOID`
- `SCAN`
- `RETURN_HOME`
- `DONE`

Transition events are emitted as `state_transition` in run logs.

## Input

Command source precedence:
1. `--command "..."`
2. `ROBOAI_COMMAND`
3. `--command-file <path>` (default: `demo/mvp_command.txt`)
4. Built-in fallback command

Plan mode source precedence:
1. `--plan-mode primitive|waypoint`
2. `ROBOAI_PLAN_MODE`
3. `--mode-file <path>` (default: `demo/mvp_mode.txt`)
4. Auto-infer from command text (`go to`/`face`/`explore` -> waypoint)

## Task Abstraction (Primitive Mode)

High-level phrases compile to primitive plans:
- `scan the room`
- `go forward and avoid obstacles`
- `patrol for 10 seconds`
- `return to base`

## Architecture

- `webots_project/controllers/roboai_controller/roboai_controller.py`: main loop, mode routing, artifact export.
- `planner_text.py`: primitive plan generation.
- `waypoint_planner.py`: waypoint plan generation (deterministic fallback + optional T5).
- `path_planner.py`: occupancy-grid A* global path planner.
- `frontier.py`: frontier extraction (free-to-unknown boundaries).
- `executor.py`: shared execution engine for primitive + waypoint ops (closed-loop path following + local obstacle correction + frontier exploration).
- `state.py`: odometry estimator.
- `pose_fusion.py`: odometry confidence tracking + optional landmark-based pose correction.
- `occupancy_grid.py`: map update.
- `logger.py`: structured JSON logs.

## Run Demo

### Prerequisites

- Python 3.10+
- Webots (world authored in R2023b format)
- `pip install -r requirements.txt`

Optional planning extras:
- `pip install -r models/planning/t5_plan/requirements.txt`

### Reproducible Scenario

1. Open `webots_project/worlds/roboAI_fixed.wbt` in Webots.
2. Leave `demo/mvp_command.txt` as-is (or edit it).
3. Run simulation.

## Webots Scenario Matrix

Available worlds:
- `webots_project/worlds/world_empty.wbt`
- `webots_project/worlds/world_obstacles.wbt`
- `webots_project/worlds/world_office.wbt`

Randomized world variants:
```bash
python scripts/generate_random_worlds.py --count 10 --seed 123
```
Outputs generated worlds under:
- `webots_project/worlds/generated/`

Supervisor-based evaluation controller:
- `webots_project/controllers/roboai_supervisor/roboai_supervisor.py`

Supervisor evaluation outputs:
- `reports/supervisor_eval.json`

## Practical Run + Debug Flow

If running from the Webots GUI (without shell env vars), use demo files:
- Command file: `demo/mvp_command.txt`
- Mode file: `demo/mvp_mode.txt` (`primitive` or `waypoint`)

Recommended quick test:
1. Set `demo/mvp_mode.txt` to `waypoint`.
2. Set `demo/mvp_command.txt` to `go to the door and face 90 degrees`.
3. Open `webots_project/worlds/world_office.wbt`.
4. Click `Reset`, then `Play`.
5. Verify console prints:
   - `Plan mode: waypoint`
   - `Plan: [{'op': 'goto', ...}, {'op': 'face', ...}, ...]`

After run:
```bash
python scripts/log_summary.py
python scripts/evaluate_mvp.py
```

Useful debug artifacts:
- `data/logs/run_*.json` (latest event trace)
- `reports/run_summary.md`
- `reports/latest_explanation.md`
- `reports/occupancy_map.png`
- `reports/benchmark_mvp.md`

Troubleshooting:
- If spawn looks wrong, close and reopen the world from disk before rerun.
- `WARNING ... remote control library ... not found` is benign for this project.
- `Forced termination` on reset is typically expected when restarting controllers quickly.

## Output Artifacts

- `data/logs/run_*.json`: structured event logs.
- `reports/latest_demo_artifact.md`: command + resolved plan + log path.
- `reports/latest_explanation.md`: explicit explanation output (intent, interpreted plan, execution evidence).
- `reports/occupancy_map.npy`: occupancy probability array.
- `reports/occupancy_map.png`: occupancy heatmap snapshot.
- `reports/occupancy_map_binary.png`: binary occupancy snapshot.

Sensor-fusion-ish outputs (without EKF/SLAM claims):
- `spa_tick.pose_confidence`: confidence score for current odometry-based pose estimate.
- `pose_correction` events: optional landmark correction metadata when a named goal is reached.

Camera perception output (if a camera device named `camera` exists in the robot/world):
- `camera_marker` events with dominant color label and score (simple colored-marker detection).

## Benchmarks And Summaries

KPI benchmark page:
```bash
python scripts/evaluate_mvp.py
```
Outputs:
- `reports/benchmark_mvp.md`
- `reports/benchmark_mvp.json`

Run summary page:
```bash
python scripts/log_summary.py
```
Outputs:
- `reports/run_summary.md`
- `reports/run_summary.json`

## Status

Working now:
- Primitive mode execution.
- Waypoint mode execution (`goto`/`face`/`wait`/`stop`).
- Goal-directed semantic lookup (`station A`, `charging dock`, `door`).
- A* path planning + waypoint path following for `goto`.
- Frontier exploration loop (`explore`) with nearest-frontier target selection.
- Obstacle-aware local correction while tracking waypoint paths.
- Occupancy map export.
- Explanation artifact generation.
- Benchmark and run-summary generation.

Partial:
- `return_base` is currently stubbed.
- Turn calibration remains open-loop.

Closed-loop `goto` logging:
- `goto_start`
- `goto_progress`
- `goto_done`
- `goal_error_m` (inside `goto_progress`/`goto_done`)

Pose estimation language used in this repo:
- Odometry + lidar-based map updates are implemented.
- Optional landmark-based pose correction is implemented.
- No EKF or full SLAM is claimed.

## Experimental Module

`models/planning/t5_plan/` is the experimental training/inference module.
In waypoint mode, set `ROBOAI_USE_WAYPOINT_LLM=1` to attempt model-driven planning; otherwise deterministic fallback planning is used.
