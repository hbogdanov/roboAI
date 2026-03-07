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

## What The Robot Does

- Reads an NL command.
- Builds a plan according to runtime mode.
- Executes plan on a differential-drive robot in Webots.
- Uses IR for obstacle-aware behavior.
- Uses encoders for odometry state (`x`, `y`, `theta`).
- Updates an occupancy grid from lidar scans.
- Writes JSON logs plus markdown artifacts.

## Input

Command source precedence:
1. `--command "..."`
2. `ROBOAI_COMMAND`
3. `--command-file <path>` (default: `demo/mvp_command.txt`)
4. Built-in fallback command

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
- `executor.py`: shared execution engine for primitive + waypoint ops.
- `state.py`: odometry estimator.
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

## Output Artifacts

- `data/logs/run_*.json`: structured event logs.
- `reports/latest_demo_artifact.md`: command + resolved plan + log path.
- `reports/latest_explanation.md`: explicit explanation output (intent, interpreted plan, execution evidence).
- `reports/occupancy_map.npy`: occupancy probability array.
- `reports/occupancy_map.png`: occupancy heatmap snapshot.
- `reports/occupancy_map_binary.png`: binary occupancy snapshot.

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
- Occupancy map export.
- Explanation artifact generation.
- Benchmark and run-summary generation.

Partial:
- `return_base` is currently stubbed.
- Turn calibration remains open-loop.

## Experimental Module

`models/planning/t5_plan/` is the experimental training/inference module.
In waypoint mode, set `ROBOAI_USE_WAYPOINT_LLM=1` to attempt model-driven planning; otherwise deterministic fallback planning is used.
