# RoboAI MVP Benchmark

## Project Boundary
RoboAI is a Webots-based mobile robot system that converts natural-language commands into executable motion plans, performs obstacle-aware navigation with onboard sensors, and logs trajectories / sensor data for analysis.

## KPI Summary
- Runs evaluated: **4**
- Command success rate: **75.0%**
- Path completion (avg): **87.5%**
- Obstacle avoidance success: **100.0%**
- Planning parse success: **100.0%**
- Map generation output: **0.0%**

## Metric Definitions
- `planning_parse_success`: `plan_built.plan` exists and contains only allowed MVP ops.
- `path_completion`: fraction of planned steps with matching completion events in the log.
- `command_success`: parse success and full path completion.
- `obstacle_avoidance_success`: max `front` level during forward ticks < `0.95`.
- `map_generation_output`: run includes `map_saved` event with `.npy` or `.png` output.

## Per-Run Table
| Run | Command Success | Path Completion | Obstacle Avoidance | Parse Success | Map Output | Distance (m) |
|---|---:|---:|---:|---:|---:|---:|
| `run_20251005_161349.json` | Yes | 100.0% | Yes | Yes | No | 0.112 |
| `run_20251005_205036.json` | No | 50.0% | Yes | Yes | No | 0.112 |
| `run_20251005_205040.json` | Yes | 100.0% | Yes | Yes | No | 0.112 |
| `run_20251027_192949.json` | Yes | 100.0% | Yes | Yes | No | 0.112 |
