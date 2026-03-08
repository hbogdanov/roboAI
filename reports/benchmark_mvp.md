# RoboAI MVP Benchmark

## Project Boundary
RoboAI is a Webots-based mobile robot system that converts natural-language commands into executable motion plans, performs obstacle-aware navigation with onboard sensors, and logs trajectories / sensor data for analysis.

## KPI Summary
- Runs evaluated: **21**
- Command success rate: **47.6%**
- Path completion (avg): **77.4%**
- Obstacle avoidance success: **76.2%**
- Planning parse success: **100.0%**
- Map generation output: **81.0%**
- Goto success (`goto_done/goto_start`): **80.3%**
- Collision warnings (total): **31**
- Goto stuck events (total): **33**
- Collision burst escapes (total): **6**
- Goto failed (total): **7**
- Time in AVOID (total): **31.79 s**
- Replans per goal (avg): **0.0**

## Metric Definitions
- `planning_parse_success`: `plan_built.plan` exists and contains only allowed MVP ops.
- `path_completion`: fraction of planned steps with matching completion events in the log.
- `command_success`: parse success + full path completion + `goto_done/goto_start` = 1.0 when goto is used.
- `obstacle_avoidance_success`: no `collision_warning`, no `goto_stuck`, no `collision_burst_escape`.
- `avoid_time_s`: time accumulated from `spa_tick.behavior_state == AVOID`.
- `replans_count`: `path_planned - goto_start` (extra global replans beyond initial per-goal plan).
- `goto_success_ratio`: `goto_done_count / goto_start_count`.
- `map_generation_output`: run includes `map_saved` event with `.npy` or `.png` output.

## Per-Run Table
| Run | Command Success | Path Completion | Goto Success | GotoFailed | CollWarn | GotoStuck | BurstEsc | AVOID(s) | Replans/Goal | Parse Success | Map Output | Distance (m) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `run_20251005_161349.json` | Yes | 100.0% | 100.0% | 0 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | No | 0.112 |
| `run_20251005_205036.json` | No | 50.0% | 100.0% | 0 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | No | 0.112 |
| `run_20251005_205040.json` | Yes | 100.0% | 100.0% | 0 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | No | 0.112 |
| `run_20251027_192949.json` | Yes | 100.0% | 100.0% | 0 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | No | 0.112 |
| `run_20260307_175715.json` | Yes | 100.0% | 100.0% | 0 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | Yes | 0.112 |
| `run_20260307_175730.json` | Yes | 100.0% | 100.0% | 0 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | Yes | 0.022 |
| `run_20260307_180004.json` | Yes | 100.0% | 100.0% | 0 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | Yes | 0.007 |
| `run_20260307_180035.json` | Yes | 100.0% | 100.0% | 0 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | Yes | 0.007 |
| `run_20260307_180049.json` | Yes | 100.0% | 100.0% | 0 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | Yes | 0.007 |
| `run_20260307_180513.json` | Yes | 100.0% | 100.0% | 0 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | Yes | 0.0 |
| `run_20260307_181336.json` | No | 33.3% | 0.0% | 0 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | Yes | 0.056 |
| `run_20260307_185816.json` | No | 33.3% | 0.0% | 0 | 3 | 11 | 0 | 7.27 | 0.0 | Yes | Yes | 1.058 |
| `run_20260307_190317.json` | No | 33.3% | 0.0% | 0 | 28 | 11 | 0 | 10.341 | 0.0 | Yes | Yes | 0.717 |
| `run_20260307_191047.json` | No | 33.3% | 0.0% | 0 | 0 | 11 | 0 | 7.176 | 0.0 | Yes | Yes | 0.375 |
| `run_20260307_195119.json` | No | 66.7% | 0.0% | 1 | 0 | 0 | 3 | 3.51 | 0.0 | Yes | Yes | 0.042 |
| `run_20260307_195209.json` | Yes | 100.0% | 100.0% | 0 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | Yes | 0.0 |
| `run_20260307_195300.json` | No | 100.0% | 16.7% | 1 | 0 | 0 | 3 | 3.493 | 0.0 | Yes | Yes | 0.043 |
| `run_20260307_221538.json` | No | 75.0% | 100.0% | 2 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | Yes | 0.0 |
| `run_20260307_221609.json` | No | 66.7% | 100.0% | 1 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | Yes | 0.001 |
| `run_20260307_221623.json` | No | 66.7% | 100.0% | 1 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | Yes | 0.001 |
| `run_20260307_221715.json` | No | 66.7% | 100.0% | 1 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | Yes | 0.001 |
