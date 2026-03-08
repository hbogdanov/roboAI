# RoboAI MVP Benchmark

## Project Boundary
RoboAI is a Webots-based mobile robot system that converts natural-language commands into executable motion plans, performs obstacle-aware navigation with onboard sensors, and logs trajectories / sensor data for analysis.

## KPI Summary
- Runs evaluated: **40**
- Command success rate: **25.0%**
- Path completion (avg): **69.0%**
- Obstacle avoidance success: **45.0%**
- Planning parse success: **100.0%**
- Map generation output: **90.0%**
- Goto success (`goto_done/goto_start`): **65.8%**
- Collision warnings (total): **236**
- Goto stuck events (total): **71**
- Collision burst escapes (total): **54**
- Goto failed (total): **22**
- Time in AVOID (total): **124.797 s**
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
| `run_20260307_222721.json` | No | 66.7% | 100.0% | 1 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | Yes | 0.001 |
| `run_20260307_222922.json` | No | 66.7% | 100.0% | 1 | 0 | 0 | 0 | 0.0 | 0.0 | Yes | Yes | 0.001 |
| `run_20260307_224018.json` | No | 66.7% | 0.0% | 1 | 0 | 1 | 0 | 0.551 | 0.0 | Yes | Yes | 0.003 |
| `run_20260307_224251.json` | No | 66.7% | 0.0% | 1 | 0 | 2 | 1 | 3.316 | 0.0 | Yes | Yes | 0.08 |
| `run_20260307_224903.json` | No | 66.7% | 0.0% | 1 | 0 | 2 | 3 | 4.723 | 0.0 | Yes | Yes | 0.046 |
| `run_20260307_225146.json` | No | 66.7% | 0.0% | 1 | 0 | 3 | 0 | 1.347 | 0.0 | Yes | Yes | 0.076 |
| `run_20260307_230033.json` | No | 66.7% | 0.0% | 1 | 0 | 3 | 1 | 3.203 | 0.0 | Yes | Yes | 0.07 |
| `run_20260307_230100.json` | No | 66.7% | 0.0% | 1 | 0 | 3 | 1 | 3.14 | 0.0 | Yes | Yes | 0.07 |
| `run_20260307_230428.json` | No | 33.3% | 0.0% | 0 | 15 | 1 | 2 | 4.996 | 0.0 | Yes | Yes | 0.281 |
| `run_20260307_230812.json` | No | 66.7% | 0.0% | 1 | 21 | 2 | 4 | 7.911 | 0.0 | Yes | Yes | 0.181 |
| `run_20260307_231053.json` | No | 66.7% | 0.0% | 1 | 26 | 6 | 4 | 10.323 | 0.0 | Yes | Yes | 0.509 |
| `run_20260307_231555.json` | No | 66.7% | 0.0% | 1 | 26 | 6 | 4 | 10.334 | 0.0 | Yes | Yes | 0.509 |
| `run_20260307_232516.json` | No | 66.7% | 0.0% | 1 | 18 | 0 | 5 | 5.803 | 0.0 | Yes | Yes | 0.117 |
| `run_20260307_232623.json` | No | 66.7% | 0.0% | 1 | 18 | 0 | 5 | 5.816 | 0.0 | Yes | Yes | 0.117 |
| `run_20260307_232810.json` | No | 33.3% | 0.0% | 0 | 6 | 0 | 3 | 3.443 | 0.0 | Yes | Yes | 0.115 |
| `run_20260307_233627.json` | No | 66.7% | 0.0% | 1 | 37 | 1 | 5 | 7.963 | 0.0 | Yes | Yes | 0.161 |
| `run_20260307_233756.json` | No | 66.7% | 0.0% | 1 | 26 | 6 | 4 | 10.307 | 0.0 | Yes | Yes | 0.509 |
| `run_20260307_234205.json` | No | 33.3% | 0.0% | 0 | 6 | 1 | 3 | 4.94 | 0.0 | Yes | Yes | 0.699 |
| `run_20260307_234422.json` | No | 33.3% | 0.0% | 0 | 6 | 1 | 3 | 4.891 | 0.0 | Yes | Yes | 0.699 |
