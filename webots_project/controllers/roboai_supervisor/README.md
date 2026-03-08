# RoboAI Supervisor Evaluation

Controller:
- `roboai_supervisor.py`

Purpose:
- Reset robot starts across trials
- Randomize obstacle positions (for known DEF obstacle nodes)
- Run timed trials
- Export aggregate trial summary with navigation metrics from controller logs

Environment variables:
- `ROBOAI_EVAL_TRIALS` (default `3`)
- `ROBOAI_EVAL_RUN_SECONDS` (default `25`)
- `ROBOAI_EVAL_SEED` (default `42`)

Output:
- `reports/supervisor_eval.json`

Summary fields include:
- `success_rate`
- `avg_final_goal_error_m`
- `avg_replans`
- `avg_runtime_s`
- `avg_collision_count`
- `avg_path_efficiency`
