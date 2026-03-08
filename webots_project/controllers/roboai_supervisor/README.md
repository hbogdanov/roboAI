# RoboAI Supervisor Evaluation

Controller:
- `roboai_supervisor.py`

Purpose:
- Reset robot starts across trials
- Randomize obstacle positions (for known DEF obstacle nodes)
- Run timed trials
- Export aggregate trial summary

Environment variables:
- `ROBOAI_EVAL_TRIALS` (default `3`)
- `ROBOAI_EVAL_RUN_SECONDS` (default `25`)
- `ROBOAI_EVAL_SEED` (default `42`)

Output:
- `reports/supervisor_eval.json`
