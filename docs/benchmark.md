# Benchmark Runtime

The benchmark runtime is the primary RoboAI implementation.

Main entrypoints:

- `python -m roboai.app.run_demo`
- `python -m roboai.app.run_batch`
- `python -m roboai.app.run_ablations`

Core benchmark capabilities:

- occupancy-grid mapping
- frontier extraction and ranking
- A*, RRT, and RRT*
- disturbance and noise experiments
- policy and planner ablations
- replayable media and report generation

This is the most complete environment for evaluating RoboAI algorithms.
