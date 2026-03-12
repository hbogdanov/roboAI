# RoboAI

RoboAI - Autonomous Exploration and Path Planning Benchmark Platform

Built a deterministic 2D robotics benchmark for occupancy-grid mapping, frontier-based exploration, and collision-aware path planning. The platform simulates lidar sensing, updates an exploration map online, plans routes to frontiers using A*, RRT, and RRT*, and exports replayable demos plus benchmarking metrics across indoor maps.

## Features

- deterministic 2D simulation
- simulated lidar
- occupancy grid mapping
- frontier exploration
- A*, RRT, and RRT* planning
- waypoint following
- coverage and efficiency metrics
- GIF / MP4 demo export

## Structure

```text
src/roboai/
  core/
  sim/
  app/
tests/
demo/
reports/
```

## Quick Start

```bash
pip install -e .
python -m roboai.app.run_demo --map office --planner astar --seed 7
```

Outputs are written under `demo/` and `reports/`.

`run_demo` uses map-specific default coverage goals and step budgets so each built-in map produces a stable, presentable exploration run without extra tuning.

Built-in maps:

- `empty`
- `office`
- `cluttered`
- `narrow`
- `maze`

## Results

Typical outputs after running a demo:

- final map image: `reports/final_map_<map>_<planner>.png`
- trajectory and exploration replay: `demo/demo_<map>_<planner>.gif` and `demo/demo_<map>_<planner>.mp4`
- metrics: `reports/metrics_<map>_<planner>_seed<seed>.json`

Example artifacts already produced in this repo:

- [final_map_office_astar.png](/abs/path/c:/Users/Ivan/roboAI/reports/final_map_office_astar.png)
- [demo_office_astar.gif](/abs/path/c:/Users/Ivan/roboAI/demo/demo_office_astar.gif)
- [metrics_office_astar_seed7.json](/abs/path/c:/Users/Ivan/roboAI/reports/metrics_office_astar_seed7.json)

Example metrics:

| map | planner | success | coverage | path length | collisions | replans |
| --- | --- | --- | --- | --- | --- | --- |
| empty | astar | true | 0.802 | 5.76 | 0 | 2 |
| office | astar | true | 0.654 | 11.93 | 0 | 3 |
| cluttered | astar | true | 0.754 | 14.67 | 2 | 5 |
| narrow | astar | true | 0.601 | 14.29 | 1 | 4 |
| maze | astar | true | 0.451 | 22.17 | 0 | 7 |
