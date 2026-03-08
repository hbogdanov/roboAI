# RoboAI Run Explanation

## Intent
- Received command: `go to station A and face 90 degrees`
- Planning mode: `waypoint`

## Interpreted Plan
```json
[
  {
    "op": "goto",
    "x": 0.35,
    "y": 0.45,
    "goal": "station_a",
    "accept_radius": 0.1
  },
  {
    "op": "face",
    "theta_deg": 90.0
  },
  {
    "op": "stop"
  }
]
```

## Execution Evidence
- `plan_built`: `1`
- `plan_loaded`: `1`
- `state_transition`: `3`
- `spa_tick`: `38`
- `turn_done`: `0`
- `scan`: `0`
- `goto_done`: `0`
- `goto_failed`: `1`
- `goto_abort`: `1`
- `goto_start`: `0`
- `goto_progress`: `0`
- `goto_stuck`: `0`
- `goto_recovery_tick`: `0`
- `path_planned`: `0`
- `path_plan_failed`: `0`
- `goal_snapped`: `0`
- `goal_clearance_checked`: `1`
- `lidar_avoid`: `0`
- `frontier_detected`: `0`
- `frontier_selected`: `0`
- `frontier_reached`: `0`
- `frontier_failed`: `0`
- `explore_done`: `0`
- `face_done`: `1`
- `pose_correction`: `0`
- `camera_marker`: `4`
- `collision_warning`: `0`
- `collision_burst_escape`: `0`
- `stop`: `2`

## Summary
The system explains behavior by exposing the resolved plan and concrete execution events from runtime logs (rather than free-form text generation).
