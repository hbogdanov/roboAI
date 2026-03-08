# RoboAI Run Explanation

## Intent
- Received command: `go to station A and face 90 degrees`
- Planning mode: `waypoint`

## Interpreted Plan
```json
[
  {
    "op": "goto",
    "x": 0.3,
    "y": 0.52,
    "goal": "station_a",
    "accept_radius": 0.14
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
- `state_transition`: `12`
- `spa_tick`: `1407`
- `turn_done`: `0`
- `scan`: `0`
- `goto_done`: `0`
- `goto_failed`: `0`
- `goto_abort`: `0`
- `goto_start`: `5`
- `goto_progress`: `1355`
- `goto_stuck`: `1`
- `goto_recovery_tick`: `49`
- `path_planned`: `4`
- `path_plan_failed`: `0`
- `goal_snapped`: `1`
- `goal_clearance_checked`: `5`
- `lidar_avoid`: `17`
- `frontier_detected`: `0`
- `frontier_selected`: `0`
- `frontier_reached`: `0`
- `frontier_failed`: `0`
- `explore_done`: `0`
- `face_done`: `0`
- `pose_correction`: `0`
- `camera_marker`: `141`
- `collision_warning`: `6`
- `collision_burst_escape`: `3`
- `stop`: `1`

## Summary
The system explains behavior by exposing the resolved plan and concrete execution events from runtime logs (rather than free-form text generation).
