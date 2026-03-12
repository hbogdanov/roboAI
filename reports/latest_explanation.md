# RoboAI Run Explanation

## Intent
- Received command: `explore the room and build a map for 60 seconds`
- Planning mode: `waypoint`

## Interpreted Plan
```json
[
  {
    "op": "explore",
    "seconds": 60.0
  },
  {
    "op": "stop"
  }
]
```

## Execution Evidence
- `plan_built`: `1`
- `plan_loaded`: `1`
- `state_transition`: `127`
- `spa_tick`: `939`
- `turn_done`: `0`
- `scan`: `0`
- `goto_done`: `0`
- `goto_failed`: `0`
- `goto_abort`: `0`
- `goto_start`: `29`
- `goto_progress`: `889`
- `goto_stuck`: `2`
- `goto_recovery_tick`: `46`
- `path_planned`: `29`
- `path_plan_failed`: `0`
- `goal_snapped`: `9`
- `goal_clearance_checked`: `29`
- `lidar_avoid`: `16`
- `frontier_detected`: `40`
- `frontier_selected`: `40`
- `frontier_reached`: `0`
- `frontier_failed`: `0`
- `explore_done`: `1`
- `face_done`: `0`
- `pose_correction`: `0`
- `camera_marker`: `94`
- `collision_warning`: `0`
- `collision_burst_escape`: `2`
- `stop`: `2`

## Summary
The system explains behavior by exposing the resolved plan and concrete execution events from runtime logs (rather than free-form text generation).
