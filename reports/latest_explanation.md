# RoboAI Run Explanation

## Intent
- Received command: `go to the door and face 90 degrees`
- Planning mode: `waypoint`

## Interpreted Plan
```json
[
  {
    "op": "goto",
    "x": 0.42,
    "y": -0.35,
    "goal": "door",
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
- `state_transition`: `23`
- `spa_tick`: `625`
- `turn_done`: `0`
- `scan`: `0`
- `goto_done`: `0`
- `goto_start`: `22`
- `goto_progress`: `518`
- `goto_stuck`: `11`
- `goto_recovery_tick`: `107`
- `path_planned`: `17`
- `path_plan_failed`: `0`
- `frontier_detected`: `0`
- `frontier_selected`: `0`
- `frontier_reached`: `0`
- `explore_done`: `0`
- `face_done`: `0`
- `pose_correction`: `0`
- `camera_marker`: `63`
- `collision_warning`: `0`
- `collision_burst_escape`: `0`
- `stop`: `1`

## Summary
The system explains behavior by exposing the resolved plan and concrete execution events from runtime logs (rather than free-form text generation).
