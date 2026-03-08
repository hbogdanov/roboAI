# MVP Demo Scenario

## Scenario

Command source file:
- `demo/mvp_command.txt`

Default command:
```text
Go forward for 3 seconds, turn left 90, scan, then stop.
```

## Expected artifacts after one run

- `data/logs/run_YYYYMMDD_HHMMSS.json`
- `reports/latest_demo_artifact.md`

## Official Demo Presets

- `demo1` goal navigation:
  - command file: `demo/demo1_goal_navigation.txt`
  - command: `go to station A and face 90 degrees`
- `demo2` explore + map:
  - command file: `demo/demo2_explore_map.txt`
  - command: `explore the room and build a map`
- `demo3` go, inspect, return:
  - command file: `demo/demo3_go_scan_return.txt`
  - command: `go to the door, scan, and return to base`

Select preset:
```bash
python scripts/select_demo.py --demo demo1
python scripts/select_demo.py --demo demo2
python scripts/select_demo.py --demo demo3
```
