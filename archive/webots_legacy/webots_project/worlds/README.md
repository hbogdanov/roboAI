# Webots Worlds

Scenario worlds:
- `world_empty.wbt`: baseline environment with Supervisor enabled.
- `world_obstacles.wbt`: fixed obstacles + red marker + Supervisor.
- `world_office.wbt`: office-like layout + blue marker + Supervisor.

Randomized variants can be generated from `world_obstacles.wbt`:
```bash
python scripts/generate_random_worlds.py --count 10 --seed 123
```

Generated files are written to:
- `webots_project/worlds/generated/`
