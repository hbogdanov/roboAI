# Experimental T5 Planner

This folder contains an experimental NL-to-JSON planning stack for model training and schema validation.

Schema here uses ops like:
- `goto`
- `face`
- `wait`
- `stop`

The current Webots runtime controller can optionally consume this schema in waypoint mode (`ROBOAI_PLAN_MODE=waypoint` + `ROBOAI_USE_WAYPOINT_LLM=1`). By default, waypoint mode uses deterministic fallback parsing.

## Files

- `synth_data.py`: generate synthetic train/val JSONL data.
- `t5_model.py`: fine-tune `t5-small`.
- `inference.py`: `nl_to_plan(...)` entrypoint.
- `validate_and_decode.py`, `schema.py`: strict JSON schema validation.

## Quickstart

```bash
python models/planning/t5_plan/synth_data.py
python models/planning/t5_plan/t5_model.py
```
