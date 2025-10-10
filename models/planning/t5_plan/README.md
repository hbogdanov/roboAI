# T5-based NL → JSON Planner

## Files
- `synth_data.py`: generate synthetic (input, target) pairs to `data/train.jsonl`, `data/val.jsonl`
- `t5_model.py`: fine-tune `t5-small` and save to `t5-plan/`
- `inference.py`: `nl_to_plan(...)` entrypoint
- `validate_and_decode.py`, `schema.py`: strict JSON validation

## Quickstart
```bash
# (optional) create data
python models/planning/t5_plan/synth_data.py

# (optional) train
python models/planning/t5_plan/t5_model.py
