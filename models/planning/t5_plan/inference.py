import os, uuid
from pathlib import Path
from typing import Dict, Tuple
import torch

from transformers import T5ForConditionalGeneration, T5TokenizerFast
from .validate_and_decode import decode_and_validate

# Try to load fine-tuned model if present; otherwise fall back to base and few-shot examples
_THIS_DIR = Path(__file__).resolve().parent
_FINETUNED_DIR = _THIS_DIR / "t5-plan"
_BASE_NAME = "t5-small"

_tok = None
_model = None
_loaded = False

FEW_SHOT_HEADER = (
    "You are a planner. Convert the instruction and context to a STRICT JSON plan.\n"
    "ONLY output valid JSON. No comments, no extra text.\n\n"
    "Example:\n"
    "INSTRUCTION:\nmove to station_a then stop\n\n"
    "ROBOT_STATE:\npose=(x=0.0,y=0.0,theta=0deg)\n\n"
    "GOAL_LIBRARY:\nstation_a=(0.8,0.9)\n\n"
    "CONSTRAINTS:\nspeed_limit=0.5, avoid=[]\n\n"
    "OUTPUT JSON:\n"
    "{\"plan_id\":\"000\",\"steps\":[{\"op\":\"goto\",\"x\":0.8,\"y\":0.9},{\"op\":\"stop\"}],"
    "\"constraints\":{\"avoid\":[],\"speed_limit\":0.5}}\n\n"
)

def _ensure_loaded():
    global _tok, _model, _loaded
    if _loaded: return
    if _FINETUNED_DIR.exists() and (_FINETUNED_DIR / "config.json").exists():
        _tok = T5TokenizerFast.from_pretrained(str(_FINETUNED_DIR))
        _model = T5ForConditionalGeneration.from_pretrained(str(_FINETUNED_DIR))
    else:
        _tok = T5TokenizerFast.from_pretrained(_BASE_NAME)
        _model = T5ForConditionalGeneration.from_pretrained(_BASE_NAME)
    _model.eval()
    _loaded = True

def _build_prompt(instr: str, pose: Tuple[float,float,float],
                  goal_library: Dict[str, Tuple[float,float]],
                  constraints: Dict):
    gl = "; ".join([f"{k}=({v[0]},{v[1]})" for k,v in goal_library.items()])
    avoid = constraints.get("avoid", [])
    speed = constraints.get("speed_limit", 0.5)
    prompt = (
        f"{FEW_SHOT_HEADER}"
        f"INSTRUCTION:\n{instr}\n\n"
        f"ROBOT_STATE:\npose=(x={pose[0]},y={pose[1]},theta={pose[2]}deg)\n\n"
        f"GOAL_LIBRARY:\n{gl}\n\n"
        f"CONSTRAINTS:\nspeed_limit={speed}, avoid={avoid}\n\n"
        f"OUTPUT JSON:\n"
    )
    return prompt

@torch.inference_mode()
def nl_to_plan(instr: str,
               pose: Tuple[float,float,float],
               goal_library: Dict[str, Tuple[float,float]],
               constraints: Dict) -> Dict:
    """
    Returns a dict matching PLAN_SCHEMA. On failure, returns a conservative fallback plan.
    """
    _ensure_loaded()
    prompt = _build_prompt(instr, pose, goal_library, constraints)
    ids = _tok(prompt, return_tensors="pt", truncation=True).input_ids
    out = _model.generate(
        ids,
        max_new_tokens=256,
        num_beams=4,
        early_stopping=True,
        no_repeat_ngram_size=3
    )
    text = _tok.decode(out[0], skip_special_tokens=True)

    # Try strict decode; if it fails, produce a safe fallback
    try:
        plan = decode_and_validate(text)
        return plan
    except Exception:
        try:
            gx, gy = next(iter(goal_library.values()))
        except StopIteration:
            gx, gy = 0.0, 0.0
        return {
            "plan_id": str(uuid.uuid4()),
            "steps": [{"op":"goto","x":float(gx),"y":float(gy)}, {"op":"stop"}],
            "constraints": {
                "avoid": constraints.get("avoid", []),
                "speed_limit": float(constraints.get("speed_limit", 0.5))
            }
        }
