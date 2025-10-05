from typing import List, Dict
import json, re

Plan = List[Dict]
USE_LLM = False

def _safe_json_array(text: str):
    text = re.sub(r"^```.*?\n|\n```$", "", text, flags=re.S)
    m = re.search(r"\[.*\]", text, flags=re.S)
    if not m: raise ValueError("No JSON array found")
    return json.loads(m.group(0))

def stub_plan(user_text: str) -> Plan:
    t = user_text.lower()
    plan: Plan = []
    if "left" in t:  plan.append({"op": "turn", "dir": "left",  "deg": 90})
    if "right" in t: plan.append({"op": "turn", "dir": "right", "deg": 90})
    if "forward" in t or "ahead" in t:
        # extract a seconds number if present
        m = re.search(r"(\d+(\.\d+)?)\s*(sec|second)", t)
        secs = float(m.group(1)) if m else 2.0
        plan.insert(0, {"op": "forward", "seconds": secs})
    if "scan" in t: plan.append({"op": "scan", "sensor": "ir"})
    if "return" in t: plan.append({"op": "return_base"})
    if not plan: plan = [{"op": "forward", "seconds": 2.0}]
    if plan[-1].get("op") != "stop": plan.append({"op": "stop"})
    return plan

def _llm_plan(user_text: str) -> Plan:
    from transformers import pipeline
    gen = pipeline("text2text-generation", model="google/flan-t5-small")
    prompt = (
        "Translate the robot instruction into a STRICT JSON array. "
        "Allowed ops: forward(seconds), turn(dir,deg), scan(sensor), return_base, stop. "
        "Example: [{\"op\":\"forward\",\"seconds\":2},{\"op\":\"turn\",\"dir\":\"left\",\"deg\":90},{\"op\":\"stop\"}] "
        f"Instruction: {user_text}\nJSON:"
    )
    out = gen(prompt, max_new_tokens=128)[0]["generated_text"]
    try:
        return _safe_json_array(out)
    except Exception:
        return stub_plan(user_text)

def get_plan(user_text: str) -> Plan:
    if USE_LLM:
        try: return _llm_plan(user_text)
        except Exception: pass
    return stub_plan(user_text)
