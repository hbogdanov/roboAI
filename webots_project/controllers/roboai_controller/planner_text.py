from __future__ import annotations
from typing import List, Dict, Any
import json, re

# A Plan is a JSON array of step dicts with specific ops.
Plan = List[Dict[str, Any]]

# Flip this to True to use the LLM path.
USE_LLM = True

# Allowed ops and field normalization

_ALLOWED_OPS = {"forward", "turn", "scan", "return_base", "wait", "stop"}

def _normalize_step(step: Dict[str, Any]) -> Dict[str, Any]:
    """Coerce fields to the right types and prune extras."""
    op = str(step.get("op", "")).lower().strip()
    s: Dict[str, Any] = {"op": op}

    if op == "forward":
        secs = step.get("seconds", 2.0)
        try:
            secs = float(secs)
        except Exception:
            secs = 2.0
        s["seconds"] = max(0.1, float(secs))

    elif op == "turn":
        # "dir" in {"left","right"} and "deg" (positive number)
        direction = str(step.get("dir", "left")).lower()
        if direction not in {"left", "right"}:
            # try to infer sign from deg if provided
            try:
                d = float(step.get("deg", 90))
                direction = "left" if d >= 0 else "right"
                s["deg"] = abs(d)
            except Exception:
                direction = "left"
        s["dir"] = direction
        if "deg" not in s:
            try:
                deg = abs(float(step.get("deg", 90)))
            except Exception:
                deg = 90.0
            s["deg"] = max(1.0, float(deg))

    elif op == "scan":
        sensor = str(step.get("sensor", "ir")).lower()
        s["sensor"] = sensor

    elif op == "wait":
        secs = step.get("seconds", 1.0)
        try:
            secs = float(secs)
        except Exception:
            secs = 1.0
        s["seconds"] = max(0.05, float(secs))

    elif op in {"return_base", "stop"}:
        pass  # no extra fields

    else:
        # unknown op -> will be filtered by validator
        pass

    return s

def _validate_and_fix(plan: Plan) -> Plan:
    """Keep only allowed ops; ensure last step is stop; normalize fields."""
    cleaned: Plan = []
    for raw in plan:
        step = _normalize_step(raw)
        if step.get("op") in _ALLOWED_OPS:
            cleaned.append(step)
    if not cleaned:
        cleaned = [{"op": "forward", "seconds": 2.0}]
    if cleaned[-1]["op"] != "stop":
        cleaned.append({"op": "stop"})
    return cleaned

# Robust JSON extraction from LLM text

_JSON_ARRAY_RE = re.compile(r"\[\s*\{.*?\}\s*\]", re.S)

def _safe_json_array(text: str) -> Plan:
    """
    Extract the FIRST JSON array from text. Accepts fenced code blocks.
    Raises ValueError if nothing parseable is found.
    """
    # Strip code fences like ```json ... ```
    t = text.strip()
    t = re.sub(r"^```(?:json)?\s*|\s*```$", "", t, flags=re.I | re.M)
    m = _JSON_ARRAY_RE.search(t)
    if not m:
        raise ValueError("No JSON array found in model output.")
    return json.loads(m.group(0))

# Simple rule-based fallback

def stub_plan(user_text: str) -> Plan:
    t = user_text.lower()
    plan: Plan = []
    # extract an inline seconds number for forward
    m = re.search(r"(\d+(?:\.\d+)?)\s*(sec|second|seconds|s)\b", t)
    secs = float(m.group(1)) if m else 2.0

    if "forward" in t or "ahead" in t or "go" in t or "move" in t:
        plan.append({"op": "forward", "seconds": secs})
    if "left" in t:
        plan.append({"op": "turn", "dir": "left", "deg": 90})
    if "right" in t:
        plan.append({"op": "turn", "dir": "right", "deg": 90})
    if "scan" in t:
        plan.append({"op": "scan", "sensor": "ir"})
    if "return" in t or "base" in t or "home" in t:
        plan.append({"op": "return_base"})
    if not plan:
        plan = [{"op": "forward", "seconds": 2.0}]
    if plan[-1].get("op") != "stop":
        plan.append({"op": "stop"})
    return _validate_and_fix(plan)

# LLM path (still legacy ops to fit executor)

def _llm_plan(user_text: str) -> Plan:
    """
    Use an LLM (FLAN-T5 small) to map NL → legacy ops.
    We instruct the model to return ONLY a JSON array with allowed ops.
    """
    from transformers import pipeline
    gen = pipeline("text2text-generation", model="google/flan-t5-small")

    prompt = (
        "Translate the following ROBOT INSTRUCTION into a STRICT JSON array of steps.\n"
        "Allowed ops ONLY: \n"
        "  - forward(seconds: number)\n"
        "  - turn(dir: 'left'|'right', deg: number)\n"
        "  - scan(sensor: string)\n"
        "  - return_base\n"
        "  - wait(seconds: number)\n"
        "  - stop\n"
        "Return ONLY the JSON array (no comments, no extra text).\n\n"
        "Example:\n"
        "Instruction: go forward 2 seconds then turn left 90 and stop\n"
        "Output:\n"
        "[{\"op\":\"forward\",\"seconds\":2}, {\"op\":\"turn\",\"dir\":\"left\",\"deg\":90}, {\"op\":\"stop\"}]\n\n"
        f"Instruction: {user_text}\nOutput:\n"
    )

    out = gen(prompt, max_new_tokens=160)[0]["generated_text"]
    try:
        parsed = _safe_json_array(out)
        return _validate_and_fix(parsed)
    except Exception:
        # Fallback to rules if parsing fails
        return stub_plan(user_text)

# Public entrypoint

def get_plan(user_text: str) -> Plan:
    if USE_LLM:
        try:
            return _llm_plan(user_text)
        except Exception:
            # strongly prefer returning a valid plan instead of throwing
            return stub_plan(user_text)
    return stub_plan(user_text)
