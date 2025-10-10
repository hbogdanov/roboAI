import json, re
from jsonschema import Draft202012Validator
from .schema import PLAN_SCHEMA

def _strip_code_fences(text: str) -> str:
    t = text.strip()
    t = re.sub(r"^```json\s*|\s*```$", "", t, flags=re.MULTILINE)
    t = re.sub(r"^```\s*|\s*```$", "", t, flags=re.MULTILINE)
    return t.strip()

def _extract_first_json_obj(text: str) -> str:
    # keep first {...} block
    m = re.search(r"\{.*\}", text, flags=re.DOTALL)
    return m.group(0) if m else text

def clean_to_json_str(text: str) -> str:
    return _extract_first_json_obj(_strip_code_fences(text))

def decode_and_validate(text: str) -> dict:
    raw = clean_to_json_str(text)
    obj = json.loads(raw)
    Draft202012Validator(PLAN_SCHEMA).validate(obj)
    return obj
