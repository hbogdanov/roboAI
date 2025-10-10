PLAN_SCHEMA = {
    "type": "object",
    "required": ["plan_id", "steps", "constraints"],
    "properties": {
        "plan_id": {"type": "string"},
        "steps": {
            "type": "array",
            "items": {
                "type": "object",
                "required": ["op"],
                "properties": {
                    "op": {"type": "string", "enum": ["goto", "face", "wait", "stop"]},
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "theta_deg": {"type": "number"},
                    "seconds": {"type": "number"},
                },
                "additionalProperties": False
            }
        },
        "constraints": {
            "type": "object",
            "properties": {
                "avoid": {"type": "array", "items": {"type": "string"}},
                "speed_limit": {"type": "number"}
            },
            "additionalProperties": True
        }
    },
    "additionalProperties": False
}
