from __future__ import annotations

from typing import Dict, Tuple, Any
import os
import re
import uuid
import sys

from config import REPO_ROOT


GOAL_LIBRARY: Dict[str, Tuple[float, float]] = {
    "charging_dock": (1.5, -0.7),
    "conveyor_belt": (1.3, -0.4),
    "station_a": (0.8, 0.9),
    "station_b": (-0.6, 0.4),
    "door": (0.0, -1.2),
}


def _fallback_waypoint_plan(command: str) -> Dict[str, Any]:
    t = command.lower()
    steps = []

    dst = next((k for k in GOAL_LIBRARY.keys() if k in t), None)
    if dst is not None:
        x, y = GOAL_LIBRARY[dst]
        steps.append({"op": "goto", "x": x, "y": y})

    m_face = re.search(r"face\s*(-?\d+(?:\.\d+)?)", t)
    if m_face:
        steps.append({"op": "face", "theta_deg": float(m_face.group(1))})

    if "wait" in t:
        m_wait = re.search(r"(\d+(?:\.\d+)?)\s*(?:sec|second|seconds|s)\b", t)
        secs = float(m_wait.group(1)) if m_wait else 1.0
        steps.append({"op": "wait", "seconds": secs})

    if not steps:
        # conservative default
        steps = [{"op": "wait", "seconds": 1.0}]

    if steps[-1]["op"] != "stop":
        steps.append({"op": "stop"})

    return {
        "plan_id": str(uuid.uuid4()),
        "steps": steps,
        "constraints": {"avoid": [], "speed_limit": 0.5},
    }


def get_waypoint_plan(command: str) -> Dict[str, Any]:
    """
    Return a schema-like waypoint plan dict with keys:
      - plan_id
      - steps: [{op: goto|face|wait|stop, ...}]
      - constraints

    Uses the experimental T5 module when enabled, with safe fallback.
    """
    use_llm = os.getenv("ROBOAI_USE_WAYPOINT_LLM", "0").strip().lower() in {"1", "true", "yes", "on"}
    if not use_llm:
        return _fallback_waypoint_plan(command)

    try:
        if REPO_ROOT not in sys.path:
            sys.path.insert(0, REPO_ROOT)

        from models.planning.t5_plan.inference import nl_to_plan  # pylint: disable=import-error

        return nl_to_plan(
            instr=command,
            pose=(0.0, 0.0, 0.0),
            goal_library=GOAL_LIBRARY,
            constraints={"avoid": [], "speed_limit": 0.5},
        )
    except Exception:
        return _fallback_waypoint_plan(command)
