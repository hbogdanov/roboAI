from __future__ import annotations

from typing import Dict, Tuple, Any
import os
import re
import uuid
import sys

from config import REPO_ROOT


GOAL_LIBRARY: Dict[str, Tuple[float, float]] = {
    # Coordinates tuned for ~1.5m x 1.5m RectangleArena scenarios.
    "charging_dock": (0.48, -0.48),
    "conveyor_belt": (0.48, 0.02),
    "station_a": (0.35, 0.45),
    "station_b": (-0.42, 0.28),
    # Door is wall-adjacent; executor uses a relaxed acceptance radius.
    "door": (0.42, -0.35),
}


def _goal_aliases() -> Dict[str, str]:
    return {
        "station a": "station_a",
        "station_a": "station_a",
        "station b": "station_b",
        "station_b": "station_b",
        "charging dock": "charging_dock",
        "charging_dock": "charging_dock",
        "dock": "charging_dock",
        "conveyor belt": "conveyor_belt",
        "conveyor_belt": "conveyor_belt",
        "door": "door",
    }


def _resolve_goal_name(text: str) -> str:
    alias_map = _goal_aliases()
    t = text.lower().replace("-", " ").strip()
    for alias, canonical in alias_map.items():
        if alias in t:
            return canonical
    return ""


def _fallback_waypoint_plan(command: str) -> Dict[str, Any]:
    t = command.lower()
    steps = []

    if "explore" in t and "map" in t:
        m_secs = re.search(r"(\d+(?:\.\d+)?)\s*(?:sec|second|seconds|s)\b", t)
        secs = float(m_secs.group(1)) if m_secs else 20.0
        steps.append({"op": "explore", "seconds": secs})

    dst = _resolve_goal_name(t)
    if dst:
        x, y = GOAL_LIBRARY[dst]
        accept_radius = 0.14 if dst == "door" else 0.10
        steps.append({"op": "goto", "x": x, "y": y, "goal": dst, "accept_radius": accept_radius})

    m_face = re.search(r"face(?:\s+to)?\s*(-?\d+(?:\.\d+)?)", t)
    if not m_face:
        m_face = re.search(r"face(?:\s+to)?\s*(-?\d+(?:\.\d+)?)\s*(?:deg|degree|degrees)?", t)
    if not m_face:
        m_face = re.search(r"(?:heading|theta)\s*(-?\d+(?:\.\d+)?)", t)
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
        "constraints": {"avoid": [], "speed_limit": 0.35},
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
            constraints={"avoid": [], "speed_limit": 0.35},
        )
    except Exception:
        return _fallback_waypoint_plan(command)
