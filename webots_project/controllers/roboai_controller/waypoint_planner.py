from __future__ import annotations

from typing import Dict, Tuple, Any, Optional
import argparse
import json
import os
import re
import uuid
import sys

from config import REPO_ROOT, GOALS_DIR, DEFAULT_WORLD_NAME, DEFAULT_WORLD_FILE


DEFAULT_GOAL_LIBRARY: Dict[str, Tuple[float, float]] = {
    "door": (0.42, -0.35),
    "charging_dock": (-0.55, 0.55),
    "conveyor_belt": (0.48, 0.02),
    "station_a": (0.35, 0.45),
    "station_b": (-0.42, 0.28),
}

WORLD_GOALS_MAP = {
    "world_office": "goals_world_office.json",
    "world_obstacles": "goals_world_obstacles.json",
    "world_empty": "goals_world_empty.json",
}

_CURRENT_GOAL_LIBRARY: Dict[str, Tuple[float, float]] = dict(DEFAULT_GOAL_LIBRARY)
_CURRENT_GOALS_SOURCE: str = "default_fallback"


def _read_mode_file(path: str) -> str:
    if path and os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            return f.read().strip()
    return ""


def resolve_world_name() -> str:
    """
    World precedence:
      1) --world-name
      2) ROBOAI_WORLD_NAME
      3) --world-file (or demo/mvp_world.txt)
      4) DEFAULT_WORLD_NAME
    """
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--world-name", type=str, default="")
    parser.add_argument("--world-file", type=str, default=DEFAULT_WORLD_FILE)
    args, _ = parser.parse_known_args(sys.argv[1:])

    cli = (args.world_name or "").strip()
    if cli:
        return cli

    env = os.getenv("ROBOAI_WORLD_NAME", "").strip()
    if env:
        return env

    file_val = _read_mode_file((args.world_file or "").strip())
    if file_val:
        return file_val

    return DEFAULT_WORLD_NAME


def _normalize_goal_library(raw: Dict[str, Any]) -> Dict[str, Tuple[float, float]]:
    out: Dict[str, Tuple[float, float]] = {}
    for k, v in raw.items():
        if not isinstance(k, str):
            continue
        if not isinstance(v, (list, tuple)) or len(v) != 2:
            continue
        try:
            out[k] = (float(v[0]), float(v[1]))
        except Exception:
            continue
    return out


def _load_goal_library_from_file(path: str) -> Optional[Dict[str, Tuple[float, float]]]:
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
        lib = _normalize_goal_library(raw if isinstance(raw, dict) else {})
        return lib if lib else None
    except Exception:
        return None


def load_goal_library() -> Tuple[Dict[str, Tuple[float, float]], str]:
    """
    Goals precedence:
      1) --goals-file
      2) ROBOAI_GOALS_FILE
      3) world-specific file from resolved world name
      4) DEFAULT_GOAL_LIBRARY
    """
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--goals-file", type=str, default="")
    args, _ = parser.parse_known_args(sys.argv[1:])

    cli_file = (args.goals_file or "").strip()
    if cli_file:
        lib = _load_goal_library_from_file(cli_file)
        if lib is not None:
            return lib, cli_file

    env_file = os.getenv("ROBOAI_GOALS_FILE", "").strip()
    if env_file:
        lib = _load_goal_library_from_file(env_file)
        if lib is not None:
            return lib, env_file

    world_name = resolve_world_name().strip().lower()
    mapped = WORLD_GOALS_MAP.get(world_name)
    if mapped:
        default_path = os.path.join(GOALS_DIR, mapped)
        lib = _load_goal_library_from_file(default_path)
        if lib is not None:
            return lib, default_path

    return dict(DEFAULT_GOAL_LIBRARY), "default_fallback"


def get_goal_library() -> Dict[str, Tuple[float, float]]:
    return dict(_CURRENT_GOAL_LIBRARY)


def get_goals_source() -> str:
    return _CURRENT_GOALS_SOURCE


def get_goal_xy(goal_name: Optional[str]) -> Optional[Tuple[float, float]]:
    if not goal_name:
        return None
    return _CURRENT_GOAL_LIBRARY.get(str(goal_name))


def _goal_aliases(goal_library: Dict[str, Tuple[float, float]]) -> Dict[str, str]:
    aliases: Dict[str, str] = {}
    for name in goal_library.keys():
        n = name.lower().strip()
        aliases[n] = name
        aliases[n.replace("_", " ")] = name
    # Common synonyms.
    if "charging_dock" in goal_library:
        aliases["dock"] = "charging_dock"
        aliases["charging dock"] = "charging_dock"
    return aliases


def _resolve_goal_name(text: str, goal_library: Dict[str, Tuple[float, float]]) -> str:
    alias_map = _goal_aliases(goal_library)
    t = text.lower().replace("-", " ").strip()
    for alias, canonical in alias_map.items():
        if alias in t:
            return canonical
    return ""


def _fallback_waypoint_plan(command: str, goal_library: Dict[str, Tuple[float, float]]) -> Dict[str, Any]:
    t = command.lower()
    steps = []

    if "explore" in t and "map" in t:
        m_secs = re.search(r"(\d+(?:\.\d+)?)\s*(?:sec|second|seconds|s)\b", t)
        secs = float(m_secs.group(1)) if m_secs else 20.0
        steps.append({"op": "explore", "seconds": secs})

    dst = _resolve_goal_name(t, goal_library=goal_library)
    if dst:
        x, y = goal_library[dst]
        accept_radius = 0.14 if dst == "door" else 0.10
        steps.append({"op": "goto", "x": x, "y": y, "goal": dst, "accept_radius": accept_radius})

    if "scan" in t:
        steps.append({"op": "scan", "sensor": "ir"})

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

    if "return to base" in t or "return base" in t or "return home" in t:
        steps.append({"op": "return_base"})

    if not steps:
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
    """
    global _CURRENT_GOAL_LIBRARY, _CURRENT_GOALS_SOURCE
    _CURRENT_GOAL_LIBRARY, _CURRENT_GOALS_SOURCE = load_goal_library()

    use_llm = os.getenv("ROBOAI_USE_WAYPOINT_LLM", "0").strip().lower() in {"1", "true", "yes", "on"}
    if not use_llm:
        return _fallback_waypoint_plan(command, goal_library=_CURRENT_GOAL_LIBRARY)

    try:
        if REPO_ROOT not in sys.path:
            sys.path.insert(0, REPO_ROOT)

        from models.planning.t5_plan.inference import nl_to_plan  # pylint: disable=import-error

        return nl_to_plan(
            instr=command,
            pose=(0.0, 0.0, 0.0),
            goal_library=_CURRENT_GOAL_LIBRARY,
            constraints={"avoid": [], "speed_limit": 0.35},
        )
    except Exception:
        return _fallback_waypoint_plan(command, goal_library=_CURRENT_GOAL_LIBRARY)
