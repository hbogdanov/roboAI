from __future__ import annotations

import json
from pathlib import Path

import numpy as np


FEATURE_KEYS = [
    "bias",
    "distance",
    "heading_penalty",
    "region_size",
    "info_gain",
    "semantic_value",
    "blocked_penalty",
    "revisit_penalty",
    "uncertainty_penalty",
    "beacon_bonus",
]

DEFAULT_WEIGHTS = {
    "bias": 0.0,
    "distance": -1.0,
    "heading_penalty": -0.55,
    "region_size": 0.18,
    "info_gain": 0.42,
    "semantic_value": 0.85,
    "blocked_penalty": -0.9,
    "revisit_penalty": -0.75,
    "uncertainty_penalty": -0.85,
    "beacon_bonus": 0.65,
}


def feature_vector(features: dict[str, float]) -> np.ndarray:
    return np.asarray([float(features.get(key, 0.0)) for key in FEATURE_KEYS], dtype=float)


def score_frontier(features: dict[str, float], weights: dict[str, float] | None = None) -> float:
    model = weights or DEFAULT_WEIGHTS
    return float(feature_vector(features) @ feature_vector(model))


def fit_linear_frontier_model(samples: list[dict[str, float]], targets: list[float], ridge: float = 1e-3) -> dict[str, float]:
    if not samples or not targets:
        return dict(DEFAULT_WEIGHTS)
    x = np.vstack([feature_vector(sample) for sample in samples])
    y = np.asarray(targets, dtype=float)
    eye = np.eye(x.shape[1], dtype=float)
    coeffs = np.linalg.solve(x.T @ x + ridge * eye, x.T @ y)
    return {key: float(value) for key, value in zip(FEATURE_KEYS, coeffs)}


def save_frontier_model(path: str | Path, weights: dict[str, float]) -> None:
    Path(path).write_text(json.dumps(weights, indent=2), encoding="utf-8")


def load_frontier_model(path: str | Path) -> dict[str, float]:
    return json.loads(Path(path).read_text(encoding="utf-8"))
