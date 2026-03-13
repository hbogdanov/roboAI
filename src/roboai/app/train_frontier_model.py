from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from roboai.core.frontier import frontier_regions, _frontier_features  # noqa: PLC2701
from roboai.core.frontier_model import fit_linear_frontier_model, save_frontier_model
from roboai.core.occupancy_grid import FREE, OCCUPIED, OccupancyGrid
from roboai.sim.grid2d.maps import built_in_map, built_in_semantic_grid


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--maps", nargs="+", default=["empty", "office", "cluttered", "narrow", "maze"])
    parser.add_argument("--samples-per-map", type=int, default=40)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--out", default="reports/frontier_model_weights.json")
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    samples: list[dict[str, float]] = []
    targets: list[float] = []
    for map_name in args.maps:
        obstacle_grid = built_in_map(map_name)
        semantic_grid = built_in_semantic_grid(map_name)
        for _ in range(args.samples_per_map):
            occ = _sample_partial_observation(obstacle_grid, rng)
            regions = frontier_regions(occ)
            if not regions:
                continue
            robot_xy = (
                float(rng.uniform(1.0, max(1.2, occ.width * occ.resolution - 1.0))),
                float(rng.uniform(1.0, max(1.2, occ.height * occ.resolution - 1.0))),
            )
            robot_theta = float(rng.uniform(-np.pi, np.pi))
            for region in regions[:8]:
                world_points = np.asarray([occ.grid_to_world(gx, gy) for gx, gy in region], dtype=float)
                centroid = world_points.mean(axis=0)
                representative = min(
                    world_points,
                    key=lambda point: float(np.hypot(point[0] - centroid[0], point[1] - centroid[1])),
                )
                distance = float(np.hypot(representative[0] - robot_xy[0], representative[1] - robot_xy[1]))
                heading = float(np.arctan2(representative[1] - robot_xy[1], representative[0] - robot_xy[0]))
                heading_penalty = abs(_wrap_angle(heading - robot_theta)) * 0.75
                features = _frontier_features(
                    grid=occ,
                    representative=(float(representative[0]), float(representative[1])),
                    distance=distance,
                    heading_penalty=heading_penalty,
                    region_size=float(len(region)),
                    blocked_penalty=0.0,
                    revisit_penalty=0.0,
                    semantic_grid=semantic_grid,
                    localization_uncertainty=float(rng.uniform(0.0, 0.6)),
                )
                utility = (
                    0.55 * features["info_gain"]
                    + 0.35 * features["semantic_value"]
                    + 0.2 * features["beacon_bonus"]
                    + 0.12 * features["region_size"]
                    - 1.0 * features["distance"]
                    - 0.5 * features["heading_penalty"]
                    - 0.8 * features["uncertainty_penalty"]
                )
                samples.append(features)
                targets.append(float(utility))

    weights = fit_linear_frontier_model(samples, targets)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    save_frontier_model(args.out, weights)
    print(json.dumps(weights, indent=2))


def _sample_partial_observation(obstacle_grid: np.ndarray, rng: np.random.Generator) -> OccupancyGrid:
    height, width = obstacle_grid.shape
    occ = OccupancyGrid(width=width, height=height, resolution=0.2)
    center_x = int(rng.integers(8, max(9, width - 8)))
    center_y = int(rng.integers(8, max(9, height - 8)))
    radius_x = int(rng.integers(6, 14))
    radius_y = int(rng.integers(6, 14))
    x0 = max(1, center_x - radius_x)
    x1 = min(width - 1, center_x + radius_x)
    y0 = max(1, center_y - radius_y)
    y1 = min(height - 1, center_y + radius_y)
    for gy in range(y0, y1):
        for gx in range(x0, x1):
            occ.set_cell(gx, gy, OCCUPIED if obstacle_grid[gy, gx] else FREE)
    return occ


def _wrap_angle(angle: float) -> float:
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return float(angle)


if __name__ == "__main__":
    main()
