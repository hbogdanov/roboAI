from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path
from time import perf_counter

import numpy as np

from roboai.app.run_demo import BASE_PLANNERS, _estimated_pose, _planner_sequence, _trajectory_length
from roboai.core.control.local_avoidance import emergency_stop
from roboai.core.control.waypoint_follower import WaypointFollower
from roboai.core.frontier import frontier_cells, rank_frontier_targets
from roboai.core.metrics import write_metrics_json
from roboai.core.occupancy_grid import OccupancyGrid, UNKNOWN
from roboai.core.planners.smoothing import path_quality, shortcut_smooth_path
from roboai.core.types import Pose2D, RenderFrame, RunMetrics
from roboai.sim.grid2d.env import Grid2DEnv
from roboai.sim.grid2d.lidar import SimulatedLidar
from roboai.sim.grid2d.maps import built_in_map, built_in_semantic_grid
from roboai.sim.grid2d.renderer import save_run_artifacts


START_PAIRS = {
    "empty": [Pose2D(0.8, 0.8, 0.0), Pose2D(10.0, 10.0, np.pi)],
    "office": [Pose2D(1.2, 1.2, 0.0), Pose2D(13.5, 9.2, np.pi)],
    "cluttered": [Pose2D(0.8, 0.8, 0.0), Pose2D(11.0, 11.0, np.pi)],
    "narrow": [Pose2D(1.0, 1.2, 0.0), Pose2D(13.0, 9.5, np.pi)],
    "maze": [Pose2D(1.0, 1.0, 0.0), Pose2D(14.0, 14.0, np.pi)],
}


def run_multi_demo(
    map_name: str,
    planner_name: str,
    seed: int,
    max_steps: int = 260,
    coverage_goal: float = 0.75,
    write_artifacts: bool = True,
) -> RunMetrics:
    np.random.seed(seed)
    obstacle_grid = built_in_map(map_name)
    semantic_grid = built_in_semantic_grid(map_name)
    starts = START_PAIRS[map_name]
    envs = [
        Grid2DEnv(obstacle_grid=obstacle_grid, resolution=0.2, robot_radius=0.16),
        Grid2DEnv(obstacle_grid=obstacle_grid, resolution=0.2, robot_radius=0.16),
    ]
    for env, pose in zip(envs, starts):
        env.reset(pose)

    occ = OccupancyGrid(width=envs[0].width, height=envs[0].height, resolution=envs[0].resolution)
    lidars = [SimulatedLidar(num_beams=181, max_range=6.5), SimulatedLidar(num_beams=181, max_range=6.5)]
    followers = [WaypointFollower(), WaypointFollower()]
    blocked_targets = [[], []]
    revisit_counts: dict[tuple[int, int], int] = {}
    pose_drifts = [np.zeros(3, dtype=float), np.zeros(3, dtype=float)]
    frames: list[RenderFrame] = []
    coverage_history: list[float] = []
    replans = 0
    replan_triggers = 0
    recovery_events = 0
    collisions = 0
    near_conflicts = 0
    duplicate_frontier_assignments = 0
    total_raw_path_length = 0.0
    total_smoothed_path_length = 0.0
    total_turn_count = 0
    last_plan: list[tuple[float, float]] = []
    localization_uncertainty = 0.05
    stop_reason = "max_steps"
    started_at = perf_counter()
    robot_explored_masks = [np.zeros_like(occ.grid, dtype=bool), np.zeros_like(occ.grid, dtype=bool)]

    for step_idx in range(max_steps):
        estimated_poses = []
        scans = []
        for idx, env in enumerate(envs):
            estimated_pose = _estimated_pose(env.pose, 0.01, pose_drifts[idx])
            scan = lidars[idx].scan(env, env.pose)
            local_occ = OccupancyGrid(width=occ.width, height=occ.height, resolution=occ.resolution)
            local_occ.grid[:] = occ.grid
            local_occ.update_from_scan(estimated_pose, scan)
            robot_explored_masks[idx] |= local_occ.grid != UNKNOWN
            occ.update_from_scan(estimated_pose, scan)
            estimated_poses.append(estimated_pose)
            scans.append(scan)
        localization_uncertainty = min(1.0, localization_uncertainty + 0.01)
        coverage = occ.known_ratio()
        coverage_history.append(coverage)
        if step_idx % 2 == 0:
            frames.append(
                RenderFrame(
                    occupancy=occ.grid.copy(),
                    trajectory=list(envs[0].trajectory),
                    planner_path=list(last_plan),
                    frontier_points=[occ.grid_to_world(gx, gy) for gx, gy in frontier_cells(occ)],
                    robot_pose=envs[0].pose,
                    coverage=coverage,
                    semantic_overlay=semantic_grid,
                    robot_poses=[env.pose for env in envs],
                    localization_uncertainty=localization_uncertainty,
                    additional_trajectories=[list(envs[1].trajectory)],
                )
            )
        if coverage >= coverage_goal:
            stop_reason = "coverage_goal_reached"
            break

        assigned_targets: list[tuple[float, float] | None] = [None, None]
        for idx, env in enumerate(envs):
            if not followers[idx].is_done():
                continue
            replan_triggers += 1
            targets = rank_frontier_targets(
                occ,
                (estimated_poses[idx].x, estimated_poses[idx].y),
                robot_theta=estimated_poses[idx].theta,
                policy="semantic_information_gain",
                blocked_targets=blocked_targets[idx],
                revisit_counts=revisit_counts,
                semantic_grid=semantic_grid,
                localization_uncertainty=localization_uncertainty,
            )
            targets = [target for target in targets if all(other is None or np.hypot(target[0] - other[0], target[1] - other[1]) > 0.8 for other in assigned_targets)]
            for target in targets:
                result = None
                for _, planner in _planner_sequence(planner_name):
                    candidate = planner(
                        grid=occ,
                        start_xy=(estimated_poses[idx].x, estimated_poses[idx].y),
                        goal_xy=target,
                        robot_radius=env.robot_radius,
                        allow_unknown=False,
                    )
                    if candidate.success:
                        result = candidate
                        break
                if result is None:
                    continue
                smoothed = shortcut_smooth_path(occ, result.path, env.robot_radius, allow_unknown=False)
                raw_length, _ = path_quality(result.path)
                smooth_length, turn_count = path_quality(smoothed)
                total_raw_path_length += raw_length
                total_smoothed_path_length += smooth_length
                total_turn_count += turn_count
                followers[idx].set_path(smoothed[1:])
                assigned_targets[idx] = target
                if any(other is not None and np.hypot(target[0] - other[0], target[1] - other[1]) <= 1.0 for other in assigned_targets[:idx]):
                    duplicate_frontier_assignments += 1
                blocked_targets[idx].append(target)
                revisit_counts[occ.world_to_grid(*target)] = revisit_counts.get(occ.world_to_grid(*target), 0) + 1
                last_plan = smoothed
                replans += 1
                break

        for idx, env in enumerate(envs):
            linear, angular = followers[idx].command(estimated_poses[idx])
            if emergency_stop(scans[idx].ranges, stop_distance=0.16, scan_angles=scans[idx].angles, front_angle=0.45):
                linear = 0.0
                angular = 1.0
            other = envs[1 - idx].pose
            if np.hypot(env.pose.x - other.x, env.pose.y - other.y) <= env.robot_radius * 2.5:
                linear = 0.0
                angular = 0.8 if idx == 0 else -0.8
                recovery_events += 1
            if np.hypot(env.pose.x - other.x, env.pose.y - other.y) <= env.robot_radius * 4.0:
                near_conflicts += 1
            _, collision = env.step(linear=linear, angular=angular, dt=0.2)
            if collision:
                collisions += 1
                replan_triggers += 1
                followers[idx].set_path([])

    runtime_seconds = perf_counter() - started_at
    known_cells = int(np.count_nonzero(occ.grid != UNKNOWN))
    overlap_cells = int(np.count_nonzero(robot_explored_masks[0] & robot_explored_masks[1]))
    union_cells = int(np.count_nonzero(robot_explored_masks[0] | robot_explored_masks[1]))
    metrics = RunMetrics(
        map_name=map_name,
        planner_name=planner_name,
        planner_policy="cooperative",
        frontier_policy="semantic_information_gain",
        seed=seed,
        success=bool(coverage_history and coverage_history[-1] >= coverage_goal),
        stop_reason=stop_reason,
        steps=len(coverage_history),
        runtime_seconds=runtime_seconds,
        coverage=coverage_history[-1] if coverage_history else 0.0,
        path_length=sum(_trajectory_length(env.trajectory) for env in envs),
        raw_path_length=total_raw_path_length,
        smoothed_path_length=total_smoothed_path_length,
        path_turn_count=total_turn_count,
        collisions=collisions,
        replans=replans,
        replan_triggers=replan_triggers,
        recovery_events=recovery_events,
        disturbance_name="none",
        disturbance_events=0,
        range_noise_std=0.0,
        dropout_prob=0.0,
        pose_noise_std=0.01,
        semantic_mode="enabled",
        final_localization_uncertainty=localization_uncertainty,
        robot_count=2,
        time_to_coverage_step=_time_to_coverage_step(coverage_history, coverage_goal),
        explored_overlap_ratio=float(overlap_cells / union_cells) if union_cells else 0.0,
        duplicate_frontier_assignments=duplicate_frontier_assignments,
        near_conflicts=near_conflicts,
        explored_cells=known_cells,
        known_cells=known_cells,
        coverage_history=coverage_history,
    )
    if write_artifacts and frames:
        Path("demo").mkdir(exist_ok=True)
        Path("reports").mkdir(exist_ok=True)
        save_run_artifacts(
            demo_path=Path("demo") / f"demo_{map_name}_{planner_name}_multi.gif",
            video_path=Path("demo") / f"demo_{map_name}_{planner_name}_multi.mp4",
            final_map_path=Path("reports") / f"final_map_{map_name}_{planner_name}_multi.png",
            coverage_path=Path("reports") / f"coverage_{map_name}_{planner_name}_multi_seed{seed}.png",
            obstacle_grid=obstacle_grid,
            resolution=envs[0].resolution,
            frames=frames,
        )
        write_metrics_json(Path("reports") / f"metrics_{map_name}_{planner_name}_multi_seed{seed}.json", metrics)
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, choices=sorted(START_PAIRS))
    parser.add_argument("--planner", default="hybrid", choices=sorted(list(BASE_PLANNERS) + ["hybrid"]))
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-steps", type=int, default=260)
    parser.add_argument("--coverage-goal", type=float, default=0.75)
    args = parser.parse_args()
    print(json.dumps(asdict(run_multi_demo(args.map, args.planner, args.seed, args.max_steps, args.coverage_goal)), indent=2))


def _time_to_coverage_step(coverage_history: list[float], coverage_goal: float) -> int:
    for idx, coverage in enumerate(coverage_history, start=1):
        if coverage >= coverage_goal:
            return idx
    return len(coverage_history)


if __name__ == "__main__":
    main()
