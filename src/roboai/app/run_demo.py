from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path
from time import perf_counter

import numpy as np

from roboai.core.control.local_avoidance import emergency_stop
from roboai.core.control.waypoint_follower import WaypointFollower
from roboai.core.frontier import frontier_cells, rank_frontier_targets
from roboai.core.metrics import write_metrics_json
from roboai.core.occupancy_grid import OccupancyGrid, UNKNOWN
from roboai.core.planners.astar import plan_astar
from roboai.core.planners.rrt import plan_rrt
from roboai.core.planners.rrt_star import plan_rrt_star
from roboai.core.planners.smoothing import path_quality, shortcut_smooth_path
from roboai.core.types import Pose2D, RenderFrame, RunMetrics
from roboai.sim.grid2d.env import Grid2DEnv
from roboai.sim.grid2d.lidar import SimulatedLidar
from roboai.sim.grid2d.maps import built_in_map
from roboai.sim.grid2d.renderer import save_run_artifacts


BASE_PLANNERS = {
    "astar": plan_astar,
    "rrt": plan_rrt,
    "rrt_star": plan_rrt_star,
}

START_POSES = {
    "empty": Pose2D(x=0.8, y=0.8, theta=0.0),
    "office": Pose2D(x=1.2, y=1.2, theta=0.0),
    "cluttered": Pose2D(x=0.8, y=0.8, theta=0.0),
    "narrow": Pose2D(x=1.0, y=1.2, theta=0.0),
    "maze": Pose2D(x=1.0, y=1.0, theta=0.0),
}

DEFAULT_COVERAGE_GOALS = {
    "empty": 0.80,
    "office": 0.65,
    "cluttered": 0.75,
    "narrow": 0.60,
    "maze": 0.45,
}

DEFAULT_MAX_STEPS = {
    "empty": 180,
    "office": 240,
    "cluttered": 260,
    "narrow": 320,
    "maze": 420,
}


def run_demo(
    map_name: str,
    planner_name: str,
    seed: int,
    max_steps: int | None = None,
    coverage_goal: float | None = None,
    frame_stride: int = 2,
    write_artifacts: bool = True,
    frontier_policy: str = "naive",
    disturbance_name: str = "none",
    range_noise_std: float = 0.0,
    dropout_prob: float = 0.0,
    pose_noise_std: float = 0.0,
) -> RunMetrics:
    np.random.seed(seed)
    requested_planner_name = planner_name
    if max_steps is None:
        max_steps = DEFAULT_MAX_STEPS.get(map_name, 300)
    if coverage_goal is None:
        coverage_goal = DEFAULT_COVERAGE_GOALS.get(map_name, 0.80)
    obstacle_grid = built_in_map(map_name)
    env = Grid2DEnv(
        obstacle_grid=obstacle_grid,
        resolution=0.2,
        robot_radius=0.16,
        disturbance_name=disturbance_name,
    )
    env.reset(START_POSES.get(map_name, Pose2D(x=0.8, y=0.8, theta=0.0)))

    occ = OccupancyGrid(width=env.width, height=env.height, resolution=env.resolution)
    lidar = SimulatedLidar(
        num_beams=181,
        max_range=6.5,
        range_noise_std=range_noise_std,
        dropout_prob=dropout_prob,
    )
    follower = WaypointFollower()

    coverage_history: list[float] = []
    replans = 0
    replan_triggers = 0
    collisions = 0
    last_plan: list[tuple[float, float]] = []
    total_raw_path_length = 0.0
    total_smoothed_path_length = 0.0
    total_turn_count = 0
    stop_reason = "max_steps"
    frames: list[RenderFrame] = []
    started_at = perf_counter()
    stagnant_steps = 0
    last_coverage = 0.0
    recovery_steps = 0
    recovery_events = 0
    blocked_targets: list[tuple[float, float]] = []
    active_target: tuple[float, float] | None = None
    active_target_start_coverage = 0.0
    active_target_steps = 0
    revisit_counts: dict[tuple[int, int], int] = {}
    pose_drift = np.zeros(3, dtype=float)

    for step_idx in range(max_steps):
        estimated_pose = _estimated_pose(env.pose, pose_noise_std, pose_drift)
        scan = lidar.scan(env, env.pose)
        occ.update_from_scan(estimated_pose, scan)
        coverage_history.append(occ.known_ratio())
        if coverage_history[-1] <= last_coverage + 1e-4:
            stagnant_steps += 1
        else:
            stagnant_steps = 0
        last_coverage = coverage_history[-1]
        current_frontiers = [occ.grid_to_world(gx, gy) for gx, gy in frontier_cells(occ)]

        if step_idx % max(1, frame_stride) == 0:
            frames.append(
                RenderFrame(
                    occupancy=occ.grid.copy(),
                    trajectory=list(env.trajectory),
                    planner_path=list(last_plan),
                    frontier_points=current_frontiers,
                    robot_pose=Pose2D(x=env.pose.x, y=env.pose.y, theta=env.pose.theta),
                    coverage=coverage_history[-1],
                )
            )

        if coverage_history[-1] >= coverage_goal:
            stop_reason = "coverage_goal_reached"
            break

        if active_target is not None:
            active_target_steps += 1
            target_reached = np.hypot(estimated_pose.x - active_target[0], estimated_pose.y - active_target[1]) <= 0.35
            target_progress = coverage_history[-1] - active_target_start_coverage
            if target_reached or (active_target_steps >= 28 and target_progress < 0.01):
                blocked_targets.append(active_target)
                active_target = None
                follower.set_path([])
                last_plan = []
                replan_triggers += 1

        if last_plan and _path_blocked_by_environment(env, last_plan):
            follower.set_path([])
            last_plan = []
            active_target = None
            replan_triggers += 1

        if follower.is_done():
            replan_triggers += 1
            targets = rank_frontier_targets(
                occ,
                (estimated_pose.x, estimated_pose.y),
                robot_theta=estimated_pose.theta,
                policy=frontier_policy,
                blocked_targets=blocked_targets,
                revisit_counts=revisit_counts,
            )
            if not targets:
                stop_reason = "frontier_exhausted"
                break
            result = None
            chosen_target = None
            planning_attempts = [
                ([target for target in targets if not _is_blocked_target(target, blocked_targets)], False),
                (targets, False),
                ([target for target in targets if not _is_blocked_target(target, blocked_targets)], True),
                (targets, True),
            ]
            chosen_backend = requested_planner_name
            for candidate_targets, allow_unknown in planning_attempts:
                if not candidate_targets:
                    continue
                for target in candidate_targets:
                    for backend_name, backend in _planner_sequence(planner_name):
                        candidate = backend(
                            grid=occ,
                            start_xy=(estimated_pose.x, estimated_pose.y),
                            goal_xy=target,
                            robot_radius=env.robot_radius,
                            allow_unknown=allow_unknown,
                        )
                        if candidate.success:
                            result = candidate
                            chosen_target = target
                            chosen_backend = backend_name
                            break
                    if result is not None:
                        break
                if result is not None:
                    break
            if result is None:
                stop_reason = "planner_failed"
                break
            raw_path = list(result.path)
            smoothed_path = shortcut_smooth_path(
                grid=occ,
                path=raw_path,
                robot_radius=env.robot_radius,
                allow_unknown=False,
            )
            raw_length, _ = path_quality(raw_path)
            smooth_length, turn_count = path_quality(smoothed_path)
            total_raw_path_length += raw_length
            total_smoothed_path_length += smooth_length
            total_turn_count += turn_count
            last_plan = smoothed_path
            follower.set_path(smoothed_path[1:])
            active_target = chosen_target
            active_target_start_coverage = coverage_history[-1]
            active_target_steps = 0
            revisit_counts[occ.world_to_grid(*chosen_target)] = revisit_counts.get(occ.world_to_grid(*chosen_target), 0) + 1
            replans += 1
            _ = chosen_backend

        linear, angular = follower.command(estimated_pose)
        if stagnant_steps >= 12:
            recovery_steps = 6
            stagnant_steps = 0
            if active_target is not None:
                blocked_targets.append(active_target)
                active_target = None
                follower.set_path([])
                last_plan = []
            replan_triggers += 1
            recovery_events += 1
        if recovery_steps > 0:
            linear = 0.0
            angular = 1.1
            recovery_steps -= 1
        if emergency_stop(scan.ranges, stop_distance=0.16, scan_angles=scan.angles, front_angle=0.45):
            linear = 0.0
            angular = 1.2

        _, collision = env.step(linear=linear, angular=angular, dt=0.2)
        if collision:
            collisions += 1
            recovery_steps = max(recovery_steps, 8)
            recovery_events += 1
            if active_target is not None:
                blocked_targets.append(active_target)
                active_target = None
            follower.set_path([])
            last_plan = []
            replan_triggers += 1

    success = bool(coverage_history and coverage_history[-1] >= coverage_goal)
    runtime_seconds = perf_counter() - started_at
    known_cells = int(np.count_nonzero(occ.grid != UNKNOWN))
    if not frames or frames[-1].coverage != coverage_history[-1]:
        frames.append(
            RenderFrame(
                occupancy=occ.grid.copy(),
                trajectory=list(env.trajectory),
                planner_path=list(last_plan),
                frontier_points=[occ.grid_to_world(gx, gy) for gx, gy in frontier_cells(occ)],
                robot_pose=Pose2D(x=env.pose.x, y=env.pose.y, theta=env.pose.theta),
                coverage=coverage_history[-1] if coverage_history else 0.0,
            )
        )

    metrics = RunMetrics(
        map_name=map_name,
        planner_name=requested_planner_name,
        planner_policy="fallback_rrt" if requested_planner_name == "hybrid" else "single",
        frontier_policy=frontier_policy,
        seed=seed,
        success=success,
        stop_reason=stop_reason,
        steps=len(coverage_history),
        runtime_seconds=runtime_seconds,
        coverage=coverage_history[-1] if coverage_history else 0.0,
        path_length=_trajectory_length(env.trajectory),
        raw_path_length=total_raw_path_length,
        smoothed_path_length=total_smoothed_path_length,
        path_turn_count=total_turn_count,
        collisions=collisions,
        replans=replans,
        replan_triggers=replan_triggers,
        recovery_events=recovery_events,
        disturbance_name=disturbance_name,
        disturbance_events=env.disturbance_events,
        range_noise_std=range_noise_std,
        dropout_prob=dropout_prob,
        pose_noise_std=pose_noise_std,
        explored_cells=known_cells,
        known_cells=known_cells,
        coverage_history=coverage_history,
    )

    if write_artifacts:
        Path("demo").mkdir(exist_ok=True)
        Path("reports").mkdir(exist_ok=True)
        save_run_artifacts(
            demo_path=Path("demo") / f"demo_{map_name}_{requested_planner_name}.gif",
            video_path=Path("demo") / f"demo_{map_name}_{requested_planner_name}.mp4",
            final_map_path=Path("reports") / f"final_map_{map_name}_{requested_planner_name}.png",
            coverage_path=Path("reports") / f"coverage_{map_name}_{requested_planner_name}_seed{seed}.png",
            obstacle_grid=obstacle_grid,
            resolution=env.resolution,
            frames=frames,
        )
        write_metrics_json(Path("reports") / f"metrics_{map_name}_{requested_planner_name}_seed{seed}.json", metrics)
    return metrics


def _trajectory_length(trajectory: list[tuple[float, float]]) -> float:
    total = 0.0
    for (x0, y0), (x1, y1) in zip(trajectory, trajectory[1:]):
        total += float(np.hypot(x1 - x0, y1 - y0))
    return total


def _is_blocked_target(target: tuple[float, float], blocked_targets: list[tuple[float, float]], radius: float = 0.75) -> bool:
    for blocked in blocked_targets:
        if float(np.hypot(target[0] - blocked[0], target[1] - blocked[1])) <= radius:
            return True
    return False


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, choices=["empty", "office", "cluttered", "narrow", "maze"])
    parser.add_argument("--planner", required=True, choices=sorted(list(BASE_PLANNERS) + ["hybrid"]))
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--coverage-goal", type=float, default=None)
    parser.add_argument("--frontier-policy", choices=["naive", "information_gain"], default="naive")
    parser.add_argument("--disturbance", choices=["none", "moving_obstacle", "temporary_block"], default="none")
    parser.add_argument("--range-noise-std", type=float, default=0.0)
    parser.add_argument("--dropout-prob", type=float, default=0.0)
    parser.add_argument("--pose-noise-std", type=float, default=0.0)
    args = parser.parse_args()

    metrics = run_demo(
        args.map,
        args.planner,
        args.seed,
        max_steps=args.max_steps,
        coverage_goal=args.coverage_goal,
        frontier_policy=args.frontier_policy,
        disturbance_name=args.disturbance,
        range_noise_std=args.range_noise_std,
        dropout_prob=args.dropout_prob,
        pose_noise_std=args.pose_noise_std,
    )
    print(json.dumps(asdict(metrics), indent=2))


def _planner_sequence(planner_name: str):
    if planner_name == "hybrid":
        return [("astar", BASE_PLANNERS["astar"]), ("rrt", BASE_PLANNERS["rrt"])]
    return [(planner_name, BASE_PLANNERS[planner_name])]


def _path_blocked_by_environment(env: Grid2DEnv, path: list[tuple[float, float]]) -> bool:
    if len(path) < 2:
        return False
    for x, y in path[1:]:
        if env.is_occupied_world(x, y):
            return True
    return False


def _estimated_pose(true_pose: Pose2D, pose_noise_std: float, drift_state: np.ndarray) -> Pose2D:
    if pose_noise_std <= 0.0:
        return Pose2D(x=true_pose.x, y=true_pose.y, theta=true_pose.theta)
    drift_state += np.random.normal(loc=0.0, scale=pose_noise_std * 0.08, size=3)
    return Pose2D(
        x=float(true_pose.x + drift_state[0]),
        y=float(true_pose.y + drift_state[1]),
        theta=float(true_pose.theta + drift_state[2] * 0.5),
    )


if __name__ == "__main__":
    main()
