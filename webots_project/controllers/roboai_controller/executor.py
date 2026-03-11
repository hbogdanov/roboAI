from __future__ import annotations

from typing import List, Dict, Optional, Tuple, Any
import math
import os

from config import TURN_SPEED
from logger import RunLogger
from navigator import steer
from path_planner import plan_world_path
from frontier import frontier_points_world
from controller import Robot
from motion import Drive
from sensors import Sensors


class PlanExecutor:
    """
    Executes mixed primitive + waypoint plans.
    Supported ops:
      - forward(seconds)
      - turn(dir, deg)
      - scan(sensor)
      - wait(seconds)
      - return_base
      - goto(x, y)
      - face(theta_deg)
      - explore(seconds)
      - stop
    """

    def __init__(self, robot: "Robot", drive: Drive, sensors: Sensors, log: RunLogger):
        self.robot = robot
        self.drive = drive
        self.sensors = sensors
        self.log = log

        self.plan: List[Dict[str, Any]] = []
        self.constraints: Dict[str, Any] = {"speed_limit": 0.5, "avoid": []}
        self.idx = 0
        self.op_timer = 0.0
        self._turn_target_theta: Optional[float] = None
        self._turn_tol_rad: float = 0.07
        self.last_cmd: Tuple[float, float] = (0.0, 0.0)

        # Shared navigation state for goto/explore.
        self._nav_target: Optional[Tuple[float, float]] = None
        self._nav_path: List[Tuple[float, float]] = []
        self._nav_wp_idx: int = 0
        self._nav_started: bool = False
        self._nav_mode: str = "goto"
        self._nav_prev_goal_error: Optional[float] = None
        self._nav_no_improve_s: float = 0.0
        self._recovery_s: float = 0.0

        # Frontier exploration state.
        self._frontier_target: Optional[Tuple[float, float]] = None
        self._frontier_refresh_s = 1.5
        self._frontier_refresh_timer = 0.0
        self.behavior_state = "IDLE"
        self.last_goal_reached: Optional[Dict[str, Any]] = None
        self._collision_burst = 0.0
        self._planner_cfg: Dict[str, Any] = {
            "inflate_cells": 2,
            "goal_clearance_cells": 0,
            "max_goal_snap_cells": 6,
            "block_unknown": True,
            "local_avoid_mode": "lidar",
            "path_stride": 1,
        }
        self._home_pose: Optional[Tuple[float, float, float]] = None
        self._last_good_nav_path: List[Tuple[float, float]] = []
        self._last_good_nav_wp_idx: int = 0
        self._nav_replan_attempts: int = 0
        self._nav_replan_limit: int = 6
        self._nav_no_progress_events: int = 0
        self._nav_collision_burst_events: int = 0
        self._last_nav_fail_reason: Optional[str] = None
        self._local_avoid_mode_runtime: str = "lidar"

    def load(self, plan: List[Dict[str, Any]], constraints: Optional[Dict[str, Any]] = None):
        self.plan = plan or [{"op": "stop"}]
        self.constraints = constraints or {"speed_limit": 0.5, "avoid": []}
        self.idx = 0
        self.op_timer = 0.0
        self._turn_target_theta = None
        self.last_cmd = (0.0, 0.0)
        self._reset_nav_state()
        self._frontier_target = None
        self._frontier_refresh_timer = 0.0
        self.last_goal_reached = None
        self._collision_burst = 0.0
        self._home_pose = None
        self._last_good_nav_path = []
        self._last_good_nav_wp_idx = 0
        self._nav_replan_attempts = 0
        self._nav_replan_limit = 6
        self._nav_no_progress_events = 0
        self._nav_collision_burst_events = 0
        self._last_nav_fail_reason = None
        self._local_avoid_mode_runtime = "lidar"
        planner_cfg = self.constraints.get("planner", {}) if isinstance(self.constraints, dict) else {}
        self._planner_cfg = {
            "inflate_cells": int(planner_cfg.get("inflate_cells", 2)),
            "goal_clearance_cells": int(planner_cfg.get("goal_clearance_cells", 0)),
            "max_goal_snap_cells": int(planner_cfg.get("max_goal_snap_cells", 6)),
            "block_unknown": bool(planner_cfg.get("block_unknown", True)),
            "replan_limit": int(planner_cfg.get("replan_limit", 6)),
            "local_avoid_mode": str(planner_cfg.get("local_avoid_mode", "lidar")).strip().lower(),
            "path_stride": int(planner_cfg.get("path_stride", 1)),
        }
        mode = str(self._planner_cfg.get("local_avoid_mode", "lidar")).strip().lower()
        self._local_avoid_mode_runtime = mode if mode in {"lidar", "ir"} else "lidar"
        self._nav_replan_limit = max(1, int(self._planner_cfg.get("replan_limit", 6)))
        self._set_state("PLAN", reason="plan_loaded")
        self.log.event(op="plan_loaded", steps=len(self.plan), constraints=self.constraints)

    def _reset_nav_state(self):
        self._nav_target = None
        self._nav_path = []
        self._nav_wp_idx = 0
        self._nav_started = False
        self._nav_mode = "goto"
        self._nav_prev_goal_error = None
        self._nav_no_improve_s = 0.0
        self._recovery_s = 0.0
        self._nav_replan_attempts = 0
        self._nav_no_progress_events = 0
        self._nav_collision_burst_events = 0
        self._last_nav_fail_reason = None
        self._last_good_nav_path = []
        self._last_good_nav_wp_idx = 0

    def _set_state(self, new_state: str, reason: str = ""):
        if self.behavior_state == new_state:
            return
        prev = self.behavior_state
        self.behavior_state = new_state
        self.log.event(op="state_transition", frm=prev, to=new_state, reason=reason)

    def _wrap_pi(self, a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _capture_home_pose_if_needed(self, state):
        if self._home_pose is not None or state is None:
            return
        self._home_pose = (float(state.x), float(state.y), float(state.theta))
        self.log.event(
            op="home_pose_set",
            home_x=self._home_pose[0],
            home_y=self._home_pose[1],
            home_theta=self._home_pose[2],
        )

    def _lidar_sector_min(
        self,
        ranges: List[float],
        angle_min: float,
        angle_inc: float,
        sector_center_deg: float,
        sector_half_width_deg: float,
        range_max: float,
    ) -> float:
        if not ranges:
            return float(range_max)
        center = math.radians(sector_center_deg)
        half = math.radians(max(0.0, sector_half_width_deg))
        vals: List[float] = []
        min_valid = 0.07
        for i, rv in enumerate(ranges):
            try:
                r = float(rv)
            except Exception:
                continue
            if not math.isfinite(r) or r < min_valid:
                continue
            ang = float(angle_min) + float(i) * float(angle_inc)
            err = self._wrap_pi(ang - center)
            if abs(err) <= half:
                vals.append(r)
        if not vals:
            return float(range_max)
        vals.sort()
        # Use a lower-percentile distance instead of raw min to reject speckle outliers.
        idx = max(0, min(len(vals) - 1, int(0.2 * (len(vals) - 1))))
        return float(vals[idx])

    def _lidar_local_avoidance(
        self,
        lidar_scan: Optional[Tuple[List[float], float, float, float]],
        base: float,
        heading_error: float = 0.0,
    ) -> Optional[Tuple[float, float, Dict[str, Any]]]:
        if lidar_scan is None:
            return None
        ranges, angle_min, angle_inc, range_max = lidar_scan
        if not ranges:
            return None

        front_min = self._lidar_sector_min(ranges, angle_min, angle_inc, 0.0, 20.0, range_max)
        left_front_min = self._lidar_sector_min(ranges, angle_min, angle_inc, 35.0, 25.0, range_max)
        right_front_min = self._lidar_sector_min(ranges, angle_min, angle_inc, -35.0, 25.0, range_max)

        front_stop_m = 0.08

        # Hard stop/rotate when front sector is blocked.
        if front_min < front_stop_m:
            turn_v = max(0.8, min(1.8, base * 1.3))
            if abs(heading_error) > 0.20:
                # Prefer turning toward goal heading when possible.
                if heading_error > 0.0:
                    l, r = -turn_v, turn_v
                    decision = "front_blocked_turn_to_goal_left"
                else:
                    l, r = turn_v, -turn_v
                    decision = "front_blocked_turn_to_goal_right"
            elif left_front_min <= right_front_min:
                # Obstacle stronger on left-front, rotate right.
                l, r = turn_v, -turn_v
                decision = "front_blocked_turn_right"
            else:
                l, r = -turn_v, turn_v
                decision = "front_blocked_turn_left"
            return l, r, {
                "front_min_m": front_min,
                "left_front_min_m": left_front_min,
                "right_front_min_m": right_front_min,
                "decision": decision,
            }

        # In narrow passages, do not side-bias off corridor walls.
        # Let path tracking handle steering unless the front is truly blocked.
        return None

    def _record_nav_failure(self, mode: str, reason: str, tx: float, ty: float, extra: Optional[Dict[str, Any]] = None):
        payload = {"op": "goto_failed", "mode": mode, "tx": tx, "ty": ty, "fail_reason": reason}
        if extra:
            payload.update(extra)
        self.log.event(**payload)
        self._last_nav_fail_reason = reason

    def _start_nav_if_needed(self, target_xy: Tuple[float, float], state, occ_grid, mode: str):
        if self._nav_target == target_xy and self._nav_mode == mode:
            return True
        self._last_nav_fail_reason = None
        self._nav_target = target_xy
        self._nav_mode = mode
        prev_path = list(self._nav_path)
        prev_wp_idx = self._nav_wp_idx
        self._nav_path = []
        self._nav_wp_idx = 0
        self._nav_started = True
        self._nav_replan_attempts += 1

        tx, ty = target_xy
        if self._nav_replan_attempts > self._nav_replan_limit:
            self.log.event(
                op="replan_limit_reached",
                mode=mode,
                tx=tx,
                ty=ty,
                attempt=self._nav_replan_attempts,
                limit=self._nav_replan_limit,
            )
            # Soft-limit behavior: keep navigating instead of hard-aborting.
            self._nav_replan_attempts = self._nav_replan_limit
        if occ_grid is not None:
            try:
                strict_block_unknown = bool(self._planner_cfg.get("block_unknown", True))
                path_world, meta = plan_world_path(
                    occ_grid=occ_grid,
                    start_xy=(float(state.x), float(state.y)),
                    goal_xy=(tx, ty),
                    block_unknown=strict_block_unknown,
                    inflate_cells=max(0, int(self._planner_cfg.get("inflate_cells", 2))),
                    goal_clearance_cells=max(0, int(self._planner_cfg.get("goal_clearance_cells", 0))),
                    max_goal_snap_cells=max(1, int(self._planner_cfg.get("max_goal_snap_cells", 6))),
                    return_meta=True,
                    smooth_path=False,
                )
                # Bootstrap fallback: early map can be mostly unknown.
                if not path_world and str(meta.get("fail_reason", "")).strip() == "unknown_path" and strict_block_unknown:
                    path_world, meta = plan_world_path(
                        occ_grid=occ_grid,
                        start_xy=(float(state.x), float(state.y)),
                        goal_xy=(tx, ty),
                        block_unknown=False,
                        inflate_cells=max(0, int(self._planner_cfg.get("inflate_cells", 2))),
                        goal_clearance_cells=max(0, int(self._planner_cfg.get("goal_clearance_cells", 0))),
                        max_goal_snap_cells=max(1, int(self._planner_cfg.get("max_goal_snap_cells", 6))),
                        return_meta=True,
                        smooth_path=False,
                    )
                    self.log.event(op="path_plan_bootstrap_unknown", mode=mode, tx=tx, ty=ty)
                if path_world:
                    stride = max(1, int(self._planner_cfg.get("path_stride", 1)))
                    self._nav_path = path_world[::stride]
                    if self._nav_path[-1] != path_world[-1]:
                        self._nav_path.append(path_world[-1])
                    self._last_good_nav_path = list(self._nav_path)
                    self._last_good_nav_wp_idx = self._nav_wp_idx
                    snapped_goal = bool(meta.get("snapped_goal", False))
                    gx_raw, gy_raw = meta.get("goal_grid_raw", (None, None))
                    gx_used, gy_used = meta.get("goal_grid_used", (None, None))
                    raw_world = None
                    used_world = None
                    if gx_raw is not None and gy_raw is not None:
                        raw_world = occ_grid.grid_to_world(int(gx_raw), int(gy_raw))
                    if gx_used is not None and gy_used is not None:
                        used_world = occ_grid.grid_to_world(int(gx_used), int(gy_used))
                    self.log.event(
                        op="path_planned",
                        mode=mode,
                        block_unknown=strict_block_unknown,
                        inflate_cells=max(0, int(self._planner_cfg.get("inflate_cells", 2))),
                        goal_clearance_cells=max(0, int(self._planner_cfg.get("goal_clearance_cells", 0))),
                        goal_snapped=snapped_goal,
                        goal_raw=raw_world,
                        goal_used=used_world,
                        nodes=len(path_world),
                        waypoints=len(self._nav_path),
                        path_stride=stride,
                        replan_attempt=self._nav_replan_attempts,
                        tx=tx,
                        ty=ty,
                    )
                    if snapped_goal:
                        self.log.event(
                            op="goal_snapped",
                            mode=mode,
                            goal_raw=raw_world,
                            goal_used=used_world,
                            tx=tx,
                            ty=ty,
                        )
                    self.log.event(
                        op="goal_clearance_checked",
                        mode=mode,
                        tx=tx,
                        ty=ty,
                        goal_snapped=snapped_goal,
                        goal_raw=raw_world,
                        goal_used=used_world,
                    )
                else:
                    fail_reason = str(meta.get("fail_reason", "")).strip() or "unknown_path"
                    if fail_reason == "unknown_path":
                        # Keep going with direct local targeting when global path is temporarily unavailable.
                        if prev_path:
                            self._nav_path = prev_path
                            self._nav_wp_idx = min(prev_wp_idx, max(0, len(prev_path) - 1))
                        elif self._last_good_nav_path:
                            self._nav_path = list(self._last_good_nav_path)
                            self._nav_wp_idx = min(self._last_good_nav_wp_idx, max(0, len(self._nav_path) - 1))
                        self.log.event(
                            op="path_plan_unavailable",
                            mode=mode,
                            tx=tx,
                            ty=ty,
                            fail_reason=fail_reason,
                            attempt=self._nav_replan_attempts,
                        )
                        self.log.event(
                            op="goal_clearance_checked",
                            mode=mode,
                            tx=tx,
                            ty=ty,
                            goal_snapped=bool(meta.get("snapped_goal", False)),
                            goal_raw=meta.get("goal_grid_raw"),
                            goal_used=meta.get("goal_grid_used"),
                            fail_reason=fail_reason,
                        )
                        self.log.event(op="goto_start", mode=mode, tx=tx, ty=ty)
                        self._set_state("NAVIGATE", reason=f"{mode}_start_no_global_path")
                        return True
                    self._record_nav_failure(
                        mode=mode,
                        reason=fail_reason,
                        tx=tx,
                        ty=ty,
                        extra={"attempt": self._nav_replan_attempts},
                    )
                    self.log.event(
                        op="goal_clearance_checked",
                        mode=mode,
                        tx=tx,
                        ty=ty,
                        goal_snapped=bool(meta.get("snapped_goal", False)),
                        goal_raw=meta.get("goal_grid_raw"),
                        goal_used=meta.get("goal_grid_used"),
                        fail_reason=fail_reason,
                    )
                    self._nav_target = None
                    self._recovery_s = 0.4
                    return False
            except Exception:
                self.log.event(op="path_plan_failed", mode=mode, tx=tx, ty=ty, fail_reason="unknown_path")
                if prev_path:
                    self._nav_path = prev_path
                    self._nav_wp_idx = min(prev_wp_idx, max(0, len(prev_path) - 1))
                elif self._last_good_nav_path:
                    self._nav_path = list(self._last_good_nav_path)
                    self._nav_wp_idx = min(self._last_good_nav_wp_idx, max(0, len(self._nav_path) - 1))
                self.log.event(
                    op="path_plan_unavailable",
                    mode=mode,
                    tx=tx,
                    ty=ty,
                    fail_reason="unknown_path",
                    attempt=self._nav_replan_attempts,
                )
                self.log.event(op="goto_start", mode=mode, tx=tx, ty=ty)
                self._set_state("NAVIGATE", reason=f"{mode}_start_plan_exception")
                return True

        self.log.event(op="goto_start", mode=mode, tx=tx, ty=ty)
        self._set_state("NAVIGATE", reason=f"{mode}_start")
        return True

    def _choose_nav_subtarget(self, state, target_xy: Tuple[float, float]) -> Tuple[float, float]:
        tx, ty = target_xy
        if not self._nav_path:
            return tx, ty

        cur_goal_error = math.hypot(tx - float(state.x), ty - float(state.y))

        while self._nav_wp_idx < len(self._nav_path):
            wx, wy = self._nav_path[self._nav_wp_idx]
            if math.hypot(float(state.x) - wx, float(state.y) - wy) <= 0.12:
                self._nav_wp_idx += 1
            else:
                break

        # Skip early path points that move away from the final goal.
        while self._nav_wp_idx < len(self._nav_path):
            wx, wy = self._nav_path[self._nav_wp_idx]
            wp_goal_error = math.hypot(tx - wx, ty - wy)
            if wp_goal_error > (cur_goal_error + 0.08):
                self._nav_wp_idx += 1
                continue
            # If a waypoint requires near about-face and barely helps goal error,
            # skip it to avoid spin-in-place behavior near obstacles/passages.
            dx = wx - float(state.x)
            dy = wy - float(state.y)
            wp_heading = math.atan2(dy, dx)
            wp_heading_err = abs(self._wrap_pi(wp_heading - float(state.theta)))
            if wp_heading_err > 2.2 and wp_goal_error > (cur_goal_error - 0.12):
                self._nav_wp_idx += 1
                continue
            break

        if self._nav_wp_idx < len(self._nav_path):
            return self._nav_path[self._nav_wp_idx]
        return tx, ty

    def _goto_control(
        self,
        state,
        ir,
        occ_grid,
        target_xy: Tuple[float, float],
        mode: str,
        dt: float,
        accept_radius: float,
        lidar_scan: Optional[Tuple[List[float], float, float, float]] = None,
    ) -> bool:
        """
        Closed-loop controller with explicit metrics/events.
        Returns True when target reached.
        """
        if not self._start_nav_if_needed(target_xy, state=state, occ_grid=occ_grid, mode=mode):
            return False
        tx, ty = target_xy
        tgt_x, tgt_y = self._choose_nav_subtarget(state=state, target_xy=target_xy)

        dx_goal = tx - float(state.x)
        dy_goal = ty - float(state.y)
        goal_error = math.hypot(dx_goal, dy_goal)
        desired_goal = math.atan2(dy_goal, dx_goal)
        heading_error = self._wrap_pi(desired_goal - float(state.theta))

        # If we are in recovery mode, back up briefly and then force a replan.
        if self._recovery_s > 0.0:
            self._recovery_s = max(0.0, self._recovery_s - max(0.0, dt))
            self._set_state("AVOID", reason=f"{mode}_recovery")

            rec_turn = 1.0 if heading_error >= 0.0 else -1.0
            self.drive.set_velocity(-rec_turn, rec_turn)
            self.last_cmd = (-rec_turn, rec_turn)
            self.log.event(
                op="goto_recovery_tick",
                mode=mode,
                goal_error_m=goal_error,
                recovery_left_s=self._recovery_s,
            )
            if self._recovery_s <= 0.0:
                self._nav_started = False
                self._nav_prev_goal_error = None
                self._nav_target = None
                self.log.event(op="goto_force_replan", mode=mode, tx=tx, ty=ty)
            return False

        dx = tgt_x - float(state.x)
        dy = tgt_y - float(state.y)
        subtarget_error = math.hypot(dx, dy)

        if goal_error <= accept_radius:
            self.drive.stop()
            self.last_cmd = (0.0, 0.0)
            self.log.event(op="goto_done", mode=mode, x=tx, y=ty, goal_error_m=goal_error, accept_radius=accept_radius)
            self._reset_nav_state()
            self._set_state("IDLE", reason=f"{mode}_done")
            return True

        desired = math.atan2(dy, dx)
        heading_error = self._wrap_pi(desired - float(state.theta))
        speed_limit = float(self.constraints.get("speed_limit", 0.35))
        # Keep waypoint motion conservative in cluttered maps.
        base_nom = max(0.18, min(0.9, speed_limit * 2.0))
        # Slow down near goal to reduce wall scraping and overshoot.
        goal_scale = max(0.45, min(1.0, goal_error / 0.6))
        base = base_nom * goal_scale
        k_heading = 1.8

        local_avoid_mode = self._local_avoid_mode_runtime
        use_lidar_avoid = local_avoid_mode != "ir"
        lidar_avoid = self._lidar_local_avoidance(
            lidar_scan=lidar_scan, base=base, heading_error=heading_error
        ) if use_lidar_avoid else None
        if lidar_avoid is not None:
            l, r, lidar_meta = lidar_avoid
            self._collision_burst += 1.0
            self._set_state("AVOID", reason=f"{mode}_lidar_local_avoid")
            self.log.event(op="lidar_avoid", mode=mode, **lidar_meta)
            if self._collision_burst >= 8.0:
                self.log.event(op="collision_burst_escape", mode=mode, burst=self._collision_burst, goal_error_m=goal_error)
                self._nav_collision_burst_events += 1
                # If lidar keeps causing burst escapes, degrade to IR mode instead of aborting.
                if local_avoid_mode == "lidar" and self._nav_collision_burst_events >= 2:
                    self._local_avoid_mode_runtime = "ir"
                    self._nav_collision_burst_events = 0
                    self.log.event(op="local_avoid_fallback", mode=mode, new_mode="ir", reason="lidar_collision_burst")
                elif self._nav_collision_burst_events >= 3:
                    self._record_nav_failure(
                        mode=mode,
                        reason="collision_burst",
                        tx=tx,
                        ty=ty,
                        extra={"burst_events": self._nav_collision_burst_events},
                    )
                    self.drive.stop()
                    self.last_cmd = (0.0, 0.0)
                    return False
                self._collision_burst = 0.0
                self._recovery_s = 0.8
                return False
        else:
            l_avoid, r_avoid, front = steer(ir, base_speed=base)
            if front >= 0.25:
                self._collision_burst += 1.0
                self._set_state("AVOID", reason=f"{mode}_front_obstacle")
                l, r = l_avoid, r_avoid
                self.log.event(op="collision_warning", front=front, mode=mode)
                # If we keep colliding in a burst, force a short escape cycle.
                if self._collision_burst >= 6.0:
                    self.log.event(op="collision_burst_escape", mode=mode, burst=self._collision_burst, goal_error_m=goal_error)
                    self._nav_collision_burst_events += 1
                    if local_avoid_mode == "lidar" and self._nav_collision_burst_events >= 2:
                        self._local_avoid_mode_runtime = "ir"
                        self._nav_collision_burst_events = 0
                        self.log.event(op="local_avoid_fallback", mode=mode, new_mode="ir", reason="lidar_collision_burst")
                    elif self._nav_collision_burst_events >= 3:
                        self._record_nav_failure(
                            mode=mode,
                            reason="collision_burst",
                            tx=tx,
                            ty=ty,
                            extra={"burst_events": self._nav_collision_burst_events},
                        )
                        self.drive.stop()
                        self.last_cmd = (0.0, 0.0)
                        return False
                    self._collision_burst = 0.0
                    self._recovery_s = 0.8
                    return False
            elif abs(heading_error) > 0.28:
                self._collision_burst = max(0.0, self._collision_burst - 0.5)
                self._set_state("NAVIGATE", reason=f"{mode}_heading_align")
                w_lim = max(0.25, base)
                w = max(-w_lim, min(w_lim, k_heading * heading_error))
                l, r = -w, w
            else:
                self._collision_burst = max(0.0, self._collision_burst - 0.75)
                self._set_state("NAVIGATE", reason=f"{mode}_track")
                corr = max(-base * 0.9, min(base * 0.9, k_heading * heading_error))
                l, r = base - corr, base + corr

        self.drive.set_velocity(l, r)
        self.last_cmd = (l, r)

        # Stuck detection: goal error not improving enough for several seconds.
        if self._nav_prev_goal_error is None:
            self._nav_prev_goal_error = goal_error
        improve = self._nav_prev_goal_error - goal_error
        # Small per-tick improvement is still progress in tight spaces.
        if improve >= 0.0003:
            self._nav_no_improve_s = 0.0
        else:
            self._nav_no_improve_s += max(0.0, dt)
        self._nav_prev_goal_error = goal_error

        if self._nav_no_improve_s >= 6.0:
            self.log.event(op="goto_stuck", mode=mode, goal_error_m=goal_error, no_improve_s=self._nav_no_improve_s, fail_reason="no_progress")
            self._nav_no_progress_events += 1
            if self._nav_no_progress_events >= 6:
                self._record_nav_failure(
                    mode=mode,
                    reason="no_progress",
                    tx=tx,
                    ty=ty,
                    extra={"stuck_events": self._nav_no_progress_events, "goal_error_m": goal_error},
                )
                self.drive.stop()
                self.last_cmd = (0.0, 0.0)
                return False
            self._nav_no_improve_s = 0.0
            self._recovery_s = 0.6

        self.log.event(
            op="goto_progress",
            mode=mode,
            x=state.x,
            y=state.y,
            tx=tx,
            ty=ty,
            target_x=tgt_x,
            target_y=tgt_y,
            wp_idx=self._nav_wp_idx,
            wp_total=len(self._nav_path),
            replan_attempt=self._nav_replan_attempts,
            no_progress_s=self._nav_no_improve_s,
            heading_error_rad=heading_error,
            subtarget_error_m=subtarget_error,
            goal_error_m=goal_error,
        )
        return False

    def _nearest_point(self, x: float, y: float, pts: List[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        if not pts:
            return None
        best = None
        best_d = float("inf")
        for px, py in pts:
            d = math.hypot(px - x, py - y)
            if d < best_d:
                best_d = d
                best = (px, py)
        return best

    def step(
        self,
        dt: float,
        ir: List[Optional[float]],
        state=None,
        occ_grid=None,
        lidar_scan: Optional[Tuple[List[float], float, float, float]] = None,
    ) -> bool:
        if self.idx >= len(self.plan):
            self.drive.stop()
            self.last_cmd = (0.0, 0.0)
            return True
        self._capture_home_pose_if_needed(state)

        step = self.plan[self.idx]
        op = str(step.get("op", "")).lower()

        if op == "forward":
            secs = float(step.get("seconds", 1.0))
            l, r, front = steer(ir)
            self.drive.set_velocity(l, r)
            self.last_cmd = (l, r)
            self.log.event(op="spa_forward_tick", l=l, r=r, front=front)
            self.op_timer += dt
            if self.op_timer >= secs:
                self.drive.stop()
                self.log.event(op="forward_done", seconds=secs)
                self.idx += 1
                self.op_timer = 0.0

        elif op == "turn":
            if state is None:
                self.drive.stop()
                self.last_cmd = (0.0, 0.0)
                self.log.event(op="turn_skip_no_state")
                self.idx += 1
            else:
                if self._turn_target_theta is None:
                    deg = abs(float(step.get("deg", 90)))
                    direction = str(step.get("dir", "left")).lower()
                    signed_deg = deg if direction == "left" else -deg
                    self._turn_target_theta = self._wrap_pi(float(state.theta) + math.radians(signed_deg))
                    self.log.event(
                        op="turn_start",
                        dir=direction,
                        deg=deg,
                        start_theta=float(state.theta),
                        target_theta=float(self._turn_target_theta),
                        tolerance_rad=float(self._turn_tol_rad),
                    )

                err = self._wrap_pi(float(self._turn_target_theta) - float(state.theta))
                if abs(err) <= self._turn_tol_rad:
                    self.drive.stop()
                    self.last_cmd = (0.0, 0.0)
                    self.log.event(op="turn_done", err=err, target_theta=float(self._turn_target_theta))
                    self.idx += 1
                    self.op_timer = 0.0
                    self._turn_target_theta = None
                else:
                    turn_v = max(0.35, min(TURN_SPEED, abs(err) * 1.8))
                    l = -turn_v if err > 0 else turn_v
                    r = turn_v if err > 0 else -turn_v
                    self.drive.set_velocity(l, r)
                    self.last_cmd = (l, r)
                    self.log.event(op="turn_tick", err=err, target_theta=float(self._turn_target_theta))

        elif op == "scan":
            self._set_state("SCAN", reason="scan_step")
            sensor_name = str(step.get("sensor", "ir")).lower()
            dist = self.sensors.read_front_distance()
            self.log.event(op="scan", sensor=sensor_name, value=dist)
            self.idx += 1
            self.last_cmd = (0.0, 0.0)

        elif op == "wait":
            self._set_state("IDLE", reason="wait_step")
            secs = float(step.get("seconds", 1.0))
            self.drive.stop()
            self.last_cmd = (0.0, 0.0)
            self.op_timer += dt
            if self.op_timer >= secs:
                self.log.event(op="wait_done", seconds=secs)
                self.idx += 1
                self.op_timer = 0.0

        elif op == "return_base":
            self._set_state("RETURN_HOME", reason="return_base_compile")
            self.log.event(op="return_base")
            if self._home_pose is None:
                if state is not None:
                    self._capture_home_pose_if_needed(state)
                else:
                    self.log.event(op="return_base_skip_no_home")
                    self.idx += 1
                    self.last_cmd = (0.0, 0.0)
                    return self.idx >= len(self.plan)

            home_x, home_y, home_theta = self._home_pose if self._home_pose is not None else (0.0, 0.0, 0.0)
            face_deg = math.degrees(home_theta)
            accept_radius = float(step.get("accept_radius", 0.12))
            compiled = [
                {"op": "goto", "x": float(home_x), "y": float(home_y), "goal": "home", "accept_radius": accept_radius},
                {"op": "face", "theta_deg": float(face_deg)},
                {"op": "stop"},
            ]
            self.plan = self.plan[: self.idx] + compiled + self.plan[self.idx + 1 :]
            self._reset_nav_state()
            self.log.event(
                op="return_base_compiled",
                home_x=float(home_x),
                home_y=float(home_y),
                home_theta=float(home_theta),
                accept_radius=accept_radius,
            )
            self.last_cmd = (0.0, 0.0)

        elif op == "goto":
            if state is None:
                self.log.event(op="goto_skip_no_state")
                self.idx += 1
            else:
                tx = float(step.get("x", state.x))
                ty = float(step.get("y", state.y))
                goal_name = str(step.get("goal", "")).strip() or None
                accept_radius = float(step.get("accept_radius", 0.10))
                reached = self._goto_control(
                    state=state,
                    ir=ir,
                    occ_grid=occ_grid,
                    target_xy=(tx, ty),
                    mode="goto",
                    dt=dt,
                    accept_radius=accept_radius,
                    lidar_scan=lidar_scan,
                )
                if reached:
                    self.last_goal_reached = {"x": tx, "y": ty, "goal": goal_name}
                    self.idx += 1
                elif self._last_nav_fail_reason:
                    self.log.event(op="goto_abort", mode="goto", tx=tx, ty=ty, fail_reason=self._last_nav_fail_reason)
                    self.drive.stop()
                    self.last_cmd = (0.0, 0.0)
                    self.idx += 1
                    self._reset_nav_state()

        elif op == "face":
            self._set_state("NAVIGATE", reason="face_step")
            if state is None:
                self.log.event(op="face_skip_no_state")
                self.idx += 1
            else:
                target_deg = float(step.get("theta_deg", 0.0))
                target_rad = math.radians(target_deg)
                err = self._wrap_pi(target_rad - float(state.theta))
                if abs(err) <= 0.10:
                    self.drive.stop()
                    self.last_cmd = (0.0, 0.0)
                    self.log.event(op="face_done", theta_deg=target_deg, err=err)
                    self.idx += 1
                else:
                    turn_v = max(0.3, min(1.8, abs(err) * 1.5))
                    l = -turn_v if err > 0 else turn_v
                    r = turn_v if err > 0 else -turn_v
                    self.drive.set_velocity(l, r)
                    self.last_cmd = (l, r)
                    self.log.event(op="face_tick", theta=state.theta, target_deg=target_deg, err=err)

        elif op == "explore":
            self._set_state("NAVIGATE", reason="explore_step")
            secs = float(step.get("seconds", 20.0))
            self.op_timer += dt
            self._frontier_refresh_timer += dt
            if self.op_timer >= secs:
                self.drive.stop()
                self.last_cmd = (0.0, 0.0)
                self.log.event(op="explore_done", seconds=secs)
                self.idx += 1
                self.op_timer = 0.0
                self._frontier_refresh_timer = 0.0
                self._frontier_target = None
                self._reset_nav_state()
                self._set_state("DONE", reason="explore_done")
            elif state is None or occ_grid is None:
                self._set_state("IDLE", reason="explore_waiting")
                self.drive.stop()
                self.last_cmd = (0.0, 0.0)
                self.log.event(op="explore_waiting_for_state")
            else:
                # Refresh frontier target periodically or when no target exists.
                need_refresh = self._frontier_target is None or self._frontier_refresh_timer >= self._frontier_refresh_s
                if need_refresh:
                    frontiers = frontier_points_world(occ_grid)
                    self.log.event(op="frontier_detected", count=len(frontiers))
                    target = self._nearest_point(float(state.x), float(state.y), frontiers)
                    self._frontier_target = target
                    self._frontier_refresh_timer = 0.0
                    if target is not None:
                        self.log.event(op="frontier_selected", x=target[0], y=target[1])

                if self._frontier_target is None:
                    self._set_state("IDLE", reason="frontier_none")
                    self.drive.stop()
                    self.last_cmd = (0.0, 0.0)
                    self.log.event(op="frontier_none")
                else:
                    explore_accept_radius = 0.14
                    env_accept = os.getenv("ROBOAI_EXPLORE_ACCEPT_RADIUS", "").strip()
                    if env_accept:
                        try:
                            explore_accept_radius = max(0.08, min(0.40, float(env_accept)))
                        except Exception:
                            pass
                    reached = self._goto_control(
                        state=state,
                        ir=ir,
                        occ_grid=occ_grid,
                        target_xy=self._frontier_target,
                        mode="explore",
                        dt=dt,
                        accept_radius=explore_accept_radius,
                        lidar_scan=lidar_scan,
                    )
                    if reached:
                        self.log.event(op="frontier_reached", x=self._frontier_target[0], y=self._frontier_target[1])
                        self._frontier_target = None
                    elif self._last_nav_fail_reason:
                        self.log.event(op="frontier_failed", fail_reason=self._last_nav_fail_reason)
                        self._frontier_target = None
                        self._reset_nav_state()

        elif op == "stop":
            self.drive.stop()
            self.log.event(op="stop")
            self.idx = len(self.plan)
            self.last_cmd = (0.0, 0.0)
            self._set_state("DONE", reason="stop_step")
            return True

        else:
            self.log.event(op="unknown_step", step=step)
            self.idx += 1

        return self.idx >= len(self.plan)
