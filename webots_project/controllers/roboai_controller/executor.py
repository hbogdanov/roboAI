from __future__ import annotations

from typing import List, Dict, Optional, Tuple, Any
import math

from config import SECS_PER_DEG, TURN_SPEED
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
        self.turn_target_secs: Optional[float] = None
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

    def load(self, plan: List[Dict[str, Any]], constraints: Optional[Dict[str, Any]] = None):
        self.plan = plan or [{"op": "stop"}]
        self.constraints = constraints or {"speed_limit": 0.5, "avoid": []}
        self.idx = 0
        self.op_timer = 0.0
        self.turn_target_secs = None
        self.last_cmd = (0.0, 0.0)
        self._reset_nav_state()
        self._frontier_target = None
        self._frontier_refresh_timer = 0.0
        self.last_goal_reached = None
        self._collision_burst = 0.0
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

    def _set_state(self, new_state: str, reason: str = ""):
        if self.behavior_state == new_state:
            return
        prev = self.behavior_state
        self.behavior_state = new_state
        self.log.event(op="state_transition", frm=prev, to=new_state, reason=reason)

    def _wrap_pi(self, a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _start_nav_if_needed(self, target_xy: Tuple[float, float], state, occ_grid, mode: str):
        if self._nav_target == target_xy and self._nav_mode == mode:
            return
        self._nav_target = target_xy
        self._nav_mode = mode
        self._nav_path = []
        self._nav_wp_idx = 0
        self._nav_started = True

        tx, ty = target_xy
        if occ_grid is not None:
            try:
                path_world = plan_world_path(
                    occ_grid=occ_grid,
                    start_xy=(float(state.x), float(state.y)),
                    goal_xy=(tx, ty),
                )
                if path_world:
                    self._nav_path = path_world[::4]
                    if self._nav_path[-1] != path_world[-1]:
                        self._nav_path.append(path_world[-1])
                    self.log.event(
                        op="path_planned",
                        mode=mode,
                        nodes=len(path_world),
                        waypoints=len(self._nav_path),
                        tx=tx,
                        ty=ty,
                    )
            except Exception:
                self.log.event(op="path_plan_failed", mode=mode, tx=tx, ty=ty)

        self.log.event(op="goto_start", mode=mode, tx=tx, ty=ty)
        self._set_state("NAVIGATE", reason=f"{mode}_start")

    def _choose_nav_subtarget(self, state, target_xy: Tuple[float, float]) -> Tuple[float, float]:
        tx, ty = target_xy
        if not self._nav_path:
            return tx, ty

        while self._nav_wp_idx < len(self._nav_path):
            wx, wy = self._nav_path[self._nav_wp_idx]
            if math.hypot(float(state.x) - wx, float(state.y) - wy) <= 0.12:
                self._nav_wp_idx += 1
            else:
                break

        if self._nav_wp_idx < len(self._nav_path):
            return self._nav_path[self._nav_wp_idx]
        return tx, ty

    def _goto_control(self, state, ir, occ_grid, target_xy: Tuple[float, float], mode: str, dt: float, accept_radius: float) -> bool:
        """
        Closed-loop controller with explicit metrics/events.
        Returns True when target reached.
        """
        self._start_nav_if_needed(target_xy, state=state, occ_grid=occ_grid, mode=mode)
        tx, ty = target_xy
        tgt_x, tgt_y = self._choose_nav_subtarget(state=state, target_xy=target_xy)

        dx_goal = tx - float(state.x)
        dy_goal = ty - float(state.y)
        goal_error = math.hypot(dx_goal, dy_goal)

        # If we are in recovery mode, back up briefly and then force a replan.
        if self._recovery_s > 0.0:
            self._recovery_s = max(0.0, self._recovery_s - max(0.0, dt))
            self._set_state("AVOID", reason=f"{mode}_recovery")
            self.drive.set_velocity(-1.2, -1.2)
            self.last_cmd = (-1.2, -1.2)
            self.log.event(op="goto_recovery_tick", mode=mode, goal_error_m=goal_error, recovery_left_s=self._recovery_s)
            if self._recovery_s <= 0.0:
                self._nav_target = None  # trigger path recompute on next tick
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

        l_avoid, r_avoid, front = steer(ir, base_speed=base)
        if front >= 0.25:
            self._collision_burst += 1.0
            self._set_state("AVOID", reason=f"{mode}_front_obstacle")
            l, r = l_avoid, r_avoid
            self.log.event(op="collision_warning", front=front, mode=mode)
            # If we keep colliding in a burst, force a short escape cycle.
            if self._collision_burst >= 6.0:
                self.log.event(op="collision_burst_escape", mode=mode, burst=self._collision_burst, goal_error_m=goal_error)
                self._collision_burst = 0.0
                self._recovery_s = 0.8
                self._nav_target = None
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
        if improve >= 0.003:
            self._nav_no_improve_s = 0.0
        else:
            self._nav_no_improve_s += max(0.0, dt)
        self._nav_prev_goal_error = goal_error

        if self._nav_no_improve_s >= 3.0:
            self.log.event(op="goto_stuck", mode=mode, goal_error_m=goal_error, no_improve_s=self._nav_no_improve_s)
            self._nav_no_improve_s = 0.0
            self._recovery_s = 0.6
            self._nav_target = None  # force replan after recovery

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

    def step(self, dt: float, ir: List[Optional[float]], state=None, occ_grid=None) -> bool:
        if self.idx >= len(self.plan):
            self.drive.stop()
            self.last_cmd = (0.0, 0.0)
            return True

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
            if self.turn_target_secs is None:
                deg = abs(float(step.get("deg", 90)))
                self.turn_target_secs = max(0.0, SECS_PER_DEG * deg)
                direction = str(step.get("dir", "left")).lower()
                if direction == "left":
                    self.drive.set_velocity(-TURN_SPEED, TURN_SPEED)
                    self.last_cmd = (-TURN_SPEED, TURN_SPEED)
                else:
                    self.drive.set_velocity(TURN_SPEED, -TURN_SPEED)
                    self.last_cmd = (TURN_SPEED, -TURN_SPEED)
                self.log.event(op="turn_start", dir=direction, deg=deg, secs=self.turn_target_secs)
            self.op_timer += dt
            if self.op_timer >= (self.turn_target_secs or 0.0):
                self.drive.stop()
                self.log.event(op="turn_done")
                self.idx += 1
                self.op_timer = 0.0
                self.turn_target_secs = None

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
            self._set_state("RETURN_HOME", reason="return_base_step")
            self.drive.stop()
            self.log.event(op="return_base")
            self.idx += 1
            self.last_cmd = (0.0, 0.0)

        elif op == "goto":
            if state is None:
                self.log.event(op="goto_skip_no_state")
                self.idx += 1
            else:
                tx = float(step.get("x", state.x))
                ty = float(step.get("y", state.y))
                goal_name = str(step.get("goal", "")).strip() or None
                accept_radius = float(step.get("accept_radius", 0.08))
                reached = self._goto_control(
                    state=state,
                    ir=ir,
                    occ_grid=occ_grid,
                    target_xy=(tx, ty),
                    mode="goto",
                    dt=dt,
                    accept_radius=accept_radius,
                )
                if reached:
                    self.last_goal_reached = {"x": tx, "y": ty, "goal": goal_name}
                    self.idx += 1

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
                    reached = self._goto_control(
                        state=state,
                        ir=ir,
                        occ_grid=occ_grid,
                        target_xy=self._frontier_target,
                        mode="explore",
                        dt=dt,
                        accept_radius=0.12,
                    )
                    if reached:
                        self.log.event(op="frontier_reached", x=self._frontier_target[0], y=self._frontier_target[1])
                        self._frontier_target = None

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
