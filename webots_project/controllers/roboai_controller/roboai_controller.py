from controller import Robot
from config import TIME_STEP_MS, LEFT_MOTOR_NAME, RIGHT_MOTOR_NAME
from motion import Drive
from sensors import Sensors
from logger import RunLogger
from state import StateEstimator
from planner_text import get_plan
from executor import PlanExecutor
from sensors import LidarWrapper
from occupancy_grid import OccupancyGrid

RUN_SECONDS = 40.0
COMMAND = "Go forward for 3 seconds, turn left 90, scan, then stop."

def main():
    print("roboai_controller (SPA + planners) loaded")
    robot = Robot()
    log = RunLogger()

    sensors = Sensors(robot)
    drive = Drive(robot, LEFT_MOTOR_NAME, RIGHT_MOTOR_NAME)
    est = StateEstimator()

    lidar = LidarWrapper(robot, name="LDS-01", timestep=TIME_STEP_MS)
    occ_grid = OccupancyGrid(width_m=20.0, height_m=20.0, resolution=0.05)

    # High-level plan
    plan = get_plan(COMMAND)
    print("Plan:", plan)
    log.event(op="plan_built", command=COMMAND, plan=plan)

    execu = PlanExecutor(robot, drive, sensors, log)
    execu.load(plan)

    dt = TIME_STEP_MS / 1000.0
    elapsed = 0.0
    while elapsed < RUN_SECONDS:
        if robot.step(TIME_STEP_MS) == -1:
            break

        # Sense
        ir = sensors.read_ir()
        enc = sensors.read_encoders()

        # State
        state = est.update(enc, dt)

        ranges, angle_min, angle_inc, range_max = lidar.read_scan()
        x, y, th = state.x, state.y, state.theta
        occ_grid.update_from_scan((x, y, th), ranges, angle_min, angle_inc, range_max)

        # Plan + Act
        done = execu.step(dt, ir)

        # Log tick state
        lcmd, rcmd = execu.last_cmd
        log.event(
            op="spa_tick",
            x=state.x, y=state.y, theta=state.theta,
            vl=state.vl, vr=state.vr,
            left_cmd=lcmd, right_cmd=rcmd
        )

        if done:
            break

        elapsed += dt

    drive.stop()
    log.event(op="stop")
    log.close()
    print("roboai_controller finished")

if __name__ == "__main__":
    main()
