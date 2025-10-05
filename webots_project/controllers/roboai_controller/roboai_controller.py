from controller import Robot 
from config import TIME_STEP_MS, LEFT_MOTOR_NAME, RIGHT_MOTOR_NAME
from motion import Drive
from sensors import Sensors
from logger import RunLogger

def main():
    print("roboai_controller loaded")
    robot = Robot()
    log = RunLogger()

    # Initialize hardware helpers
    drive = Drive(robot, LEFT_MOTOR_NAME, RIGHT_MOTOR_NAME)
    sensors = Sensors(robot)

    # Simple demo sequence
    log.event(msg="starting demo")
    drive.forward(2.0)
    log.event(op="forward", seconds=2.0)

    drive.turn_left_deg(90)
    log.event(op="turn", dir="left", deg=90)

    # Drive forward while reading optional distance sensor for ~3s
    drive.set_velocity(2.0, 2.0)
    elapsed = 0.0
    while elapsed < 3.0:
        if robot.step(TIME_STEP_MS) == -1:
            break
        dist = sensors.read_front_distance()
        log.event(op="tick", dist=dist)
        elapsed += TIME_STEP_MS / 1000.0

    drive.stop()
    log.event(op="stop")
    log.close()
    print("roboai_controller finished")

if __name__ == "__main__":
    main()