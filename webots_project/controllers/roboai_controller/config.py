import os

# Timing
TIME_STEP_MS = 64

# Motor device names
LEFT_MOTOR_NAME  = "left wheel motor"
RIGHT_MOTOR_NAME = "right wheel motor"

# E-puck IR sensors
EPUCK_IR_NAMES = [f"ps{i}" for i in range(8)]
IR_MAX = 4000.0
FRONT_THRESH = 0.25

# Flip if steering is wrong
EPUCK_LAYOUT = "A" 
if EPUCK_LAYOUT == "A":
    LEFT_GROUP  = [5, 6, 7]
    RIGHT_GROUP = [0, 1, 2]
    FRONT_GROUP = [7, 0, 1]
else:
    LEFT_GROUP  = [0, 1, 2]
    RIGHT_GROUP = [5, 6, 7]
    FRONT_GROUP = [1, 6, 7]

# Motion / avoidance
BASE_SPEED  = 2.0
AVOID_GAIN  = 3.0
# open-loop helpers (used for timed turns)
FORWARD_SPEED = 2.0
TURN_SPEED    = 1.5
SECS_PER_DEG  = 0.010

# Diff-drive kinematics (for pose estimate)
WHEEL_RADIUS_M = 0.0205
AXLE_LENGTH_M  = 0.053

# Logs to <repo>/data/logs
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
LOG_DIR = os.path.join(REPO_ROOT, "data", "logs")