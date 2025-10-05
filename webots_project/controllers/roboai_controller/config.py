import os

# Webots controller timing
TIME_STEP_MS = 64

# Motor names
LEFT_MOTOR_NAME  = "left wheel motor"
RIGHT_MOTOR_NAME = "right wheel motor"

# Optional distance sensor name
FRONT_DISTANCE_SENSOR_NAME = None

# Motion tunables
FORWARD_SPEED = 2.0
TURN_SPEED    = 1.5
SECS_PER_DEG  = 0.010  

# Logging: write to <repo>/data/logs no matter where Webots launches from
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
LOG_DIR = os.path.join(REPO_ROOT, "data", "logs")