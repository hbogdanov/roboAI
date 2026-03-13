# RoboAI ROS2 Prototype

This workspace is an early ROS2 deployment prototype for RoboAI on ROS2 Humble with TurtleBot3 Gazebo.

It is not yet full parity with the benchmark runtime in `src/roboai/`.

What is implemented:

- `/scan` input
- `/odom` input
- `/map` publication
- `/goal` publication
- `/path` publication
- `/cmd_vel` publication
- closed-loop TurtleBot3 simulation path for sensing, planning, and actuation

What is not implemented yet:

- full parity with benchmark frontier scoring
- full parity with all planner backends
- learned frontier model integration
- semantic exploration parity
- multi-robot ROS2 deployment
- polished RViz marker/debug support

## Setup

Target environment:

- Ubuntu 22.04
- ROS2 Humble
- TurtleBot3 Gazebo

From the repository root:

```bash
pip install -e .
cd roboai_ros2_ws
colcon build
source install/setup.bash
```

## Launch

Launch the RoboAI algorithm stack:

```bash
ros2 launch roboai_bringup exploration_stack.launch.py
```

If you kept the older name:

```bash
ros2 launch roboai_bringup sim_single.launch.py
```

Verify topics:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel --once
```

## Worked On My Machine

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Terminal 2:

```bash
cd /mnt/c/Users/Ivan/roboAI/roboai_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch roboai_bringup exploration_stack.launch.py
```

Terminal 3:

```bash
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/Ivan/roboAI/roboai_ros2_ws
source install/setup.bash
ros2 topic info /scan
ros2 topic info /odom
ros2 topic info /cmd_vel
```

## Node Flow

```text
/scan + /odom
  -> mapping_node
  -> /map
  -> frontier_node
  -> /goal
  -> planning_node
  -> /path
  -> control_node
  -> /cmd_vel
```
