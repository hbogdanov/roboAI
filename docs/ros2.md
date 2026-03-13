# ROS2 Prototype

The ROS2 workspace in `roboai_ros2_ws/` is an early deployment prototype for ROS2 Humble on Ubuntu 22.04.

Purpose:

- validate ROS message flow and node boundaries
- connect the RoboAI autonomy core to TurtleBot3 simulation
- support closed-loop sensing, planning, and control in Gazebo

Current scope:

- single-robot exploration stack
- `/scan`, `/odom`, `/map`, `/goal`, `/path`, `/cmd_vel`
- launchable node stack for TurtleBot3 integration

Note: The ROS2 nodes import the core RoboAI modules.

If running from the workspace, ensure the Python path includes the benchmark core:

```bash
export PYTHONPATH=$(pwd)/../src:$PYTHONPATH
```

If Gazebo GUI crashes (`gzclient`), the simulation backend (`gzserver`) may still run.

You can still validate the stack via:

```bash
ros2 topic hz /scan
ros2 topic echo /odom
```

RViz visualization is recommended for debugging.

Not yet at parity with the benchmark runtime:

- full semantic frontier scoring
- full planner parity across A*, RRT, and RRT*
- learned frontier scoring
- multi-robot ROS2 deployment
- mature RViz visualization tooling
