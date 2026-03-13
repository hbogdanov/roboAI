# RoboAI ROS2 Workspace

This workspace is the ROS2 migration scaffold for the RoboAI autonomy stack.

Target platform:

- Ubuntu 22.04
- ROS 2 Humble
- TurtleBot3 Gazebo simulation
- RViz2

This workspace is intentionally kept separate from the benchmark-only Python package in the repository root:

- ROS2 packages in `roboai_ros2_ws/src/`
- autonomy libraries remain in `src/roboai/`

The intended split is:

- ROS2 packages handle topics, parameters, launch, namespaces, and visualization
- `src/roboai` continues to own frontier scoring, planning, control, and evaluation logic

Expected bringup flow on Ubuntu:

```bash
cd roboai_ros2_ws
colcon build
source install/setup.bash
ros2 launch roboai_bringup sim_single.launch.py
```

Included packages:

- `roboai_msgs`: optional custom messages for frontiers and exploration status
- `roboai_mapping`: LaserScan/Odometry to OccupancyGrid node scaffold
- `roboai_frontier`: frontier extraction and scoring node scaffold
- `roboai_planning`: path planning node scaffold
- `roboai_control`: path follower to `/cmd_vel` node scaffold
- `roboai_manager`: high-level exploration manager scaffold
- `roboai_bringup`: launch files and parameter YAMLs
- `roboai_viz`: debug and RViz marker scaffold

Current status:

- package layout, launch files, and parameter files are present
- Python node modules are scaffolded and import the existing RoboAI autonomy libraries
- real ROS2 execution still requires Ubuntu 22.04 + ROS2 Humble + TurtleBot3 simulation
