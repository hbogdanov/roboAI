# Architecture

RoboAI is split into two layers:

- benchmark core in `src/roboai`
- deployment prototype in `roboai_ros2_ws`

The intended boundary is:

- core libraries own autonomy logic
- ROS2 packages own orchestration, topics, launch, and system integration

Core modules:

- `roboai.core.occupancy_grid`
- `roboai.core.frontier`
- `roboai.core.planners.*`
- `roboai.core.control.waypoint_follower`

ROS2 prototype flow:

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
