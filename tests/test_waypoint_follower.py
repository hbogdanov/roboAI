from roboai.core.control.waypoint_follower import WaypointFollower
from roboai.core.types import Pose2D


def test_waypoint_follower_advances_after_reaching_target():
    follower = WaypointFollower(waypoint_tolerance=0.2)
    follower.set_path([(0.5, 0.0), (1.0, 0.0)])
    linear, angular = follower.command(Pose2D(x=0.5, y=0.0, theta=0.0))
    assert linear > 0.0
    assert abs(angular) < 1.0
    assert follower.index == 1
