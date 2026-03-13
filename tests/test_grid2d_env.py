from roboai.core.types import Pose2D
from roboai.sim.grid2d.env import Grid2DEnv
from roboai.sim.grid2d.maps import make_empty


def test_env_step_moves_robot_without_collision():
    env = Grid2DEnv(obstacle_grid=make_empty(20, 20), resolution=0.2, robot_radius=0.1)
    env.reset(Pose2D(x=1.0, y=1.0, theta=0.0))
    pose, collision = env.step(linear=0.5, angular=0.0, dt=1.0)
    assert collision is False
    assert pose.x > 1.0


def test_temporary_block_disturbance_activates():
    env = Grid2DEnv(
        obstacle_grid=make_empty(24, 24),
        resolution=0.2,
        robot_radius=0.1,
        disturbance_name="temporary_block",
    )
    env.reset(Pose2D(x=1.0, y=1.0, theta=0.0))
    for _ in range(35):
        env.step(linear=0.0, angular=0.0, dt=0.2)
    assert env.disturbance_events >= 1
    assert env.dynamic_obstacle_grid.any()
