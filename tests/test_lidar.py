import numpy as np

from roboai.core.types import Pose2D
from roboai.sim.grid2d.env import Grid2DEnv
from roboai.sim.grid2d.lidar import SimulatedLidar
from roboai.sim.grid2d.maps import make_empty


def test_lidar_dropout_sets_ranges_to_max():
    np.random.seed(0)
    env = Grid2DEnv(obstacle_grid=make_empty(20, 20), resolution=0.2, robot_radius=0.1)
    env.reset(Pose2D(x=1.0, y=1.0, theta=0.0))
    lidar = SimulatedLidar(num_beams=32, max_range=4.0, dropout_prob=1.0)

    scan = lidar.scan(env, env.pose)

    assert np.allclose(scan.ranges, 4.0)
