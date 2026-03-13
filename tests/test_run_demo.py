from roboai.app.run_demo import run_demo


def test_minimal_demo_reaches_coverage_goal_on_empty_map():
    metrics = run_demo(
        map_name="empty",
        planner_name="astar",
        seed=1,
        max_steps=120,
        coverage_goal=0.80,
        write_artifacts=False,
    )
    assert metrics.success is True
    assert metrics.collisions == 0
    assert metrics.coverage >= 0.80
    assert metrics.stop_reason == "coverage_goal_reached"


def test_demo_supports_hybrid_policy_and_noise_metrics():
    metrics = run_demo(
        map_name="empty",
        planner_name="hybrid",
        frontier_policy="information_gain",
        disturbance_name="moving_obstacle",
        range_noise_std=0.01,
        dropout_prob=0.05,
        pose_noise_std=0.01,
        seed=1,
        max_steps=40,
        coverage_goal=0.20,
        write_artifacts=False,
    )
    assert metrics.frontier_policy == "information_gain"
    assert metrics.planner_policy == "fallback_rrt"
    assert metrics.disturbance_name == "moving_obstacle"
    assert metrics.range_noise_std == 0.01
    assert metrics.dropout_prob == 0.05
    assert metrics.pose_noise_std == 0.01
    assert metrics.replan_triggers >= metrics.replans
