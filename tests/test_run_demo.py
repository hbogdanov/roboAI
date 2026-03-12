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
