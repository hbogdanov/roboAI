from roboai.app.run_multi_demo import run_multi_demo


def test_multi_demo_runs_shared_map_exploration():
    metrics = run_multi_demo(
        map_name="empty",
        planner_name="hybrid",
        seed=1,
        max_steps=40,
        coverage_goal=0.2,
        write_artifacts=False,
    )

    assert metrics.robot_count == 2
    assert metrics.semantic_mode == "enabled"
    assert metrics.coverage >= 0.2
