from roboai.core.frontier_model import FEATURE_KEYS, fit_linear_frontier_model, score_frontier


def test_fit_linear_frontier_model_returns_all_weights():
    samples = [
        {
            "bias": 1.0,
            "distance": 1.0,
            "heading_penalty": 0.2,
            "region_size": 3.0,
            "info_gain": 10.0,
            "semantic_value": 4.0,
            "blocked_penalty": 0.0,
            "revisit_penalty": 0.0,
            "uncertainty_penalty": 0.1,
            "beacon_bonus": 0.5,
        },
        {
            "bias": 1.0,
            "distance": 3.0,
            "heading_penalty": 0.4,
            "region_size": 1.0,
            "info_gain": 4.0,
            "semantic_value": 0.0,
            "blocked_penalty": 1.0,
            "revisit_penalty": 0.0,
            "uncertainty_penalty": 0.6,
            "beacon_bonus": 0.0,
        },
    ]
    targets = [5.0, -2.0]

    weights = fit_linear_frontier_model(samples, targets)

    assert set(weights) == set(FEATURE_KEYS)
    assert score_frontier(samples[0], weights) > score_frontier(samples[1], weights)
