from roboai.sim.grid2d.maps import built_in_map


def test_all_built_in_maps_load():
    for name in ["empty", "office", "cluttered", "narrow", "maze"]:
        grid = built_in_map(name)
        assert grid.ndim == 2
        assert grid.shape[0] > 10
        assert grid.shape[1] > 10
