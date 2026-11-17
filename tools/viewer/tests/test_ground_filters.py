"""Tests for the grid-based morphological ground segmentation.

`grid_ground` is what stops vertical surfaces from entering the DEM: without
it, every wall/roof point lands in the grid and min-aggregation flattens it
into the floor (see `dem_min.png` in the analysis notes).
"""

from __future__ import annotations

import numpy as np
import pytest

from pc_snapshot.dem import DemAccumulator, import_dem_mapping
from pc_snapshot.config import Config
from tests.fixtures import make_scene_cloud
from pc_snapshot.decoder import decode_pointcloud2

assert import_dem_mapping() is True

from dem_mapping.ground_filters import (GridGroundFilter,  # noqa: E402
                                        available_filters, build_filter)


def _exposed_scene(seed: int = 0, n: int = 20000):
    """Ground plane at z=-1.5 plus a wall at x=6 spanning z=-1.5..2.5."""
    rng = np.random.RandomState(seed)
    ground = np.stack([
        rng.uniform(-15.0, 15.0, n),
        rng.uniform(-15.0, 15.0, n),
        np.full(n, -1.5),
    ], axis=1)
    wall = np.stack([
        np.full(n, 6.0),
        rng.uniform(-15.0, 15.0, n),
        rng.uniform(-1.5, 2.5, n),
    ], axis=1)
    return ground, wall


def test_grid_ground_is_registered():
    assert "grid_ground" in available_filters()
    assert isinstance(build_filter("grid_ground"), GridGroundFilter)


def test_wall_points_are_rejected_and_ground_kept():
    ground, wall = _exposed_scene()
    f = build_filter("grid_ground", cell=0.5, radius=2.0, height_tol=0.25,
                     max_slope=0.35, slope_radius=8.0)

    kept_ground = f(ground, {})
    kept_wall = f(wall, {})

    assert len(kept_ground) > 0.9 * len(ground)
    # only the wall's bottom 0.25 m may pass as "ground"
    assert len(kept_wall) < 0.10 * len(wall)
    assert kept_wall[:, 2].max() < -1.5 + 0.30
    assert np.all(np.abs(kept_ground[:, 2] + 1.5) <= 0.26)


def test_sloped_ground_is_preserved():
    rng = np.random.RandomState(1)
    n = 20000
    x = rng.uniform(-15.0, 15.0, n)
    y = rng.uniform(-15.0, 15.0, n)
    ramp = np.stack([x, y, 0.2 * x], axis=1)     # 20 % grade
    f = build_filter("grid_ground", cell=0.5, radius=2.0, height_tol=0.25,
                     max_slope=0.35, slope_radius=8.0)
    kept = f(ramp, {})
    assert len(kept) > 0.9 * n
    assert np.max(np.abs(kept[:, 2] - 0.2 * kept[:, 0])) < 0.26


def test_steep_slope_is_not_ground():
    rng = np.random.RandomState(2)
    n = 20000
    x = rng.uniform(0.0, 10.0, n)
    y = rng.uniform(-10.0, 10.0, n)
    cliff = np.stack([x, y, 2.0 * x], axis=1)    # 200 % grade
    f = build_filter("grid_ground", cell=0.5, radius=2.0, height_tol=0.25,
                     max_slope=0.35, slope_radius=8.0)
    kept = f(cliff, {})
    assert len(kept) < 0.5 * n


def test_empty_input_is_handled():
    f = build_filter("grid_ground")
    assert f(np.zeros((0, 3)), {}).shape == (0, 3)


def test_bad_parameters_raise():
    with pytest.raises(ValueError):
        GridGroundFilter(cell=0.0)
    with pytest.raises(ValueError):
        GridGroundFilter(radius=0.0)
    with pytest.raises(ValueError):
        GridGroundFilter(max_slope=0.0)
    with pytest.raises(ValueError):
        GridGroundFilter(radius=4.0, slope_radius=2.0)


def test_dem_defaults_to_grid_ground_and_drops_vertical_structure():
    cfg = Config()
    cfg.api_token = ""
    cfg.dem_extent = 40.0
    cfg.dem_resolution = 0.5
    cfg.dem_max_points_per_scan = 5000
    cfg.validate()
    assert cfg.dem_ground_filter == "grid_ground"

    acc = DemAccumulator(cfg)
    assert acc.enabled, acc.reason
    acc.add(decode_pointcloud2(make_scene_cloud(2500)).points, ros_stamp=1.0)
    cap = acc.capture()
    elev = cap.elevation[cap.valid]
    # the scene's wall reaches z=3.9 m: it must not show up in the DEM
    assert elev.max() < 0.5
    assert elev.min() > -0.5
