"""Modular ground-surface filtering.

The DEM algorithm itself only needs *ground* points; how a point is judged
to be ground is deliberately pluggable so it can later be replaced by a
proper classifier (plane fitting, cloth simulation, a learned model, ...)
without touching the elevation grid code.

All filters operate on an (N,3) array of points already expressed in the
map frame and return a boolean mask of the points to keep.

Context dict keys (optional, depends on the filter):
    pose_z   : sensor/vehicle altitude in the map frame (m)
    stamp    : scan timestamp (s)
    pose_xyz : sensor position (3,)
"""

import numpy as np


class GroundFilter:
    name = 'none'

    def mask(self, points, ctx):
        raise NotImplementedError

    def __call__(self, points, ctx=None):
        ctx = ctx or {}
        if points.shape[0] == 0:
            return points
        m = self.mask(points, ctx)
        return points[m]


class NoFilter(GroundFilter):
    """Keep everything (algorithm testing only - NOT ground classification)."""

    name = 'none'

    def mask(self, points, ctx):
        return np.ones(points.shape[0], dtype=bool)


class ZRangeFilter(GroundFilter):
    """Absolute elevation window in the map frame."""

    name = 'z_range'

    def __init__(self, z_min=-np.inf, z_max=np.inf):
        self.z_min = float(z_min)
        self.z_max = float(z_max)

    def mask(self, points, ctx):
        z = points[:, 2]
        return (z >= self.z_min) & (z <= self.z_max)


class HeightBandFilter(GroundFilter):
    """Keep points in a vertical band around the sensor/vehicle altitude.

    Points much higher than the vehicle (buildings, canopy, wires) and
    points far below it (pits, map drift, returns from voids) are dropped.
    """

    name = 'height_band'

    def __init__(self, below=4.0, above=3.0):
        self.below = float(below)
        self.above = float(above)

    def mask(self, points, ctx):
        pose_z = ctx.get('pose_z')
        if pose_z is None:
            return np.ones(points.shape[0], dtype=bool)
        z = points[:, 2]
        return (z >= pose_z - self.below) & (z <= pose_z + self.above)


class RadialRangeFilter(GroundFilter):
    """Keep points within a range annulus measured from the sensor origin."""

    name = 'radial_range'

    def __init__(self, min_range=0.5, max_range=60.0):
        self.min_range = float(min_range)
        self.max_range = float(max_range)

    def mask(self, points, ctx):
        origin = ctx.get('pose_xyz')
        if origin is None:
            # sensor position unknown (e.g. registered clouds without a pose
            # stream): do not gate by range
            return np.ones(points.shape[0], dtype=bool)
        p = points - np.asarray(origin)
        r = np.linalg.norm(p, axis=1)
        return (r >= self.min_range) & (r <= self.max_range)


class MaxPointsFilter(GroundFilter):
    """Random subsample - a CPU budget guard, not a ground classifier."""

    name = 'max_points'

    def __init__(self, max_points=8000, seed=0):
        self.max_points = int(max_points)
        self._rng = np.random.default_rng(seed)

    def __call__(self, points, ctx=None):
        if points.shape[0] <= self.max_points:
            return points
        idx = self._rng.choice(points.shape[0], size=self.max_points,
                               replace=False)
        idx.sort()
        return points[idx]


_FILTERS = {
    NoFilter.name: NoFilter,
    ZRangeFilter.name: ZRangeFilter,
    HeightBandFilter.name: HeightBandFilter,
    RadialRangeFilter.name: RadialRangeFilter,
    MaxPointsFilter.name: MaxPointsFilter,
}


def available_filters():
    return sorted(_FILTERS)


def build_filter(name, **params):
    """Instantiate a filter by name.

    Unknown parameters are ignored so that a YAML can carry the parameters of
    several filters at once.
    """
    import inspect
    cls = _FILTERS[name]
    accepted = set(inspect.signature(cls.__init__).parameters) - {'self'}
    kwargs = {k: v for k, v in params.items() if k in accepted}
    return cls(**kwargs)
