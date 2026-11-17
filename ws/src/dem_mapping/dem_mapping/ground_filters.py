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

from __future__ import division

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


def _rank_filter(a, k, op):
    """Separable (2k+1)x(2k+1) min/max filter over a 2-D array.

    Padded with +inf (min) / -inf (max) so the borders simply see fewer
    samples.  Separable means 2*(2k+1) shifted reductions instead of
    (2k+1)^2, which keeps a 60 m scan well under a millisecond.
    """
    if k <= 0:
        return a.copy()
    fill = np.inf if op is np.minimum else -np.inf
    out = a
    for axis in (0, 1):
        pads = [(0, 0), (0, 0)]
        pads[axis] = (k, k)
        padded = np.pad(out, pads, mode='constant', constant_values=fill)
        n = out.shape[axis]
        acc = np.full(out.shape, fill, dtype=np.float64)
        for d in range(2 * k + 1):
            sl = [slice(None), slice(None)]
            sl[axis] = slice(d, d + n)
            acc = op(acc, padded[tuple(sl)])
        out = acc
    return out


class GridGroundFilter(GroundFilter):
    """Grid-based morphological ground segmentation.

    This is the filter that makes a DEM actually a *terrain* model: the other
    filters only gate elevation (they keep walls, roofs and foliage), so every
    vertical surface ends up in the grid and, with min aggregation, gets
    flattened into the floor level.

    One scan at a time (no cross-scan state, so it works in a moving map
    frame as well as in the sensor frame):

    1. rasterise the scan's *lowest* z on a local ``cell``-metre grid,
    2. morphologically open that surface (erode then dilate with a
       ``radius``-metre kernel): everything narrower than the kernel --
       walls, poles, trunks, cars, people -- is pulled down to the local
       ground level, while the ground itself keeps its shape,
    3. reject cells whose opened surface sits more than ``max_slope`` above
       the lowest surface within ``slope_radius`` (buildings and roofs that
       are wider than the kernel survive the opening but not this test),
    4. keep the points within ``-height_tol .. +height_tol`` of the surface.

    Points with no surface estimate (isolated returns, no neighbours) are
    dropped.  Works in any frame whose third axis is roughly 'up'.
    """

    name = 'grid_ground'

    #: hard cap on raster cells; the cell size is coarsened beyond it
    MAX_CELLS = 1_000_000

    def __init__(self, cell=0.5, radius=2.0, height_tol=0.25,
                 max_slope=0.35, slope_radius=8.0):
        self.cell = float(cell)
        self.radius = float(radius)
        self.height_tol = float(height_tol)
        self.max_slope = float(max_slope)
        self.slope_radius = float(slope_radius)
        if self.cell <= 0:
            raise ValueError('cell must be > 0')
        if self.radius <= 0:
            raise ValueError('radius must be > 0')
        if self.height_tol < 0:
            raise ValueError('height_tol must be >= 0')
        if self.max_slope <= 0:
            raise ValueError('max_slope must be > 0')
        if self.slope_radius < self.radius:
            raise ValueError('slope_radius must be >= radius')

    # ------------------------------------------------------------- internals
    def _surface(self, points):
        """(N,) ground elevation under each point, NaN where unknown."""
        xy = points[:, :2]
        z = points[:, 2]
        cell = self.cell
        lo = xy.min(axis=0)
        span = xy.max(axis=0) - lo
        nx = int(np.floor(span[0] / cell)) + 1
        ny = int(np.floor(span[1] / cell)) + 1
        if nx * ny > self.MAX_CELLS:
            cell *= np.ceil(np.sqrt(nx * ny / float(self.MAX_CELLS)))
            nx = int(np.floor(span[0] / cell)) + 1
            ny = int(np.floor(span[1] / cell)) + 1

        ix = np.floor((xy[:, 0] - lo[0]) / cell).astype(np.int64)
        iy = np.floor((xy[:, 1] - lo[1]) / cell).astype(np.int64)

        # Per-cell minimum z.  np.minimum.at is ~1 us per point (29 ms for a
        # 20k scan); reduceat over a sorted index is an order of magnitude
        # cheaper and keeps the live DEM path off the streaming thread's
        # critical path.
        flat = iy * nx + ix
        z = z.astype(np.float64)
        order = np.argsort(flat, kind='stable')
        f_sorted = flat[order]
        z_sorted = z[order]
        is_new = np.empty(f_sorted.shape, dtype=bool)
        is_new[0] = True
        np.not_equal(f_sorted[1:], f_sorted[:-1], out=is_new[1:])
        starts = np.flatnonzero(is_new)
        minz = np.full(nx * ny, np.inf, dtype=np.float64)
        minz[f_sorted[starts]] = np.minimum.reduceat(z_sorted, starts)
        minz = minz.reshape(ny, nx)

        k = max(1, int(round(self.radius / cell)))
        ks = max(k, int(round(self.slope_radius / cell)))

        eroded = _rank_filter(minz, k, np.minimum)          # pull objects down
        # unobserved cells must not win the dilation below
        eroded = np.where(np.isfinite(eroded), eroded, -np.inf)
        opened = _rank_filter(eroded, k, np.maximum)        # restore plateaus

        finite = np.isfinite(opened)
        lows = self._neighbourhood_low(opened, cell, ks)
        slope_ok = finite & (opened - lows <= self.max_slope
                             * self.slope_radius + 1e-6)
        surface = np.where(slope_ok, opened, np.nan)
        return surface[iy, ix]

    @staticmethod
    def _neighbourhood_low(opened, cell, ks):
        """Lowest opened-surface value within ``ks`` cells of each cell.

        The slope test only needs a smooth 8 m-scale minimum, so it runs on a
        ~2 m block-minified copy of the raster and is upsampled again: the
        same answer for a fraction of the shifted reductions.
        """
        ny, nx = opened.shape
        factor = max(1, int(round(2.0 / cell)))
        if factor == 1 or ny < 2 * factor or nx < 2 * factor:
            return _rank_filter(np.where(np.isfinite(opened), opened, np.inf),
                                ks, np.minimum)
        ny2, nx2 = ny // factor, nx // factor
        block = np.where(np.isfinite(opened[:ny2 * factor, :nx2 * factor]),
                         opened[:ny2 * factor, :nx2 * factor], np.inf)
        coarse = block.reshape(ny2, factor, nx2, factor).min(axis=(1, 3))
        k2 = max(1, int(round(ks / float(factor))))
        low = _rank_filter(coarse, k2, np.minimum)
        low = np.repeat(np.repeat(low, factor, axis=0), factor, axis=1)
        # odd margins (grid not a multiple of ``factor``) repeat the edge value
        rows = np.minimum(np.arange(ny), low.shape[0] - 1)
        cols = np.minimum(np.arange(nx), low.shape[1] - 1)
        return low[rows][:, cols]

    def mask(self, points, ctx):
        surface = self._surface(points)
        ok = np.isfinite(surface)
        ok[ok] = np.abs(points[ok, 2] - surface[ok]) <= self.height_tol
        return ok


class MaxPointsFilter(GroundFilter):
    """Random subsample - a CPU budget guard, not a ground classifier."""

    name = 'max_points'

    def __init__(self, max_points=8000, seed=0):
        self.max_points = int(max_points)
        self._rng = np.random.RandomState(seed)

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
    GridGroundFilter.name: GridGroundFilter,
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
    accepted = set(inspect.getargspec(cls.__init__).args) - {'self'}
    kwargs = {k: v for k, v in params.items() if k in accepted}
    return cls(**kwargs)
