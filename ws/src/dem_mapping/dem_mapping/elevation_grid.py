"""Incremental 2-D elevation grid.

The grid is updated *in place* from small batches of ground points; it is
never rebuilt from historical point clouds.  State per cell:

* running aggregate (mean sum, or minimum) of the observed elevations,
* observation count,
* validity derived from the observation count (count > 0 => valid).

Unknown cells stay ``NaN`` (NoData).

All updates are vectorised NumPy operations over the incoming points, so a
scan of ~20k points costs well under a millisecond on a laptop CPU.
"""

import numpy as np


class ElevationGrid:

    AGGREGATIONS = ('mean', 'min')

    def __init__(self, resolution=0.2, size_x=300.0, size_y=300.0,
                 origin_x=-150.0, origin_y=-150.0, aggregation='min'):
        if aggregation not in self.AGGREGATIONS:
            raise ValueError(f'aggregation must be one of {self.AGGREGATIONS}')
        if resolution <= 0:
            raise ValueError('resolution must be > 0')

        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.nx = max(1, int(round(float(size_x) / self.resolution)))
        self.ny = max(1, int(round(float(size_y) / self.resolution)))
        self.aggregation = aggregation

        n = self.nx * self.ny
        self._sum = np.zeros(n, dtype=np.float64)   # for mean aggregation
        self._min = np.full(n, np.inf, dtype=np.float64)
        self._count = np.zeros(n, dtype=np.int32)

        # statistics
        self.total_points = 0
        self.total_updates = 0
        self.updates_out_of_bounds = 0

    # ------------------------------------------------------------- geometry
    def cell_index(self, xy):
        """World XY (N,2) -> flat cell index, -1 where out of bounds."""
        ix = np.floor((xy[:, 0] - self.origin_x) / self.resolution).astype(np.int64)
        iy = np.floor((xy[:, 1] - self.origin_y) / self.resolution).astype(np.int64)
        inside = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        flat = np.where(inside, iy * self.nx + ix, -1)
        return flat, inside

    def cell_centers(self, valid_only=True):
        """Return (M,2) array of cell-centre world coordinates."""
        if valid_only:
            idx = np.flatnonzero(self._count > 0)
        else:
            idx = np.arange(self.nx * self.ny)
        ix = idx % self.nx
        iy = idx // self.nx
        return np.stack([self.origin_x + (ix + 0.5) * self.resolution,
                         self.origin_y + (iy + 0.5) * self.resolution],
                        axis=1)

    # --------------------------------------------------------------- update
    def update(self, xy, z):
        """Incrementally merge a batch of observations.

        Parameters
        ----------
        xy : (N,2) float array of world coordinates (metres)
        z  : (N,)   float array of elevations (metres)

        Returns
        -------
        dict with the number of accepted / rejected points.
        """
        xy = np.asarray(xy, dtype=np.float64)
        z = np.asarray(z, dtype=np.float64)
        if xy.size == 0:
            return {'accepted': 0, 'rejected': 0}
        flat, inside = self.cell_index(xy)
        self.updates_out_of_bounds += int((~inside).sum())
        if not inside.any():
            return {'accepted': 0, 'rejected': int(xy.shape[0])}

        f = flat[inside]
        v = z[inside]
        # restrict bincount work to the touched index span (a single scan
        # usually touches far fewer cells than the whole grid)
        lo = int(f.min())
        hi = int(f.max()) + 1
        span = hi - lo

        counts = np.bincount(f - lo, minlength=span)
        self._count[lo:hi] += counts.astype(np.int32)

        if self.aggregation == 'mean':
            sums = np.bincount(f - lo, weights=v, minlength=span)
            self._sum[lo:hi] += sums
        else:  # 'min'
            # np.minimum.at handles repeated indices correctly
            np.minimum.at(self._min, f, v)

        self.total_points += int(f.size)
        self.total_updates += 1
        return {'accepted': int(f.size), 'rejected': int(xy.shape[0] - f.size)}

    # --------------------------------------------------------------- access
    def valid_mask(self):
        """(ny,nx) boolean mask of cells with at least one observation."""
        return (self._count > 0).reshape(self.ny, self.nx)

    def elevation(self):
        """(ny,nx) float32 elevation, NaN where the cell is NoData."""
        out = np.full((self.ny, self.nx), np.nan, dtype=np.float32)
        valid = self._count > 0
        if self.aggregation == 'mean':
            vals = np.where(valid, self._sum / np.maximum(self._count, 1), 0.0)
        else:
            vals = np.where(valid, self._min, 0.0)
        out.reshape(-1)[valid] = vals[valid].astype(np.float32)
        return out

    def counts(self):
        return self._count.reshape(self.ny, self.nx).copy()

    def copy_state(self):
        """Raw copies of the flat arrays; caller must hold the grid lock.

        Copying is a fast memcpy (~10 ms for a 1500x1500 grid), so the
        lock is held briefly and the expensive evaluation can happen
        outside it in :meth:`evaluate`.
        """
        return (self._sum.copy(), self._min.copy(), self._count.copy())

    def evaluate(self, state):
        """Pure evaluation of copied raw arrays -> (elev, valid, stats).

        ``state`` is the tuple returned by :meth:`copy_state`. No locking
        and no access to live grid state: safe to run concurrently with
        incoming updates.
        """
        sum_, min_, count = state
        valid_flat = count > 0
        n_valid = int(valid_flat.sum())
        elev = np.full(count.size, np.nan, dtype=np.float32)
        if n_valid:
            if self.aggregation == 'mean':
                vals = sum_[valid_flat] / count[valid_flat]
            else:
                vals = min_[valid_flat]
            elev[valid_flat] = vals.astype(np.float32)
        elev = elev.reshape(self.ny, self.nx)
        valid = valid_flat.reshape(self.ny, self.nx)

        z_min = z_max = None
        if n_valid:
            z_min = float(vals.min())
            z_max = float(vals.max())
        stats = {
            'shape': [self.ny, self.nx],
            'resolution_m': self.resolution,
            'origin': [self.origin_x, self.origin_y],
            'aggregation': self.aggregation,
            'valid_cells': n_valid,
            'coverage_m2': round(n_valid * self.resolution ** 2, 2),
            'observations': int(self.total_points),
            'updates': int(self.total_updates),
            'out_of_bounds_points': int(self.updates_out_of_bounds),
            'z_min': z_min,
            'z_max': z_max,
        }
        return elev, valid, stats

    def snapshot(self):
        """One consistent (elevation, valid mask, stats) view of the grid.

        Convenience wrapper for tests/tools: copies state and evaluates
        it. In the live pipeline the node copies under its own lock and
        evaluates outside of it.
        """
        return self.evaluate(self.copy_state())

    def stats(self):
        valid = self._count > 0          # flat mask over the flat arrays
        n_valid = int(valid.sum())
        elev = None
        z_min = z_max = None
        if n_valid:
            vals = (self._sum[valid] / self._count[valid]
                    if self.aggregation == 'mean' else self._min[valid])
            z_min = float(vals.min())
            z_max = float(vals.max())
        return {
            'shape': [self.ny, self.nx],
            'resolution_m': self.resolution,
            'origin': [self.origin_x, self.origin_y],
            'aggregation': self.aggregation,
            'valid_cells': n_valid,
            'coverage_m2': round(n_valid * self.resolution ** 2, 2),
            'observations': int(self.total_points),
            'updates': int(self.total_updates),
            'out_of_bounds_points': int(self.updates_out_of_bounds),
            'z_min': z_min,
            'z_max': z_max,
        }

    def reset(self):
        self._sum[:] = 0.0
        self._min[:] = np.inf
        self._count[:] = 0
        self.total_points = 0
        self.total_updates = 0
        self.updates_out_of_bounds = 0

    def save_npz(self, path):
        np.savez_compressed(
            path,
            elevation=self.elevation(),
            counts=self.counts(),
            valid=self.valid_mask().reshape(self.ny, self.nx),
            resolution=self.resolution,
            origin=np.array([self.origin_x, self.origin_y]),
            aggregation=np.array(self.aggregation))
        return path
