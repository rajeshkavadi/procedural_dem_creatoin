"""Voxelisation compatible with OpenPCDet / spconv's VoxelGenerator.

spconv returns coordinates as ``(batch, z, y, x)``; that ordering is what
``PointPillarScatter`` and ``PillarVFE`` expect, so it is preserved here.
"""

import numpy as np


def voxelize(points, point_cloud_range, voxel_size, grid_size,
             max_points_per_voxel=32, max_voxels=100000, rng_seed=None):
    """Convert an ``N x (3+C)`` point array into pillars.

    Args:
        points: float32 array ``[N, 4]`` (x, y, z, intensity).
        point_cloud_range: ``[x0, y0, z0, x1, y1, z1]``.
        voxel_size: ``[vx, vy, vz]``.
        grid_size: ``(GX, GY, GZ)`` ints.
        max_points_per_voxel: points kept per pillar (spconv default 32).
        max_voxels: maximum number of pillars. When exceeded, pillars are
            randomly subsampled (seeded, so runs are reproducible).
        rng_seed: optional seed for the subsampling.

    Returns:
        voxels: ``[M, T, C]`` float32
        num_points: ``[M]`` int32
        coords: ``[M, 4]`` int32 ``(batch=0, z, y, x)``
    """
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError("points must be an N x (>=3) array, got shape %r" % (points.shape,))
    if points.shape[1] == 3:
        points = np.concatenate(
            [points, np.zeros((points.shape[0], 1), dtype=np.float32)], axis=1)

    gx, gy, gz = [int(v) for v in grid_size]
    pc_min = np.asarray(point_cloud_range[:3], dtype=np.float32)
    vox = np.asarray(voxel_size, dtype=np.float32)

    if points.shape[0] == 0:
        return (_empty(points.shape[1], max_points_per_voxel), np.zeros(0, np.int32),
                np.zeros((0, 4), np.int32))

    coords = np.floor((points[:, :3] - pc_min[None, :]) / vox[None, :]).astype(np.int32)
    valid = ((coords[:, 0] >= 0) & (coords[:, 0] < gx) &
             (coords[:, 1] >= 0) & (coords[:, 1] < gy) &
             (coords[:, 2] >= 0) & (coords[:, 2] < gz))
    if not valid.any():
        return (_empty(points.shape[1], max_points_per_voxel), np.zeros(0, np.int32),
                np.zeros((0, 4), np.int32))

    coords = coords[valid]
    points = points[valid]

    # Stable sort: within a pillar the original point order is preserved, so
    # truncation to max_points_per_voxel keeps the same points as spconv.
    flat = (coords[:, 2].astype(np.int64) * gy + coords[:, 1].astype(np.int64)) * gx \
        + coords[:, 0].astype(np.int64)
    order = np.argsort(flat, kind="stable")
    sorted_flat = flat[order]
    sorted_points = points[order]

    starts = np.flatnonzero(np.r_[True, sorted_flat[1:] != sorted_flat[:-1]])
    counts = np.diff(np.r_[starts, sorted_flat.shape[0]])
    num_groups = starts.shape[0]

    if num_groups > max_voxels:
        rng = np.random.RandomState(0 if rng_seed is None else int(rng_seed))
        chosen = rng.choice(num_groups, size=max_voxels, replace=False)
        chosen.sort()
    else:
        chosen = np.arange(num_groups)

    group_starts = starts[chosen]
    group_counts = counts[chosen]
    num_out = chosen.shape[0]

    # rank of each point inside its pillar
    group_ids = np.repeat(np.arange(num_groups), counts)
    ranks = np.arange(sorted_points.shape[0]) - np.repeat(starts, counts)
    if num_groups > max_voxels:
        keep = np.isin(group_ids, chosen) & (ranks < int(max_points_per_voxel))
    else:
        keep = ranks < int(max_points_per_voxel)

    group_to_row = np.full(num_groups, -1, dtype=np.int64)
    group_to_row[chosen] = np.arange(num_out)

    rows = group_to_row[group_ids[keep]]
    point_ranks = ranks[keep]

    num_features = sorted_points.shape[1]
    voxels = np.zeros((num_out, int(max_points_per_voxel), num_features), dtype=np.float32)
    voxels[rows, point_ranks] = sorted_points[keep]

    num_points = np.bincount(rows, minlength=num_out).astype(np.int32)

    group_coords = coords[order[group_starts]]  # (x, y, z) of each pillar
    out_coords = np.zeros((num_out, 4), dtype=np.int32)
    out_coords[:, 0] = 0
    out_coords[:, 1] = group_coords[:, 2]
    out_coords[:, 2] = group_coords[:, 1]
    out_coords[:, 3] = group_coords[:, 0]

    return voxels, num_points, out_coords


def _empty(num_features, max_points_per_voxel):
    return np.zeros((0, int(max_points_per_voxel), int(num_features)), dtype=np.float32)
