"""Time-indexed pose buffer with interpolation (NumPy only).

Used to transform incoming LiDAR scans into a consistent map frame using
whatever pose stream is available (FAST-LIO2 odometry, GNSS/INS pose, ...).
Position is linearly interpolated, orientation is spherically interpolated
(quaternion slerp) between the two bracketing samples.
"""

import numpy as np


# ------------------------------------------------------------------ quaternions
def quat_normalize(q):
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    n = np.where(n < 1e-12, 1.0, n)
    return q / n


def quat_to_rot(q):
    """(...,4) xyzw quaternion -> (...,3,3) rotation matrices."""
    q = quat_normalize(q)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    r00 = 1 - 2 * (yy + zz)
    r01 = 2 * (xy - wz)
    r02 = 2 * (xz + wy)
    r10 = 2 * (xy + wz)
    r11 = 1 - 2 * (xx + zz)
    r12 = 2 * (yz - wx)
    r20 = 2 * (xz - wy)
    r21 = 2 * (yz + wx)
    r22 = 1 - 2 * (xx + yy)
    row0 = np.stack([r00, r01, r02], axis=-1)
    row1 = np.stack([r10, r11, r12], axis=-1)
    row2 = np.stack([r20, r21, r22], axis=-1)
    return np.stack([row0, row1, row2], axis=-2)


def quat_slerp(q0, q1, s):
    """Vectorised slerp between (...,4) quaternions and (...,) factors."""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    dot = np.sum(q0 * q1, axis=-1)
    flip = dot < 0.0
    q1 = np.where(flip[..., None], -q1, q1)
    dot = np.abs(dot)
    dot = np.clip(dot, -1.0, 1.0)

    theta = np.arccos(dot)
    sin_theta = np.sin(theta)
    # near-parallel: fall back to linear interpolation
    small = sin_theta < 1e-6
    w0 = np.where(small, 1.0 - s, np.sin((1.0 - s) * theta) /
                  np.where(small, 1.0, sin_theta))
    w1 = np.where(small, s, np.sin(s * theta) /
                  np.where(small, 1.0, sin_theta))
    out = q0 * w0[..., None] + q1 * w1[..., None]
    return quat_normalize(out)


class PoseBuffer:
    """Bounded, time-sorted buffer of (t, position, orientation)."""

    def __init__(self, max_span_s=10.0, max_size=200_000):
        self.max_span_s = float(max_span_s)
        self.max_size = int(max_size)
        self._t = np.zeros(0, dtype=np.float64)      # seconds
        self._p = np.zeros((0, 3), dtype=np.float64)
        self._q = np.zeros((0, 4), dtype=np.float64)  # xyzw

    def __len__(self):
        return self._t.size

    def clear(self):
        self._t = np.zeros(0, dtype=np.float64)
        self._p = np.zeros((0, 3), dtype=np.float64)
        self._q = np.zeros((0, 4), dtype=np.float64)

    def add(self, t, p, q):
        """Append one pose (seconds, (3,), xyzw). Out-of-order drops."""
        if self._t.size and t <= self._t[-1]:
            return False
        self._t = np.append(self._t, float(t))
        self._p = np.append(self._p, np.asarray(p, dtype=np.float64)[None, :],
                            axis=0)
        q = np.asarray(q, dtype=np.float64).reshape(4)
        self._q = np.append(self._q, quat_normalize(q)[None, :], axis=0)
        self._trim()
        return True

    def _trim(self):
        if self._t.size <= 1:
            return
        if self._t.size > self.max_size:
            cut = self._t.size - self.max_size
            self._t = self._t[cut:]
            self._p = self._p[cut:]
            self._q = self._q[cut:]
            return
        newest = self._t[-1]
        if newest - self._t[0] > self.max_span_s:
            keep_from = np.searchsorted(self._t, newest - self.max_span_s)
            keep_from = max(0, keep_from - 1)     # keep one older sample
            if keep_from > 0:
                self._t = self._t[keep_from:]
                self._p = self._p[keep_from:]
                self._q = self._q[keep_from:]

    # ------------------------------------------------------------- queries
    def _bracket(self, t):
        """Indices (i, j) and interpolation factor s for time t, or None."""
        if self._t.size == 0:
            return None
        j = int(np.searchsorted(self._t, t))
        if j <= 0:
            i = j
            j = min(i + 1, self._t.size - 1)
        elif j >= self._t.size:
            i = self._t.size - 1
            j = i
        else:
            i = j - 1
        dt = self._t[j] - self._t[i]
        s = 0.0 if dt <= 0 else float(np.clip((t - self._t[i]) / dt, 0.0, 1.0))
        return i, j, s

    def pose_at(self, t, max_age=0.5):
        """Return (R 3x3, p 3) at time t, or None when too far from data."""
        if self._t.size == 0:
            return None
        if abs(t - self._t[0]) > max_age and abs(t - self._t[-1]) > max_age \
                and not (self._t[0] <= t <= self._t[-1]):
            return None
        if t < self._t[0] - max_age or t > self._t[-1] + max_age:
            return None
        b = self._bracket(t)
        if b is None:
            return None
        i, j, s = b
        p = (1.0 - s) * self._p[i] + s * self._p[j]
        q = quat_slerp(self._q[i], self._q[j], np.float64(s))
        return quat_to_rot(q), p

    def transform_points(self, t_pts, pts, max_age=0.5):
        """Transform (N,3) points by the pose valid at each point time.

        ``t_pts`` is (N,) seconds.  Points whose timestamp lies outside the
        buffer (plus ``max_age``) fall back to the nearest available pose.

        Returns (transformed (N,3), n_used_exactly, n_fallback).
        """
        pts = np.asarray(pts, dtype=np.float64)
        n = pts.shape[0]
        if n == 0:
            return pts.copy(), 0, 0
        if self._t.size == 0:
            raise RuntimeError('pose buffer is empty')

        # bracketing sample for every point time
        j = np.searchsorted(self._t, t_pts)
        exact = (j < self._t.size) & (self._t[np.clip(j, 0, self._t.size - 1)]
                                      == t_pts)
        i = np.clip(j - 1, 0, self._t.size - 1)
        jj = np.clip(j, 0, self._t.size - 1)
        dt = self._t[jj] - self._t[i]
        s = np.where(dt > 0, (t_pts - self._t[i]) / np.where(dt > 0, dt, 1.0),
                     0.0)
        s = np.clip(s, 0.0, 1.0)
        # out-of-range points clamp to the ends of the buffer
        fallback = (t_pts < self._t[0] - max_age) | \
                   (t_pts > self._t[-1] + max_age)
        s = np.where(fallback, 0.0, s)
        i = np.where(fallback, np.where(t_pts < self._t[0], 0, self._t.size - 1),
                     i)
        jj = i

        p = (1.0 - s)[:, None] * self._p[i] + s[:, None] * self._p[jj]
        q = quat_slerp(self._q[i], self._q[jj], s)
        R = quat_to_rot(q)                       # (N,3,3)
        out = np.einsum('nij,nj->ni', R, pts)
        out += p
        n_exact = int((~fallback & exact).sum())
        return out, n_exact, int(fallback.sum())

    def trajectory(self):
        return self._t.copy(), self._p.copy()
