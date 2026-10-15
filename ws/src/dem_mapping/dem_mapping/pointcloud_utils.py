"""PointCloud2 -> NumPy helpers (pure Python/NumPy, no PCL/Open3D)."""

import numpy as np

# sensor_msgs/PointField datatypes
_INT8, _UINT8, _INT16, _UINT16 = 1, 2, 3, 4
_INT32, _UINT32, _FLOAT32, _FLOAT64 = 5, 6, 7, 8

_DTYPE = {
    _INT8: np.int8,
    _UINT8: np.uint8,
    _INT16: np.int16,
    _UINT16: np.uint16,
    _INT32: np.int32,
    _UINT32: np.uint32,
    _FLOAT32: np.float32,
    _FLOAT64: np.float64,
}


def _field_column(data, field, width, point_step):
    """Extract one PointField column as a float64 array of shape (width,)."""
    dt = np.dtype(_DTYPE[field.datatype])
    raw = np.frombuffer(data, dtype=np.uint8)
    raw = raw.reshape(width, point_step)
    if dt.itemsize == 1:
        col = raw[:, field.offset:field.offset + 1].copy().reshape(width)
        return col.astype(np.float64)
    # little-endian aligned view
    seg = np.ascontiguousarray(raw[:, field.offset:field.offset + dt.itemsize])
    return seg.view(dt).reshape(width).astype(np.float64)


def pointcloud2_to_xyz(msg):
    """Return (N,3) float64 XYZ extracted from a sensor_msgs/PointCloud2."""
    fields = {f.name: f for f in msg.fields}
    for need in ('x', 'y', 'z'):
        if need not in fields:
            raise KeyError(f'PointCloud2 has no field "{need}"')
    n = msg.height * msg.width
    if n == 0:
        return np.zeros((0, 3), dtype=np.float64)
    x = _field_column(bytes(msg.data), fields['x'], n, msg.point_step)
    y = _field_column(bytes(msg.data), fields['y'], n, msg.point_step)
    z = _field_column(bytes(msg.data), fields['z'], n, msg.point_step)
    return np.stack([x, y, z], axis=1)


def pointcloud2_field(msg, name):
    """Return one named field as float64 (N,), or None if absent."""
    fields = {f.name: f for f in msg.fields}
    if name not in fields:
        return None
    n = msg.height * msg.width
    if n == 0:
        return np.zeros(0, dtype=np.float64)
    return _field_column(bytes(msg.data), fields[name], n, msg.point_step)


def pointcloud2_point_times(msg):
    """Per-point time offsets in seconds relative to ``msg.header.stamp``.

    Supported encodings (checked in this order):

    * ``timestamp`` - absolute time, float64, seconds or nanoseconds
      (Livox driver PointCloud2 format),
    * ``t``         - uint32 nanoseconds since header (Ouster),
    * ``time``      - float32/float64 seconds relative to header (Velodyne).

    Returns ``None`` when the message carries no usable per-point time,
    in which case the whole scan is assumed to be stamped at header time.
    """
    fields = {f.name: f for f in msg.fields}
    hdr_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    if 'timestamp' in fields:
        ts = pointcloud2_field(msg, 'timestamp')
        if ts is None or ts.size == 0:
            return None
        med = float(np.median(ts))
        if med > 1e17:            # absolute nanoseconds
            return (ts - hdr_ns) * 1e-9
        if med > 1e14:            # absolute microseconds
            return (ts - hdr_ns * 1e-3) * 1e-6
        if med > 1e11:            # absolute milliseconds
            return (ts - hdr_ns * 1e-6) * 1e-3
        if med > 1e9:             # absolute seconds (float)
            return ts - hdr_ns * 1e-9
        return ts                 # already relative seconds

    if 't' in fields:
        t = pointcloud2_field(msg, 't')
        if t is None:
            return None
        return t * 1e-9

    if 'time' in fields:
        t = pointcloud2_field(msg, 'time')
        if t is None:
            return None
        return t

    return None
