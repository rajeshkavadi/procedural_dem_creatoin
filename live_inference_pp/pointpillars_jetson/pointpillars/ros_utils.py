"""ROS helpers: PointCloud2 -> numpy, box markers, cloud publishing.

ROS imports are local so the inference pipeline can also run completely
headless (bag mode without roscore).
"""

import numpy as np

_POINT_FIELD_DTYPES = {
    1: "i1", 2: "u1", 3: "i2", 4: "u2",
    5: "i4", 6: "u4", 7: "f4", 8: "f8",
}

_INTENSITY_NAMES = ("intensity", "i", "reflectivity", "reflectance", "intensities")


def pointcloud2_to_xyz_intensity(msg):
    """Convert a sensor_msgs/PointCloud2 to an ``N x 4`` float32 array."""
    fields = list(msg.fields or [])
    by_name = {}
    for f in fields:
        if f.name not in by_name:
            by_name[f.name] = f
    if "x" not in by_name or "y" not in by_name or "z" not in by_name:
        raise ValueError("PointCloud2 has no x/y/z fields (fields: %s)"
                         % [f.name for f in fields])

    intensity_name = None
    for name in _INTENSITY_NAMES:
        if name in by_name:
            intensity_name = name
            break

    use_names = ["x", "y", "z"] + ([intensity_name] if intensity_name else [])
    endian = ">" if msg.is_bigendian else "<"
    names, formats, offsets = [], [], []
    for name in use_names:
        f = by_name[name]
        base = _POINT_FIELD_DTYPES.get(int(f.datatype))
        if base is None:
            raise ValueError("unsupported PointField datatype %s for '%s'"
                             % (f.datatype, name))
        names.append(name)
        formats.append(base if int(f.count) <= 1 else (base, int(f.count)))
        offsets.append(int(f.offset))

    dtype = np.dtype({
        "names": names,
        "formats": [endian + fmt if isinstance(fmt, str) else (endian + fmt[0], fmt[1])
                    for fmt in formats],
        "offsets": offsets,
        "itemsize": int(msg.point_step),
    })

    width, height = int(msg.width), int(msg.height)
    point_step, row_step = int(msg.point_step), int(msg.row_step)
    raw = np.frombuffer(msg.data, dtype=np.uint8)

    if height > 1 and row_step != width * point_step:
        rows = raw[:height * row_step].reshape(height, row_step)
        raw = rows[:, :width * point_step].copy().reshape(-1)
    elif height > 1:
        raw = raw[:height * row_step]

    arr = np.frombuffer(raw.tobytes(), dtype=dtype, count=width * height)

    out = np.empty((arr.shape[0], 4), dtype=np.float32)
    out[:, 0] = arr["x"]
    out[:, 1] = arr["y"]
    out[:, 2] = arr["z"]
    if intensity_name:
        val = arr[intensity_name]
        if val.ndim > 1:
            val = val[:, 0]
        out[:, 3] = val
    else:
        out[:, 3] = 0.0

    finite = np.isfinite(out).all(axis=1)
    if not finite.all():
        out = out[finite]
    return out


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------
_BOX_LINES = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]

_COLORS = [
    (0.0, 1.0, 1.0),
    (1.0, 1.0, 0.0),
    (0.0, 1.0, 0.0),
    (1.0, 0.5, 0.0),
    (1.0, 0.0, 1.0),
    (0.5, 0.5, 1.0),
]


def boxes_to_corners_3d(boxes):
    """boxes (N,7) -> (N,8,3) corners, same layout as OpenPCDet."""
    boxes = np.asarray(boxes, dtype=np.float32)
    if boxes.shape[0] == 0:
        return np.zeros((0, 8, 3), dtype=np.float32)
    template = np.array([
        [1, 1, -1], [1, -1, -1], [-1, -1, -1], [-1, 1, -1],
        [1, 1, 1], [1, -1, 1], [-1, -1, 1], [-1, 1, 1],
    ], dtype=np.float32) / 2.0
    corners = boxes[:, None, 3:6] * template[None, :, :]
    c, s = np.cos(boxes[:, 6]), np.sin(boxes[:, 6])
    x = corners[:, :, 0] * c[:, None] - corners[:, :, 1] * s[:, None]
    y = corners[:, :, 0] * s[:, None] + corners[:, :, 1] * c[:, None]
    corners[:, :, 0] = x
    corners[:, :, 1] = y
    corners += boxes[:, None, 0:3]
    return corners


def build_marker_array(boxes, scores, labels, class_names, header, line_width=0.06):
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    arr = MarkerArray()
    clear = Marker()
    clear.header = header
    clear.action = Marker.DELETEALL
    arr.markers.append(clear)

    if boxes.shape[0] == 0:
        return arr

    corners = boxes_to_corners_3d(boxes)
    for i in range(corners.shape[0]):
        label = int(labels[i]) - 1
        color = _COLORS[label % len(_COLORS)]
        name = class_names[label] if 0 <= label < len(class_names) else "obj"

        m = Marker()
        m.header = header
        m.ns = "pointpillars_boxes"
        m.id = i * 2
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = float(line_width)
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 0.9
        for a, b in _BOX_LINES:
            for idx in (a, b):
                pt = corners[i, idx]
                m.points.append(Point(float(pt[0]), float(pt[1]), float(pt[2])))
        arr.markers.append(m)

        t = Marker()
        t.header = header
        t.ns = "pointpillars_labels"
        t.id = i * 2 + 1
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = float(boxes[i, 0])
        t.pose.position.y = float(boxes[i, 1])
        t.pose.position.z = float(boxes[i, 2] + boxes[i, 5] * 0.5 + 0.4)
        t.pose.orientation.w = 1.0
        t.scale.z = 0.5
        t.color.r, t.color.g, t.color.b = color
        t.color.a = 1.0
        t.text = "%s %.2f" % (name, float(scores[i]))
        arr.markers.append(t)
    return arr


def make_pointcloud2(points_xyz, header):
    """Create a PointCloud2 with x/y/z[/intensity] from an ``N x 3/4`` array."""
    from sensor_msgs.msg import PointCloud2, PointField

    pts = np.asarray(points_xyz, dtype=np.float32)
    if pts.shape[1] > 4:
        pts = pts[:, :4]

    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    if pts.shape[1] == 4:
        fields.append(PointField("intensity", 12, PointField.FLOAT32, 1))

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(pts.shape[0])
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 16 if pts.shape[1] == 4 else 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = np.ascontiguousarray(pts).tobytes()
    return msg
