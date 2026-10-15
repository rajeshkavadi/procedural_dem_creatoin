"""Minimal PCD (Point Cloud Data) reader/writer - pure NumPy.

Supports the subset that mapping pipelines actually produce:

* ``DATA ascii`` and ``DATA binary`` (what ``pcl::PCDWriter::writeBinary``
  emits - used by FAST-LIO's ``/map_save``),
* arbitrary per-field ``SIZE / TYPE / COUNT`` layouts,
* float/int fields of 1/2/4/8 bytes, little-endian.

``DATA binary_compressed`` (PCL's ``writeBinaryCompressed``) is *not*
supported on purpose - recompressing 100M points costs minutes and none of
the tools in this repository produce it.  The reader raises a clear error if
it meets one.
"""

import numpy as np

_TYPECHAR = {'F': 'f', 'U': 'u', 'I': 'i'}


def _np_dtype(type_char, size):
    if type_char not in _TYPECHAR:
        raise ValueError(f'unsupported PCD TYPE {type_char!r}')
    return np.dtype(f'<{_TYPECHAR[type_char]}{size}')


def read_pcd(path):
    """Read a PCD file.

    Returns
    -------
    dict with keys:
        ``fields``  list of field names (in file order)
        ``points``  np structured array with one element per point
        ``header``  the raw header lines (list of str)

    Field values are accessed as ``data['points']['x']`` etc.
    """
    with open(path, 'rb') as fh:
        header = []
        while True:
            line = fh.readline()
            if not line:
                raise ValueError(f'{path}: unexpected EOF in PCD header')
            text = line.decode('ascii', 'replace').strip()
            header.append(text)
            if text.startswith('DATA'):
                break

        def get(key, default=None):
            for h in header:
                if h.startswith(key + ' '):
                    return h.split(None, 1)[1] if len(h.split(None, 1)) > 1 \
                        else ''
            return default

        fields = get('FIELDS').split()
        sizes = [int(x) for x in get('SIZE').split()]
        types = get('TYPE').split()
        counts = [int(x) for x in (get('COUNT') or ' '.join(
            '1' for _ in fields)).split()]
        points_line = get('POINTS')
        width = get('WIDTH')
        height = get('HEIGHT', '1')
        n = int(points_line) if points_line else int(width) * int(height)
        data_kind = get('DATA').split()[0]

        dtype_fields = []
        for name, size, typ, count in zip(fields, sizes, types, counts):
            base = _np_dtype(typ, size)
            dtype_fields.append((name, base) if count == 1
                                else (name, base, (count,)))
        dtype = np.dtype(dtype_fields, align=False)

        if data_kind == 'ascii':
            import io
            text = fh.read().decode('ascii', 'replace')
            raw = np.loadtxt(io.StringIO(text), dtype=np.float64,
                             ndmin=2) if n else np.zeros((0, len(fields)))
            raw = raw[:n]
            pts = np.empty(n, dtype=dtype)
            col = 0
            for i, (name, size, typ, count) in enumerate(
                    zip(fields, sizes, types, counts)):
                arr = raw[:, col:col + count]
                if count == 1:
                    pts[name] = arr[:, 0].astype(dtype[name])
                else:
                    pts[name] = arr.astype(dtype[name])
                col += count
        elif data_kind == 'binary':
            blob = fh.read(n * dtype.itemsize)
            if len(blob) < n * dtype.itemsize:
                raise ValueError(
                    f'{path}: truncated PCD payload '
                    f'({len(blob)} of {n * dtype.itemsize} bytes)')
            pts = np.frombuffer(blob, dtype=dtype, count=n).copy()
        else:
            raise ValueError(
                f'{path}: DATA {data_kind!r} is not supported by '
                f'dem_mapping.pcd_io (convert with '
                f'pcl_convert_pcd_ascii_binary / pdal first)')

    return {'fields': fields, 'points': pts, 'header': header,
            'count': int(n)}


def write_pcd(path, columns, names, comments=()):
    """Write a binary PCD file from float32 columns.

    Parameters
    ----------
    columns : sequence of (N,) arrays (will be cast to float32)
    names   : field names, e.g. ``['x', 'y', 'z', 'intensity']``
    comments: extra ``#`` lines for the header

    Returns the number of points written.
    """
    assert len(columns) == len(names)
    arrays = [np.ascontiguousarray(np.asarray(c, dtype=np.float32))
              for c in columns]
    n = int(arrays[0].size)
    for a in arrays:
        assert a.size == n, 'all fields must have the same length'
        assert a.ndim == 1

    lines = ['# .PCD v0.7 - Point Cloud Data file format',
             'VERSION 0.7',
             f'FIELDS {" ".join(names)}',
             f'SIZE {" ".join("4" for _ in names)}',
             f'TYPE {" ".join("F" for _ in names)}',
             f'COUNT {" ".join("1" for _ in names)}',
             f'WIDTH {n}',
             'HEIGHT 1',
             'VIEWPOINT 0 0 0 1 0 0 0',
             f'POINTS {n}',
             'DATA binary']
    for c in comments:
        lines.insert(1, f'# {c}')

    dtype = np.dtype([(name, '<f4') for name in names])
    pts = np.empty(n, dtype=dtype)
    for name, arr in zip(names, arrays):
        pts[name] = arr

    with open(path, 'wb') as fh:
        fh.write(('\n'.join(lines) + '\n').encode('ascii'))
        fh.write(pts.tobytes())
    return n


def transform_pcd(path_in, path_out, rotation, translation,
                  in_names=('x', 'y', 'z'), out_extra=None, comments=()):
    """Read ``path_in``, apply ``p -> R p + t``, write ``path_out``.

    Extra fields (e.g. ``intensity``) are carried over.  ``out_extra`` may
    add new float32 fields computed from the transformed points, e.g.
    ``{'elevation': lambda xyz: xyz[:, 2]}``.
    """
    from_dem = read_pcd(path_in)
    pts = from_dem['points']
    xyz = np.stack([pts[in_names[0]], pts[in_names[1]],
                    pts[in_names[2]]], axis=1).astype(np.float64)
    xyz2 = xyz @ np.asarray(rotation, dtype=np.float64).T + \
        np.asarray(translation, dtype=np.float64)

    names = ['x', 'y', 'z']
    cols = [xyz2[:, 0], xyz2[:, 1], xyz2[:, 2]]
    for extra in from_dem['fields']:
        if extra in in_names or extra == 'rgb':
            continue
        if np.issubdtype(pts[extra].dtype, np.floating):
            names.append(extra)
            cols.append(pts[extra].astype(np.float64))
    if out_extra:
        for name, fn in out_extra.items():
            names.append(name)
            cols.append(np.asarray(fn(xyz2), dtype=np.float64))
    n = write_pcd(path_out, cols, names, comments=comments)
    return {'count': n, 'fields': names, 'source': str(path_in),
            'destination': str(path_out)}
