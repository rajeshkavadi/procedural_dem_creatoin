#!/usr/bin/env python3
"""ROS 1 ``.bag`` -> ROS 2 ``rosbag2`` converter (with optional cutting).

What it does
------------
Reads a ROS 1 bag (``#ROSBAG V2.0``), converts every message from the ROS 1
(serialization) format to CDR (the ROS 2 format) and writes a rosbag2
directory (``metadata.yaml`` + ``*.db3``/``*.mcap``).

The conversion is done with the pure-python ``rosbags`` library - **no ROS 1
installation, no ``ros1_bridge`` and no Docker are required**.  Message
definitions embedded in the bag are reused; ``std_msgs/Header`` is special
cased (ROS 1 carries a ``seq`` field that ROS 2 dropped), everything else is
a byte-copy as long as the wire formats match, otherwise fields are migrated
(renames detected, removed fields dropped, added fields defaulted).

Extras over ``rosbags-convert``
-------------------------------
* ``--start`` / ``--duration``  cut a time window out of the recording
* ``--dry-run``                 inspect the source bag without converting
* progress + ETA + per-topic byte statistics
* automatic post-conversion validation (re-read the result and compare
  message counts and payload spot checks)

Usage
-----
    # full conversion
    ros2 run dem_replay bag_convert -- \
        OutdoorRoad_dataset.bag -o OutdoorRoad_dataset_ros2

    # only the topics the DEM pipeline consumes, first 120 s
    ros2 run dem_replay bag_convert -- \
        OutdoorRoad_dataset.bag -o cut0_ros2 \
        --topics /mid360/livox/lidar /mid360/livox/imu /gnss_pose \
        --start 0 --duration 120

    # inspect only
    ros2 run dem_replay bag_convert -- OutdoorRoad_dataset.bag --dry-run

(``bag_convert`` is a plain python entry point; ``python3 -m
dem_replay.bag_convert`` works too.  The only dependency is
``pip install rosbags``.)
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

try:
    from rosbags.convert.converter import create_connections_converters
    from rosbags.highlevel import AnyReader
    from rosbags.rosbag2 import StoragePlugin, Writer
    from rosbags.rosbag2.enums import CompressionFormat, CompressionMode
except ImportError as exc:                                       # pragma: no cover
    sys.exit(
        f'ERROR: the python package "rosbags" is missing ({exc}).\n'
        '       install it with:  pip install --user rosbags')


HUMAN = {'B': 1, 'kB': 1e3, 'MB': 1e6, 'GB': 1e9, 'TB': 1e12}


def hbytes(n: float) -> str:
    for unit in ('GB', 'MB', 'kB'):
        if n >= HUMAN[unit]:
            return f'{n / HUMAN[unit]:.2f} {unit}'
    return f'{int(n)} B'


def htime(seconds: float) -> str:
    seconds = max(0, int(seconds))
    if seconds >= 60:
        return f'{seconds // 60}m{seconds % 60:02d}s'
    return f'{seconds}s'


def select_connections(reader, include, exclude):
    """Filter reader connections by topic list(s)."""
    conns = list(reader.connections)
    if include:
        missing = [t for t in include
                   if not any(c.topic == t for c in conns)]
        if missing:
            raise SystemExit(
                f'ERROR: topic(s) {missing} not in source bag.\n'
                f'       available: {sorted({c.topic for c in conns})}')
        conns = [c for c in conns if c.topic in include]
    if exclude:
        conns = [c for c in conns if c.topic not in exclude]
    if not conns:
        raise SystemExit('ERROR: topic filters removed every connection.')
    return conns


def inspect(src: Path) -> dict:
    """Cheap source-bag report (reads index/metadata only, no payloads)."""
    with AnyReader([src]) as reader:
        topics = {}
        for name, info in sorted(reader.topics.items()):
            topics[name] = {'type': info.msgtype, 'count': info.msgcount,
                            'connections': len(info.connections)}
        return {
            'path': str(src),
            'format': 'rosbag1' if src.suffix == '.bag' else 'rosbag2',
            'duration_s': round(reader.duration / 1e9, 3),
            'start_epoch_s': round(reader.start_time / 1e9, 6),
            'message_count': reader.message_count,
            'topics': topics,
        }


def convert(src: Path, dst: Path, *, storage: str, version: int,
            compress: str, compress_mode: str, dst_typestore: str,
            include, exclude, start: float, duration: float,
            progress: bool, validate: bool) -> dict:
    """Convert ``src`` -> ``dst`` and return a JSON-able report."""

    plugin = StoragePlugin.MCAP if storage == 'mcap' else StoragePlugin.SQLITE3
    writer = Writer(dst, version=version, storage_plugin=plugin)
    if compress != 'none':
        writer.set_compression(CompressionMode[compress_mode.upper()],
                               CompressionFormat[compress.upper()])

    from rosbags.typesys import Stores, get_typestore
    typestore = None if dst_typestore == 'copy' \
        else get_typestore(Stores(dst_typestore))

    report = {
        'source': str(src),
        'destination': str(dst),
        'storage': storage,
        'bagfile_version': version,
        'compression': compress if compress != 'none' else None,
        'typestore': dst_typestore,
        'window_s': None if (start == 0.0 and duration is None)
                    else {'start': start, 'duration': duration},
        'topics': {},
        'messages': 0,
        'bytes_in': 0,
        'bytes_out': 0,
        'elapsed_s': None,
        'validated': False,
    }

    t_begin = time.monotonic()
    next_print = t_begin + 5.0

    with AnyReader([src]) as reader:
        conns = select_connections(reader, include, exclude)
        t0 = reader.start_time
        t_end = reader.end_time
        span_ns = reader.duration
        start_ns = t0 + int(start * 1e9)
        stop_ns = (start_ns + int(duration * 1e9)) if duration else None
        if start and start_ns >= t_end:
            raise SystemExit('ERROR: --start is beyond the end of the bag.')

        report['source_topics'] = {c.topic: c.msgtype for c in conns}
        report['source_duration_s'] = round(reader.duration / 1e9, 3)

        with writer:
            connmap, convmap = create_connections_converters(
                conns, typestore, reader, writer)

            stats = {c.topic: {'count': 0, 'bytes_in': 0, 'bytes_out': 0,
                               'first_ts': None, 'last_ts': None}
                     for c in conns}
            total = 0
            bytes_in = bytes_out = 0
            for rconn, ts, data in reader.messages(
                    connections=conns, start=start_ns, stop=stop_ns):
                out = convmap[rconn.msgtype](data)
                writer.write(connmap[(rconn.id, rconn.owner)], ts, out)
                st = stats[rconn.topic]
                st['count'] += 1
                st['bytes_in'] += len(data)
                st['bytes_out'] += len(out)
                if st['first_ts'] is None:
                    st['first_ts'] = ts
                st['last_ts'] = ts
                total += 1
                bytes_in += len(data)
                bytes_out += len(out)

                if progress and time.monotonic() >= next_print:
                    next_print = time.monotonic() + 5.0
                    lo = start_ns if start_ns is not None else t0
                    hi = stop_ns if stop_ns is not None else t_end
                    frac = min(1.0, max(0.0, (ts - lo) / (hi - lo))) \
                        if hi > lo else 1.0
                    el = time.monotonic() - t_begin
                    eta = el * (1.0 - frac) / frac if frac > 0.01 else 0.0
                    print(f'  [{frac * 100:5.1f}%] {total:>7} msgs  '
                          f'{hbytes(bytes_out)} written  '
                          f'{hbytes(bytes_out / max(el, 1e-6))}/s  '
                          f'eta {htime(eta)}', flush=True)

    report['messages'] = total
    report['bytes_in'] = bytes_in
    report['bytes_out'] = bytes_out
    report['elapsed_s'] = round(time.monotonic() - t_begin, 2)
    report['topics'] = {
        t: {**s,
            'first_s': None if s['first_ts'] is None
                       else round((s['first_ts'] - t0) / 1e9, 3),
            'last_s': None if s['last_ts'] is None
                      else round((s['last_ts'] - t0) / 1e9, 3),
            'type': report['source_topics'][t]}
        for t, s in stats.items()
    }
    for s in report['topics'].values():
        s.pop('first_ts', None)
        s.pop('last_ts', None)

    if validate:
        report['validation'] = validate_bag(dst, stats, span_ns)
        report['validated'] = report['validation']['ok']
    return report


def validate_bag(dst: Path, expect, span_ns: int) -> dict:
    """Re-open the converted bag: compare metadata counts and round-trip
    the first message of every topic (deserialize it with the ROS 2 schema).
    """
    out = {'ok': False, 'topics': {}}
    try:
        with AnyReader([dst]) as reader:
            counts = {t: i.msgcount for t, i in reader.topics.items()}
            for topic, exp in expect.items():
                got = counts.get(topic, 0)
                out['topics'][topic] = {
                    'expected': exp['count'], 'found': got,
                    'count_ok': got == exp['count']}
            out['duration_s'] = round(reader.duration / 1e9, 3)
            out['source_duration_s'] = round(span_ns / 1e9, 3)

            # spot check: first message per topic must deserialise
            wanted = set(out['topics'])
            for c, _ts, data in reader.messages(connections=reader.connections):
                if c.topic in wanted:
                    wanted.discard(c.topic)
                    try:
                        msg = reader.deserialize(data, c.msgtype)
                        entry = out['topics'][c.topic]
                        entry['deserializes'] = True
                        entry['msgtype'] = c.msgtype
                        entry['msg_head'] = repr(msg)[:200]
                    except Exception as exc:
                        out['topics'][c.topic]['deserializes'] = False
                        out['topics'][c.topic]['error'] = str(exc)
                    if not wanted:
                        break
            out['ok'] = all(
                t.get('count_ok') and t.get('deserializes')
                for t in out['topics'].values())
    except Exception as exc:                                     # pragma: no cover
        out['error'] = f'{type(exc).__name__}: {exc}'
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='bag_convert',
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('source', type=Path, help='ROS 1 .bag (or rosbag2 dir)')
    ap.add_argument('-o', '--output', type=Path, default=None,
                    help='destination rosbag2 directory (must not exist)')
    ap.add_argument('--storage', choices=['sqlite3', 'mcap'],
                    default='sqlite3', help='rosbag2 storage backend')
    ap.add_argument('--bag-version', type=int, choices=[8, 9], default=8,
                    help='rosbag2 bagfile version (8 reads on Humble, '
                         '9 needs a newer rosbag2; default: 8)')
    ap.add_argument('--compress', choices=['none', 'zstd'], default='none',
                    help='compress the rosbag2 output (needs more CPU at '
                         'playback time)')
    ap.add_argument('--compress-mode', choices=['file', 'message'],
                    default='file')
    ap.add_argument('--typestore', default='copy',
                    help="'copy' keeps source definitions (default), or any "
                         "rosbags store name (ros2_humble, ...) to migrate "
                         "messages into that schema")
    ap.add_argument('--topics', nargs='+', default=None,
                    help='only convert these topics')
    ap.add_argument('--exclude-topics', nargs='+', default=None,
                    help='convert everything except these topics')
    ap.add_argument('--start', type=float, default=0.0,
                    help='seconds into the bag where conversion starts')
    ap.add_argument('--duration', type=float, default=None,
                    help='convert at most this many seconds (cut window)')
    ap.add_argument('--dry-run', action='store_true',
                    help='only print the source bag report')
    ap.add_argument('--no-validate', action='store_true',
                    help='skip the post-conversion validation pass')
    ap.add_argument('--json', type=Path, default=None,
                    help='write the report to this JSON file')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args(argv)

    if not args.source.exists():
        raise SystemExit(f'ERROR: source {args.source} does not exist')

    if args.dry_run:
        report = inspect(args.source)
        print(json.dumps(report, indent=2))
        if args.json:
            args.json.write_text(json.dumps(report, indent=2))
        return 0

    if args.output is None:
        raise SystemExit('ERROR: -o/--output is required (or use --dry-run)')
    if args.output.exists():
        raise SystemExit(f'ERROR: output {args.output} exists already')

    report = convert(
        args.source, args.output,
        storage=args.storage, version=args.bag_version,
        compress=args.compress, compress_mode=args.compress_mode,
        dst_typestore=args.typestore,
        include=args.topics, exclude=args.exclude_topics,
        start=args.start, duration=args.duration,
        progress=not args.quiet, validate=not args.no_validate)

    if not args.quiet:
        print(f"\nconverted {report['messages']} messages "
              f"({hbytes(report['bytes_in'])} -> {hbytes(report['bytes_out'])}) "
              f"in {htime(report['elapsed_s'])}")
        print(f"{'topic':<28} {'msgs':>8} {'in':>10} {'out':>10}  window (s)")
        for topic, s in sorted(report['topics'].items()):
            win = f"{s['first_s']} .. {s['last_s']}"
            print(f"{topic:<28} {s['count']:>8} {hbytes(s['bytes_in']):>10} "
                  f"{hbytes(s['bytes_out']):>10}  {win}")
        if report.get('validation'):
            v = report['validation']
            print(f"validation: {'OK' if v['ok'] else 'FAILED'} "
                  f"({len(v.get('topics', {}))} topics re-read)")
            if not v['ok']:
                print(json.dumps(v, indent=2))
        print(f"\nbag: {args.output}\ntry: ros2 bag info {args.output}")

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2))

    if report.get('validation') and not report['validation']['ok']:
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
