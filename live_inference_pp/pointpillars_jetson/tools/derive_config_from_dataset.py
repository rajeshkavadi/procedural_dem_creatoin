#!/usr/bin/env python3
"""Recompute the training PCR / anchors from ``custom_infos_train.pkl``.

This mirrors sections 5 and 7 of ship_pointpillars_scratch_livox_data.ipynb:

  * keep only the requested classes (the notebook trained with Car only),
  * anchor size  = mean (l, w, h) of that class' boxes, rounded to 2 decimals,
  * anchor bottom = mean (z - h/2), rounded to 2 decimals,
  * point cloud range = box extents + 10 m, snapped to a grid whose cell is
    ``VOXEL_XY * 16`` (the backbone needs a feature map divisible by 16).

Usage:
    python3 tools/derive_config_from_dataset.py --data-root /path/to/custom_livox_data
    python3 tools/derive_config_from_dataset.py --data-root ... --out config/pointpillar_car.yaml --force
"""

import argparse
import math
import pickle
import sys
from pathlib import Path

import numpy as np
import yaml

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))


def find_infos(root):
    candidates = [
        root / "custom_infos_train.pkl",
        root / "training" / "custom_infos_train.pkl",
    ]
    candidates += sorted(root.rglob("custom_infos_train.pkl"))
    for c in candidates:
        if c.is_file():
            return c
    return None


def collect_boxes(infos, wanted):
    boxes, names = [], []
    for info in infos:
        annos = info.get("annos")
        if not annos or len(annos.get("name", [])) == 0:
            continue
        b = np.asarray(annos["gt_boxes_lidar"], dtype=np.float64)
        n = np.asarray([str(x) for x in annos["name"]])
        keep = np.isin(n, wanted)
        if keep.any():
            boxes.append(b[keep])
            names.append(n[keep])
    if not boxes:
        return np.zeros((0, 7)), np.zeros((0,), dtype="<U32")
    return np.concatenate(boxes, axis=0), np.concatenate(names, axis=0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-root", required=True,
                    help="folder containing custom_infos_train.pkl")
    ap.add_argument("--classes", default="Car", help="comma separated classes (default Car)")
    ap.add_argument("--voxel-xy", type=float, default=0.64,
                    help="VOXEL_XY from the training Config cell (default 0.64)")
    ap.add_argument("--config", default=str(PKG_ROOT / "config" / "pointpillar_car.yaml"),
                    help="base YAML to update")
    ap.add_argument("--out", default=None, help="output YAML (default: overwrite --config)")
    ap.add_argument("--force", action="store_true", help="overwrite an existing output file")
    args = ap.parse_args()

    root = Path(args.data_root).expanduser().resolve()
    infos_path = find_infos(root)
    if infos_path is None:
        print("ERROR: custom_infos_train.pkl not found under %s" % root)
        return 1
    print("infos:", infos_path)

    with open(str(infos_path), "rb") as f:
        infos = pickle.load(f)
    print("train frames:", len(infos))

    wanted = [c.strip() for c in args.classes.split(",") if c.strip()]
    boxes, names = collect_boxes(infos, wanted)
    if boxes.shape[0] == 0:
        found = sorted(set(str(n) for info in infos
                           for n in (info.get("annos", {}) or {}).get("name", [])))
        print("ERROR: no annotations for %r. Classes present: %s" % (wanted, found))
        return 1

    present = [c for c in wanted if np.any(names == c)]
    if len(present) != 1:
        print("WARNING: the checkpoint has conv_cls outputs for ONE class. Derived classes: %s"
              % present)

    anchors = {}
    for c in present:
        b = boxes[names == c]
        anchors[c] = {
            "size": [round(float(v), 2) for v in b[:, 3:6].mean(axis=0)],
            "bottom": round(float((b[:, 2] - b[:, 5] / 2.0).mean()), 2),
            "n": int(b.shape[0]),
        }
        print("%-10s n=%5d mean l,w,h=%s mean bottom z=%.2f"
              % (c, anchors[c]["n"], anchors[c]["size"], anchors[c]["bottom"]))

    pad = 10.0
    x0 = float(boxes[:, 0].min() - pad)
    x1 = float(boxes[:, 0].max() + pad)
    y0 = float(boxes[:, 1].min() - pad)
    y1 = float(boxes[:, 1].max() + pad)
    z0 = float((boxes[:, 2] - boxes[:, 5] / 2.0).min() - 1.0)
    z1 = float((boxes[:, 2] + boxes[:, 5] / 2.0).max() + 1.0)

    vxy = float(args.voxel_xy)
    cell = vxy * 16.0
    x0 = math.floor(x0 / cell) * cell
    x1 = math.ceil(x1 / cell) * cell
    y0 = math.floor(y0 / cell) * cell
    y1 = math.ceil(y1 / cell) * cell
    z0 = float(math.floor(z0))
    z1 = float(math.ceil(z1))
    pcr = [round(v, 4) for v in (x0, y0, z0, x1, y1, z1)]

    gx = int(round((x1 - x0) / vxy))
    gy = int(round((y1 - y0) / vxy))
    if gx % 8 or gy % 8:
        print("ERROR: derived BEV grid %dx%d is not divisible by 8" % (gx, gy))
        return 1

    print("POINT_CLOUD_RANGE:", pcr)
    print("VOXEL_SIZE: [%.4f, %.4f, %.4f] | BEV grid: %d x %d"
          % (vxy, vxy, z1 - z0, gx, gy))

    cfg_path = Path(args.out or args.config)
    if cfg_path.exists() and not args.force:
        print("\nRefusing to overwrite %s (use --force)." % cfg_path)
        return 1
    base = {}
    if cfg_path.is_file() or Path(args.config).is_file():
        src = cfg_path if cfg_path.is_file() else Path(args.config)
        with open(str(src), "r") as f:
            base = yaml.safe_load(f) or {}

    first = present[0]
    base["class_names"] = [first]
    base["point_cloud_range"] = pcr
    base["voxel_size_xy"] = vxy
    base["anchor_sizes"] = anchors[first]["size"]
    base["anchor_bottom_height"] = anchors[first]["bottom"]

    cfg_path.parent.mkdir(parents=True, exist_ok=True)
    with open(str(cfg_path), "w") as f:
        f.write("# Derived from %s by derive_config_from_dataset.py\n" % infos_path)
        yaml.safe_dump(base, f, sort_keys=False)
    print("wrote", cfg_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
