#!/usr/bin/env python3
"""Smoke test: load the checkpoint, build pillars from synthetic points, infer.

Run this right after install. It does not need ROS or a bag.

    python3 tools/check_setup.py
    python3 tools/check_setup.py --device cpu --frames 5
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from pointpillars.config import ConfigError, expected_anchor_count, load_config  # noqa: E402
from pointpillars.pipeline import PointPillarDetector  # noqa: E402


def make_frame(cfg, num_ground=40000, num_clusters=6, seed=0):
    rng = np.random.RandomState(seed)
    pcr = cfg["point_cloud_range"]
    x = rng.uniform(pcr[0], pcr[3], num_ground).astype(np.float32)
    y = rng.uniform(pcr[1], pcr[4], num_ground).astype(np.float32)
    z = (pcr[2] + 0.4 + rng.normal(0.0, 0.05, num_ground)).astype(np.float32)
    intensity = rng.uniform(0.0, 1.0, num_ground).astype(np.float32)

    chunks = [np.stack([x, y, z, intensity], axis=1)]
    l, w, h = cfg["anchor_sizes"]
    for _ in range(num_clusters):
        cx = rng.uniform(pcr[0] + 20, pcr[3] - 20)
        cy = rng.uniform(pcr[1] + 20, pcr[4] - 20)
        cz = pcr[2] + 2.0
        n = 250
        px = cx + rng.uniform(-l / 2, l / 2, n)
        py = cy + rng.uniform(-w / 2, w / 2, n)
        pz = cz + rng.uniform(-h / 2, h / 2, n)
        chunks.append(np.stack([px, py, pz, rng.uniform(0, 1, n)], axis=1).astype(np.float32))

    pts = np.concatenate(chunks, axis=0).astype(np.float32)
    rng.shuffle(pts)
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(PKG_ROOT / "config" / "pointpillar_car.yaml"))
    ap.add_argument("--checkpoint", default=None)
    ap.add_argument("--device", default=None, choices=[None, "auto", "cpu", "cuda"])
    ap.add_argument("--frames", type=int, default=3)
    args = ap.parse_args()

    print("torch:", torch.__version__, "| CUDA build:", torch.version.cuda,
          "| cuda available:", torch.cuda.is_available())
    if torch.cuda.is_available():
        print("GPU:", torch.cuda.get_device_name(0))

    try:
        cfg = load_config(args.config)
        det = PointPillarDetector(cfg, checkpoint=args.checkpoint, device=args.device)
    except (ConfigError, RuntimeError) as e:
        print("\nSETUP CHECK FAILED:\n%s" % e)
        return 2

    n_params = sum(p.numel() for p in det.model.parameters())
    print("parameters: %.2f M" % (n_params / 1e6))
    print("expected anchors: %d | conv_cls outputs: %d"
          % (expected_anchor_count(cfg), det.model.dense_head.conv_cls.weight.shape[0]))

    pts = make_frame(cfg)
    print("synthetic frame: %d points" % pts.shape[0])

    times = []
    total_dets = 0
    for i in range(max(1, args.frames)):
        t0 = time.time()
        out = det.detect(pts)
        dt = (time.time() - t0) * 1000.0
        times.append(dt)
        assert out["boxes"].ndim == 2 and out["boxes"].shape[1] == 7, out["boxes"].shape
        assert np.isfinite(out["boxes"]).all(), "non-finite boxes"
        assert np.isfinite(out["scores"]).all(), "non-finite scores"
        total_dets += out["boxes"].shape[0]
        print("frame %d: %d detections | voxels=%d | total=%.1f ms (infer %.1f ms)"
              % (i, out["boxes"].shape[0], out["num_voxels"], dt,
                 out["timings"]["inference"]))

    if total_dets == 0:
        print("note: 0 detections on synthetic points is expected; this checks the "
              "pipeline, not detection accuracy. Use a real bag to judge quality.")

    print("\nSETUP CHECK PASSED")
    if len(times) > 1:
        print("steady-state: %.1f ms/frame (%.1f FPS)"
              % (np.mean(times[1:]), 1000.0 / max(1e-6, np.mean(times[1:]))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
