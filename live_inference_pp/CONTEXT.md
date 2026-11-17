# CONTEXT.md — PointPillars ROS inference on Jetson Nano

Working notes for whoever continues this. Read this before changing anything in
`pointpillars_jetson/`.

## Objective

Play a ROS `.bag` (or subscribe live) and run **PointPillars** inference with
`checkpoint_epoch_20.pth`, on an NVIDIA Jetson Nano, publishing 3D boxes to
RViz. The original request also asked for an install script and a repo that can
be cloned and verified on the Jetson.

The reference project `livox_detection/` does the same thing with
**CenterPoint**; it was used only for ROS patterns (subscriber, MarkerArray,
RViz config). The model here is PointPillars, as requested.

## What the checkpoint is

Analysed by unpickling it directly (no torch available on the analysis host):

| field | value |
| --- | --- |
| format | legacy (non-zip) torch pickle — needs `torch.load(..., map_location="cpu")` |
| top keys | `epoch=20`, `it=50560`, `version=pcdet+0.6.0+233f849`, `model_state` |
| `model_state` | 127 tensors, OpenPCDet `PointPillar` names (`vfe.*`, `backbone_2d.*`, `dense_head.*`) |
| classes | **1** (`conv_cls` output = 2 = 1 anchor × 2 rotations × 1 class) |
| box output | 14 = 2 anchors × 7; dir output 4 = 2 anchors × 2 dir bins |
| VFE | `vfe.pfn_layers.0.linear.weight (64, 10)` → 4 point features + 6 absolute-xyz features |

Consequences baked into the code:

* `class_names: [Car]` — the checkpoint cannot predict more classes.
* One anchor size, two rotations (0 and π/2), one bottom height.
* No spconv and no compiled `iou3d_nms`: voxelisation is numpy, NMS is pure
  torch rotated-BEV IoU. This is deliberate — building OpenPCDet's CUDA
  extensions on a Nano takes hours and lots of RAM.

## Training config recovered from the notebook

From `ship_pointpillars_scratch_livox_data.ipynb` (sections 5/7 and the anchor
patch around lines 982–1012):

```
KEEP_CLASSES        = ["Car"]
VOXEL_XY            = 0.64
LAYER_NUMS          = [3, 5, 5]
LAYER_STRIDES       = [2, 2, 2]
NUM_FILTERS         = [64, 128, 256]
UPSAMPLE_STRIDES    = [1, 2, 4]
NUM_UPSAMPLE_FILTERS= [128, 128, 128]
feature_map_stride  = 2
NUM_DIR_BINS        = 2
dir_offset          = 0.78539
SCORE_THRESH        = 0.1
NMS_THRESH          = 0.01
NMS_PRE_MAXSIZE     = 4096
NMS_POST_MAXSIZE    = 500
```

The notebook patches the anchor generator so the shift count is exactly the
feature-map size (no `arange(...+1e-5)` off-by-one when the PCR spans ~1 km).
`model.py::_generate_anchors` reproduces the patched behaviour, including the
non-centered stride `(x1 - x0) / (fw - 1)`.

Notebook comments imply a BEV grid of **1600 × 1280** for a PCR spanning
1024.0 × 819.2 m with `VOXEL_XY = 0.64`. Exact origin unknown.

## MISSING values (the one thing blocking a real run)

The checkpoint stores weights only. These were derived from the dataset by the
notebook and are **not** in the checkpoint:

* `point_cloud_range` — printed by section 7 as `POINT_CLOUD_RANGE: [...]`
* `anchor_sizes` — `Car n=... mean l,w,h=[...]`
* `anchor_bottom_height` — `... mean bottom z=...`

They are intentionally left `null` in `config/pointpillar_car.yaml`. The loader
(`config.py`) refuses to guess and fails with instructions instead. Two ways to
fill them:

1. Paste the three notebook lines into the YAML.
2. `python3 tools/derive_config_from_dataset.py --data-root /path/to/custom_livox_data --force`
   — reproduces the notebook rules from `custom_infos_train.pkl`: keep Car,
   ±10 m pad, snap x/y to a grid of `VOXEL_XY * 16`, z to integers, anchor =
   mean l/w/h rounded to 2 dp, bottom = mean `z - h/2`.

**Do not invent these numbers.** A wrong PCR shifts every anchor; wrong anchor
size changes every decoded box. Detection quality collapses silently.

## Code layout

```
pointpillars/model.py        PointPillar: PillarVFE, PointPillarScatter,
                             BaseBEVBackbone, AnchorHeadSingle. Parameter names
                             match the checkpoint; verify with strict load.
pointpillars/voxelizer.py    numpy pillarization; coords emitted as
                             (batch, z, y, x) to match spconv / scatter indices.
pointpillars/postprocess.py  anchor decode is in the head; here: rotated BEV
                             IoU (Sutherland–Hodgman, chunked), greedy NMS,
                             sigmoid + score filter + class ids (1-based).
pointpillars/pipeline.py     checkpoint loading, intensity auto-scale, optional
                             z rotation/offset, timings, inverse transform of
                             decoded boxes.
pointpillars/ros_utils.py    PointCloud2 -> Nx4 (no ros_numpy), MarkerArray
                             boxes/labels, PointCloud2 publishing.
tools/bag_inference.py       bag playback (real-time pacing) or live subscriber.
tools/check_setup.py         synthetic frame smoke test, no ROS needed.
tools/derive_config_from_dataset.py
tools/rviz.rviz              topics /pointpillars/{points,boxes}
install_deps.sh              Jetson: apt + pip + torch/ROS/config checks
```

### Why `sensor_msgs` conversion is hand-written

`ros_numpy` is not installed by default on JetPack images and pulls extra
dependencies. `pointcloud2_to_xyz_intensity` handles padded `row_step`,
big-endian data, and looks for `intensity`/`i`/`reflectivity` field names.

### Intensity handling

Training normalised Livox intensities to 0..1 (the notebook divides by the
max). Bags usually report 0..255. `intensity_scale: auto` divides by 255 when
the frame max is >1.5, leaves 0..1 untouched, and warns once if everything is
zero. Override with a fixed number when the sensor convention is known.

### Coordinate conventions

Boxes are `[x, y, z, dx, dy, dz, heading]` in the LiDAR frame. OpenPCDet's
`dir_offset = π/4` and 2 direction bins are applied in
`AnchorHeadSingle.forward`. `boxes_to_corners_3d` in `ros_utils.py` mirrors
OpenPCDet's corner order, so `vis_ros.py`-style markers line up.

## Verification status

* Verified in this environment: checkpoint loads into the architecture with no
  missing tensors, synthetic inference runs, BEV IoU/NMS behave, grid/anchor
  count matches `conv_cls`.
* Not verified: real bag accuracy (blocked on the missing config values), and
  any timing on actual Jetson hardware. Treat README performance notes as
  expectations, not measurements.

## Jetson notes

* JetPack 4.6 / Ubuntu 18.04 / ROS Melodic / Python 3.6 is the target. Keep
  code Python 3.6 compatible (`%` formatting, no walrus, no f-strings needed).
* `pip` has no aarch64 Jetson torch wheels on PyPI. Torch must come from the
  NVIDIA wheel matching the L4T release; `install_deps.sh` detects and explains
  this rather than downloading a wrong wheel.
* 4 GB Nano: model + 1600×1280 BEV activations are tight. Knobs: lower
  `max_voxels` (host→GPU transfer), raise `score_thresh` (smaller NMS input),
  and `--device cpu` as a correctness fallback.
* The first frame is much slower (CUDA context/cuDNN autotune). Benchmark with
  `check_setup.py --frames 5` and look at the steady-state line.

## If detections look wrong

1. `check_setup.py` passes? Weights/architecture are fine.
2. Confirm PCR origin and anchor size/bottom against the notebook — most common
   failure.
3. Check intensity scale and z placement (`z_offset`, `z_rotation_deg`).
4. Confirm the bag frame matches training (Livox `livox_frame`, +x forward).
5. `NMS_THRESH 0.01` is aggressive by design; raise it temporarily with
   `--nms-thresh 0.1` to see whether boxes are being merged away.
