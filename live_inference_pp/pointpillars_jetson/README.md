# PointPillars ROS inference (Jetson / Livox)

Runs `checkpoint_epoch_20.pth` (OpenPCDet PointPillar, single class **Car**,
trained by `ship_pointpillars_scratch_livox_data.ipynb`) on a ROS bag or a live
`PointCloud2` topic and publishes 3D boxes for RViz.

The model, voxelizer, anchor decoding and rotated NMS are re-implemented with
**plain PyTorch + numpy**. There is no OpenPCDet, spconv or CUDA extension to
build, so installation on a Jetson Nano is just `pip` + the NVIDIA PyTorch
wheel. It also runs on CPU for testing.

```
pointpillars_jetson/
├── config/pointpillar_car.yaml     # inference config (3 values must be filled)
├── install_deps.sh                 # apt + pip + torch/ROS checks
├── pointpillars/
│   ├── model.py                    # PointPillar (same parameter names as the checkpoint)
│   ├── voxelizer.py                # spconv-compatible pillarization
│   ├── postprocess.py              # anchor decode, rotated BEV IoU, NMS
│   ├── pipeline.py                 # checkpoint load + end-to-end detect()
│   └── ros_utils.py                # PointCloud2 <-> numpy, RViz markers
└── tools/
    ├── bag_inference.py            # rosbag playback or live subscription
    ├── check_setup.py              # synthetic smoke test
    ├── derive_config_from_dataset.py
    └── rviz.rviz
```

## 1. Install

On the Jetson, inside this folder:

```bash
chmod +x install_deps.sh
./install_deps.sh
source /opt/ros/melodic/setup.bash        # or noetic
```

If PyTorch is missing, the script prints the NVIDIA download page. Install the
wheel matching your L4T/JetPack release, then re-run the script. No swap file
or long compilation is needed.

## 2. Fill the config (required)

The checkpoint contains weights only. The **point-cloud range** and **anchor
size / bottom height** were derived from the training data by the notebook and
are not stored in the `.pth`. They must match the training run exactly:

| key | notebook output |
| --- | --- |
| `point_cloud_range` | section 7: `POINT_CLOUD_RANGE: [...]` |
| `anchor_sizes` | section 5/7: `Car ... mean l,w,h=[...]` |
| `anchor_bottom_height` | section 5/7: `... mean bottom z=...` |

`voxel_size_xy` is `VOXEL_XY` from the notebook's Config cell (default `0.64`).

Either paste the values into `config/pointpillar_car.yaml`, or regenerate them
from the dataset infos on any machine that has them:

```bash
python3 tools/derive_config_from_dataset.py \
    --data-root /path/to/custom_livox_data --force
```

The script reproduces the notebook's rules (Car only, ±10 m padding, grid
snapped to `VOXEL_XY * 16`). It refuses to guess: if the keys are still `null`,
`check_setup.py` and `bag_inference.py` stop with an explanation.

## 3. Verify

```bash
python3 tools/check_setup.py
```

This loads the checkpoint, builds a synthetic frame inside the configured
range, runs inference and prints per-frame latency. It needs neither ROS nor a
bag. `SETUP CHECK PASSED` means the weights, grid, anchors and NMS all work.

## 4. Run on a bag

```bash
# publish boxes + points for RViz, honouring bag timestamps
python3 tools/bag_inference.py --bag /path/to/recording.bag

# headless (no roscore needed), write detections to JSON
python3 tools/bag_inference.py --bag rec.bag --no-publish --save-json det.json

# faster than real time
python3 tools/bag_inference.py --bag rec.bag --rate 0
```

The node auto-selects the first `sensor_msgs/PointCloud2` topic if `--topic`
does not exist in the bag.

Visualize:

```bash
rviz -d tools/rviz.rviz
```

`tools/rviz.rviz` uses fixed frame `livox_frame`. If your bag publishes a
different frame (check with `rosbag info`), set *Fixed Frame* to it in RViz.

Topics:

| topic | type | content |
| --- | --- | --- |
| `/pointpillars/boxes` | `visualization_msgs/MarkerArray` | 3D boxes + labels/scores |
| `/pointpillars/points` | `sensor_msgs/PointCloud2` | received cloud (for RViz) |

## 5. Live mode

```bash
# terminal 1
roscore
# terminal 2
python3 tools/bag_inference.py --topic /livox/lidar
# terminal 3
rosbag play recording.bag
```

Or subscribe to a real LiDAR driver directly. Frames are processed
synchronously with `queue_size=1`, so slow inference drops frames instead of
building latency.

## Useful options

```bash
--checkpoint PATH     override weights
--device cpu|cuda|auto
--score-thresh 0.3    override the trained 0.1
--nms-thresh 0.01     OpenPCDet's config value; raise it to keep more overlaps
--max-frames N        stop early
--intensity-scale 255 fixed intensity normalisation (config default is auto)
```

Sensor alignment (same idea as the reference `livox_detection` node):

```yaml
z_offset: 0.0          # add to z if the ground plane is not at the training height
z_rotation_deg: 0.0    # rotate points about z to match the training frame
```

## Performance on a Jetson Nano

The training BEV grid is very large (the notebook comments mention
`1600 x 1280` cells for a ~1 km scene, roughly 300x the area of KITTI). A
full-grid forward pass is on the order of several hundred GFLOP, so expect
**several seconds per frame** on a 4 GB Nano, plus CUDA/cuDNN warm-up on the
first frame. `check_setup.py` prints the real numbers for your board and
config; do not assume real-time playback. Mitigations:

* play the bag slower so the node keeps up: `--rate 0.2` (or `--rate 0` for
  as-fast-as-possible),
* lower `max_voxels` (e.g. `60000`) to reduce host→GPU work,
* raise `--score-thresh` to shrink the NMS candidate set,
* use `--device cpu` only to confirm correctness, not for real-time use.

The original 2 GB Jetson Nano may run out of memory with a 1600x1280 grid; a
4 GB model is recommended (or an Orin Nano, where inference is much faster).

## Notes

* Intensity: the notebook normalised training intensities to `0..1`. For Livox
  bags reporting `0..255`, `intensity_scale: auto` divides by 255. If your bags
  use another convention, set a fixed number.
* The checkpoint is a legacy (pre-1.6) torch pickle. Loading sets
  `weights_only=False` on torch >= 2.6; it still loads on torch 1.8.
* `NMS_THRESH 0.01` is the value in the training config. It is aggressive by
  design for anchor-based PointPillars; override per run if boxes disappear.
