"""Configuration loading / validation for the PointPillars ROS node."""

import math
import os
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "pointpillar_car.yaml"

REQUIRED_KEYS = (
    "class_names",
    "point_cloud_range",
    "voxel_size_xy",
    "anchor_sizes",
    "anchor_bottom_height",
)


class ConfigError(RuntimeError):
    pass


DEFAULTS = {
    "checkpoint": "../checkpoint_epoch_20.pth",
    "device": "auto",
    "num_point_features": 4,
    "anchor_rotations": [0.0, 1.57],
    "feature_map_stride": 2,
    "num_dir_bins": 2,
    "dir_offset": 0.78539,
    "dir_limit_offset": 0.0,
    "use_direction_classifier": True,
    "score_thresh": 0.1,
    "nms_thresh": 0.01,
    "nms_pre_maxsize": 4096,
    "nms_post_maxsize": 500,
    "max_points_per_voxel": 32,
    "max_voxels": 100000,
    "intensity_scale": "auto",
    "z_offset": 0.0,
    "z_rotation_deg": 0.0,
    "topic": "/livox/lidar",
}


def _as_floats(value, name, count):
    try:
        vals = [float(v) for v in value]
    except (TypeError, ValueError):
        raise ConfigError("%s must be a list of %d numbers, got %r" % (name, count, value))
    if len(vals) != count:
        raise ConfigError("%s must contain exactly %d numbers, got %d" % (name, count, len(vals)))
    return vals


def resolve_checkpoint(path):
    path = os.path.expanduser(str(path))
    p = Path(path)
    candidates = []
    if p.is_absolute():
        candidates.append(p)
    else:
        candidates += [PROJECT_ROOT / p, Path.cwd() / p]
    for c in candidates:
        if c.is_file():
            return str(c)
    raise ConfigError(
        "checkpoint not found: %s\n"
        "Tried:\n  %s\n"
        "Pass --checkpoint /path/to/checkpoint_epoch_20.pth or set POINTPILLARS_CHECKPOINT."
        % (path, "\n  ".join(str(c) for c in candidates)))


def load_config(path=None):
    """Load YAML config, fill defaults and derive grid/voxel sizes."""
    path = Path(path) if path else DEFAULT_CONFIG
    if not path.is_file():
        raise ConfigError("config file not found: %s" % path)

    with open(str(path), "r") as f:
        user = yaml.safe_load(f) or {}
    if not isinstance(user, dict):
        raise ConfigError("config root must be a mapping: %s" % path)

    cfg = dict(DEFAULTS)
    cfg.update(user)
    cfg["config_path"] = str(path)

    missing = [k for k in REQUIRED_KEYS if cfg.get(k) in (None, "", [])]
    if missing:
        raise ConfigError(
            "config is missing required values: %s\n\n"
            "The checkpoint does NOT store the training point-cloud range or the\n"
            "anchor size/bottom height - they were derived from the dataset by\n"
            "ship_pointpillars_scratch_livox_data.ipynb (sections 5 and 7) and must\n"
            "match that training run exactly.\n\n"
            "Fill these in %s, e.g.:\n"
            "  point_cloud_range: [x0, y0, z0, x1, y1, z1]   # printed as 'POINT_CLOUD_RANGE:'\n"
            "  anchor_sizes: [length, width, height]        # 'Car ... mean l,w,h=[...]'\n"
            "  anchor_bottom_height: <mean bottom z>        # same line, 'mean bottom z='\n\n"
            "Alternative: run tools/derive_config_from_dataset.py against the dataset's\n"
            "custom_infos_train.pkl - it reproduces the notebook rules and writes the keys."
            % (", ".join(missing), path))

    pcr = _as_floats(cfg["point_cloud_range"], "point_cloud_range", 6)
    if not (pcr[3] > pcr[0] and pcr[4] > pcr[1] and pcr[5] > pcr[2]):
        raise ConfigError("point_cloud_range must be [x0,y0,z0,x1,y1,z1] with max > min: %r" % (pcr,))
    cfg["point_cloud_range"] = pcr

    vxy = float(cfg["voxel_size_xy"])
    if vxy <= 0:
        raise ConfigError("voxel_size_xy must be positive")
    vz = float(pcr[5] - pcr[2])
    cfg["voxel_size"] = [vxy, vxy, vz]

    gx = int(round((pcr[3] - pcr[0]) / vxy))
    gy = int(round((pcr[4] - pcr[1]) / vxy))
    gz = int(round((pcr[5] - pcr[2]) / vz))
    if gx <= 0 or gy <= 0 or gz != 1:
        raise ConfigError("invalid BEV grid %r x %r x %r for range %r" % (gx, gy, gz, pcr))
    for v in (gx, gy):
        if v % 8 != 0:
            raise ConfigError(
                "BEV grid %dx%d is not divisible by 8; the stride-2 backbone cannot "
                "match its upsampled feature maps. Check point_cloud_range/voxel_size_xy."
                % (gx, gy))
    cfg["grid_size"] = [gx, gy, gz]

    fms = int(cfg["feature_map_stride"])
    if (gx // fms) < 2 or (gy // fms) < 2:
        raise ConfigError("feature map too small: %dx%d" % (gx // fms, gy // fms))

    sizes = _as_floats(cfg["anchor_sizes"], "anchor_sizes", 3)
    if min(sizes) <= 0:
        raise ConfigError("anchor_sizes must be positive lengths (l, w, h)")
    cfg["anchor_sizes"] = sizes
    cfg["anchor_bottom_height"] = float(cfg["anchor_bottom_height"])
    cfg["anchor_bottom_heights"] = [cfg["anchor_bottom_height"]]

    rot = _as_floats(cfg["anchor_rotations"], "anchor_rotations", len(cfg["anchor_rotations"]))
    cfg["anchor_rotations"] = rot

    if cfg.get("class_names") and not isinstance(cfg["class_names"], (list, tuple)):
        cfg["class_names"] = [cfg["class_names"]]
    cfg["class_names"] = [str(c) for c in cfg["class_names"]]
    cfg["num_class"] = len(cfg["class_names"])
    cfg["num_point_features"] = int(cfg["num_point_features"])

    scale = cfg["intensity_scale"]
    if isinstance(scale, str) and scale.lower() != "auto":
        try:
            scale = float(scale)
        except ValueError:
            raise ConfigError("intensity_scale must be 'auto' or a number")
        cfg["intensity_scale"] = scale

    cfg["nms_thresh"] = float(cfg["nms_thresh"])
    cfg["score_thresh"] = float(cfg["score_thresh"])
    cfg["max_points_per_voxel"] = int(cfg["max_points_per_voxel"])
    cfg["max_voxels"] = int(cfg["max_voxels"])

    cfg["checkpoint_path"] = resolve_checkpoint(cfg["checkpoint"])
    return cfg


def model_config(cfg):
    """Subset of the config consumed by pointpillars.model.PointPillar."""
    keys = (
        "class_names", "num_class", "point_cloud_range", "voxel_size", "grid_size",
        "anchor_sizes", "anchor_rotations", "anchor_bottom_heights", "feature_map_stride",
        "dir_offset", "dir_limit_offset", "num_dir_bins", "use_direction_classifier",
        "num_point_features",
    )
    return {k: cfg[k] for k in keys}


def grid_str(cfg):
    gx, gy, gz = cfg["grid_size"]
    return "%dx%dx%d" % (gx, gy, gz)


def expected_anchor_count(cfg):
    """Anchors per feature location = one size x rotations x bottom heights."""
    fw = cfg["grid_size"][0] // int(cfg["feature_map_stride"])
    fh = cfg["grid_size"][1] // int(cfg["feature_map_stride"])
    return fw * fh * len(cfg["anchor_rotations"]) * len(cfg["anchor_bottom_heights"])
