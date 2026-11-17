"""End-to-end PointPillars inference for one LiDAR frame."""

import time

import numpy as np
import torch

from .config import model_config
from .model import PointPillar
from .postprocess import postprocess
from .voxelizer import voxelize


def _strip_module(state):
    return {k.replace("module.", "", 1) if k.startswith("module.") else k: v
            for k, v in state.items()}


def load_model_state(path):
    """Load either the training checkpoint dict or a bare state_dict."""
    try:
        ckpt = torch.load(path, map_location="cpu", weights_only=False)
    except TypeError:  # torch < 1.13 has no weights_only
        ckpt = torch.load(path, map_location="cpu")

    if isinstance(ckpt, dict) and "model_state" in ckpt:
        state = ckpt["model_state"]
        meta = {k: ckpt[k] for k in ("epoch", "it", "version") if k in ckpt}
    elif isinstance(ckpt, dict) and "model_state_dict" in ckpt:
        state = ckpt["model_state_dict"]
        meta = {}
    elif isinstance(ckpt, dict):
        state = ckpt
        meta = {}
    else:
        raise RuntimeError("unsupported checkpoint format in %s" % path)

    return _strip_module(state), meta


class PointPillarDetector(object):
    def __init__(self, cfg, checkpoint=None, device=None):
        self.cfg = cfg
        self.class_names = list(cfg["class_names"])

        self.device = torch.device(self._pick_device(device or cfg.get("device", "auto")))
        self.model = PointPillar(model_config(cfg))

        ckpt_path = checkpoint or cfg["checkpoint_path"]
        state, meta = load_model_state(ckpt_path)
        missing, unexpected = self.model.load_state_dict(state, strict=False)
        missing = [k for k in missing if k != "global_step"]
        if missing:
            raise RuntimeError(
                "checkpoint does not match this architecture; missing tensors:\n  %s"
                % "\n  ".join(missing[:20]))
        if unexpected:
            print("[pointpillars] ignoring %d unexpected checkpoint tensors (e.g. %s)"
                  % (len(unexpected), unexpected[0]))

        head = self.model.dense_head
        expected_cls = head.num_anchors_per_location * cfg["num_class"]
        if head.conv_cls.weight.shape[0] != expected_cls:
            raise RuntimeError(
                "class/anchor mismatch: checkpoint conv_cls outputs %d, config expects %d "
                "(class_names=%r, anchors/location=%d). Use the same single 'Car' class "
                "configuration the checkpoint was trained with."
                % (head.conv_cls.weight.shape[0], expected_cls,
                   self.class_names, head.num_anchors_per_location))

        self.model.to(self.device)
        self.model.eval()

        self.meta = meta
        if meta:
            print("[pointpillars] checkpoint: epoch=%s it=%s version=%s"
                  % (meta.get("epoch", "?"), meta.get("it", "?"), meta.get("version", "?")))
        print("[pointpillars] device=%s | grid=%s | anchors/loc=%d | classes=%r"
              % (self.device, cfg["grid_size"], head.num_anchors_per_location, self.class_names))

        self._z_rotation = np.deg2rad(float(cfg.get("z_rotation_deg", 0.0)))
        self._z_offset = float(cfg.get("z_offset", 0.0))
        self._warned_intensity = False

    @staticmethod
    def _pick_device(name):
        name = str(name).lower()
        if name == "auto":
            return "cuda" if torch.cuda.is_available() else "cpu"
        if name.startswith("cuda") and not torch.cuda.is_available():
            print("[pointpillars] WARNING: CUDA requested but unavailable -> CPU")
            return "cpu"
        return name

    # ------------------------------------------------------------------
    def _intensity_scale(self, intensity):
        scale = self.cfg.get("intensity_scale", "auto")
        if isinstance(scale, float) or isinstance(scale, int):
            return float(scale)
        if intensity.size == 0:
            return 1.0
        vmax = float(np.max(intensity))
        if vmax <= 1.5:
            return 1.0
        return 255.0 if vmax <= 255.0 else vmax

    def preprocess(self, points):
        pts = np.asarray(points, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] < 3:
            raise ValueError("points must be N x (>=3), got %r" % (pts.shape,))
        if pts.shape[1] == 3:
            pts = np.concatenate([pts, np.zeros((pts.shape[0], 1), np.float32)], axis=1)
        else:
            pts = pts[:, :4].copy()

        finite = np.isfinite(pts).all(axis=1)
        if not finite.all():
            pts = pts[finite]

        if self._z_rotation != 0.0 and pts.shape[0]:
            c, s = np.cos(self._z_rotation), np.sin(self._z_rotation)
            x, y = pts[:, 0].copy(), pts[:, 1].copy()
            pts[:, 0] = c * x - s * y
            pts[:, 1] = s * x + c * y
        if self._z_offset != 0.0:
            pts[:, 2] += self._z_offset

        scale = self._intensity_scale(pts[:, 3])
        if scale != 1.0:
            pts[:, 3] = np.clip(pts[:, 3] / scale, 0.0, 1.0)
        if not self._warned_intensity and pts.shape[0] and np.max(pts[:, 3]) <= 1e-6:
            print("[pointpillars] WARNING: all intensities are ~0; the model was trained "
                  "with intensity in 0..1. Check the bag's intensity field / --intensity-scale.")
            self._warned_intensity = True

        pcr = self.cfg["point_cloud_range"]
        keep = ((pts[:, 0] >= pcr[0]) & (pts[:, 0] <= pcr[3])
                & (pts[:, 1] >= pcr[1]) & (pts[:, 1] <= pcr[4]))
        pts = pts[keep]

        voxels, num_points, coords = voxelize(
            pts,
            point_cloud_range=pcr,
            voxel_size=self.cfg["voxel_size"],
            grid_size=self.cfg["grid_size"],
            max_points_per_voxel=self.cfg["max_points_per_voxel"],
            max_voxels=self.cfg["max_voxels"],
        )
        return voxels, num_points, coords, pts.shape[0]

    def _to_device(self, arr, dtype=None):
        t = torch.from_numpy(arr)
        if dtype is not None:
            t = t.to(dtype)
        return t.to(self.device, non_blocking=True)

    def _boxes_back(self, boxes):
        """Undo the z-rotation/z-offset applied during preprocessing."""
        if boxes.shape[0] == 0:
            return boxes
        if self._z_offset != 0.0:
            boxes[:, 2] -= self._z_offset
        if self._z_rotation != 0.0:
            c, s = np.cos(-self._z_rotation), np.sin(-self._z_rotation)
            x, y = boxes[:, 0].copy(), boxes[:, 1].copy()
            boxes[:, 0] = c * x - s * y
            boxes[:, 1] = s * x + c * y
            boxes[:, 6] -= self._z_rotation
        return boxes

    # ------------------------------------------------------------------
    def detect(self, points):
        timings = {}
        t0 = time.perf_counter()
        voxels, num_points, coords, num_in_range = self.preprocess(points)
        timings["preprocess"] = (time.perf_counter() - t0) * 1000.0

        empty_boxes = np.zeros((0, 7), dtype=np.float32)
        empty_scores = np.zeros((0,), dtype=np.float32)
        empty_labels = np.zeros((0,), dtype=np.int64)
        if voxels.shape[0] == 0:
            timings["inference"] = 0.0
            timings["postprocess"] = 0.0
            timings["total"] = timings["preprocess"]
            return {"boxes": empty_boxes, "scores": empty_scores, "labels": empty_labels,
                    "num_points": int(num_in_range), "num_voxels": 0,
                    "timings": timings}

        t1 = time.perf_counter()
        voxels_t = self._to_device(voxels, torch.float32)
        num_points_t = self._to_device(num_points, torch.int32)
        coords_t = self._to_device(coords, torch.int32)

        if self.device.type == "cuda":
            torch.cuda.synchronize()
        with torch.no_grad():
            cls_preds, box_preds = self.model(voxels_t, num_points_t, coords_t)
        if self.device.type == "cuda":
            torch.cuda.synchronize()
        timings["inference"] = (time.perf_counter() - t1) * 1000.0

        t2 = time.perf_counter()
        boxes, scores, labels = postprocess(
            cls_preds, box_preds,
            score_thresh=self.cfg["score_thresh"],
            nms_thresh=self.cfg["nms_thresh"],
            nms_pre_maxsize=self.cfg["nms_pre_maxsize"],
            nms_post_maxsize=self.cfg["nms_post_maxsize"],
        )
        boxes = boxes.detach().cpu().numpy().astype(np.float32)
        scores = scores.detach().cpu().numpy().astype(np.float32)
        labels = labels.detach().cpu().numpy().astype(np.int64)
        boxes = self._boxes_back(boxes)
        timings["postprocess"] = (time.perf_counter() - t2) * 1000.0
        timings["total"] = timings["preprocess"] + timings["inference"] + timings["postprocess"]

        return {"boxes": boxes, "scores": scores, "labels": labels,
                "num_points": int(num_in_range), "num_voxels": int(voxels.shape[0]),
                "timings": timings}
