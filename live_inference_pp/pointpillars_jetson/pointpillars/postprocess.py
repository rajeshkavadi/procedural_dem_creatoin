"""Anchor decoding, rotated-BEV IoU and NMS for PointPillars inference.

The IoU/NMS here replace OpenPCDet's compiled ``iou3d_nms`` CUDA extension
with plain torch, so no build step is required on the Jetson.
"""

import numpy as np
import torch


# ---------------------------------------------------------------------------
# Rotated BEV IoU (convex polygon clipping)
# ---------------------------------------------------------------------------
_QUAD = np.array([
    [-0.5, -0.5],
    [0.5, -0.5],
    [0.5, 0.5],
    [-0.5, 0.5],
], dtype=np.float32)  # counter-clockwise


def boxes_to_corners_bev(boxes):
    """boxes: (N, >=7) [x, y, z, dx, dy, dz, heading] -> (N, 4, 2) CCW corners."""
    x, y = boxes[:, 0], boxes[:, 1]
    dx, dy = boxes[:, 3], boxes[:, 4]
    heading = boxes[:, 6]
    quad = torch.as_tensor(_QUAD, dtype=boxes.dtype, device=boxes.device)
    lx = quad[None, :, 0] * dx[:, None]
    ly = quad[None, :, 1] * dy[:, None]
    cos, sin = torch.cos(heading)[:, None], torch.sin(heading)[:, None]
    rx = lx * cos - ly * sin + x[:, None]
    ry = lx * sin + ly * cos + y[:, None]
    return torch.stack([rx, ry], dim=-1)


def _polygon_area(poly, valid):
    counts = valid.sum(dim=1)
    idx = torch.arange(poly.shape[1], device=poly.device).unsqueeze(0)
    nxt = torch.where(idx + 1 < counts.unsqueeze(1), idx + 1, torch.zeros_like(idx))

    x, y = poly[..., 0], poly[..., 1]
    xn = torch.gather(x, 1, nxt)
    yn = torch.gather(y, 1, nxt)
    cross = x * yn - xn * y
    return 0.5 * torch.abs((cross * valid.to(cross.dtype)).sum(dim=1))


def _intersection_area(poly_a, poly_b):
    """Convex polygon intersection area for P independent pairs.

    poly_a, poly_b: (P, 4, 2) counter-clockwise.
    """
    p = poly_a.shape[0]
    device = poly_a.device
    poly = poly_a
    valid = torch.ones((p, 4), dtype=torch.bool, device=device)

    for e in range(4):
        s = poly.shape[1]
        a = poly_b[:, e, :].unsqueeze(1)             # (P, 1, 2)
        b = poly_b[:, (e + 1) % 4, :].unsqueeze(1)   # (P, 1, 2)

        edge = b - a
        rel = poly - a
        cross = edge[:, :, 0] * rel[:, :, 1] - edge[:, :, 1] * rel[:, :, 0]
        inside = (cross >= 0) & valid

        # previous vertex, wrapping to the last *valid* vertex
        counts = valid.sum(dim=1)
        idx = torch.arange(s, device=device).unsqueeze(0)
        prev = torch.where(idx == 0, (counts - 1).unsqueeze(1), idx - 1)
        prev = prev.clamp(0, s - 1)
        poly_prev = torch.gather(poly, 1, prev.unsqueeze(-1).expand(-1, -1, 2))
        inside_prev = torch.gather(inside, 1, prev)
        valid_prev = torch.gather(valid, 1, prev)

        crossing = (inside != inside_prev) & valid & valid_prev

        seg = poly - poly_prev
        denom = seg[..., 0] * edge[:, :, 1] - seg[..., 1] * edge[:, :, 0]
        d1 = poly_prev - a
        num = -(d1[..., 0] * edge[:, :, 1] - d1[..., 1] * edge[:, :, 0])
        safe = denom.abs() > 1e-9
        t = torch.where(safe, num / torch.where(safe, denom, torch.ones_like(denom)),
                        torch.zeros_like(denom))
        t = t.clamp(0.0, 1.0).unsqueeze(-1)
        inter = poly_prev + t * seg

        enter = crossing & inside
        leave = crossing & (~inside)
        emit_curr = inside

        slot0 = torch.where(enter.unsqueeze(-1), inter, poly)
        slot0 = torch.where(leave.unsqueeze(-1), inter, slot0)
        valid0 = (enter | emit_curr | leave) & valid
        slot1 = poly
        valid1 = enter & valid

        base0 = (2 * torch.arange(s, device=device)).unsqueeze(0).expand(p, -1)
        base1 = base0 + 1
        out = poly.new_zeros((p, 2 * s, 2))
        out_valid = torch.zeros((p, 2 * s), dtype=torch.bool, device=device)
        out.scatter_(1, base0.unsqueeze(-1).expand(-1, -1, 2), slot0)
        out.scatter_(1, base1.unsqueeze(-1).expand(-1, -1, 2), slot1)
        out_valid.scatter_(1, base0, valid0)
        out_valid.scatter_(1, base1, valid1)

        # compact valid vertices to the front, preserving order
        total = 2 * s
        key = torch.arange(total, device=device, dtype=torch.float32).unsqueeze(0)
        key = key + (~out_valid).to(torch.float32) * float(total)
        perm = torch.argsort(key, dim=1)
        poly = torch.gather(out, 1, perm.unsqueeze(-1).expand(-1, -1, 2))
        valid = torch.gather(out_valid, 1, perm)
        ar = torch.arange(total, device=device).unsqueeze(0)
        valid = valid & (ar < out_valid.sum(dim=1).unsqueeze(1))

    return _polygon_area(poly, valid)


def boxes_iou_bev(boxes_a, boxes_b, pair_chunk=8192, grid_chunk=512):
    """Rotated BEV IoU matrix (Na, Nb) computed with polygon clipping."""
    na, nb = boxes_a.shape[0], boxes_b.shape[0]
    iou = boxes_a.new_zeros((na, nb))
    if na == 0 or nb == 0:
        return iou

    ca = boxes_to_corners_bev(boxes_a)
    cb = boxes_to_corners_bev(boxes_b)
    area_a = (boxes_a[:, 3] * boxes_a[:, 4]).clamp(min=1e-6)
    area_b = (boxes_b[:, 3] * boxes_b[:, 4]).clamp(min=1e-6)
    radius_a = 0.5 * torch.sqrt(boxes_a[:, 3] ** 2 + boxes_a[:, 4] ** 2)
    radius_b = 0.5 * torch.sqrt(boxes_b[:, 3] ** 2 + boxes_b[:, 4] ** 2)

    for i0 in range(0, na, grid_chunk):
        i1 = min(i0 + grid_chunk, na)
        for j0 in range(0, nb, grid_chunk):
            j1 = min(j0 + grid_chunk, nb)
            dx = boxes_a[i0:i1, 0:1] - boxes_b[j0:j1, 0].unsqueeze(0)
            dy = boxes_a[i0:i1, 1:2] - boxes_b[j0:j1, 1].unsqueeze(0)
            reach = radius_a[i0:i1].unsqueeze(1) + radius_b[j0:j1].unsqueeze(0)
            cand = (dx * dx + dy * dy) <= (reach * reach)
            if not cand.any():
                continue

            ii, jj = cand.nonzero(as_tuple=True)
            for k0 in range(0, ii.shape[0], pair_chunk):
                k1 = min(k0 + pair_chunk, ii.shape[0])
                ia = ii[k0:k1] + i0
                ib = jj[k0:k1] + j0
                inter = _intersection_area(ca[ia], cb[ib])
                union = area_a[ia] + area_b[ib] - inter
                vals = inter / union.clamp(min=1e-6)
                vals = vals.clamp(min=0.0, max=1.0)
                iou[ia, ib] = vals
    return iou


def nms_bev(boxes, scores, thresh, pre_maxsize=4096, post_maxsize=500):
    """Greedy NMS in BEV, same policy as OpenPCDet's ``nms_gpu``."""
    if boxes.shape[0] == 0:
        return (torch.zeros(0, dtype=torch.long, device=boxes.device),
                boxes.new_zeros((0,)))
    k = min(int(pre_maxsize), boxes.shape[0])
    top_scores, top_idx = torch.topk(scores, k=k)
    boxes_top = boxes[top_idx]

    iou = boxes_iou_bev(boxes_top, boxes_top)
    overlap = iou > float(thresh)

    suppressed = torch.zeros(k, dtype=torch.bool, device=boxes.device)
    keep = []
    for i in range(k):
        if suppressed[i]:
            continue
        keep.append(i)
        if len(keep) >= int(post_maxsize):
            break
        suppressed |= overlap[i]
        suppressed[i] = True

    keep = torch.as_tensor(keep, dtype=torch.long, device=boxes.device)
    return top_idx[keep], top_scores[keep]


# ---------------------------------------------------------------------------
# Post processing
# ---------------------------------------------------------------------------
def postprocess(cls_preds, box_preds, score_thresh=0.1, nms_thresh=0.01,
                nms_pre_maxsize=4096, nms_post_maxsize=500):
    """cls_preds: (1, N, C) logits, box_preds: (1, N, 7) decoded boxes."""
    cls_scores = torch.sigmoid(cls_preds[0])
    scores, labels = torch.max(cls_scores, dim=-1)
    labels = labels + 1  # OpenPCDet class ids are 1-based
    boxes = box_preds[0]

    mask = scores >= float(score_thresh)
    boxes = boxes[mask]
    scores = scores[mask]
    labels = labels[mask]

    if boxes.shape[0] == 0:
        return boxes, scores, labels

    selected, selected_scores = nms_bev(
        boxes, scores, nms_thresh,
        pre_maxsize=nms_pre_maxsize, post_maxsize=nms_post_maxsize,
    )
    return boxes[selected], selected_scores, labels[selected]
