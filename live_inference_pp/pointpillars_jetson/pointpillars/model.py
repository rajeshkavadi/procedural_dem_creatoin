"""Standalone PointPillars (OpenPCDet layout) for inference.

The module/parameter names below are kept identical to OpenPCDet's
``PointPillar`` so that a training checkpoint produced by
``ship_pointpillars_scratch_livox_data.ipynb`` loads without any renaming:

    vfe.*, backbone_2d.*, dense_head.*, global_step

Only standard ``torch`` ops are used (no CUDA extensions, no spconv), which is
what makes this runnable on a Jetson Nano without compiling anything.
"""

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


# ---------------------------------------------------------------------------
# Pillar feature encoder (pcdet/models/backbones_3d/vfe/pillar_vfe.py)
# ---------------------------------------------------------------------------
class PFNLayer(nn.Module):
    def __init__(self, in_channels, out_channels, use_norm=True, last_layer=False):
        super(PFNLayer, self).__init__()
        self.last_vfe = last_layer
        self.use_norm = use_norm
        if not self.last_vfe:
            out_channels = out_channels // 2

        if self.use_norm:
            self.linear = nn.Linear(in_channels, out_channels, bias=False)
            self.norm = nn.BatchNorm1d(out_channels, eps=1e-3, momentum=0.01)
        else:
            self.linear = nn.Linear(in_channels, out_channels, bias=True)

        self.part = 50000

    def forward(self, inputs):
        if inputs.shape[0] > self.part:
            num_parts = inputs.shape[0] // self.part
            part_linear_out = [
                self.linear(inputs[num_part * self.part:(num_part + 1) * self.part])
                for num_part in range(num_parts + 1)
            ]
            x = torch.cat(part_linear_out, dim=0)
        else:
            x = self.linear(inputs)

        if self.use_norm:
            x = self.norm(x.permute(0, 2, 1)).permute(0, 2, 1)
        x = F.relu(x)

        x_max = torch.max(x, dim=1, keepdim=True)[0]
        if self.last_vfe:
            return x_max
        x_repeat = x_max.repeat(1, inputs.shape[1], 1)
        return torch.cat([x, x_repeat], dim=2)


class PillarVFE(nn.Module):
    def __init__(self, num_point_features, voxel_size, point_cloud_range,
                 use_norm=True, with_distance=False, use_absolute_xyz=True,
                 num_filters=(64,)):
        super(PillarVFE, self).__init__()
        self.use_norm = use_norm
        self.with_distance = with_distance
        self.use_absolute_xyz = use_absolute_xyz

        num_point_features += 6 if self.use_absolute_xyz else 3
        if self.with_distance:
            num_point_features += 1

        self.num_filters = list(num_filters)
        num_filters_all = [num_point_features] + self.num_filters

        pfn_layers = []
        for i in range(len(num_filters_all) - 1):
            pfn_layers.append(
                PFNLayer(
                    num_filters_all[i],
                    num_filters_all[i + 1],
                    self.use_norm,
                    last_layer=(i >= len(num_filters_all) - 2),
                )
            )
        self.pfn_layers = nn.ModuleList(pfn_layers)

        self.voxel_x = voxel_size[0]
        self.voxel_y = voxel_size[1]
        self.voxel_z = voxel_size[2]
        self.x_offset = self.voxel_x / 2 + point_cloud_range[0]
        self.y_offset = self.voxel_y / 2 + point_cloud_range[1]
        self.z_offset = self.voxel_z / 2 + point_cloud_range[2]

    def get_output_feature_dim(self):
        return self.num_filters[-1]

    @staticmethod
    def _paddings_indicator(actual_num, max_num, axis=0):
        actual_num = torch.unsqueeze(actual_num, axis + 1)
        max_num_shape = [1] * len(actual_num.shape)
        max_num_shape[axis + 1] = -1
        max_num = torch.arange(max_num, dtype=torch.int, device=actual_num.device).view(max_num_shape)
        return actual_num.int() > max_num

    def forward(self, voxel_features, voxel_num_points, coords):
        # voxel_features: [M, T, 4]   coords: [M, 4] = (batch, z, y, x)
        points_mean = voxel_features[:, :, :3].sum(dim=1, keepdim=True) / \
            voxel_num_points.type_as(voxel_features).view(-1, 1, 1)
        f_cluster = voxel_features[:, :, :3] - points_mean

        f_center = torch.zeros_like(voxel_features[:, :, :3])
        f_center[:, :, 0] = voxel_features[:, :, 0] - (
            coords[:, 3].to(voxel_features.dtype).unsqueeze(1) * self.voxel_x + self.x_offset)
        f_center[:, :, 1] = voxel_features[:, :, 1] - (
            coords[:, 2].to(voxel_features.dtype).unsqueeze(1) * self.voxel_y + self.y_offset)
        f_center[:, :, 2] = voxel_features[:, :, 2] - (
            coords[:, 1].to(voxel_features.dtype).unsqueeze(1) * self.voxel_z + self.z_offset)

        if self.use_absolute_xyz:
            features = [voxel_features, f_cluster, f_center]
        else:
            features = [voxel_features[..., 3:], f_cluster, f_center]

        if self.with_distance:
            features.append(torch.norm(voxel_features[:, :, :3], 2, 2, keepdim=True))

        features = torch.cat(features, dim=-1)

        voxel_count = features.shape[1]
        mask = self._paddings_indicator(voxel_num_points, voxel_count, axis=0)
        mask = torch.unsqueeze(mask, -1).type_as(voxel_features)
        features = features * mask

        for pfn in self.pfn_layers:
            features = pfn(features)

        # OpenPCDet uses .squeeze(); keep it 2-D even for a single pillar.
        return features.reshape(-1, features.shape[-1])


# ---------------------------------------------------------------------------
# Scatter pillars to a BEV feature map
# ---------------------------------------------------------------------------
class PointPillarScatter(nn.Module):
    def __init__(self, num_bev_features, grid_size):
        super(PointPillarScatter, self).__init__()
        self.num_bev_features = num_bev_features
        self.nx, self.ny, self.nz = grid_size
        assert self.nz == 1

    def forward(self, pillar_features, coords):
        batch_size = int(coords[:, 0].max().item()) + 1
        batch_spatial_features = []
        for batch_idx in range(batch_size):
            spatial_feature = torch.zeros(
                self.num_bev_features,
                self.nz * self.nx * self.ny,
                dtype=pillar_features.dtype,
                device=pillar_features.device,
            )
            batch_mask = coords[:, 0] == batch_idx
            this_coords = coords[batch_mask, :]
            indices = this_coords[:, 1] + this_coords[:, 2] * self.nx + this_coords[:, 3]
            indices = indices.type(torch.long)
            pillars = pillar_features[batch_mask, :].t()
            spatial_feature[:, indices] = pillars
            batch_spatial_features.append(spatial_feature)

        batch_spatial_features = torch.stack(batch_spatial_features, 0)
        return batch_spatial_features.view(batch_size, self.num_bev_features * self.nz, self.ny, self.nx)


# ---------------------------------------------------------------------------
# 2D BEV backbone (pcdet/models/backbones_2d/base_bev_backbone.py)
# ---------------------------------------------------------------------------
class BaseBEVBackbone(nn.Module):
    def __init__(self, num_bev_features, layer_nums, layer_strides, num_filters,
                 upsample_strides, num_upsample_filters):
        super(BaseBEVBackbone, self).__init__()
        assert len(layer_nums) == len(layer_strides) == len(num_filters)
        assert len(upsample_strides) == len(num_upsample_filters)

        self.blocks = nn.ModuleList()
        self.deblocks = nn.ModuleList()
        c_in_list = [num_bev_features] + list(num_filters[:-1])

        for idx in range(len(layer_nums)):
            cur_layers = [
                nn.ZeroPad2d(1),
                nn.Conv2d(c_in_list[idx], num_filters[idx], kernel_size=3,
                          stride=layer_strides[idx], padding=0, bias=False),
                nn.BatchNorm2d(num_filters[idx], eps=1e-3, momentum=0.01),
                nn.ReLU(),
            ]
            for _ in range(layer_nums[idx]):
                cur_layers.extend([
                    nn.Conv2d(num_filters[idx], num_filters[idx], kernel_size=3,
                              padding=1, bias=False),
                    nn.BatchNorm2d(num_filters[idx], eps=1e-3, momentum=0.01),
                    nn.ReLU(),
                ])
            self.blocks.append(nn.Sequential(*cur_layers))

            stride = int(upsample_strides[idx])
            if stride >= 1:
                # Upstream (233f849) uses ConvTranspose2d for stride 1 too,
                # which is why the checkpoint's first deblock is (64,128,1,1).
                self.deblocks.append(nn.Sequential(
                    nn.ConvTranspose2d(num_filters[idx], num_upsample_filters[idx],
                                       stride, stride=stride, bias=False),
                    nn.BatchNorm2d(num_upsample_filters[idx], eps=1e-3, momentum=0.01),
                    nn.ReLU(),
                ))
            else:
                stride = int(np.round(1.0 / stride))
                self.deblocks.append(nn.Sequential(
                    nn.Conv2d(num_filters[idx], num_upsample_filters[idx],
                              stride, stride=stride, bias=False),
                    nn.BatchNorm2d(num_upsample_filters[idx], eps=1e-3, momentum=0.01),
                    nn.ReLU(),
                ))

        self.num_bev_features = sum(num_upsample_filters)

    def forward(self, x):
        ups = []
        for i in range(len(self.blocks)):
            x = self.blocks[i](x)
            ups.append(self.deblocks[i](x))
        if len(ups) > 1:
            x = torch.cat(ups, dim=1)
        else:
            x = ups[0]
        return x


# ---------------------------------------------------------------------------
# Anchor head (AnchorHeadSingle + ResidualCoder decoding)
# ---------------------------------------------------------------------------
def _generate_anchors(grid_size_2d, point_cloud_range, anchor_sizes,
                      anchor_rotations, anchor_bottom_heights,
                      feature_map_stride, align_center=False):
    """Reproduces the patched OpenPCDet anchor grid.

    The notebook patches ``anchor_generator.py`` so the number of shifts is
    exactly ``feature_map_size`` (the 1e-5 guard in ``torch.arange`` silently
    drops the last row when the range spans ~1 km in float32).
    """
    fw = int(grid_size_2d[0] // feature_map_stride)
    fh = int(grid_size_2d[1] // feature_map_stride)

    if align_center:
        x_stride = (point_cloud_range[3] - point_cloud_range[0]) / fw
        y_stride = (point_cloud_range[4] - point_cloud_range[1]) / fh
        x_offset = x_stride / 2
        y_offset = y_stride / 2
    else:
        x_stride = (point_cloud_range[3] - point_cloud_range[0]) / (fw - 1)
        y_stride = (point_cloud_range[4] - point_cloud_range[1]) / (fh - 1)
        x_offset = 0.0
        y_offset = 0.0

    x_shifts = (torch.arange(fw, dtype=torch.float32) * float(x_stride)
                + float(point_cloud_range[0] + x_offset))
    y_shifts = (torch.arange(fh, dtype=torch.float32) * float(y_stride)
                + float(point_cloud_range[1] + y_offset))
    z_shifts = torch.tensor(anchor_bottom_heights, dtype=torch.float32)

    anchor_sizes = torch.as_tensor(anchor_sizes, dtype=torch.float32)
    if anchor_sizes.ndim == 1:
        anchor_sizes = anchor_sizes.reshape(1, 3)  # single (l, w, h) anchor
    anchor_rotations = torch.as_tensor(anchor_rotations, dtype=torch.float32).reshape(-1)

    try:
        gx, gy, gz = torch.meshgrid(x_shifts, y_shifts, z_shifts, indexing="ij")
    except TypeError:  # torch < 1.10
        gx, gy, gz = torch.meshgrid([x_shifts, y_shifts, z_shifts])

    anchors = torch.stack((gx, gy, gz), dim=-1)  # [fx, fy, nz, 3]
    anchors = anchors[:, :, :, None, :].repeat(1, 1, 1, anchor_sizes.shape[0], 1)
    anchor_size = anchor_sizes.view(1, 1, 1, -1, 3).repeat(
        anchors.shape[0], anchors.shape[1], anchors.shape[2], 1, 1)
    anchors = torch.cat((anchors, anchor_size), dim=-1)

    anchors = anchors[:, :, :, :, None, :].repeat(
        1, 1, 1, 1, anchor_rotations.shape[0], 1)
    anchor_rotation = anchor_rotations.view(1, 1, 1, 1, -1, 1).repeat(
        anchors.shape[0], anchors.shape[1], anchors.shape[2], anchor_sizes.shape[0], 1, 1)
    anchors = torch.cat((anchors, anchor_rotation), dim=-1)  # [fx, fy, nz, nsize, nrot, 7]

    anchors = anchors.permute(2, 1, 0, 3, 4, 5).contiguous()  # [z, y, x, size, rot, 7]
    anchors[..., 2] += anchors[..., 5] / 2                    # bottom -> center
    return anchors


class AnchorHeadSingle(nn.Module):
    def __init__(self, input_channels, num_class, grid_size, point_cloud_range,
                 anchor_sizes, anchor_rotations, anchor_bottom_heights,
                 feature_map_stride=2, dir_offset=0.78539, num_dir_bins=2,
                 use_direction_classifier=True, align_center=False):
        super(AnchorHeadSingle, self).__init__()
        self.num_class = num_class
        num_anchor_sizes = 1 if np.asarray(anchor_sizes).ndim == 1 else len(anchor_sizes)
        self.num_anchors_per_location = (len(anchor_rotations)
                                         * num_anchor_sizes
                                         * len(anchor_bottom_heights))
        self.dir_offset = dir_offset
        self.dir_limit_offset = 0.0
        self.num_dir_bins = num_dir_bins

        self.conv_cls = nn.Conv2d(input_channels,
                                  self.num_anchors_per_location * self.num_class, kernel_size=1)
        self.conv_box = nn.Conv2d(input_channels,
                                  self.num_anchors_per_location * 7, kernel_size=1)
        self.conv_dir_cls = None
        if use_direction_classifier:
            self.conv_dir_cls = nn.Conv2d(
                input_channels, self.num_anchors_per_location * self.num_dir_bins, kernel_size=1)

        self.anchors = _generate_anchors(
            grid_size_2d=grid_size[:2],
            point_cloud_range=point_cloud_range,
            anchor_sizes=anchor_sizes,
            anchor_rotations=anchor_rotations,
            anchor_bottom_heights=anchor_bottom_heights,
            feature_map_stride=feature_map_stride,
            align_center=align_center,
        )
        self._anchors_device = None

    def _anchors_on(self, device):
        if self._anchors_device != device:
            self.anchors = self.anchors.to(device)
            self._anchors_device = device
        return self.anchors

    @staticmethod
    def _limit_period(val, offset, period):
        return val - torch.floor(val / period + offset) * period

    def _decode(self, box_encodings, anchors):
        xa, ya, za = anchors[..., 0:1], anchors[..., 1:2], anchors[..., 2:3]
        dxa, dya, dza = anchors[..., 3:4], anchors[..., 4:5], anchors[..., 5:6]
        ra = anchors[..., 6:7]
        xt, yt, zt = box_encodings[..., 0:1], box_encodings[..., 1:2], box_encodings[..., 2:3]
        dxt, dyt, dzt = box_encodings[..., 3:4], box_encodings[..., 4:5], box_encodings[..., 5:6]
        rt = box_encodings[..., 6:7]

        diagonal = torch.sqrt(dxa ** 2 + dya ** 2)
        xg = xt * diagonal + xa
        yg = yt * diagonal + ya
        zg = zt * dza + za
        dxg = torch.exp(dxt) * dxa
        dyg = torch.exp(dyt) * dya
        dzg = torch.exp(dzt) * dza
        rg = rt + ra
        return torch.cat([xg, yg, zg, dxg, dyg, dzg, rg], dim=-1)

    def forward(self, spatial_features_2d):
        cls_preds = self.conv_cls(spatial_features_2d).permute(0, 2, 3, 1).contiguous()
        box_preds = self.conv_box(spatial_features_2d).permute(0, 2, 3, 1).contiguous()
        dir_preds = None
        if self.conv_dir_cls is not None:
            dir_preds = self.conv_dir_cls(spatial_features_2d).permute(0, 2, 3, 1).contiguous()

        batch_size = cls_preds.shape[0]
        anchors = self._anchors_on(cls_preds.device).reshape(-1, 7)
        num_anchors = anchors.shape[0]
        batch_anchors = anchors.view(1, -1, 7).repeat(batch_size, 1, 1)

        batch_cls = cls_preds.view(batch_size, num_anchors, -1).float()
        batch_box = box_preds.view(batch_size, num_anchors, -1)
        batch_box = self._decode(batch_box, batch_anchors)

        if dir_preds is not None:
            dir_preds = dir_preds.view(batch_size, num_anchors, -1)
            dir_labels = torch.max(dir_preds, dim=-1)[1]
            period = 2 * np.pi / self.num_dir_bins
            dir_rot = self._limit_period(
                batch_box[..., 6] - self.dir_offset,
                self.dir_limit_offset,
                period,
            )
            batch_box[..., 6] = dir_rot + self.dir_offset + period * dir_labels.to(batch_box.dtype)

        return batch_cls, batch_box


# ---------------------------------------------------------------------------
# Full detector
# ---------------------------------------------------------------------------
class PointPillar(nn.Module):
    """Inference-only network with OpenPCDet-compatible parameter names."""

    def __init__(self, model_cfg):
        super(PointPillar, self).__init__()
        self.register_buffer("global_step", torch.LongTensor(1).zero_())

        self.grid_size = tuple(model_cfg["grid_size"])       # (GX, GY, 1)
        pcr = list(model_cfg["point_cloud_range"])
        voxel_size = list(model_cfg["voxel_size"])
        num_point_features = int(model_cfg["num_point_features"])

        self.vfe = PillarVFE(
            num_point_features=num_point_features,
            voxel_size=voxel_size,
            point_cloud_range=pcr,
            use_norm=True,
            with_distance=False,
            use_absolute_xyz=True,
            num_filters=(64,),
        )
        self.map_to_bev = PointPillarScatter(64, self.grid_size)
        self.backbone_2d = BaseBEVBackbone(
            num_bev_features=64,
            layer_nums=[3, 5, 5],
            layer_strides=[2, 2, 2],
            num_filters=[64, 128, 256],
            upsample_strides=[1, 2, 4],
            num_upsample_filters=[128, 128, 128],
        )
        self.dense_head = AnchorHeadSingle(
            input_channels=384,
            num_class=int(model_cfg["num_class"]),
            grid_size=self.grid_size,
            point_cloud_range=pcr,
            anchor_sizes=model_cfg["anchor_sizes"],
            anchor_rotations=model_cfg["anchor_rotations"],
            anchor_bottom_heights=model_cfg["anchor_bottom_heights"],
            feature_map_stride=int(model_cfg.get("feature_map_stride", 2)),
            dir_offset=float(model_cfg.get("dir_offset", 0.78539)),
            num_dir_bins=int(model_cfg.get("num_dir_bins", 2)),
            use_direction_classifier=bool(model_cfg.get("use_direction_classifier", True)),
        )

    def forward(self, voxels, num_points, coords):
        pillar_features = self.vfe(voxels, num_points, coords)
        spatial = self.map_to_bev(pillar_features, coords)
        spatial_features_2d = self.backbone_2d(spatial)
        return self.dense_head(spatial_features_2d)
