"""Procedural surface classification from two accumulated rasters.

A bare-earth DEM can only ever show the ground, which looks like a
featureless disc from a single viewpoint: walls, trees and cars are exactly
what the ground filter removes.  This module combines the two layers the
accumulator keeps for the same cells --

* ``ground``: lowest return that passed the ground filter (bare earth),
* ``top``:   highest return of *any* point (canopy, roof, wall crest),

-- into a height-above-ground model and four classes:

===========  ==========================================================
class        rule
===========  ==========================================================
ground       ``top - ground < low_min_m`` (road, pavement, lawn)
low object   ``low_min_m <= h < low_max_m`` (kerb, car roof, person)
building     ``h >= low_max_m`` with a *smooth* top compared with its
             tall neighbours (roof line, wall crest, container)
vegetation   ``h >= low_max_m`` with a ragged top (canopy, branches)
===========  ==========================================================

The building/vegetation split only compares cells that are themselves tall,
so the ground in front of a wall cannot make the wall look "ragged".  It is
deliberately rule-based -- no learned model, no PDAL -- so every input is a
number the operator can inspect and tune.

This is a *visibility* map, not a 3D reconstruction: one viewpoint cannot
see behind a wall, so a building shows as its visible facade crest rather
than as a filled footprint, and a dense canopy shows the canopy top.
"""

from dataclasses import dataclass
from typing import Tuple

import numpy as np

# Class codes stored in the classification raster.
NO_DATA = 0
GROUND = 1
LOW_OBJECT = 2
BUILDING = 3
VEGETATION = 4

CLASS_NAMES = {
    NO_DATA: "no data",
    GROUND: "ground",
    LOW_OBJECT: "low object",
    BUILDING: "building",
    VEGETATION: "vegetation",
}

# Display colours (RGB 0-1) for the rendered legend.
CLASS_COLORS = {
    GROUND: (0.42, 0.45, 0.50),      # asphalt grey
    LOW_OBJECT: (0.98, 0.75, 0.20),  # amber
    BUILDING: (0.40, 0.62, 0.92),    # steel blue
    VEGETATION: (0.28, 0.76, 0.34),  # green
}


@dataclass(frozen=True)
class SurfaceClasses:
    """Classification raster plus the summary used in titles and JSON."""

    classes: np.ndarray          # (ny, nx) uint8 of class codes
    height: np.ndarray           # (ny, nx) float32, NaN = no data (nDSM)
    stats: dict
    # Continuous bare-earth surface (observed cells as measured, gaps
    # interpolated) and the mask of cells that really were observed.
    ground_filled: np.ndarray = None
    ground_known: np.ndarray = None


def _shift(a: np.ndarray, dy: int, dx: int, fill) -> np.ndarray:
    """``out[r, c] = a[r - dy, c - dx]``, ``fill`` outside the array."""
    out = np.full_like(a, fill)
    ny, nx = a.shape
    ys, ye = max(0, dy), ny + min(0, dy)
    xs, xe = max(0, dx), nx + min(0, dx)
    if ys >= ye or xs >= xe:
        return out
    out[ys:ye, xs:xe] = a[ys - dy:ye - dy, xs - dx:xe - dx]
    return out


_OFFSETS = tuple((dy, dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1)
                 if (dy, dx) != (0, 0))


def inpaint_nearest(values: np.ndarray, valid: np.ndarray,
                    iterations: int = 60) -> Tuple[np.ndarray, np.ndarray]:
    """Grow valid cells into their empty neighbours (nearest-value fill).

    Ground is unobserved under trees, behind walls and under parked cars.
    Filling those cells from the nearest observed ground still lets the
    height of an object standing there be measured; the iteration count
    bounds how far (in cells) a filled value may travel.
    """
    filled = np.array(values, dtype=np.float64, copy=True)
    known = np.array(valid, dtype=bool, copy=True)
    for _ in range(int(iterations)):
        if known.all() or not known.any():
            break
        acc = np.zeros_like(filled)
        cnt = np.zeros(filled.shape, dtype=np.float64)
        for dy, dx in _OFFSETS:
            k = _shift(known, dy, dx, False)
            acc += np.where(k, _shift(filled, dy, dx, 0.0), 0.0)
            cnt += k
        grow = (~known) & (cnt > 0)
        if not grow.any():
            break
        filled[grow] = acc[grow] / cnt[grow]
        known |= grow
    return filled, known


def _spread(raster: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """3x3 max-minus-min of ``raster`` over valid neighbours (inf if <2)."""
    hi = np.full(raster.shape, -np.inf)
    lo = np.full(raster.shape, np.inf)
    cnt = np.zeros(raster.shape, dtype=np.float64)
    for dy, dx in _OFFSETS:
        v = _shift(valid, dy, dx, False)
        r = _shift(raster, dy, dx, 0.0)
        hi = np.where(v & (r > hi), r, hi)
        lo = np.where(v & (r < lo), r, lo)
        cnt += v
    out = np.full(raster.shape, np.inf)
    enough = cnt >= 1
    out[enough] = hi[enough] - lo[enough]
    return out


def classify_surface(ground: np.ndarray, ground_valid: np.ndarray,
                     top: np.ndarray, top_valid: np.ndarray,
                     *, resolution: float = 0.25,
                     low_min_m: float = 0.3,
                     low_max_m: float = 2.0,
                     rough_m: float = 0.5) -> SurfaceClasses:
    """Classify cells from the bare-earth and top-surface rasters.

    Parameters
    ----------
    ground, ground_valid : bare-earth raster and its validity mask
    top, top_valid       : highest-return raster and its validity mask
    resolution           : cell size in metres (for the reported areas)
    low_min_m            : below this height above ground -> ground
    low_max_m            : below this height -> low object, above -> tall
    rough_m              : top spread above which a tall cell is "ragged"
                           (vegetation) instead of "smooth" (building)
    """
    if ground.shape != top.shape:
        raise ValueError("ground and top rasters must have the same shape")

    classes = np.zeros(top.shape, dtype=np.uint8)
    height = np.full(top.shape, np.nan, dtype=np.float64)
    filled = np.full(top.shape, np.nan, dtype=np.float64)
    known = np.zeros(top.shape, dtype=bool)
    observed = ground_valid | top_valid
    if not observed.any():
        return SurfaceClasses(classes=classes,
                              height=height.astype(np.float32),
                              stats=_summary(classes, resolution),
                              ground_filled=filled.astype(np.float32),
                              ground_known=known)

    # Work only on the occupied window: a 200 m grid is mostly empty and the
    # neighbourhood work is per-cell.
    rows = np.flatnonzero(observed.any(axis=1))
    cols = np.flatnonzero(observed.any(axis=0))
    r0, r1 = max(0, rows[0] - 2), min(observed.shape[0], rows[-1] + 3)
    c0, c1 = max(0, cols[0] - 2), min(observed.shape[1], cols[-1] + 3)

    sub_classes, sub_height, sub_filled, sub_known = _classify(
        ground[r0:r1, c0:c1], ground_valid[r0:r1, c0:c1],
        top[r0:r1, c0:c1], top_valid[r0:r1, c0:c1],
        low_min_m=low_min_m, low_max_m=low_max_m, rough_m=rough_m)
    classes[r0:r1, c0:c1] = sub_classes
    height[r0:r1, c0:c1] = sub_height
    filled[r0:r1, c0:c1] = sub_filled
    known[r0:r1, c0:c1] = sub_known

    stats = _summary(classes, resolution)
    stats["ground_observed_cells"] = int(ground_valid.sum())
    stats["interpolated_cells"] = int((known & ~ground_valid).sum())
    return SurfaceClasses(classes=classes,
                          height=height.astype(np.float32),
                          stats=stats,
                          ground_filled=filled.astype(np.float32),
                          ground_known=known)


def _classify(ground: np.ndarray, ground_valid: np.ndarray,
              top: np.ndarray, top_valid: np.ndarray, *, low_min_m: float,
              low_max_m: float, rough_m: float):
    ground_filled, ground_known = _ground_surface(ground, ground_valid)

    height = np.full(top.shape, np.nan, dtype=np.float64)
    usable = top_valid & ground_known
    height[usable] = (np.asarray(top[usable], dtype=np.float64)
                      - ground_filled[usable])

    classes = np.zeros(top.shape, dtype=np.uint8)
    if not usable.any():
        return classes, height, ground_filled, ground_known

    h = height
    low = usable & (h >= float(low_min_m))
    tall = usable & (h >= float(low_max_m))

    smooth = np.zeros(top.shape, dtype=bool)
    if tall.any():
        spread = _spread(np.where(tall, top, 0.0), tall)
        smooth = tall & (spread <= float(rough_m))

    classes[usable] = GROUND
    classes[low] = LOW_OBJECT
    classes[tall & smooth] = BUILDING
    classes[tall & ~smooth] = VEGETATION
    return classes, height, ground_filled, ground_known


def _ground_surface(ground: np.ndarray, valid: np.ndarray,
                    iterations: int = 12):
    """Bare-earth raster with holes filled where nothing was observed.

    Nearest-observation growth first (a few cells around each gap, which
    keeps local relief), then a least-squares plane through all observed
    ground for gaps wider than that (behind walls, under dense canopy).
    """
    filled, known = inpaint_nearest(np.where(valid, ground, 0.0), valid,
                                    iterations=iterations)
    missing = ~known
    if missing.any() and int(valid.sum()) >= 3:
        ys, xs = np.nonzero(valid)
        A = np.column_stack([xs, ys, np.ones(len(xs), dtype=np.float64)])
        coef, _, _, _ = np.linalg.lstsq(A, ground[valid], rcond=None)
        gy, gx = np.nonzero(missing)
        filled[gy, gx] = (coef[0] * gx + coef[1] * gy + coef[2])
        known |= missing
    return filled, known


def _summary(classes: np.ndarray, resolution: float) -> dict:
    cell_area = float(resolution) ** 2
    total = int((classes != NO_DATA).sum())
    counts = {}
    for code in (GROUND, LOW_OBJECT, BUILDING, VEGETATION):
        n = int((classes == code).sum())
        counts[CLASS_NAMES[code]] = {
            "cells": n,
            "area_m2": round(n * cell_area, 1),
            "share": (round(n / total, 4) if total else 0.0),
        }
    tall = counts["building"]["cells"] + counts["vegetation"]["cells"]
    return {
        "cells": total,
        "classes": counts,
        "tall_cells": tall,
        "building_share_of_tall": (round(counts["building"]["cells"] / tall, 4)
                                   if tall else 0.0),
    }
