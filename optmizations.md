# Optimizations for smoother streaming

This document summarizes concrete optimizations to make the bag replay → FAST-LIO → DEM pipeline stream more seamlessly (lower latency, less jitter, fewer drops).

## Current bottlenecks

### 1) Replay pacing (`dem_replay/replay_node.py`)
Issues:
- `_wait_paced()` sleeps in up to 20ms chunks and does active condition checks (coarse granularity, scheduling jitter).
- Rate changes recompute `base_wall` from `t_last_bag` but don't fully compensate for accumulated sleep errors.
- Worker polls while paused (minor CPU usage).

Suggestions:
- Use `time.perf_counter()` for pacing (higher resolution, monotonic). 
- Do a single sleep toward target with early wake + small spin/busy-wait remainder for precision.
- Track pacing error (integral) to smooth rate transitions.
- Consider a leaky-bucket style pacer to absorb OS jitter.
- Avoid holding locks during long sleeps.

Example sketch:
```python
target = base + (bag_ts - t0_bag)/1e9 / rate
now = time.perf_counter()
if now >= target - 1e-6: return True
rem = target - now
if rem > 0.005: time.sleep(rem - 0.002)
# spin remainder
end = time.perf_counter() + max(0, target - time.perf_counter())
while time.perf_counter() < end: pass
```

### 2) `dem_node` hot path (`dem_mapping/dem_mapping/dem_node.py`)
Per-scan flow does parsing, transform, filtering, and grid update under lock. Issues:
- Pose interpolation in `pose` mode is expensive (`PoseBuffer.transform_points()` does full `searchsorted`, vectorized slerp, `np.einsum`). 
- Grid update held under `self._lock` (necessary for thread-safety of grid arrays).
- All cloud callbacks serialized via `MutuallyExclusiveCallbackGroup` (limits cross-scan parallelism).
- Allocations from intermediate arrays/copies add GC pressure.

Suggestions:
- **Prefer `registered` mode** (default with FAST-LIO): skips expensive per-point pose interpolation entirely.
- Reduce lock hold time: copy grid state for publishing outside lock (already done well via `copy_state()`/`evaluate()`), and keep only `grid.update()` under lock.
- Pre-allocate/reuse buffers for point arrays where possible.
- Cache frequently accessed parameters/lookups.
- Consider adaptive downsampling: if latency rises, lower effective `max_points_per_scan` dynamically.
- Move heavy path to C++ (rclcpp) for zero-copy PointCloud2 and lower overhead.

### 3) `PoseBuffer` (`dem_mapping/dem_mapping/pose_buffer.py`)
Uses `np.append` on every `add()` creating new arrays; quaternions normalized on every insert. Also does full-array ops in queries.
- O(n) allocations per pose insertion, GC churn over long runs.
- `transform_points` does expensive per-point bracketing even if many points fall back to ends.

Suggestions:
- **Circular buffer**: pre-allocate arrays of `max_size`, track head/count/indexing. Avoid allocations/copies.
- Lazy or batched normalization.
- Cache last bracket index when poses are monotonic and queries are in order.
- Early-out paths for the common case (most points in range, monotonic time).
- Optionally use float32 where precision allows.

Sketch (circular):
```python
self._t = np.zeros(max_size, dtype=np.float64)
self._p = np.zeros((max_size,3), dtype=np.float64)
self._q = np.zeros((max_size,4), dtype=np.float64)
self._head = 0; self._count = 0
```
Update all indexing to modulo and handle wrap-around.

### 4) Grid update (`dem_mapping/dem_mapping/elevation_grid.py`)
Already well-optimized (span-limited `bincount`, `np.minimum.at`, `copy_state()`/`evaluate()` split). Minor tweaks:
- Use integer arithmetic where possible.
- Ensure contiguous arrays (`np.ascontiguousarray`) passed in.
- For very large grids, consider 32-bit floats; current use of float64 for sums is fine for mean but can be trimmed.
- Avoid unnecessary copies in hot paths.

### 5) QoS, threading, backpressure
Current design is sensible. Suggestions:
- **Cloud sub**: BEST_EFFORT depth 10 is good to drop stale scans under load. Tune depth based on measured latency.
- **Replay pub**: RELIABLE depth 100 helps when consumers are slow to attach; consider tuning.
- **Callback groups/executor**: Current split (subs/pubs/heavy/light) is good. Try tuning thread count for target hardware.
- **Monitoring**: Add explicit drop counters (QoS drops), queue depths, and end-to-end latency (sensor stamp → grid update). dem_node already has timing histograms - expose via stats topic.
- **End-to-end BEST_EFFORT** where possible to reduce latency variance.

### 6) FAST-LIO / IMU
- Ensure IMU units correct (imu_relay g→m/s²) - already done.
- Keep `use_sim_time` consistent across nodes.
- Avoid extra copies of registered clouds.
- Apply known patches if rebuilding (as noted in repo docs) but not directly a streaming optimization.

## Quick wins (high impact, low effort)
1. Switch to `registered` mode (default). 
2. Tune replay pacing to use `perf_counter()` + early-wake + spin.
3. Shrink fixed grid if trajectory smaller than configured size.
4. Reduce viz/publish rates (`image_rate`/`cloud_rate`) when not needed.
5. Add basic latency/drop metrics to logs/stats.

## Structural improvements (bigger)
1. **Circular PoseBuffer** - eliminate allocations in pose stream.
2. **Time-window batching** - accumulate scans over 50-200ms windows and process as batch for better vectorization/amortization.
3. **Worker pool for processing** - enqueue work from callbacks to background threads, keep callbacks fast.
4. **Zero-copy / C++ migration** for dem_node hot path.
5. **Tiled/sliding grid** for unbounded areas (evict distant cells) to cap memory.
6. **Adaptive QoS/throttling** based on measured latency.

## Recommended immediate changes
For most seamless streaming with minimal code changes:
- **Replay**: replace sleep-based pacing with perf_counter + early wake + small spin.
- **PoseBuffer**: convert append-based storage to circular arrays (big win on long runs).
- **Tuning**: use `registered` mode, right-size grid, dial down publish rates.

The codebase is already well-optimized (vectorized NumPy, smart locking, callback group separation). Biggest gains come from removing allocations in hot paths and tightening timing.
