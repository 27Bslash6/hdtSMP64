# CUDA Optimization Roadmap

**Branch:** `feature/cuda-batched-collisions`
**Last Updated:** 2025-01-10

---

## Executive Summary

Multi-phase GPU optimization to achieve "infinite" scaling of physics actors:

| Phase | Target | Status | Effort | Actor Capacity |
|-------|--------|--------|--------|----------------|
| 1 | Batched Collisions | **COMPLETE** | 2 weeks | ~20 actors |
| 2 | Batched Internal Updates | **COMPLETE** | 2 weeks | ~20 actors |
| 3 | Double-Buffer Leaf AABBs | **PLANNED** | 2 weeks | **50+ actors** |
| 4-5 | GPU Trees + Broadphase | FUTURE | 6-8 weeks | 100+ actors |
| 6 | XPBD GPU Solver | ROADMAP | 8-12 weeks | **500+ actors** |

**Key Insight:** Phase 3 is the critical unlock. By accepting 2-frame collision latency, we eliminate a 12.2ms sync that currently dominates frame time.

---

## Current State (After Phase 1+2)

### Profile Summary (c3 trace)

```
+--------------------------------------------------+
|  GlobalGpuSync            19.0% (12.2ms)         | ← BOTTLENECK
|  BulletCollisionDetection 31.6% (20.3ms)         |
|  SolveConstraints         13.4% (8.6ms)          | ← Future bottleneck
|  LaunchInternalUpdates     3.8% (2.5ms)          |
|  BatchedLaunchCollisions   1.1% (0.7ms)          |
+--------------------------------------------------+
```

### What's Been Achieved

| Phase | Change | Impact |
|-------|--------|--------|
| 1 | 925K collision kernels → ~100 | -32% frame time |
| 2 | 117 graph launches → 5 direct | -37% (GraphLaunch eliminated) |

---

## Phase 3: Double-Buffer Leaf AABBs (NEXT)

**Workplan:** `cuda-gpu-tree-updates.md`
**Effort:** 2 weeks
**Goal:** Eliminate 12.2ms GlobalGpuSync

### The Problem

```
GPU: Vertices → Leaf AABBs
         │
      SYNC ← 12.2ms waiting for GPU!
         │
CPU: Tree propagation (104μs)
```

The CPU tree update needs leaf AABBs from GPU. Currently we block 12.2ms waiting.

### The Solution

Use **previous frame's** leaf AABBs. No sync needed.

```
GPU: Vertices → Leaf AABBs [buffer N]
         │                      │
         │ (NO SYNC)            │ async copy
         │                      v
CPU: Tree uses [buffer N-1]
```

### Trade-off

| Metric | Before | After |
|--------|--------|-------|
| Collision latency | 1 frame | 2 frames |
| Sync time | 12.2ms | **0ms** |
| Frame time (10 actors) | ~22ms | **~10ms** |

2-frame latency at 60 FPS = 33ms. Imperceptible for cloth/hair physics.

### Implementation Summary

1. Double-buffer leaf AABB storage
2. GPU writes to current buffer
3. CPU tree update reads previous buffer
4. Async copy (non-blocking)
5. Swap buffers at frame end

---

## Phase 4-5: GPU Trees + Broadphase (FUTURE)

**Status:** Design only - implement if Phase 3 insufficient

### Phase 4: GPU Tree Propagation

Move BVH tree propagation from CPU to GPU:
- Bottom-up AABB merge kernel
- Level-by-level or atomic updates
- Eliminates CPU tree overhead entirely

**Note:** Expert panel identified this is more complex than originally planned. The existing tree is n-ary (not binary BVH). Would require either:
- Converting to binary BVH representation, OR
- Atomic propagation with parent indices

### Phase 5: GPU Broadphase

Move collision pair gathering to GPU:
- Spatial hash grid for uniform-size objects
- Or BVH traversal for mixed sizes
- Embarrassingly parallel pair generation

### Expected Gain

| Metric | After Phase 3 | After Phase 4-5 |
|--------|---------------|-----------------|
| Tree overhead | 104μs | 0μs |
| Broadphase | ~1ms | ~0.1ms |
| **Actor capacity** | 50+ | **100+** |

---

## Phase 6: XPBD GPU Solver (ROADMAP)

**Workplan:** `cuda-xpbd-solver.md`
**Effort:** 8-12 weeks
**Goal:** Replace Bullet CPU solver for "infinite" scaling

### Why XPBD?

Bullet's Sequential Impulse solver is inherently serial:
```
for each constraint:  // Can't parallelize!
    solve(constraint)
```

XPBD is massively parallel:
```
parallel_for each constraint:  // GPU-native!
    compute_correction()
```

### Scaling Comparison

| Actors | Bullet (CPU) | XPBD (GPU) |
|--------|--------------|------------|
| 50 | 14ms | 0.8ms |
| 100 | 28ms | 1.5ms |
| 500 | 140ms | 5ms |

### Decision Gate

**Do not start Phase 6 until:**
1. Phase 3 complete and validated
2. Bullet solver confirmed as bottleneck (>30% frame time)
3. 100+ actor scaling actually required
4. 8-12 week effort justified

Phase 3's 50-actor capacity may be sufficient for most users.

---

## Pipeline Evolution

### Before Optimization (Baseline)

```
Frame: 45ms+ for 10 actors

CPU: bones → [per-body graph launch x117] → SYNC → tree → broadphase
                     ↓
GPU: [per-pair kernel x925K]
                     ↓
CPU: SYNC → apply results → Bullet solve → integrate
```

### After Phase 1+2 (Current)

```
Frame: ~22ms for 10 actors

CPU: bones
       ↓
GPU: batched vertex/AABB kernels
       ↓
CPU: SYNC (12.2ms) ← BOTTLENECK
       ↓
CPU: tree (104μs) → broadphase
       ↓
GPU: batched collision kernels
       ↓
CPU: apply results (from N-1) → Bullet solve → integrate
```

### After Phase 3 (Target)

```
Frame: ~10ms for 10 actors

CPU: bones
       ↓
GPU: batched vertex/AABB → async copy
       ↓                        ↓
CPU: tree (uses N-1 AABBs)     (no sync!)
       ↓
CPU: broadphase → launch collision batch
       ↓
GPU: collision kernels
       ↓
CPU: apply results (from N-2) → Bullet solve → integrate
```

### After Phase 6 (Blue Sky)

```
Frame: <5ms for 100 actors

CPU: bones → upload
       ↓
GPU: vertices → AABBs → tree → broadphase → collision → XPBD solve → integrate
       ↓
CPU: download final positions
```

---

## Scaling Projections

| Phase | 10 Actors | 50 Actors | 100 Actors | 500 Actors |
|-------|-----------|-----------|------------|------------|
| Current | 22ms | OOM | OOM | OOM |
| Phase 3 | **10ms** | 25ms | OOM | OOM |
| Phase 4-5 | 8ms | 15ms | 30ms | OOM |
| Phase 6 | 2ms | 5ms | **10ms** | **25ms** |

---

## Latency Budget

All phases accept increased collision latency for performance:

| Phase | Collision Latency | At 60 FPS | Acceptable? |
|-------|-------------------|-----------|-------------|
| Current | 1 frame | 16ms | Yes |
| Phase 3 | 2 frames | 33ms | Yes (cloth/hair) |
| Phase 6 | 3 frames | 50ms | Yes (visual physics) |

For gameplay-critical physics (player collision), maintain separate low-latency path.

---

## Risk Summary

| Phase | Risk Level | Primary Risk | Mitigation |
|-------|------------|--------------|------------|
| 3 | **Low** | 2-frame latency artifacts | Test extensively; user accepts |
| 4-5 | Medium | N-ary tree complexity | May need architecture rethink |
| 6 | **High** | Visual behavior change | A/B testing; Bullet fallback |

---

## Decision Tree

```
Start
  │
  ├─ Phase 3: Double-buffer AABBs (2 weeks)
  │     │
  │     ├─ 10 actors < 10ms? ──Yes──► Ship it!
  │     │
  │     └─ No
  │           │
  │           ├─ Need 50+ actors? ──No──► Optimize elsewhere
  │           │
  │           └─ Yes
  │                 │
  │                 ├─ Phase 4-5: GPU trees (6-8 weeks)
  │                 │     │
  │                 │     ├─ 50 actors < 16ms? ──Yes──► Ship it!
  │                 │     │
  │                 │     └─ No
  │                 │           │
  │                 │           └─ Need 100+ actors?
  │                 │                 │
  │                 │                 └─ Yes
  │                 │                       │
  │                 │                       └─ Phase 6: XPBD (8-12 weeks)
  │                 │
  │                 └─ Skip to Phase 6 if solver is bottleneck
```

---

## Files Reference

| Workplan | Phase | Status |
|----------|-------|--------|
| `cuda-batched-collisions.md` | 1 | COMPLETE |
| `cuda-batched-internal-updates.md` | 2 | COMPLETE |
| `cuda-gpu-tree-updates.md` | 3 | PLANNED |
| `cuda-xpbd-solver.md` | 6 | ROADMAP |
| `GPU_PHYSICS_ARCHITECTURE.md` | All | Reference |

---

## Non-CUDA Impact

All changes are guarded by `#ifdef CUDA`. NOCUDA builds:
- Unaffected by any optimization
- Use CPU-only paths
- No performance regression
- Remain the fallback for non-NVIDIA GPUs
