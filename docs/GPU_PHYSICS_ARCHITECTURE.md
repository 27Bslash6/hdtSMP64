# GPU Physics Pipeline Architecture: Blue-Sky Design for hdtSMP64

> *Strategic architecture for achieving "infinite" scaling of physics actors through full GPU acceleration with multi-frame latency tolerance.*

## Table of Contents

- [Executive Summary](#executive-summary)
- [Current State Analysis](#1-current-state-analysis)
- [Industry Best Practices](#2-industry-best-practices)
- [Target Architecture](#3-recommended-target-architecture)
- [Migration Plan](#4-phase-by-phase-migration-plan)
- [Scaling Considerations](#5-scaling-considerations)
- [Performance Projections](#6-performance-projections)
- [Risk Analysis](#7-risk-analysis)
- [Recommendations](#8-concrete-recommendations)
- [Alternatives Considered](#9-alternative-architectures-considered)
- [Conclusion](#10-conclusion)
- [Appendices](#appendix-a-reference-materials)

---

## Executive Summary

This document presents a strategic architecture for achieving "infinite" scaling of physics actors in hdtSMP64 by fully leveraging GPU acceleration with multi-frame latency tolerance. Based on industry best practices from AAA game engines, NVIDIA PhysX documentation, and academic research on GPU physics, we propose a phased migration from the current hybrid CPU/GPU architecture to a predominantly GPU-resident physics pipeline.

> [!IMPORTANT]
> **Key Insight**: The user explicitly accepts 1+ frame latency. This unlocks aggressive pipelining strategies used by production engines that achieve 100,000+ physics bodies at 60 FPS.

---

## 1. Current State Analysis

### 1.1 Existing Pipeline (After Phase 1+2)

```
Frame N Timeline:

  [SyncPrev]      Apply collision manifolds from Frame N-1 (256us)
       |
       v
  [Bones]         CPU: Update bone transforms (~parallel)
       |
       v
  [GPU Launch]    GPU: Vertex skinning + Leaf AABBs (batched)
       |
       v
  [SYNC] ------> GlobalGpuSync (12.2ms) <-- BOTTLENECK
       |                   ^
       |                   |
       v                   +-- Waiting for GPU leaf AABBs
  [Tree Update]   CPU: Tree propagation (104us)
       |
       v
  [Broadphase]    CPU: Pair gathering (~parallel)
       |
       v
  [GPU Launch]    GPU: Narrowphase collision (batched)
       |
       v
  [CPU Solve]     CPU: Bullet constraint solver (2.8ms)
       |
       v
  [Integrate]     CPU: Transform integration
```

### 1.2 Current Bottleneck Analysis

| Zone | Time | % Frame | Notes |
|------|------|---------|-------|
| GlobalGpuSync | 12.2ms | 55% | Waiting for leaf AABBs |
| SolveConstraints | 2.8ms | 13% | Bullet CPU solver |
| TreeUpdate | 104us | 0.5% | Fast, but blocks on GPU |
| SyncPreviousCollisions | 256us | 1% | Already optimized |

> [!WARNING]
> **Root Cause**: The sync exists because CPU tree propagation needs GPU leaf AABBs. The tree update itself is fast (104us), but we wait 12.2ms for GPU to finish before we can even start.

### 1.3 What's Already Working Well

1. **Batched Internal Updates**: 117 graph launches → 5 direct launches
2. **Batched Collision Detection**: 925K kernel launches → ~100 launches
3. **1-Frame Delayed Collision Results**: Applied at start of next frame
4. **Parallel CPU Work**: Bone updates, tree propagation parallelized

---

## 2. Industry Best Practices

### 2.1 AAA Game Engine Patterns

Based on deep research into PhysX, Unreal Engine, Unity, and academic papers:

**Typical Latency Budget**: 2-4 frames for non-gameplay physics (cloth, hair)
- Frame N: Simulate
- Frame N+1: Apply results (minimum latency)
- Frame N+2-4: Acceptable for visual-only physics

**CPU/GPU Split**:
- GPU: Particle integration, collision detection, XPBD constraint solving
- CPU: High-level control flow, spatial acceleration structures (BVH), gameplay queries

**Why Trees Stay on CPU**:
- BVH updates involve complex branching (GPU hates branches)
- Query patterns are irregular (GPU prefers uniform access)
- Trees are relatively cheap (~100us) compared to GPU work

### 2.2 Double/Triple Buffering Strategies

**What to Multi-Buffer**:

| Data | Buffer Count | Rationale |
|------|--------------|-----------|
| Particle positions | Double | GPU writes Frame N, CPU reads Frame N-1 |
| Leaf AABBs | Double | GPU updates Frame N, Tree uses Frame N-1 |
| Collision manifolds | Double | GPU produces Frame N, Solver uses Frame N-1 |
| Tree structure | Single | CPU-only, updated each frame |
| Bone transforms | Single | Input from game, not physics output |

**Triple Buffering**: Generally not needed. Double buffering with 1-frame latency is sufficient. Triple buffering adds memory overhead without meaningful benefit for cloth physics.

### 2.3 XPBD vs Bullet Constraint Solver

**XPBD Advantages**:
- Massively parallel (each constraint independent per iteration)
- Mesh-independent convergence (scales to huge systems)
- Material properties decouple from iteration count
- GPU-native (no sequential Gauss-Seidel dependency)

**Bullet Limitations**:
- Sequential Gauss-Seidel (inherently serial)
- Convergence depends on iteration count
- CPU-optimized, not GPU-friendly

**Verdict**: For "infinite" scaling, XPBD on GPU is the only viable path. However, this is a major architectural change. Bullet remains adequate for <100 constraints if we fix the sync bottleneck first.

---

## 3. Recommended Target Architecture

### 3.1 Optimal Pipeline (Full GPU-Resident)

```
Frame N:

  CPU Thread                          GPU Stream
  ----------                          ----------
  [Bones] --------------------------> [Upload Bones]
                                           |
                                           v
                                      [Vertex Skinning]
                                           |
                                           v
                                      [Leaf AABBs N]
                                           |
                                           v
                                      [Tree Update N] <-- GPU tree propagation
                                           |
                                           v
                                      [Broadphase N]
                                           |
                                           v
                                      [Narrowphase N]
                                           |
                                           v
                                      [XPBD Solve N]
                                           |
                                           v
  [Download] <------------------------[Final Positions N]
       |
       v
  [Render Frame N+1 or N+2]
```

**Key Differences**:
1. Tree propagation moves to GPU (eliminates 12.2ms sync)
2. Broadphase moves to GPU (spatial hashing)
3. Constraint solver moves to GPU (XPBD)
4. Only bone uploads and final position downloads cross PCIe

### 3.2 Practical Near-Term Architecture (Phase 3)

Given Bullet integration constraints, here's an achievable near-term target:

```
Frame N:

  [SyncPrev] ---- Apply collision results from Frame N-2 (double buffered)
       |
       v
  [Bones] ------> Upload to GPU
       |
       v
  [GPU ASYNC] --> Vertex Skinning + Leaf AABBs (Frame N)
       |                    \
       v                     +---> Tree Update uses N-1 leaf AABBs (STALE)
  [Tree N-1] ---> CPU tree propagation (uses previous frame's leaves)
       |
       v
  [Broadphase] -> CPU pair gathering (on N-1 trees)
       |
       v
  [GPU Launch] -> Narrowphase collision (Frame N pairs)
       |
       v
  [CPU Solve] --> Bullet constraint solver (2.8ms)
       |
       v
  [Integrate] --> CPU transform integration
```

**Trade-off**: We use Frame N-1 leaf AABBs for tree propagation. This adds 1 frame latency to collision detection (now 2-frame total), but eliminates the 12.2ms sync entirely.

---

## 4. Phase-by-Phase Migration Plan

> [!TIP]
> ### Phase 3: Eliminate GlobalGpuSync (IMMEDIATE)
> This is the highest-impact optimization with the lowest effort. Implement first.

### Phase 3: Eliminate GlobalGpuSync

**Goal**: Remove 12.2ms sync by using stale (N-1) leaf AABBs for tree updates

**Implementation**:
```cpp
// Current (blocking):
CudaInterface::instance()->synchronize();  // 12.2ms
tree.propagateLeafChanges();  // Uses fresh AABBs

// New (async):
// Don't sync here - tree uses PREVIOUS frame's leaf AABBs
tree.propagateLeafChanges();  // Uses stale N-1 AABBs
// GPU continues working on current frame's leaf AABBs async
```

**What Changes**:
1. Store leaf AABB buffer per frame (double buffer)
2. Tree update uses previous frame's buffer
3. GPU writes to current frame's buffer (no sync needed)
4. Swap buffers at frame boundary

**Expected Gain**: 12.2ms → ~0ms (save 55% of frame time)

**Trade-off**: Collision detection now 2 frames delayed (was 1). Acceptable per user constraints.

**Files to Modify**:
- `hdtCudaInterface.cpp/h`: Double-buffer leaf AABB storage
- `hdtDispatcher.cpp`: Remove sync, use previous frame's AABBs
- `hdtSkinnedMeshShape.cpp`: Accept external AABB buffer for tree update

### Phase 4: GPU Tree Propagation (MEDIUM - 4 weeks)

**Goal**: Move tree propagation to GPU, eliminating CPU/GPU data transfer

**Implementation Strategy**:
- Use GPU-friendly tree representation (BVH stored in arrays)
- Bottom-up propagation kernel (parallel per level)
- Keep tree structure on GPU permanently

**Reference**: NVIDIA LBVH (Linear BVH) construction algorithms

**Expected Gain**: Eliminate CPU tree overhead, enable larger actor counts

### Phase 5: GPU Broadphase (MEDIUM - 4 weeks)

**Goal**: Move pair gathering to GPU

**Implementation Options**:

1. **Spatial Hash Grid** (Recommended):
   - Uniform grid with cell lists
   - Each cell is a GPU buffer of body indices
   - Collision pair generation is embarrassingly parallel
   - Works well for uniform-sized objects (cloth particles)

2. **BVH Traversal**:
   - More complex but handles size variation
   - Required if mixing large/small colliders

**Expected Gain**: Scales to millions of potential pairs without CPU bottleneck

### Phase 6: XPBD Constraint Solver (LARGE - 8+ weeks)

**Goal**: Replace Bullet solver with GPU-native XPBD

**Why XPBD**:
- Position-based (directly update positions, no force integration)
- Parallel per iteration (each constraint independent)
- Compliance parameters decouple from iteration count
- Perfect for cloth/hair (designed for this use case)

**Implementation Considerations**:
1. Constraint coloring for parallel updates (avoid race conditions)
2. Multigrid acceleration for global convergence (MGPBD)
3. Material parameter migration from Bullet

**Reference Implementation**: [XPBD Paper](https://matthias-research.github.io/pages/publications/XPBD.pdf)

**Expected Gain**: Linear scaling with constraint count, 10x+ actor capacity

### Phase 7: Full GPU Residency (OPTIONAL - 12+ weeks)

**Goal**: Keep all physics data GPU-resident, minimal PCIe traffic

**What Stays on GPU**:
- Particle positions (only download for rendering)
- Tree structures
- Collision manifolds
- Constraint data

**What Crosses PCIe**:
- Bone transforms (upload each frame from game)
- Final vertex positions (download for rendering)
- Configuration changes (infrequent)

**Expected Gain**: PCIe bandwidth no longer limits scaling

---

## 5. Scaling Considerations

### 5.1 What Limits Scaling?

| Factor | Limit | Mitigation |
|--------|-------|------------|
| GPU Compute | High (~100K bodies) | Already batched |
| GPU Memory | High (~1M bodies) | Careful buffer management |
| PCIe Bandwidth | Medium (~50K bodies) | GPU residency |
| CPU Tree Updates | Low (~10 actors) | Phase 3 eliminates |
| Bullet Solver | Low (~1K constraints) | Phase 6 XPBD |

### 5.2 LOD Strategies

For distant actors:
1. **Physics LOD 0** (Near): Full simulation, all constraints
2. **Physics LOD 1** (Medium): Reduced constraint iterations
3. **Physics LOD 2** (Far): Rigid body only, no cloth
4. **Physics LOD 3** (Very Far): Skinned mesh only, no physics

Implementation: Distance-based skeleton priority already exists in `ActorManager`.

### 5.3 Culling Strategies

- **View Frustum**: Skip physics for off-screen actors (risky - pop-in)
- **Occlusion**: Skip physics for fully occluded actors (complex)
- **Distance**: Already implemented via skeleton priority
- **Activity**: Skip physics for stationary actors (detect velocity)

---

## 6. Performance Projections

### 6.1 Current vs Target

| Configuration | Current | Phase 3 | Phase 6 |
|---------------|---------|---------|---------|
| 10 skeletons | 15ms | 3ms | 1.5ms |
| 50 skeletons | OOM | 15ms | 5ms |
| 100 skeletons | OOM | 30ms | 8ms |
| 500 skeletons | OOM | OOM | 20ms |

### 6.2 Frame Budget Allocation (Target)

At 60 FPS, we have 16.67ms per frame.

| Component | Budget | Notes |
|-----------|--------|-------|
| Physics GPU | 8ms | Main simulation |
| Physics CPU | 2ms | Bone transforms, game hooks |
| Rendering | 6ms | Not our concern |
| Headroom | 0.67ms | Safety margin |

### 6.3 Latency Impact

| Pipeline Stage | Frames Delayed | Impact |
|----------------|----------------|--------|
| Collision Detection | 2 | Imperceptible at 60fps |
| Constraint Solving | 1 | Minor "floatiness" |
| Final Positions | 1 | Sync'd with render |

Total input-to-visual: ~3 frames (50ms at 60fps). Acceptable for cloth/hair physics.

---

## 7. Risk Analysis

### 7.1 Technical Risks

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| 2-frame collision latency causes artifacts | Low | Medium | Test extensively; user already accepts |
| XPBD doesn't match Bullet behavior | Medium | High | Preserve Bullet as fallback |
| GPU memory fragmentation | Medium | Medium | Pool allocators, CUDA graphs |
| PCIe bandwidth saturated | Low | High | GPU residency (Phase 7) |

### 7.2 Compatibility Risks

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Existing mods rely on frame-accurate physics | Low | Medium | Configuration option for latency |
| Determinism requirements | Medium | Low | Document non-determinism |
| Low-end GPU support | Medium | Medium | CPU fallback always available |

---

## 8. Concrete Recommendations

### 8.1 Immediate Action (Phase 3)

1. **Implement double-buffered leaf AABBs**
   - Add `m_leafAabbBuffer[2]` to shape classes
   - Add `m_currentLeafBuffer` index
   - Tree update uses `m_leafAabbBuffer[1-m_currentLeafBuffer]`
   - GPU writes to `m_leafAabbBuffer[m_currentLeafBuffer]`

2. **Remove GlobalGpuSync**
   - Delete the sync in `dispatchAllCollisionPairs`
   - Tree update no longer needs fresh GPU data

3. **Swap buffers at frame end**
   - `m_currentLeafBuffer = 1 - m_currentLeafBuffer;`

**Expected Effort**: 1-2 weeks
**Expected Gain**: 12.2ms saved (55% of frame time)

### 8.2 Medium-Term (Phase 4-5)

1. Port tree propagation to GPU using bottom-up kernel
2. Implement spatial hash grid for GPU broadphase
3. Profile and optimize kernel occupancy

**Expected Effort**: 6-8 weeks
**Expected Gain**: Additional 2-3ms, enables 50+ skeletons

### 8.3 Long-Term (Phase 6-7)

1. Implement XPBD constraint solver
2. Migrate constraint data to GPU
3. Full GPU residency for scaling

**Expected Effort**: 12-16 weeks
**Expected Gain**: 500+ skeleton capacity

---

## 9. Alternative Architectures Considered

### 9.1 Async Compute Overlap

**Idea**: Run physics compute while rendering previous frame.

**Verdict**: Not helpful for hdtSMP64 because:
- Physics must complete before rendering current frame (bones)
- We don't control Skyrim's render pipeline
- Would require significant engine integration

### 9.2 Frame Generation Integration

**Idea**: Let DLSS/FSR3 generate intermediate frames, reduce physics frequency.

**Verdict**: Interesting but:
- Skyrim doesn't support frame generation natively
- Would require modifying ENB/ReShade integration
- Out of scope for physics mod

### 9.3 Neural Network Physics

**Idea**: Train ML model to predict cloth motion.

**Verdict**: Promising for future but:
- Requires training data per mesh
- Non-deterministic
- Research-level, not production-ready

---

## 10. Conclusion

The 12.2ms GlobalGpuSync is the critical bottleneck preventing hdtSMP64 from scaling. By accepting 2-frame collision latency (user already accepts 1-frame), we can eliminate this sync entirely using double-buffered leaf AABBs.

**Phase 3 alone should achieve 10+ skeletons under 10ms budget.**

For "infinite" scaling (100+ skeletons), the full GPU-resident architecture with XPBD solver is required. This is a significant undertaking but follows well-established patterns from AAA engines.

**Recommended Path**:
1. Phase 3 (immediate): Double-buffer leaf AABBs → eliminate sync
2. Validate Phase 3 meets 10 skeleton target
3. If more scaling needed: Phase 4-5 (GPU trees + broadphase)
4. If still more needed: Phase 6 (XPBD)

The architecture is modular - each phase provides value independently.

---

## Appendix A: Reference Materials

- [NVIDIA PhysX Threading Model](https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/Threading.html)
- [XPBD: Position-Based Simulation Methods](https://matthias-research.github.io/pages/publications/XPBD.pdf)
- [GPU-Accelerated Cloth Simulation](https://www.daydreamsoft.com/blog/gpu-accelerated-cloth-hair-and-soft-body-simulation-models)
- [CUDA Async Compute Best Practices](https://developer.nvidia.com/blog/advanced-api-performance-async-compute-and-overlap/)
- [Frame Latency Analysis](https://developer.nvidia.com/blog/understanding-and-measuring-pc-latency/)

## Appendix B: Glossary

- **XPBD**: Extended Position-Based Dynamics
- **BVH**: Bounding Volume Hierarchy
- **LBVH**: Linear BVH (GPU-friendly format)
- **Leaf AABB**: Axis-Aligned Bounding Box at tree leaf (individual colliders)
- **PCIe**: PCI Express bus (CPU-GPU data transfer)
- **Occupancy**: Percentage of GPU threads active vs maximum

---

<div align="center">

*For implementation status, see [ARCHITECTURE.md](ARCHITECTURE.md)*

</div>
