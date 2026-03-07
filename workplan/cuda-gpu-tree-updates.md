# Phase 3: Double-Buffer Leaf AABBs

**Branch:** `feature/cuda-batched-collisions`
**Status:** PLANNED
**Goal:** Eliminate 12.2ms GlobalGpuSync by using stale (N-1) leaf AABBs
**Effort:** 2 weeks
**Expected Gain:** 55% frame time reduction (12.2ms → ~0ms)

---

## Executive Summary

The expert panel review identified that the original plan (GPU tree propagation with binary BVH) was incompatible with the existing n-ary tree structure. Deep analysis revealed a much simpler solution:

**The sync exists because CPU tree propagation needs GPU leaf AABBs. But we accept 1-frame latency. So use PREVIOUS frame's leaf AABBs - no sync needed.**

This is the industry-standard approach used by AAA engines for cloth/hair physics.

---

## Problem Statement

### Current Pipeline (After Phase 1+2)

```
Frame N:
  GPU: Vertex skinning → Leaf AABBs
              │
           SYNC ← 12.2ms (55% of frame time!)
              │
  CPU: Tree propagation (104μs) ← needs fresh leaf AABBs
              │
  CPU: Broadphase pair gathering
              │
  GPU: Narrowphase collision
              │
  CPU: Constraint solver
```

### Root Cause

The tree propagation code calls `updateBoundingBoxes()` which reads GPU-computed leaf AABBs:

```cpp
// hdtCudaInterface.cpp - CudaColliderTree::update()
void update() {
    updateBoundingBoxes(*m_tree, m_nodeAabbs);  // Reads from GPU buffer
}
```

This requires the GPU to have finished computing leaf AABBs, hence the 12.2ms sync.

---

## Solution: Double-Buffer Leaf AABBs

### Key Insight

- Tree propagation (104μs) is CPU-bound and fast
- It only needs leaf AABBs, not the actual vertex positions
- We already accept 1-frame latency on collision results
- Adding 1 more frame of AABB latency is imperceptible for cloth/hair

### New Pipeline

```
Frame N:
  GPU: Vertex skinning → Leaf AABBs [write to buffer N]
              │                              │
              │ (NO SYNC)                    │ async copy
              │                              v
  CPU: Tree propagation ← uses buffer N-1 (stale AABBs)
              │
  CPU: Broadphase (uses N-1 tree)
              │
  GPU: Narrowphase (uses N vertices, N-1 pairs)
              │
  CPU: Constraint solver
              │
  [Frame end: swap buffers]
```

### Latency Analysis

| Stage | Before | After | Impact |
|-------|--------|-------|--------|
| Leaf AABBs | Frame N | Frame N-1 | +1 frame |
| Tree AABBs | Frame N | Frame N-1 | +1 frame |
| Broadphase pairs | Frame N | Frame N-1 | +1 frame |
| Narrowphase collision | Frame N | Frame N | No change |
| Collision manifolds | Frame N-1 | Frame N-1 | No change |
| **Total collision latency** | **1 frame** | **2 frames** | Acceptable |

At 60 FPS, 2 frames = 33ms input-to-collision. For cloth/hair physics, this is imperceptible.

---

## Implementation

### Data Structure Changes

```cpp
// hdtCudaInterface.h - Add to CudaColliderTree or CudaPerVertexShape/CudaPerTriangleShape

class CudaColliderTree {
    // Existing
    CudaBuffer<cuAabb, Aabb> m_nodeAabbs;

    // NEW: Double-buffered leaf AABBs for async pipeline
    CudaBuffer<cuAabb, Aabb> m_leafAabbBuffers[2];
    int m_currentLeafBuffer = 0;  // GPU writes here
    int m_readLeafBuffer = 1;     // CPU reads here (previous frame)

    // Host-side pinned buffer for async download
    std::vector<Aabb> m_hostLeafAabbs;
};
```

### Task 1: Add Double-Buffer Infrastructure
**File:** `hdtCudaInterface.h`, `hdtCudaInterface.cpp`
**Effort:** Small

1. Add `m_leafAabbBuffers[2]` to shape classes
2. Add `m_currentLeafBuffer` index
3. Add `m_hostLeafAabbs` pinned host buffer
4. Initialize both buffers in constructor

```cpp
void CudaColliderTree::initDoubleBuffer() {
    size_t leafCount = countLeafNodes();
    m_leafAabbBuffers[0].resize(leafCount);
    m_leafAabbBuffers[1].resize(leafCount);
    m_hostLeafAabbs.resize(leafCount);
    m_currentLeafBuffer = 0;
    m_readLeafBuffer = 1;
}
```

### Task 2: Modify Kernel to Write to Current Buffer
**File:** `hdtCudaCollision.cu`, `hdtCudaInterface.cpp`
**Effort:** Small

Change `kernelBoundingBoxReduce` output to use current buffer:

```cpp
void CudaColliderTree::launchLeafUpdate(cudaStream_t stream) {
    kernelBoundingBoxReduce<<<...>>>(
        m_numNodes,
        m_nodeData.ptr(),
        boundingBoxes,
        m_leafAabbBuffers[m_currentLeafBuffer].ptr()  // Write to current
    );
}
```

### Task 3: Async Copy to Host
**File:** `hdtCudaInterface.cpp`
**Effort:** Small

After kernel launch, queue async copy (non-blocking):

```cpp
void CudaColliderTree::queueLeafDownload(cudaStream_t stream) {
    cudaMemcpyAsync(
        m_hostLeafAabbs.data(),
        m_leafAabbBuffers[m_currentLeafBuffer].ptr(),
        m_hostLeafAabbs.size() * sizeof(Aabb),
        cudaMemcpyDeviceToHost,
        stream
    );
}
```

### Task 4: Tree Update Uses Previous Buffer
**File:** `hdtCudaInterface.cpp`, `hdtSkinnedMeshShape.cpp`
**Effort:** Medium

Modify `updateBoundingBoxes` to use the read buffer (previous frame):

```cpp
void CudaColliderTree::update() {
    // Use PREVIOUS frame's leaf AABBs (no sync needed!)
    updateBoundingBoxes(*m_tree, m_hostLeafAabbs.data());
}
```

### Task 5: Remove GlobalGpuSync
**File:** `hdtDispatcher.cpp`
**Effort:** Small

Delete the sync that was blocking for fresh AABBs:

```cpp
void CollisionDispatcher::dispatchAllCollisionPairs(...) {
    // ... launch internal updates ...

    // REMOVE THIS:
    // CudaInterface::instance()->synchronize();  // Was 12.2ms!

    // Tree update now uses previous frame's AABBs (already downloaded)
    updateTrees();

    // ... gather pairs, launch collisions ...
}
```

### Task 6: Swap Buffers at Frame End
**File:** `hdtDispatcher.cpp` or `hdtSkinnedMeshWorld.cpp`
**Effort:** Small

At end of frame, swap the buffer indices:

```cpp
void CollisionDispatcher::endFrame() {
    // Swap double buffers
    for (auto& shape : m_shapes) {
        shape->swapLeafBuffers();
    }
}

void CudaColliderTree::swapLeafBuffers() {
    m_readLeafBuffer = m_currentLeafBuffer;
    m_currentLeafBuffer = 1 - m_currentLeafBuffer;
}
```

### Task 7: Handle First Frame
**File:** `hdtCudaInterface.cpp`
**Effort:** Small

On first frame, both buffers are empty. Initialize with a sync:

```cpp
void CudaColliderTree::update() {
    if (m_firstFrame) {
        // First frame: must sync to get initial AABBs
        cudaStreamSynchronize(stream);
        m_firstFrame = false;
    }
    updateBoundingBoxes(*m_tree, m_hostLeafAabbs.data());
}
```

---

## Files to Modify

| File | Changes |
|------|---------|
| `hdtCudaInterface.h` | Add double-buffer members to tree classes |
| `hdtCudaInterface.cpp` | Implement buffer management, async copy |
| `hdtCudaCollision.cu` | Point kernel output to current buffer |
| `hdtDispatcher.cpp` | Remove sync, add buffer swap |
| `hdtSkinnedMeshShape.cpp` | Tree update uses host buffer |

---

## Testing Strategy

### Correctness Tests

1. **Verify physics behavior unchanged**
   - Record collision manifold counts before/after
   - Visual inspection of cloth/hair motion
   - No new interpenetration artifacts

2. **Verify 2-frame latency is acceptable**
   - Test with fast-moving objects
   - Test with rapid direction changes
   - Compare to 1-frame latency baseline

### Performance Tests

1. **Profile with Tracy**
   - Verify `GlobalGpuSync` zone eliminated
   - Verify no new sync points introduced
   - Measure total frame time reduction

2. **Scaling test**
   - 10 actors: target <5ms
   - 20 actors: target <10ms
   - 50 actors: target <25ms

---

## Expected Results

| Metric | Before | After | Change |
|--------|--------|-------|--------|
| `GlobalGpuSync` | 12.2ms (19%) | **0ms** | -100% |
| `SyncAndTreeUpdates` | 12.4ms (19%) | 0.2ms (~0%) | -98% |
| Frame time (10 actors) | ~22ms | ~10ms | -55% |
| Collision latency | 1 frame | 2 frames | +1 frame |

---

## Risk Analysis

### Risk 1: 2-Frame Latency Causes Artifacts
**Probability:** Low
**Impact:** Medium
**Mitigation:**
- AABB margins already account for inter-frame motion
- Cloth/hair doesn't move fast enough to notice
- User explicitly accepts latency

### Risk 2: First Frame Edge Case
**Probability:** Low
**Impact:** Low
**Mitigation:**
- Sync on first frame only
- Subsequent frames are async

### Risk 3: Buffer Swap Race Condition
**Probability:** Medium
**Impact:** High
**Mitigation:**
- Ensure async copy completes before swap
- Use CUDA events to track copy completion
- Swap happens at well-defined frame boundary

---

## Success Criteria

### Performance
- [ ] `GlobalGpuSync` eliminated (0ms)
- [ ] Total frame time reduced by 50%+
- [ ] 10 actors under 10ms budget

### Correctness
- [ ] No visual physics artifacts
- [ ] No new crashes or instability
- [ ] Collision manifold counts within 5% of baseline

### Code Quality
- [ ] Tracy instrumentation on buffer operations
- [ ] Clean buffer lifecycle management
- [ ] No memory leaks on buffer resize

---

## Relationship to Other Phases

This replaces the original "GPU Tree Updates" plan which was incompatible with the existing n-ary tree structure.

**Previous plan (ABANDONED):**
- Move tree propagation to GPU
- Required binary BVH conversion
- 4+ weeks effort, high risk

**New plan (THIS DOCUMENT):**
- Keep tree propagation on CPU
- Double-buffer leaf AABBs
- 2 weeks effort, low risk
- Same performance benefit (eliminate sync)

For further scaling beyond 50+ actors, see:
- `cuda-xpbd-solver.md` - Phase 6: GPU constraint solver
- `cuda-optimization-roadmap.md` - Full pipeline overview
