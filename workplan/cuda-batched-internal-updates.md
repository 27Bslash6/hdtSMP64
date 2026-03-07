# CUDA Batched Internal Updates Implementation Plan

**Branch:** `feature/cuda-batched-collisions`
**Status:** COMPLETE
**Goal:** Reduce ~117 graph launches per frame to direct kernel launches (no graphs)

---

## Results Summary (2025-01-09)

### Phase 2 Complete: Batched Internal Updates

| Metric | Before (c2) | After (c3) | Change |
|--------|-------------|------------|--------|
| `GraphLaunch` | 37.5% (565K calls) | **ELIMINATED** | -37.5% |
| `LaunchInternalUpdates` | 6.4% | 3.8% | -40% |
| `BatchedInternalUpdates` | N/A | 3.8% | +3.8% |
| `BatchedBonesToDevice` | N/A | 1.2% | +1.2% |
| `BatchedKernelLaunches` | N/A | 2.5% | +2.5% |

**Phase 2 delivers elimination of all per-body graph launches.**

### New Bottleneck Identified: GlobalGpuSync

| Metric | c3 Value | Issue |
|--------|----------|-------|
| `GlobalGpuSync` | 19.0% (12.2ms) | CPU blocking on GPU tree updates |
| `SyncAndTreeUpdates` | 19.2% (12.4ms) | Total sync + tree overhead |
| `UpdateAabbs` | 7.2% (4.6ms) | Per-bone AABB computation |

**Next optimization target:** GPU tree updates (see `cuda-gpu-tree-updates.md`)

---

## Problem Statement (SOLVED)

### Performance Before (c2 trace)

| Metric | Value | Issue |
|--------|-------|-------|
| `GraphLaunch` | 37.5% of frame (565K calls) | 130us per graph launch |
| Bodies per frame | ~117 | Each gets its own graph launch |
| **Overhead per frame** | **~15ms** | 117 x 130us = pure API overhead |

### Root Cause (FIXED)

Each body had a captured CUDA graph containing:
1. `m_bones.toDevice()` - Bone transform transfer
2. `kernelBodyUpdate` - Transform vertices by bones
3. `kernelPerVertexUpdate` + `kernelBoundingBoxReduce` - Update vertex colliders + BVH
4. `kernelPerTriangleUpdate` + `kernelBoundingBoxReduce` - Update triangle colliders + BVH

**Fixed by:** Batched direct kernel launches instead of per-body graphs.

---

## Architecture (IMPLEMENTED)

### Old Flow (Per-Body Graphs) - REMOVED

```
LaunchInternalUpdates (parallel_for_each over bodies)
    +-- For EACH body:
            +-- cuGraphLaunch(body->m_graphExec)
                    |-- memcpy: bones to device
                    |-- kernelBodyUpdate<<<>>>
                    |-- kernelPerVertexUpdate<<<>>>
                    |-- kernelBoundingBoxReduce<<<>>>
                    |-- kernelPerTriangleUpdate<<<>>>
                    +-- kernelBoundingBoxReduce<<<>>>
```

### New Flow (Batched Direct Launches) - IMPLEMENTED

```
Phase 1: GATHER (CPU, parallel)
    +-- For each body:
            +-- updateBones() - prepare bone transforms on CPU
            +-- addInternalUpdate() - add to batch

Phase 2: UPLOAD (batched)
    +-- BatchedBonesToDevice: all bone data in single stream

Phase 3: COMPUTE (direct kernel launches)
    +-- BatchedKernelLaunches:
            |-- kernelBodyUpdate (per body)
            |-- kernelPerVertexUpdate (per body)
            +-- kernelPerTriangleUpdate (per body)

Phase 4: SYNC (global)
    +-- GlobalGpuSync: wait for tree updates to complete
```

---

## Implementation Details

### BatchedInternalUpdateManager

```cpp
class BatchedInternalUpdateManager {
public:
    void beginBatch();
    void addBody(std::shared_ptr<CudaBody> body,
                 std::shared_ptr<CudaPerVertexShape> vertexShape,
                 std::shared_ptr<CudaPerTriangleShape> triangleShape);
    void launchBatch();

private:
    std::vector<InternalUpdateWork> m_workUnits;
    void* m_batchStream = nullptr;  // Dedicated stream for batched ops
    int m_batchStreamDevice = -1;
};
```

### Key Fix: Dedicated Batch Stream

The original implementation crashed due to CUDA stream context issues:
- Per-body streams were created during `parallel_for_each` on worker threads
- Using these streams from the main thread caused driver crashes

**Solution:** Created dedicated `m_batchStream` lazily on main thread:
```cpp
void BatchedInternalUpdateManager::ensureBatchStream() {
    if (m_batchStream == nullptr) {
        cudaStreamCreate(&m_batchStream);
        m_batchStreamDevice = CudaInterface::currentDevice;
    }
}
```

---

## Files Modified

| File | Changes |
|------|---------|
| `hdtCudaInterface.h` | Added `BatchedInternalUpdateManager`, `InternalUpdateWork` |
| `hdtCudaInterface.cpp` | Implemented batched internal update API |
| `hdtDispatcher.cpp` | Use `beginInternalUpdateBatch()` / `launchInternalUpdateBatch()` |

---

## Success Criteria - ALL MET

### Performance
- [x] `GraphLaunch` eliminated from profile (0%)
- [x] `LaunchInternalUpdates` reduced from 6.4% to 3.8% (-40%)
- [x] Per-body graph launch overhead eliminated (~15ms -> 0)

### Correctness
- [x] Physics behavior unchanged
- [x] No crashes with variable body counts
- [x] Handles body add/remove mid-session
- [x] TOCTOU race conditions fixed (validity flags, mutex protection)

### Code Quality
- [x] Clean separation between gather/upload/launch phases
- [x] Tracy instrumentation on all phases
- [x] Thread-safe with proper mutex usage

---

## Lessons Learned

1. **CUDA streams have thread affinity** - Don't use streams created on worker threads from the main thread
2. **Shared_ptr lifetime != object member lifetime** - CudaBody being alive doesn't mean SkinnedMeshBody members are valid
3. **Multi-layered defense** - Use mutex + validity flag + null checks for robust lifetime management
4. **Measure before optimizing** - Graph launch overhead (not graph execution) was the real cost
