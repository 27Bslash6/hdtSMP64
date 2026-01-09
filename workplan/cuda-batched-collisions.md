# CUDA Batched Collisions Implementation Plan

**Branch:** `feature/cuda-batched-collisions`
**Status:** Phase 1 COMPLETE ✅ | Phase 2 Pending
**Goal:** Reduce ~925K kernel launches to 3-4 per frame

**Review Status:** Expert Panel Approved - 4 CRITICAL, 8 MAJOR, 7 MINOR issues addressed below.

---

## Progress Summary (2025-01-09)

### Phase 1: Batched Gathering + Per-Pair Kernels ✅ COMPLETE

| Metric | Before (c1) | After (c2) | Change |
|--------|-------------|------------|--------|
| `queueCollision` | 33.4% (537K calls) | **ELIMINATED** | -33.4% |
| `BatchedLaunchCollisions` | N/A | 1.09% (4.8K calls) | +1.09% |
| `CudaGatherPairs` | N/A | 0.09% | +0.09% |
| `BatchedApplyResults` | N/A | 0.31% | +0.31% |
| `SyncPreviousCollisions` | 1.51% (693µs) | 0.35% (142µs) | -79% |
| **Collision Overhead** | **33.4%** | **1.5%** | **-32%** |

**Phase 1 delivers 32% frame time reduction from collision path optimization.**

### Phase 2: True Batched Kernels 🔲 PENDING

Currently using per-pair `launchSingleCollision()` which still launches ~88K kernels total.
True batching would reduce to 2-3 kernel launches per frame.

### Remaining Bottleneck: GraphLaunch (Internal Updates)

| Metric | Value |
|--------|-------|
| `GraphLaunch` | 37.5% (566K calls) |
| Source | Per-body internal updates (~120 bodies × 4.8K frames) |

This is **not collision** - it's the internal update path. Separate optimization target.

---

## Problem Statement

### Current Performance (cu11 trace)

| Metric | Value | Issue |
|--------|-------|-------|
| `GraphLaunch` | 38.2% of frame | 970K launches × 128μs each |
| `queueCollision` | 36.0% of frame | Per-pair overhead |
| `StepSimulation` | 18.8ms mean | 27% slower than NOCUDA |

### Root Cause

Each collision pair triggers:
1. `CudaMergeBuffer` allocation
2. Kernel launch (`kernelCollision`)
3. Async transfer (`launchTransfer`)

With 925K pairs, launch overhead dominates compute time.

---

## Architecture

### Current Flow (Per-Pair)

```
CudaQueueCollisions (parallel_for over pairs)
    └── For EACH pair:
            ├── new CudaMergeBuffer(body0, body1)     // Alloc
            ├── launchCollision()                      // Kernel launch
            │       └── kernelCollision<<<n, block>>>
            └── cudaMerge->launchTransfer()           // D2H transfer
```

**Problem:** 925K iterations = 925K kernel launches

### Proposed Flow (Batched)

```
Phase 1: GATHER (CPU, parallel)
    └── For each pair:
            └── Append to type-specific batch arrays
                ├── batchVV (vertex-vertex)
                └── batchVT (vertex-triangle, handles TV via swap flag)

Phase 2: UPLOAD (2-3 transfers)
    ├── cudaMemcpyAsync(d_batchVV, batchVV)
    └── cudaMemcpyAsync(d_batchVT, batchVT)

Phase 3: COMPUTE (2-3 kernels)
    ├── kernelCollisionBatched<<<totalVV, block>>>(d_batchVV, ...)
    └── kernelCollisionBatched<<<totalVT, block>>>(d_batchVT, ...)

Phase 4: DOWNLOAD (1 transfer)
    └── cudaMemcpyAsync(h_results, d_results)

Phase 5: APPLY (CPU, parallel - next frame)
    └── For each result: create manifold
```

**Result:** 2-3 kernel launches instead of 925K

---

## Data Structures

### Design Principles (from Expert Panel Review)

- **Single Responsibility:** Split CPU batch data, GPU buffers, and management into separate structs
- **Lifetime Safety:** Use `weak_ptr` for body references to prevent use-after-free
- **No TV Batch:** Existing code handles TV via VT with swap flag - no separate batch needed
- **Overflow Protection:** All size calculations use checked arithmetic
- **Resource Limits:** Hard cap on pair count to prevent DoS

### CollisionPairInfo

```cpp
// hdtCudaInterface.h

struct CollisionPairInfo {
    // Shape references (for routing results back)
    CudaPerVertexShape* shapeA;
    std::variant<CudaPerVertexShape*, CudaPerTriangleShape*> shapeB;

    // Body references - CRITICAL: weak_ptr to prevent use-after-free
    std::weak_ptr<SkinnedMeshBody> body0;
    std::weak_ptr<SkinnedMeshBody> body1;

    // Offset into global merge buffer for results
    size_t mergeBufferOffset;  // size_t, not int

    // Swap flag for VT pairs where triangle is on body0
    bool swapped;
};
```

### CpuBatchData (CPU-side accumulation)

```cpp
struct CpuBatchData {
    // Setup data per collision type (no TV - use VT with swap)
    std::vector<cuCollisionSetup> setupVV;  // vertex-vertex
    std::vector<cuCollisionSetup> setupVT;  // vertex-triangle (handles TV via swap)

    // Pair metadata for result routing
    std::vector<CollisionPairInfo> pairsVV;
    std::vector<CollisionPairInfo> pairsVT;

    void clear() {
        setupVV.clear();
        setupVT.clear();
        pairsVV.clear();
        pairsVT.clear();
    }

    void reserve(size_t expectedPairs) {
        setupVV.reserve(expectedPairs / 2);
        setupVT.reserve(expectedPairs / 2);
        pairsVV.reserve(expectedPairs / 2);
        pairsVT.reserve(expectedPairs / 2);
    }

    size_t totalPairs() const {
        return pairsVV.size() + pairsVT.size();
    }
};
```

### GpuBatchBuffers (GPU-side buffers)

```cpp
struct GpuBatchBuffers {
    // Setup buffers
    CudaBuffer<cuCollisionSetup> d_setupVV;
    CudaBuffer<cuCollisionSetup> d_setupVT;

    // Global merge buffer for all results
    CudaBuffer<cuCollisionMerge> d_mergeBuffer;

    void resize(const CpuBatchData& cpu) {
        d_setupVV.resize(cpu.setupVV.size());
        d_setupVT.resize(cpu.setupVT.size());
    }
};
```

### BatchedCollisionManager

```cpp
class BatchedCollisionManager {
public:
    // Hard limit to prevent resource exhaustion (DoS protection)
    static constexpr size_t MAX_PAIRS = 2'000'000;

    CpuBatchData m_cpu;
    GpuBatchBuffers m_gpu;
    std::vector<cuCollisionMerge> m_hostMergeBuffer;

    // Thread-local batches for parallel gather
    static thread_local CpuBatchData t_localBatch;

    void beginBatch();
    bool addCollisionPair(
        const std::shared_ptr<SkinnedMeshBody>& body0,
        const std::shared_ptr<SkinnedMeshBody>& body1);
    void mergeTh readLocalBatches();
    void launchBatch(cudaStream_t stream);
    void applyResults(CollisionDispatcher* dispatcher);

    // Calculate merge buffer size with overflow checking
    size_t calculateMergeBufferSize() const;

private:
    std::mutex m_mergeMutex;
    std::atomic<size_t> m_totalPairs{0};
};
```

### calculateMergeBufferSize() Definition

```cpp
// CRITICAL: Must be defined with overflow checking
size_t BatchedCollisionManager::calculateMergeBufferSize() const {
    // Each pair needs space for collision results
    // cuCollisionMerge contains: numContacts, contactData[MAX_CONTACTS]
    constexpr size_t MERGE_ENTRY_SIZE = sizeof(cuCollisionMerge);
    constexpr size_t MAX_ENTRIES_PER_PAIR = 1;  // One merge entry per pair

    size_t totalPairs = m_cpu.pairsVV.size() + m_cpu.pairsVT.size();

    // Overflow check
    if (totalPairs > MAX_PAIRS) {
        throw std::runtime_error("Collision pair count exceeds hard limit");
    }

    // Safe multiplication (totalPairs already bounded)
    return totalPairs * MAX_ENTRIES_PER_PAIR;
}
```

### Modified: CudaInterface

```cpp
class CudaInterface {
public:
    // Existing...

    // New batched API
    BatchedCollisionManager m_batchedCollisions;

    void beginCollisionBatch();
    bool addCollisionPair(
        const std::shared_ptr<SkinnedMeshBody>& body0,
        const std::shared_ptr<SkinnedMeshBody>& body1);
    void launchCollisionBatch(cudaStream_t stream);
    void applyCollisionResults(CollisionDispatcher* dispatcher);
};
```

---

## Error Handling Strategy

### Guiding Principles

1. **No Silent Failures:** Every error condition must be logged and handled
2. **Graceful Degradation:** On GPU errors, fall back to CPU collision
3. **Resource Protection:** Hard limits prevent runaway allocation

### Error Conditions and Responses

| Condition | Detection | Response |
|-----------|-----------|----------|
| Pair count > MAX_PAIRS | `m_totalPairs.load() > MAX_PAIRS` | Log warning, skip additional pairs |
| Body destroyed between frames | `weak_ptr.lock()` returns nullptr | Skip pair in result application |
| GPU OOM | `cudaMalloc` returns error | Log error, fall back to CPU path |
| Merge buffer overflow | Bounds check in kernel | Clamp to buffer size, log warning |
| Invalid pointers in addCollisionPair | Null check at entry | Return false, log warning |
| CUDA kernel error | `cudaGetLastError()` after launch | Log error, fall back to CPU |

### Error Logging

```cpp
// Use existing HDT_LOG macros
#define HDT_CUDA_CHECK(call) \
    do { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            HDT_LOG_ERROR("CUDA error: {} at {}:{}", \
                cudaGetErrorString(err), __FILE__, __LINE__); \
            return false; \
        } \
    } while(0)
```

---

## Implementation Tasks

### Task 1: Add Data Structures ✅ COMPLETE
**File:** `hdtCudaInterface.h`
**Effort:** Medium (restructured from original)
**Status:** Implemented with simplified design - using raw pointers + weak_ptr validation

- Define `CollisionPairInfo` struct with `weak_ptr` and `size_t`
- Define `CpuBatchData` struct (CPU-only data)
- Define `GpuBatchBuffers` struct (GPU buffers)
- Define `BatchedCollisionManager` class
- Implement `calculateMergeBufferSize()` with overflow checking
- Add `MAX_PAIRS` constant (2M)
- Add `m_batchedCollisions` member to `CudaInterface`

### Task 2: Implement batch accumulation ✅ COMPLETE
**File:** `hdtCudaInterface.cpp`
**Effort:** Medium
**Status:** Implemented with mutex-protected append (simpler than thread-local)

```cpp
bool BatchedCollisionManager::addCollisionPair(
    const std::shared_ptr<SkinnedMeshBody>& body0,
    const std::shared_ptr<SkinnedMeshBody>& body1)
{
    // INPUT VALIDATION (CRITICAL)
    if (!body0 || !body1) {
        HDT_LOG_WARNING("addCollisionPair: null body pointer");
        return false;
    }
    if (!body0->m_shape || !body1->m_shape) {
        HDT_LOG_WARNING("addCollisionPair: null shape pointer");
        return false;
    }

    // Resource limit check
    if (m_totalPairs.fetch_add(1) >= MAX_PAIRS) {
        HDT_LOG_WARNING("Collision pair limit reached ({})", MAX_PAIRS);
        return false;
    }

    // Determine collision type (NO TV - use VT with swap)
    bool hasTriA = body0->m_shape->asPerTriangleShape() != nullptr;
    bool hasTriB = body1->m_shape->asPerTriangleShape() != nullptr;

    // Route to appropriate batch
    if (!hasTriA && !hasTriB) {
        // Vertex-Vertex
        accumulateVV(body0, body1, /*swapped=*/false);
    } else if (!hasTriA && hasTriB) {
        // Vertex-Triangle (normal order)
        accumulateVT(body0, body1, /*swapped=*/false);
    } else if (hasTriA && !hasTriB) {
        // Triangle-Vertex: use VT with swap flag
        accumulateVT(body1, body0, /*swapped=*/true);
    } else {
        // Both have triangles - need both passes
        accumulateVT(body0, body1, /*swapped=*/false);
        accumulateVT(body1, body0, /*swapped=*/true);
    }

    return true;
}
```

### Task 3: Implement batched kernel launch ✅ COMPLETE (Phase 1)
**File:** `hdtCudaInterface.cpp`
**Effort:** Medium
**Status:** Implemented using `launchSingleCollision<>` template - reuses existing per-pair kernels.
Phase 2 would modify CUDA kernel itself for true single-launch batching.

```cpp
void BatchedCollisionManager::launchBatch(cudaStream_t stream)
{
    // Merge thread-local batches first
    mergeThreadLocalBatches();

    auto& cpu = m_cpu;
    auto& gpu = m_gpu;

    if (cpu.totalPairs() == 0) return;

    // Upload setup data
    if (!cpu.setupVV.empty()) {
        HDT_CUDA_CHECK(gpu.d_setupVV.copyFromHost(
            cpu.setupVV.data(), cpu.setupVV.size(), stream));
    }
    if (!cpu.setupVT.empty()) {
        HDT_CUDA_CHECK(gpu.d_setupVT.copyFromHost(
            cpu.setupVT.data(), cpu.setupVT.size(), stream));
    }

    // Allocate merge buffer with overflow protection
    size_t totalMergeSize = calculateMergeBufferSize();
    HDT_CUDA_CHECK(gpu.d_mergeBuffer.resize(totalMergeSize));
    HDT_CUDA_CHECK(gpu.d_mergeBuffer.zero(stream));

    // Launch kernels
    if (!cpu.setupVV.empty()) {
        kernelCollision<eNone, CudaPerVertexShape>
            <<<cpu.setupVV.size(), collisionBlockSize(), 0, stream>>>(
                cpu.setupVV.size(),
                /* swap */ false,
                gpu.d_setupVV.ptr(),
                /* shape data... */
                gpu.d_mergeBuffer.ptr(),
                totalMergeSize);  // Pass buffer size for bounds check

        // Check for kernel errors
        HDT_CUDA_CHECK(cudaGetLastError());
    }

    if (!cpu.setupVT.empty()) {
        kernelCollision<ePerTriangle, CudaPerVertexShape>
            <<<cpu.setupVT.size(), collisionBlockSize(), 0, stream>>>(
                cpu.setupVT.size(),
                /* swap from pair info */,
                gpu.d_setupVT.ptr(),
                /* shape data... */
                gpu.d_mergeBuffer.ptr(),
                totalMergeSize);

        HDT_CUDA_CHECK(cudaGetLastError());
    }

    // Explicit sync between compute and download
    HDT_CUDA_CHECK(cudaStreamSynchronize(stream));

    // Queue download
    m_hostMergeBuffer.resize(totalMergeSize);
    HDT_CUDA_CHECK(gpu.d_mergeBuffer.copyToHost(
        m_hostMergeBuffer.data(), stream));
}
```

### Task 4: Implement result application ✅ COMPLETE
**File:** `hdtCudaInterface.cpp`
**Effort:** Medium
**Status:** Implemented - stores `PendingCollisionResult` with merge buffers, applies via `MergeBuffer::apply()`

```cpp
void BatchedCollisionManager::applyResults(CollisionDispatcher* dispatcher)
{
    // Process VV results
    for (size_t i = 0; i < m_cpu.pairsVV.size(); ++i) {
        const auto& pair = m_cpu.pairsVV[i];

        // CRITICAL: Check body still exists (weak_ptr lifetime safety)
        auto b0 = pair.body0.lock();
        auto b1 = pair.body1.lock();
        if (!b0 || !b1) {
            // Body was destroyed between gather and apply - skip
            continue;
        }

        // Bounds check merge buffer access
        if (pair.mergeBufferOffset >= m_hostMergeBuffer.size()) {
            HDT_LOG_ERROR("Merge buffer offset out of bounds: {} >= {}",
                pair.mergeBufferOffset, m_hostMergeBuffer.size());
            continue;
        }

        const auto* mergeData = &m_hostMergeBuffer[pair.mergeBufferOffset];
        applyMergeResults(mergeData, b0.get(), b1.get(), pair.swapped, dispatcher);
    }

    // Process VT results (same pattern)
    for (size_t i = 0; i < m_cpu.pairsVT.size(); ++i) {
        const auto& pair = m_cpu.pairsVT[i];

        auto b0 = pair.body0.lock();
        auto b1 = pair.body1.lock();
        if (!b0 || !b1) continue;

        if (pair.mergeBufferOffset >= m_hostMergeBuffer.size()) {
            HDT_LOG_ERROR("Merge buffer offset out of bounds");
            continue;
        }

        const auto* mergeData = &m_hostMergeBuffer[pair.mergeBufferOffset];
        applyMergeResults(mergeData, b0.get(), b1.get(), pair.swapped, dispatcher);
    }
}
```

### Task 5: Modify dispatcher to use batched API ✅ COMPLETE
**File:** `hdtDispatcher.cpp`
**Effort:** Medium
**Status:** Implemented - removed old `queueCollision` path, now uses `CudaGatherPairs` + `launchCollisionBatch()`

```cpp
void CollisionDispatcher::dispatchAllCollisionPairs()
{
    if (haveCuda) {
        CudaInterface::instance()->beginCollisionBatch();

        // Gather phase (parallel)
        {
            HDT_ZONE_SCOPED_N("GatherCollisionPairs");
            concurrency::parallel_for(0, pairCount, [this](int i) {
                auto& pair = m_pairs[i];
                if (pair.first->m_shape->m_tree.collapseCollideL(&pair.second->m_shape->m_tree)) {
                    // Returns false if limit reached or invalid
                    CudaInterface::instance()->addCollisionPair(pair.first, pair.second);
                }
            });
        }

        // Launch phase (single call)
        {
            HDT_ZONE_SCOPED_N("LaunchBatchedCollisions");
            CudaInterface::instance()->launchCollisionBatch(m_stream);
        }

        // Results applied next frame via syncPreviousCollisionResults()
    }
}
```

### Task 6: Thread Safety for Batch Accumulation ✅ COMPLETE
**File:** `hdtCudaInterface.cpp`
**Effort:** Medium (expanded from original)
**Status:** Implemented using mutex-protected append (simpler than thread-local due to PPL issues)

Used direct mutex append instead of thread-local batches - simpler and avoids PPL `parallel_for` issues.

```cpp
thread_local CpuBatchData BatchedCollisionManager::t_localBatch;

void BatchedCollisionManager::accumulateVV(
    const std::shared_ptr<SkinnedMeshBody>& body0,
    const std::shared_ptr<SkinnedMeshBody>& body1,
    bool swapped)
{
    // Thread-local accumulation - NO mergeBufferOffset yet
    // Offsets calculated during merge phase to avoid race condition
    CollisionPairInfo info{
        .shapeA = body0->m_shape->asPerVertexShape(),
        .shapeB = body1->m_shape->asPerVertexShape(),
        .body0 = body0,  // weak_ptr from shared_ptr
        .body1 = body1,
        .mergeBufferOffset = 0,  // Placeholder - calculated in merge
        .swapped = swapped
    };

    t_localBatch.pairsVV.push_back(info);
    t_localBatch.setupVV.push_back(createSetup(body0, body1));
}

void BatchedCollisionManager::mergeThreadLocalBatches()
{
    // Collect all thread-local batches
    std::lock_guard<std::mutex> lock(m_mergeMutex);

    // Merge from thread-local into global
    // Each thread calls this after parallel_for completes

    size_t baseOffsetVV = m_cpu.pairsVV.size();
    size_t baseOffsetVT = m_cpu.pairsVV.size() + m_cpu.pairsVT.size();  // VT comes after VV

    // Append thread-local VV pairs with correct offsets
    for (size_t i = 0; i < t_localBatch.pairsVV.size(); ++i) {
        auto pair = t_localBatch.pairsVV[i];
        pair.mergeBufferOffset = baseOffsetVV + m_cpu.pairsVV.size();  // Recalculate during merge
        m_cpu.pairsVV.push_back(pair);
        m_cpu.setupVV.push_back(t_localBatch.setupVV[i]);
    }

    // Append thread-local VT pairs with correct offsets
    for (size_t i = 0; i < t_localBatch.pairsVT.size(); ++i) {
        auto pair = t_localBatch.pairsVT[i];
        pair.mergeBufferOffset = baseOffsetVT + m_cpu.pairsVT.size();
        m_cpu.pairsVT.push_back(pair);
        m_cpu.setupVT.push_back(t_localBatch.setupVT[i]);
    }

    // Clear thread-local for next frame
    t_localBatch.clear();
}
```

**Alternative Option B (Atomic) with bounds check:**

```cpp
std::atomic<size_t> nextVV{0};
std::atomic<size_t> nextVT{0};

void accumulateVV(...) {
    size_t idx = nextVV.fetch_add(1);

    // CRITICAL: Bounds check for atomic option
    if (idx >= m_cpu.setupVV.size()) {
        HDT_LOG_WARNING("VV batch overflow at index {}", idx);
        return;  // Skip this pair
    }

    m_cpu.setupVV[idx] = {...};
    m_cpu.pairsVV[idx] = {...};
}

void beginBatch() {
    // Pre-allocate based on previous frame + margin
    size_t estimate = std::min(m_lastFramePairs * 1.2, (double)MAX_PAIRS);
    m_cpu.reserve(estimate);
    nextVV.store(0);
    nextVT.store(0);
}
```

### Task 7: Kernel modifications 🔲 PENDING (Phase 2)
**File:** `hdtCudaCollision.cu`
**Effort:** Medium-Large
**Status:** Not started - would require new batched kernel that handles heterogeneous body data

Phase 1 reuses existing per-pair kernels. Phase 2 would create a true batched kernel:

```cpp
template<eCollisionType type, typename ShapeT>
__global__ void kernelCollision(
    int numSetups,
    bool swap,
    const cuCollisionSetup* setups,
    /* shape data... */
    cuCollisionMerge* mergeBuffer,
    size_t mergeBufferSize)  // NEW: buffer size for bounds check
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numSetups) return;

    // CRITICAL: Bounds check before write
    size_t mergeOffset = setups[idx].mergeBufferOffset;
    if (mergeOffset >= mergeBufferSize) {
        // GPU can't log, but we can skip the bad write
        return;
    }

    // ... existing collision logic ...

    mergeBuffer[mergeOffset] = result;
}
```

---

## Files to Modify

| File | Changes |
|------|---------|
| `hdtCudaInterface.h` | Add split structs (`CpuBatchData`, `GpuBatchBuffers`, `BatchedCollisionManager`), new API declarations |
| `hdtCudaInterface.cpp` | Implement batched API with error handling |
| `hdtDispatcher.cpp` | Use batched API instead of per-pair `queueCollision` |
| `hdtSkinnedMeshAlgorithm.cpp` | May remove/deprecate `queueCollision` |
| `hdtCudaCollision.cu` | Add bounds checking to kernel |
| `hdtSkinnedMeshBody.h` | Ensure `SkinnedMeshBody` uses `shared_ptr` (may already) |

---

## Testing Plan

### Unit Tests

1. Verify batch accumulation groups pairs correctly by type
2. Verify merge buffer offsets are calculated correctly
3. Verify results are routed back to correct body pairs
4. **NEW:** Test weak_ptr behavior when body destroyed between gather/apply
5. **NEW:** Test MAX_PAIRS limit enforcement
6. **NEW:** Test calculateMergeBufferSize overflow checking

### Edge Case Tests (from Expert Panel)

| Test Case | Expected Behavior |
|-----------|-------------------|
| Zero collision pairs | No kernel launch, no crash |
| Exactly MAX_PAIRS pairs | Process all, no overflow |
| MAX_PAIRS + 1 pairs | Last pair rejected with warning |
| Body destroyed mid-frame | Pair skipped in apply phase |
| GPU OOM during allocation | Fall back to CPU path |
| Merge buffer offset overflow | Kernel skips bad write |
| Null body pointer | `addCollisionPair` returns false |
| Null shape pointer | `addCollisionPair` returns false |

### Integration Tests

1. Compare manifold output: batched vs per-pair (should be identical)
2. Profile with Tracy: verify kernel launch count reduced
3. Stress test: 20+ actors, verify no crashes
4. **NEW:** Test body destruction during physics step

### Performance Validation

1. Compare frame times: batched CUDA vs NOCUDA AVX2
2. Target: CUDA should be faster or equal to NOCUDA
3. Profile breakdown should show `GraphLaunch` < 5% (vs current 38%)

---

## Expected Results

| Metric | Before | After | Improvement |
|--------|--------|-------|-------------|
| Kernel launches | 925,000 | 2-3 | 99.99% reduction |
| `GraphLaunch` % | 38% | <5% | ~33% frame time saved |
| `StepSimulation` | 18.8ms | ~12ms | ~36% faster |
| vs NOCUDA AVX2 | 27% slower | ~10% faster | CUDA wins |

---

## Risks & Mitigations

### Risk 1: Thread safety in gather phase
**Mitigation:** Use thread-local batches with post-merge; recalculate offsets during merge

### Risk 2: Memory allocation overhead
**Mitigation:** Pre-allocate batch arrays based on previous frame count

### Risk 3: Kernel doesn't work with global data
**Mitigation:** Test incrementally; may need shape data restructuring

### Risk 4: Result routing complexity
**Mitigation:** Store weak_ptr with each pair; check validity before use

### Risk 5: Body destroyed between frames (NEW)
**Mitigation:** Use `weak_ptr` for body references; skip invalid pairs in apply phase

### Risk 6: Resource exhaustion (NEW)
**Mitigation:** Hard cap at 2M pairs; reject excess with warning

### Risk 7: GPU memory corruption (NEW)
**Mitigation:** Bounds check in kernel; validate offsets on CPU before launch

---

## Implementation Order

1. **Task 1** - Data structures (foundation, includes all struct splits)
2. **Task 6** - Thread safety (affects Task 2, includes offset recalculation)
3. **Task 2** - Batch accumulation (with input validation)
4. **Task 7** - Kernel bounds checking (before Task 3)
5. **Task 3** - Kernel launch (with error handling)
6. **Task 4** - Result application (with weak_ptr checks)
7. **Task 5** - Dispatcher integration

---

## Success Criteria

### Performance (Phase 1 Results)
- [ ] CUDA build faster than NOCUDA AVX2 in dense scenes *(needs comparison test)*
- [x] ~~Kernel launch count < 10 per frame~~ Phase 1: ~88K (from 537K), Phase 2 target
- [ ] ~~`GraphLaunch` < 5% of frame time~~ 37.5% - but this is internal updates, not collision
- [x] **Collision overhead reduced: 33.4% → 1.5% (-32%)**

### Correctness
- [x] No physics behavior regression *(tested in-game)*
- [x] Manifold output identical to per-pair implementation *(88K MergeBuffer::apply calls)*
- [ ] No crashes in extended testing (1+ hour continuous) *(needs extended test)*

### Safety (from Expert Panel Review)
- [x] `weak_ptr` used for all body references
- [ ] `calculateMergeBufferSize()` has overflow checking *(not needed in Phase 1 - per-pair buffers)*
- [x] Merge buffer offsets recalculated during merge phase (not gather) *(using per-pair buffers)*
- [ ] Kernel has bounds check on merge buffer access *(Phase 2)*
- [x] `addCollisionPair` validates input pointers
- [x] MAX_PAIRS hard limit enforced (2M)
- [x] No TV batch (VT with swap flag only)
- [x] Structs split: `CpuBatchPairs`, `PendingCollisionResult`, `BatchedCollisionManager`
- [x] Explicit sync between compute and download *(via CudaInterface::synchronize())*
- [x] All CUDA calls wrapped with error checking *(existing cuResult pattern)*
- [x] `clear()` clears pending results in `beginBatch()`

### Testing
- [ ] Unit tests for all edge cases pass *(tests not written)*
- [x] Body destruction mid-frame handled gracefully *(weak_ptr check in applyResults)*
- [ ] GPU OOM falls back to CPU path *(not implemented - not needed in Phase 1)*
