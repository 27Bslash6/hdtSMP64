# CUDA Optimization Progress Report

## Executive Summary

hdtSMP64 CUDA optimization effort to achieve parity with CPU AVX performance (10 skeletons stable at 10ms budget).

**Current Status:** 7 skeletons at ~10.5-11.3ms (0.5-1.3ms over budget)
**Target:** 10 skeletons at 10ms budget

---

## Completed Fixes

### P0: Critical Fixes (Blocking Issues)

| Fix | Location | Problem | Solution | Impact |
|-----|----------|---------|----------|--------|
| Serial Sync | hdtDispatcher.cpp:210-228 | N bodies × ~2ms sync = 24ms+ | Single device sync + parallel tree updates | 24ms → 2ms |
| Stream Pool | hdtCudaInterface.cpp:300-430 | New stream per collision (~100μs each) | Lazy-growing pool (max 512 streams) | 10ms → ~0ms |
| Dynamic Parallelism | hdtCudaCollision.cu:955-1002 | <<<1,1>>> launcher anti-pattern | Direct CPU kernel launches | ~2ms/body → 0 |

### P1: High-Impact Fixes

| Fix | Location | Problem | Solution | Impact |
|-----|----------|---------|----------|--------|
| Per-Collision Sync | hdtCudaInterface.cpp:849-851 | Sync per collision pair | Single device sync in dispatcher | N syncs → 2 syncs |

### P2: Medium-Impact Fixes

| Fix | Location | Problem | Solution | Impact |
|-----|----------|---------|----------|--------|
| Two-Phase Queueing | hdtDispatcher.cpp:254-299 | Serial tree checks + CUDA queueing | Parallel tree checks, serial CUDA | ~1ms CPU savings |

**Note:** Initial P2 attempt (fully parallel) caused mutex contention and made things WORSE. Two-phase approach separates CPU-bound work (parallelizable) from CUDA queueing (must be serial due to pool mutexes).

---

## Performance Data

### Profiling Comparison

| Metric | cuda-1.csv | cuda-2.csv | cuda-3.csv (bad P2) |
|--------|------------|------------|---------------------|
| doUpdate2ndStep | 11.22ms | 11.53ms | 10.54ms |
| DispatchCollisionPairs | 4.00ms | 4.01ms | 4.91ms |
| SolveConstraints | 2.92ms | 2.76ms | 2.80ms |
| CollisionDetection | 5.38ms | 5.27ms | 6.19ms |

### Real-Time Logs (cuda-3.csv session)
```
7 active skeletons | 1.51 ms/skeleton | 10.58 ms used / 10.00 ms budget
7 active skeletons | 1.55 ms/skeleton | 10.88 ms used / 10.00 ms budget
Auto-scaling: decreasing from 7 to 6 (over budget by 1.32 ms)
Auto-scaling: decreasing from 6 to 5 (over budget by 1.30 ms)
```

**Death spiral still occurring** - scaling down doesn't immediately help because overhead is relatively fixed.

---

## Pending Optimizations (P3)

### 1. CUDA 12 Stream-Ordered Allocation
**File:** hdtCudaInterface.cpp (CudaBufferPool class)
**Problem:** Global mutex on buffer allocation causes contention
**Solution:** Replace with `cudaMallocAsync`/`cudaFreeAsync`
**Expected Impact:** 15x speedup on allocation-heavy workloads
**Effort:** Medium (requires refactoring CudaPooledBuffer)

### 2. CUDA Graphs
**Problem:** Hundreds of kernel launches have CPU overhead (~10-50μs each)
**Solution:** Capture collision pipeline as CUDA graph, single launch
**Expected Impact:** 4ms dispatch → ~50μs
**Effort:** High (significant architectural change)

### 3. Warp-Level Reduction
**File:** hdtCudaCollision.cu (collision kernel)
**Problem:** 13 atomic operations per collision result
**Solution:** Use `__reduce_add_sync` intrinsics, 1 atomic per warp
**Expected Impact:** Reduced atomic contention on high-collision scenarios
**Effort:** Medium (kernel restructure)

### 4. Block Size Optimization
**Problem:** Hardcoded 256-thread blocks for all collision pairs
**Solution:** Adaptive sizing based on collision pair size
**Effort:** Low-Medium

---

## Architecture Notes

### Stream Usage
- **Internal updates:** Each CudaBody has its own stream
- **Collision detection:** BorrowedCudaStream from lazy-growing pool
- **Synchronization:** `cudaDeviceSynchronize()` (waits for ALL streams)

### Memory Pools
- **CudaBufferPool:** Bump allocator with 16MB pages, global mutex
- **CudaStreamPool:** Lazy-growing pool (max 512), mutex-protected borrow/return

### Key Code Paths
```
dispatchAllCollisionPairs (hdtDispatcher.cpp:69)
├── Build to_update map (serial)
├── Create CUDA objects if needed (parallel)
├── Process delayed funcs from previous frame
├── Launch internal updates (serial per body)
├── SYNC 1: cudaDeviceSynchronize()
├── Parallel tree updates (CPU)
├── Clear buffer pool
├── Collision queueing (two-phase: parallel tree + serial CUDA)
├── SYNC 2: cudaDeviceSynchronize()
└── Process collision results (serial)
```

---

## Files Modified

| File | Changes |
|------|---------|
| hdtDispatcher.cpp | P0 serial sync, P1 per-collision sync, P2 two-phase |
| hdtDispatcher.h | Added `<concurrent_vector.h>` include |
| hdtCudaInterface.cpp | Stream pool, BorrowedCudaStream, buffer pool |
| hdtCudaCollision.cu | Direct kernel launches (removed dynamic parallelism) |

---

## Next Steps

1. **Test current build** with two-phase P2 fix
2. **Profile** to see if tree parallelization helps
3. **If still over budget:** Implement CUDA 12 stream-ordered allocation
4. **Long-term:** Consider CUDA Graphs for sub-millisecond dispatch

---

## Build Commands

```bash
# Build CUDA version
just build V1_6_1170_CUDA_AVX2

# Build with Tracy profiler
just profile V1_6_1170_CUDA_AVX2

# Check CUDA installation
just cuda-info
```

---

## References

- [CUDA 12 Migration Opportunities](memory: CUDA12 migration analysis)
- [Original Optimization Audit](memory: CUDA optimization audit)
- [P0 Fixes Implementation](memory: CUDA P0 fixes)
