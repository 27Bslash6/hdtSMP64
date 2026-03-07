# CUDA Optimization TODO

## Completed

- [x] **P0:** Serial sync fix - single device sync instead of N per-body syncs
- [x] **P0:** Stream pool - lazy-growing pool instead of creating per collision
- [x] **P0:** Dynamic parallelism removal - CPU kernel launches instead of <<<1,1>>>
- [x] **P1:** Per-collision sync removal - 2 syncs per frame instead of N
- [x] **P2:** Two-phase collision queueing - parallel tree checks, serial CUDA

## In Progress

- [ ] **Test:** Validate two-phase P2 fix improves performance without contention

## Pending (P3)

- [ ] **CUDA 12 Stream-Ordered Allocation**
  - Replace `CudaBufferPool` with `cudaMallocAsync`/`cudaFreeAsync`
  - Eliminates global mutex contention
  - File: `hdtCudaInterface.cpp:209-300`

- [ ] **CUDA Graphs**
  - Capture collision pipeline as graph
  - Single graph launch instead of hundreds of kernel launches
  - Expected: 4ms → 50μs dispatch overhead

- [ ] **Warp-Level Reduction**
  - Replace 13 atomics per collision with `__reduce_add_sync`
  - File: `hdtCudaCollision.cu` collision kernel

- [ ] **Adaptive Block Sizing**
  - Dynamic block size based on collision pair size
  - Currently hardcoded at 256 threads

## Deferred

- [ ] Zero-copy memory optimization (getZ() usage analysis done, low priority)
- [ ] Green contexts for SM partitioning (game integration consideration)

## Metrics Target

| Metric | Current | Target |
|--------|---------|--------|
| Active skeletons | 7 | 10 |
| Frame time | 10.5-11.3ms | <10ms |
| ms/skeleton | 1.5-1.6ms | <1.0ms |

## Quick Reference

```bash
# Build
just build V1_6_1170_CUDA_AVX2

# Test in-game
smp timing 200    # Profile 200 frames
smp metrics       # Toggle continuous logging
smp gpu           # Toggle CUDA on/off for comparison
```
