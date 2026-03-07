# hdtSMP64 Optimization Action Plan

**Synthesis Date:** 2026-01-08
**Source Documents:** CUDA_TODO.md, CUDA_OPTIMIZATION_PROGRESS.md, OPTIMIZATION_REPORT.md, ECOSYSTEM_RECOMMENDATIONS.md
**Target:** 10 skeletons at 10ms budget (currently 7 skeletons at 10.5-11.3ms)

---

## 1. Current State Summary

### What's Done (P0-P2 CUDA Fixes)

| Fix | Status | Impact Achieved |
|-----|--------|-----------------|
| Serial sync fix (single device sync) | COMPLETE | 24ms -> 2ms overhead |
| Stream pool (lazy-growing) | COMPLETE | Eliminated 10ms+ stream creation |
| Dynamic parallelism removal | COMPLETE | Eliminated ~2ms/body overhead |
| Per-collision sync removal | COMPLETE | N syncs -> 2 syncs/frame |
| Two-phase collision queueing | COMPLETE | ~1ms CPU savings (needs validation) |

### What's Blocked

| Item | Blocker | Action Required |
|------|---------|-----------------|
| P2 validation | Needs in-game testing | Run `smp timing 200` with new build |
| Further CUDA work | P2 results unknown | Wait for validation data |

### What's Pending

| Category | Items |
|----------|-------|
| CUDA P3 | Stream-ordered allocation, CUDA Graphs, Warp reduction, Adaptive blocks |
| CPU Parallelization | 8 sequential bottlenecks identified (B1-B8) |
| Ecosystem | Tracy (partial), mimalloc, Highway SIMD, oneTBB |

---

## 2. Gap Analysis

### The Performance Gap

```
Current:  10.5 - 11.3 ms/frame @ 7 skeletons
Target:   10.0 ms/frame @ 10 skeletons
Gap:      0.5 - 1.3 ms to close (at CURRENT skeleton count)
          PLUS support 3 more skeletons
```

### Gap Breakdown by Component

| Component | Current | Target | Gap | Source |
|-----------|---------|--------|-----|--------|
| ms/skeleton | 1.5-1.6ms | <1.0ms | 0.5-0.6ms | CUDA_TODO |
| Total frame | 10.5-11.3ms | <10ms | 0.5-1.3ms | Progress log |
| Skeleton capacity | 7 | 10 | +3 | Requirements |

### Death Spiral Problem

The system exhibits a **death spiral** where:
1. Over budget -> auto-scale down skeletons
2. Fixed overhead doesn't decrease proportionally
3. Still over budget -> scale down more
4. Performance degrades rather than stabilizes

**Root cause:** Fixed overhead dominates variable work. Must reduce fixed overhead, not just per-skeleton cost.

### What the Gap Consists Of

Based on profiling data (cuda-3.csv):

| Phase | Time | % of Total |
|-------|------|------------|
| DispatchCollisionPairs | 4.91ms | 46.5% |
| SolveConstraints | 2.80ms | 26.5% |
| CollisionDetection | 6.19ms | 58.7% |
| **doUpdate2ndStep (total)** | 10.54ms | 100% |

**Key insight:** Collision-related work (dispatch + detection) is ~5-6ms. This is where most gains must come from.

---

## 3. Prioritized Action Matrix

### Tier 0: Validation (BLOCKING)

| ID | Action | Source | Expected Impact | Effort | Dependencies | Priority |
|----|--------|--------|-----------------|--------|--------------|----------|
| **V1** | Validate P2 two-phase fix | CUDA_TODO | Confirm ~1ms saving | 30 min | Build ready | **P0** |
| **V2** | Profile with Tracy | ECOSYSTEM | Identify actual bottlenecks | 2 hrs | Tracy integrated | **P0** |

### Tier 1: CPU Quick Wins (Parallel Loops)

| ID | Action | Source | Expected Impact | Effort | Dependencies | Priority |
|----|--------|--------|-----------------|--------|--------------|----------|
| **C1** | Parallelize vertex skinning (B1) | OPTIMIZATION | 0.4ms (~23% of physics) | Low | None | **P0** |
| **C2** | Parallelize system update loop (B2) | OPTIMIZATION | ~Nx for N systems | Low | None | **P0** |
| **C3** | Fix predictUnconstraintMotion (B3) | OPTIMIZATION | 2-3x this function | Low | None | **P1** |
| **C4** | Fix integrateTransforms (B4) | OPTIMIZATION | 2-3x this function | Low | None | **P1** |
| **C5** | Parallelize distance calcs (B5) | OPTIMIZATION | 1.5-2x this function | Low | None | **P2** |
| **C6** | Use parallel_sort for skeletons (B6) | OPTIMIZATION | Minor | Low | None | **P3** |

**Note:** C1 already partially implemented per memory (kParallelThreshold = 64). Verify active.

### Tier 2: CUDA Improvements

| ID | Action | Source | Expected Impact | Effort | Dependencies | Priority |
|----|--------|--------|-----------------|--------|--------------|----------|
| **G1** | CUDA 12 stream-ordered allocation | CUDA_TODO | 15x alloc speedup | Medium | P2 validated | **P2** |
| **G2** | Warp-level reduction | CUDA_TODO | Reduced atomics (13->1/warp) | Medium | None | **P2** |
| **G3** | Adaptive block sizing | CUDA_TODO | Better small-pair efficiency | Low-Med | None | **P3** |
| **G4** | CUDA Graphs | CUDA_TODO | 4ms -> 50us dispatch | High | Architecture change | **P3** |

### Tier 3: Ecosystem Integration

| ID | Action | Source | Expected Impact | Effort | Dependencies | Priority |
|----|--------|--------|-----------------|--------|--------------|----------|
| **E1** | Complete Tracy instrumentation | ECOSYSTEM | Identify real bottlenecks | Low | Partial done | **P0** |
| **E2** | mimalloc integration | ECOSYSTEM | 10-20% alloc speedup | Low | None | **P1** |
| **E3** | Highway SIMD for vertex skinning | ECOSYSTEM | 20-40% vertex speedup | Medium | Post-C1 | **P2** |
| **E4** | oneTBB evaluation | ECOSYSTEM | Better scaling 16+ cores | Medium | Optional | **P3** |

### Tier 4: Strategic (Deferred)

| ID | Action | Source | Expected Impact | Effort | Dependencies | Priority |
|----|--------|--------|-----------------|--------|--------------|----------|
| **S1** | XPBD algorithm | ECOSYSTEM | 50% fewer iterations | High | Major refactor | Deferred |
| **S2** | SoA data layout | OPTIMIZATION | 1.5x SIMD efficiency | High | Breaking change | Deferred |
| **S3** | Enable delayed collisions | OPTIMIZATION | CPU/GPU overlap | Medium | Visual tuning | Deferred |

---

## 4. Recommended Execution Sequence

### Week 1: Validate and Profile

```
Day 1-2: Validation & Profiling
    [V1] Build V1_6_1170_CUDA_AVX2
         Run: just build V1_6_1170_CUDA_AVX2
         Test: smp timing 200 (in-game)
         Decision: P2 effective? (expect ~1ms improvement)

    [V2/E1] Tracy profiling session
         Build: just profile V1_6_1170_CUDA_AVX2
         Connect Tracy profiler
         Capture 200+ frames under load
         Identify actual top 5 hotspots
```

**GO/NO-GO Decision Point #1:**
- If P2 reduced collision queueing time: Continue to Tier 1
- If P2 made things worse: Revert and investigate mutex contention

### Week 1-2: CPU Parallelization

```
Day 3-5: CPU Quick Wins
    [C1] Verify vertex skinning parallel_for active
         Check: hdtSkinnedMeshBody.cpp internalUpdate
         If threshold too high (>64), consider lowering

    [C2] Parallelize system update loop
         File: hdtSkinnedMeshWorld.cpp:126-132
         Change: for -> parallel_for_each

    [C3/C4] Fix physics overrides
         File: hdtSkinnedMeshWorld.cpp:170-222
         Change: for -> parallel_for in both functions

Day 6-7: Integration & Validation
    [E2] Add mimalloc
         Simple: link mimalloc-override.lib
         Test stability

    Re-profile with Tracy
    Measure improvement
```

**Expected Outcome Week 1-2:** 0.5-1.0ms reduction (40-60% of gap)

### Week 2-3: CUDA Refinements (If Needed)

```
IF still over budget after CPU work:

    [G1] Stream-ordered allocation
         Replace CudaBufferPool with cudaMallocAsync
         Remove global mutex contention

    [G2] Warp-level reduction
         Modify collision kernel
         Use __reduce_add_sync intrinsics
         13 atomics -> 1 atomic per warp
```

**GO/NO-GO Decision Point #2:**
- If under 10ms: Stop, document, ship
- If 10.0-10.3ms: Consider G3 (adaptive blocks)
- If still >10.5ms: Investigate deeper (CUDA Graphs or algorithmic change)

### Week 3+: Advanced Optimizations (As Needed)

```
IF still not meeting targets:

    [E3] Highway SIMD for vertex skinning
         Major refactor of hot path
         20-40% vertex transform speedup

    [G4] CUDA Graphs
         Architectural change
         Capture entire collision pipeline
         Single graph launch
```

---

## 5. Decision Points

### Decision Point #1: P2 Validation (Day 2)

| Outcome | Metric | Action |
|---------|--------|--------|
| Success | Collision queueing <1.5ms | Proceed with CPU parallelization |
| Neutral | No change | Investigate, may need P1 rework |
| Failure | Performance worse | Revert P2, analyze contention |

### Decision Point #2: Post-CPU Parallelization (End of Week 2)

| Outcome | Metric | Action |
|---------|--------|--------|
| Target met | <10ms @ 10 skeletons | Document, ship, celebrate |
| Close | 10.0-10.5ms | Apply G1/G2 for final push |
| Not close | >10.5ms | Deeper analysis, consider architecture changes |

### Decision Point #3: CUDA Work Evaluation (End of Week 3)

| Outcome | Metric | Action |
|---------|--------|--------|
| Target met | <10ms @ 10 skeletons | Ship |
| Still over | >10ms | Consider CUDA Graphs or XPBD |
| Diminishing returns | Small gains, high effort | Accept current state, document limitations |

---

## 6. Risk Assessment

### Low Risk (Proceed Confidently)

| Item | Risk | Mitigation |
|------|------|------------|
| C1-C4 (parallel loops) | Thread safety | Operations are embarrassingly parallel, no shared writes |
| E1 (Tracy) | Overhead | TRACY_ENABLE only in profiling builds |
| E2 (mimalloc) | Compatibility | Global override is well-tested |

### Medium Risk (Proceed with Testing)

| Item | Risk | Mitigation |
|------|------|------------|
| G1 (stream-ordered alloc) | CUDA 12 requirement | Verify driver compatibility |
| G2 (warp reduction) | Kernel correctness | Unit test collision results |
| P2 validation | May not improve | Have revert plan ready |

### High Risk (Careful Planning Required)

| Item | Risk | Mitigation |
|------|------|------------|
| G4 (CUDA Graphs) | Architecture change | Prototype separately, benchmark extensively |
| S1 (XPBD) | Major refactor | Long-term project, not for current release |
| S2 (SoA layout) | Breaking API changes | Would require major version bump |

### Specific Risks to Monitor

1. **Death spiral persistence:** If fixed overhead doesn't decrease, may need fundamentally different approach
2. **Mutex contention:** CudaBufferPool and CudaStreamPool have mutexes that may contend under parallel load
3. **Driver compatibility:** CUDA 12 features require specific driver versions
4. **Visual quality:** Some optimizations (delayed collisions) may introduce visual artifacts

---

## 7. Success Metrics

### Primary Metrics

| Metric | Current | Target | How to Measure |
|--------|---------|--------|----------------|
| Active skeletons | 7 | 10 | `smp` console command |
| Frame time | 10.5-11.3ms | <10ms | `smp timing 200` |
| ms/skeleton | 1.5-1.6ms | <1.0ms | Frame time / skeleton count |
| Death spiral | Occurs | Eliminated | Stable skeleton count under load |

### Secondary Metrics

| Metric | Current | Target | How to Measure |
|--------|---------|--------|----------------|
| Collision dispatch | 4.91ms | <3ms | Tracy zone timing |
| Vertex skinning | ~0.4ms | <0.3ms | Tracy zone timing |
| Alloc contention | Unknown | <5% lock wait | Tracy lock analysis |

### Validation Protocol

```bash
# Build optimized version
just build V1_6_1170_CUDA_AVX2

# In-game testing sequence:
smp reset                  # Clean state
smp on                     # Enable physics
# Load scene with 10+ NPCs
smp timing 200             # Profile 200 frames
smp stats                  # Check current state
smp                        # Verify skeleton counts
```

---

## 8. Quick Reference

### Build Commands

```bash
# Standard build
just build V1_6_1170_CUDA_AVX2

# Build with profiling
just profile V1_6_1170_CUDA_AVX2

# Run tests
just test

# Check CUDA
just cuda-info
```

### Console Commands

```
smp              # Status overview
smp timing 200   # Profile N frames
smp metrics      # Toggle continuous logging
smp stats        # Performance stats
smp gpu          # Toggle CUDA
smp reset        # Full reset
```

### Key Files

| Purpose | File |
|---------|------|
| Collision dispatch | hdtDispatcher.cpp |
| Vertex skinning | hdtSkinnedMeshBody.cpp |
| Physics world | hdtSkinnedMeshWorld.cpp |
| CUDA interface | hdtCudaInterface.cpp |
| CUDA kernels | hdtCudaCollision.cu |
| Actor management | ActorManager.cpp |

---

## 9. Summary: The Path to 10ms

```
                    CURRENT STATE
                         |
                    10.5-11.3ms
                    7 skeletons
                         |
            +------------+------------+
            |                         |
     [V1] Validate P2            [E1] Tracy
     (confirm 1ms gain)          (find truth)
            |                         |
            +------------+------------+
                         |
            +------------+------------+
            |            |            |
        [C1-C4]      [E2]         [G1-G2]
       CPU Loops    mimalloc    CUDA Alloc
        (0.5ms)     (0.1ms)      (0.3ms)
            |            |            |
            +------------+------------+
                         |
                    10.0-10.5ms
                    8-9 skeletons
                         |
            +------------+------------+
            |                         |
     If needed:              If target met:
        [E3] Highway              SHIP IT
        [G3-G4] CUDA
            |                         |
            +------------+------------+
                         |
                      <10ms
                   10 skeletons
                         |
                       DONE
```

---

**Next Immediate Action:** Run validation build and `smp timing 200` to confirm P2 effectiveness before proceeding with additional optimizations.
