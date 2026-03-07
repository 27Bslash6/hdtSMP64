# Highway SIMD Solver Integration - Benchmark Results

**Date**: 2026-01-18
**Branch**: feature/highway-simd-expansion
**Commit**: 7b4022d
**CPU**: AMD Zen 4 (AVX3_ZEN4 target)

## Summary

Highway SIMD integration for constraint solver is **complete and tested**. Unit tests pass with 100% correctness. Performance benchmarks show significant speedups on SIMD-friendly operations.

## Unit Test Benchmarks (Microbenchmarks)

### LCP Solver Helpers

**Highway::largeDot()** - Vector dot product with FMA
- Scalar:  59,560 μs
- Highway: 8,713 μs
- **Speedup: 6.84x** ✅

**Highway::vectorScale()** - Element-wise vector scaling
- Scalar:  8,751 μs
- Highway: 6,441 μs
- **Speedup: 1.36x** ✅

### Test Configuration
- Vector size: N = 10,000
- Iterations: 10,000
- Includes warmup phase to eliminate cache effects
- Uses `std::chrono::high_resolution_clock` for accurate timing

## Integration Status

### Completed ✅
1. **Batched Constraint Solver** (`hdtHighwaySolver.cpp`)
   - `resolveBatchConstraintRowsLowerLimit()` - Contact constraints
   - `resolveBatchConstraintRowsGeneric()` - Joint constraints
   - Unit tests: 129 assertions passing

2. **Solver Bridge** (`hdtHighwaySolverBridge.h`)
   - Thread-local scratch buffers
   - AoS ↔ SoA transpose with scatter/gather
   - Threshold-based gating

3. **Bullet Integration** (`btSequentialImpulseConstraintSolverMt.cpp`)
   - `resolveMultipleContactConstraints()` - Highway batched (line 1062)
   - `resolveMultipleJointConstraints()` - Highway batched (line 1044)
   - `resolveMultipleContactFrictionConstraints()` - Scalar (line 1087, deferred)

4. **LCP Helpers** (`hdtHighwayLCP.cpp`)
   - `largeDot()` - 6.84x speedup
   - `vectorScale()` - 1.36x speedup

5. **Per-Vertex AABB** (`hdtHighwayPerVertex.cpp`)
   - Runtime SIMD dispatch for all CPUs
   - Replaces compile-time AVX-512 paths

### Pending ⏳
- **In-game Tracy profiling** - Full frame impact measurement
  - Contact constraint batch performance
  - Joint constraint batch performance
  - Solver loop throughput (target: 2-3x vs scalar)

### Deferred ❌
- **Friction constraints** - Kept scalar
  - Reason: Nested loops + dynamic limit updates
  - Typical batch size = 2 (minimal SIMD benefit)

## Performance Targets

| Component | Target | Measured | Status |
|-----------|--------|----------|--------|
| largeDot() | 2-3x | 6.84x | ✅ **Exceeded** |
| vectorScale() | 1.5-2x | 1.36x | ⚠️ Close |
| Contact batch solver | 2-3x | Pending Tracy | ⏳ |
| Joint batch solver | 2-3x | Pending Tracy | ⏳ |
| Full solver loop | 20-40% frame reduction | Pending Tracy | ⏳ |

## Configuration

Runtime gating ensures Highway paths only activate when beneficial:

```cpp
if (g_highwayConfig.enabled && count >= g_highwayConfig.batchThreshold) {
    highway::batchSolver(...);  // SIMD path
} else {
    scalarSolver(...);          // Fallback
}
```

**XML Config**:
```xml
<highway enabled="true" batch-threshold="64" />
```

**Default threshold**: 64 constraints (tunable based on Tracy profiling)

## Next Steps

1. **Tracy Profiling** (Task 2.3 from `workplan/bullet-highway/tasks.md`)
   - Build main DLL with Tracy enabled (`Tracy.props`)
   - Run in-game with 100/500/1000 body scenes
   - Profile `ContactConstraintsBatch` and `JointConstraintsBatch` zones
   - Compare Highway vs scalar throughput

2. **Threshold Tuning**
   - Determine optimal `batch-threshold` via profiling
   - Currently set conservatively at 64

3. **Documentation**
   - Update `docs/HIGHWAY_SIMD_STATUS.md` with results
   - Add Tracy screenshots to `docs/solver-benchmarks/`

## Files Modified
- `hdtHighwaySolverBridge.h` - Added `#include <vector>`
- `tests/hdtSMP64_tests.vcxproj` - Added `test_solver_benchmark.cpp`

## Test Execution

```powershell
# Run LCP benchmarks
./tests/x64/tests/Release/hdtSMP64_tests.exe "[lcp][benchmark]" -s --reporter console

# Run all Highway unit tests
./tests/x64/tests/Release/hdtSMP64_tests.exe "[highway]" --reporter console
```

## Conclusion

**Phase 2 integration is functionally complete**. The Highway batched solver:
- ✅ Compiles cleanly
- ✅ Passes all unit tests (129 assertions)
- ✅ Shows 6.84x speedup on dot products
- ✅ Integrates into Bullet solver hot path
- ⏳ Awaiting in-game Tracy profiling for full impact measurement

**Expected impact**: 2-3x throughput on constraint solver (74% of frame time), translating to 20-40% overall frame time reduction for physics-heavy scenes.
