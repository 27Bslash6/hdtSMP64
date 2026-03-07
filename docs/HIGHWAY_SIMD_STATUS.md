# Highway SIMD Integration Status

## Overview

Google Highway SIMD abstraction has been integrated to replace fragmented SSE4/AVX2/AVX-512 intrinsics with portable, runtime-dispatched SIMD code.

## What's Implemented

### 1. Vertex Skinning (`hdtHighwaySkinning.cpp`)
- **Function**: `highway::batchSkinVertices()`
- **Called from**: `SkinnedMeshBody::internalUpdate()` (NOCUDA path)
- **What it does**: Transforms vertices from skin space to world space using bone matrices and weights
- **Intrinsics replaced**: SSE4 in `calcVertexState()` loop
- **Gating**: `g_highwayConfig.enabled && count >= g_highwayConfig.batchThreshold`

### 2. AABB Collision Detection (`hdtHighwayAABB.cpp`)
- **Function**: `highway::batchCollideWith()`
- **Called from**: `Aabb::collideWithMany()` in `hdtAABB.h`
- **What it does**: Tests reference AABB against N candidate AABBs
- **Intrinsics replaced**: `collideWith2()` (AVX2) and `collideWith4()` (AVX-512)
- **Gating**: `g_highwayConfig.enabled && count >= g_highwayConfig.batchThreshold`

### 3. SoA Buffer (`hdtSoABuffer.h/cpp`)
- **Purpose**: Structure-of-Arrays vertex buffer for SIMD-friendly memory access
- **Used by**: Highway skinning path
- **Features**: 64-byte aligned allocation via Highway's `AllocateAlignedBytes()`

### 4. Configuration (`config.h/cpp`)
- **XML Element**: `<highway enabled="true" batch-threshold="64" />`
- **Global**: `g_highwayConfig`
- **Logging**: Reports detected SIMD target on startup: `[HIGHWAY] SIMD target: AVX2 (batch-threshold=64)`

## Files With Raw Intrinsics (Potential Gaps)

Three files were cleaned up as part of the AVX build-variant elimination refactor (2026-03):

| File | Status | Notes |
|------|--------|-------|
| `hdtGroupConstraintSolver.cpp` | **Cleaned** | AVX2 (`_mm256_*`) replaced with plain scalar C++. Hot path handled by `hdtHighwaySolverBridge.h`. |
| `hdtLCP.cpp` | **Cleaned** | SSE4.1 (`_mm_dp_ps`, `_mm_hadd_ps`) replaced with scalar. |
| `hdtSkinnedMeshAlgorithm.cpp` | **Cleaned** | SSE4.1 geometry ops replaced with `btVector3` methods. |

Remaining files with raw intrinsics:

| File | Intrinsic Count | Hot Path? | Highway Candidate? |
|------|-----------------|-----------|-------------------|
| `hdtSkinnedMeshShape.cpp` | 28 | Yes - AABB updates | **Partial** - AVX-512 path is compile-time gated (`#ifdef __AVX512F__`) but the file has no per-file `/arch:AVX512` override, so that path is unreachable under the current build. Should migrate to Highway. |
| `hdtCollisionAlgorithm.cpp` | 3 | Medium | No - Already minimal |
| `hdtSkinnedMeshWorld.cpp` | 1 | No | No - Single use |

### Bullet Library Files (Do Not Modify)
- `LinearMath/*` - Bullet's math library
- `BulletDynamics/*` - Bullet's constraint solvers
- `BulletCollision/*` - Bullet's collision detection
- `Bullet3*` - Bullet 3 code

## Remaining Gaps

### High Priority

1. **`PerVertexShape::internalUpdate()` in `hdtSkinnedMeshShape.cpp`**
   - Has `#ifdef __AVX512F__` compile-time dispatch, but the file has no per-file `/arch:AVX512`
     compiler flag — that branch is **unreachable** under the current build system
   - Should migrate to Highway `HWY_DYNAMIC_DISPATCH` so the AVX-512 path is actually usable
   - Computes per-vertex AABBs from collider data

### Completed (as of 2026-03 refactor)

- **`hdtGroupConstraintSolver.cpp`** — AVX2 intrinsics removed; scalar fallback for low counts;
  hot path handled by `hdtHighwaySolverBridge.h`
- **`hdtLCP.cpp`** — SSE4.1 matrix factorization intrinsics replaced with scalar
- **`hdtSkinnedMeshAlgorithm.cpp`** — SSE4.1 geometry ops replaced with `btVector3` methods

## Integration Patterns

### Current Pattern (Recommended)
```cpp
// In hot path function
if (g_highwayConfig.enabled && count >= g_highwayConfig.batchThreshold) {
    highway::batchOperation(...);  // Has internal Tracy zone
    return;
}
// Legacy SSE fallback
```

### Tracy Instrumentation
- Highway functions have internal `HDT_ZONE_SCOPED_N()` zones
- Don't add duplicate zones in calling code

### Testing
- Unit tests in `tests/unit/test_aabb_simd.cpp` (disabled in standalone builds)
- Unit tests in `tests/unit/test_highway_skinning.cpp`
- Config tests in `tests/unit/test_config.cpp`

## Build Configuration

Highway files (`hdtHighwayAABB.cpp`, `hdtHighwaySkinning.cpp`) are compiled with `/arch:AVX512`
regardless of the build configuration. This enables Highway to generate all SIMD targets
(SSE4, AVX2, AVX-512) in a single binary.

**Runtime dispatch**: Highway's `HWY_DYNAMIC_DISPATCH` checks CPUID at startup and selects
the best available target. A single DLL works on all CPUs:
- SSE4-only CPU → uses SSE4 path
- AVX2 CPU → uses AVX2 path
- AVX-512 CPU → uses AVX3 path

**Per-file compiler settings** in `hdtSMP64.vcxproj`:
```xml
<ClCompile Include="hdtSkinnedMesh\hdtHighwayAABB.cpp">
  <EnableEnhancedInstructionSet>AdvancedVectorExtensions512</EnableEnhancedInstructionSet>
</ClCompile>
```

## Log Output Example
```
[CONFIG] logLevel=2 (Warning) - messages above level 2 filtered
[HIGHWAY] SIMD target: AVX3 (batch-threshold=64)
```

On AVX-512 capable CPUs, you'll see `AVX3`. On AVX2 CPUs, you'll see `AVX2`.

## Future Work

1. **Phase 2**: Migrate `PerVertexShape::internalUpdate()` to Highway (fixes dead AVX-512 code path)
2. ~~**Phase 3**: Evaluate LCP solver for Highway migration~~ — completed 2026-03 (converted to scalar)
3. ~~**Phase 4**: Profile collision checking for vectorization opportunities~~ — `hdtSkinnedMeshAlgorithm.cpp` converted to scalar 2026-03
