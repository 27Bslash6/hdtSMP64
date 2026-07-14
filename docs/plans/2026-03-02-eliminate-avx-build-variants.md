# Design: Eliminate AVX Build Variants

**Date**: 2026-03-02
**Status**: Approved

## Problem

The build matrix has 112 configurations:
`{VERSION}_{CUDA/NOCUDA}_{NoAVX/AVX/AVX2/AVX512}[_DEBUG]`

The AVX dimension is redundant. Highway SIMD already compiles all Highway files with
`/arch:AVX512` and does **runtime dispatch** — one binary supports SSE4 through AVX512 automatically.
The AVX variants exist for three remaining files that still use raw intrinsics:

| File | Intrinsics | ISA required |
|------|-----------|-------------|
| `hdtSkinnedMeshAlgorithm.cpp` | ~38 SSE4.1 intrinsics | SSE4.1 (not AVX2) |
| `hdtLCP.cpp` | ~40 SSE4.1 intrinsics | SSE4.1 (not AVX2) |
| `hdtGroupConstraintSolver.cpp` | ~35 AVX2 intrinsics | **AVX2** ← the blocker |

Only `hdtGroupConstraintSolver.cpp` actually requires AVX2 at compile time. The other two
use SSE4.1 which is available on all Skyrim-era hardware (Intel Nehalem 2008+).

## Decision

Replace all remaining raw intrinsics with Highway or scalar equivalents, then collapse
the AVX dimension from the build matrix entirely.

**CUDA/NOCUDA split is retained** because:
- NOCUDA builds don't require CUDA Toolkit (nvcc) — CI and toolkit-free devs need this
- CUDA builds are optional release artifacts requiring the full CUDA Toolkit

**Result**: 112 → 18 configurations
```
{VERSION}_CUDA          # 8 game versions × CUDA
{VERSION}_NOCUDA        # 8 game versions × NOCUDA
V1_6_1179_CUDA_DEBUG    # Latest version debug only
V1_6_1179_NOCUDA_DEBUG
```

## Architecture

### Phase 1 — hdtSkinnedMeshAlgorithm.cpp

**What**: Replace `cross_product()` and `checkCollide()` geometry ops (SSE4.1).

**Patterns**:
- `cross_product()`: shuffle + mul + sub → `hn::Mul`, `hn::Sub`, `hn::Per4LanesShuffle`
  or equivalent using `FixedTag<float, 4>`
- `checkCollide()`: `_mm_dp_ps` (dot product), `_mm_sqrt_ps`, `_mm_div_ps` →
  manual multiply-accumulate with `hn::MulAdd`, `hn::Sqrt`, `hn::Div`

**Approach**: Use `FixedTag<float, 4>` (not `foreach_target`) since these are per-collision-pair
ops that are already called in a loop — the loop structure isn't amenable to batch Highway
without a larger refactor. The goal here is just to eliminate raw intrinsics, not to improve
the batch structure (that's a separate concern).

**Testing**: Build passes. Existing collision tests pass. No regression in simulation behavior.

### Phase 2 — hdtLCP.cpp

**What**: Replace matrix solver SSE4.1 intrinsics (~40 remaining after `largeDot`/`vectorScale`
were migrated previously).

**Patterns**:
- Cholesky factorization inner loops: `_mm_dp_ps` (4-float dot), `_mm_hadd_ps` →
  manual `hn::MulAdd` accumulation
- Matrix row operations: `_mm_loadu_ps`, `_mm_storeu_ps` → `hn::LoadU`, `hn::StoreU`

**Approach**: `foreach_target` since these are inner loops where runtime dispatch pays off.
Follow the existing pattern from `hdtHighwayLCP.cpp` (which already handles `largeDot`
and `vectorScale`).

**Numerical sensitivity**: The Cholesky solver is numerically sensitive. Verify that
results are bit-for-bit equivalent (or within float epsilon) against the SSE4.1 path
before removing the old code. The existing `test_bullet_solver_configs.cpp` provides
coverage.

**Testing**: Solver tests pass. No NaN/inf in simulation. Constraint solve count unchanged.

### Phase 3 — hdtGroupConstraintSolver.cpp

**What**: Remove the AVX2 dual-body packing trick from the scalar path.

**Context**: `gResolveSingleConstraintRowGeneric_avx256` and `gResolveSingleConstraintRowLowerLimit_avx256`
are the **scalar path** (one constraint at a time), called only when constraint count < batch
threshold (64). The Highway batch bridge (`hdtHighwaySolverBridge.h`) already owns the hot path.
At batch sizes < 64, AVX2 SIMD provides minimal benefit.

**Approach**: Replace with plain scalar C++. Two sequential body operations instead of one
packed 8-wide operation. The `pack256`/`unpack256` helpers get deleted. The `FMADD`/`FMNADD`
macros go away. Rename functions: drop `_avx256` suffix.

This is the pragmatic choice: the batch solver bridge handles >64 constraints in SIMD;
the scalar path handles <64 where SIMD throughput doesn't matter.

**Testing**: Simulation stability unchanged. Frame time within noise (scalar path rarely hit
in practice with typical Skyrim NPC loads).

### Phase 4 — Kill Build Variants

**vcxproj** (script-driven, not by hand — file is 10k+ lines):
- Write a Python script to parse XML and remove all configs matching `*_NoAVX*`, `*_AVX_*`,
  `*_AVX2_*`, `*_AVX512_*`
- Keep `*_CUDA`, `*_NOCUDA`, `*_CUDA_DEBUG`, `*_NOCUDA_DEBUG`
- Global `EnableEnhancedInstructionSet` → `NotSet` (SSE2 baseline)
- Highway files retain per-file `AdvancedVectorExtensions512` override (unchanged)
- Remaining DEBUG configs: `V1_6_1179` only (latest version)

**hdtSMP64.sln**: Remove the 94 deleted configuration entries.

**build.yml**:
- Drop the `avx` matrix dimension entirely
- Build: `V1_6_1170_NOCUDA` (latest stable AE, no toolkit needed in CI)
- Artifact names: `hdtSMP64-{version}-{cuda}` (no avx component)
- Cache keys: strip avx component

**justfile**:
- `default_config := "V1_6_1170_NOCUDA"`
- `build-all-nocuda`: builds `V1_6_1170_NOCUDA` only (or all versions)
- `build-all-cuda`: builds `V1_6_1170_CUDA` only (or all versions)
- Remove per-AVX entries from `configs` recipe

**CLAUDE.md**: Update build config documentation to reflect new naming.

## Constraints

- Highway files continue to use per-file `/arch:AVX512` — this is what enables all SIMD
  targets for runtime dispatch. Do not remove this.
- SSE4.1 is the practical minimum for Skyrim hardware. `NotSet` (SSE2) is the compiler
  baseline; Highway's runtime dispatch provides SSE4.1/AVX2/AVX512 as available.
- CUDA/NOCUDA split is preserved for build-time reasons (nvcc dependency).

## Success Criteria

- [ ] Zero `_mm256` or `_mm512` intrinsics in non-Highway files
- [ ] Zero `_mm_dp_ps`, `_mm_hadd_ps` or similar SSE4.1 intrinsics in non-Highway files
- [ ] All existing unit tests pass
- [ ] Build succeeds for `V1_6_1170_NOCUDA` and `V1_6_1170_CUDA`
- [ ] 112 → 18 configurations in vcxproj
- [ ] CI green (build.yml uses new config names)
- [ ] `smp timing` shows no regression vs baseline
