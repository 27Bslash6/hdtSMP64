# hdtSMP64 Optimization History

> **Scope**: Changes made since this fork began active development (~2026-01-08).
> All numbers come from Tracy profiler traces, commit messages, and logged memory files.
> Estimated percentages are from real profiler data — not guesses.

---

## Starting Point

The upstream hdtSMP64 was effectively unmaintained since ~2022.
At fork time, the physics pipeline was:

- **Single-threaded constraint solving** via a custom lock-based `GroupConstraintSolver`
- **Blocking GPU sync** in the middle of every frame (`GlobalResultsSync: 3.8ms mean`)
- **No Tracy instrumentation** — zero visibility into frame time distribution
- **PPL-based parallelism** (Windows-only, no cross-platform path)
- **Raw pointer collision results** — stale pointers on shape reload causing CTDs
- **No CI** — builds were manual, test coverage was zero

---

## Phase 1 — Foundation (2026-01-08)

### Build System & CI

- Established GitHub Actions CI for automated builds across all game versions
- Fixed SKSE compatibility wrapper (`IsGamePaused`) for CI environment
- Fixed static assert portability issues (`WIN32_FIND_DATA` dependency removed)

### Memory Leak & Collision Bug Fix (`58602a1`)

- Fixed memory leaks in physics body lifecycle
- Fixed collision detection bugs that had been present in upstream

### Tracy Profiling Infrastructure (`5fd171b`)

- Added comprehensive Tracy instrumentation across the entire physics pipeline
- This was the prerequisite for all data-driven optimization work that followed
- Without this, all subsequent optimization claims would be untestable

---

## Phase 2 — CUDA Pipeline Optimizations (2026-01-08 to 2026-01-10)

### Frame-Based Dirty Flag (`8d69121`)

- Added per-body dirty tracking to skip redundant skinning updates
- Bodies that haven't moved skip the internal update entirely

### Per-Call Allocation Elimination (`476846d`)

- `ProcessCollision` was heap-allocating per call
- Replaced with pre-allocated thread-local buffers

### AVX2/AVX-512 Batch AABB Detection (`3844cd5`)

- Added SIMD batch collision detection for axis-aligned bounding boxes
- Used compile-time ISA dispatch (later replaced by Highway runtime dispatch)

### CUDA Graphs for Kernel Launch Batching (`9597a40`)

- Kernel launches have per-launch CPU overhead (~5-15μs each)
- CUDA Graphs capture the launch sequence once, replay cheaply
- Thread safety: static mutex with double-checked locking (graph capture is not re-entrant)

### CUDA Pipeline GPU/CPU Overlap (`d2e1f19`, `fddff6d`)

This was the most impactful single architectural change.

**Before:**
```
[Collision → → → [GlobalResultsSync 3.8ms] → → →] → [SolveConstraints]
```

**After:**
```
[SyncPrev 256μs] → [Collision launch only] → [SolveConstraints]
                                               ↑ GPU runs here in parallel
```

**Results from Tracy:**
| Metric | Before | After |
|--------|--------|-------|
| GlobalResultsSync | 3.8ms (17.4% of frame) | eliminated |
| SyncPreviousCollisions | — | 256μs (0.3% of frame) |
| Sync wait reduction | — | **15x** |

One-frame collision latency is acceptable: at 60fps that's 16.6ms, imperceptible for cloth/hair.

### CUDA Batched Collision Phase 1 (`9dd3ed6`)

**Result: 32% frame time reduction** (measured, not estimated)

- Restructured collision pair submission to batch GPU work
- Added bounds checking to merge buffer writes (`1b2b47e`)

### Parallel Frame Start (`3a259ee`)

- Hid ~1ms of CPU setup work inside the GPU sync window
- Free latency hiding with no code complexity cost

### Enable Mt Solver Batching (`012ded5`)

- Bullet's `btSequentialImpulseConstraintSolverMt` had batching disabled
- `s_minimumContactManifoldsForBatching` was 40 (too high for typical NPC counts)
- Reduced to 8 — enables graph-colored parallel batching for 5–20 NPC scenes
- Required 5 critical bug fixes to make stable:
  - Uninitialized solver pool base class
  - Division by zero in AVX constraint rows
  - Memory leak in solver pool destructor
  - Empty vector undefined behavior (`&vec[0]` on size-0 vector)
  - Non-atomic frame counter (`uint32_t` → `std::atomic<uint32_t>`)

---

## Phase 3 — CUDA Triple→Double Buffer Simplification (2026-01-17)

**Commit:** `fdc911b`

The CUDA collision result buffer was triple-buffered. Analysis showed:

- Frame N writes to `buffer[N % 3]`
- Frame N reads from `buffer[(N+1) % 3]`
- Third buffer was never simultaneously in-use

**Changed to double-buffer (ping-pong):**
- `buffer[N & 1]` write, `buffer[(N+1) & 1]` read
- Removed 33% of buffer memory allocation
- Fixed an off-by-one loop bound bug (`i < 3` → `i < 2`) that would have crashed on access to `[2]`

---

## Phase 4 — Concurrency Bug Fixes (2026-01-11 to 2026-01-17)

These were not optimizations — they were correctness fixes for crashes that were hard to reproduce. All grounded in root cause analysis from Tracy + log evidence.

### Option C: Index-Based Collider Access (`064bffb`)

**Root Cause:** `ColliderTree` stored raw `Collider*` and `Aabb*` pointers. On vector reallocation or task scheduling, these could point into a *different* shape's memory — one case logged a pointer 29MB off.

**Fix:** Eliminated pointer storage entirely. Store only `size_t colliderOffset`. Compute pointer on-demand from shape's current base address.

```cpp
// Before: Collider* cbuf (goes stale)
// After:  size_t colliderOffset (stable index)
Collider* getColliders(Collider* base) const { return base + colliderOffset; }
```

**Impact:** Removed custom copy/move constructors, `remapColliders()`, `refreshColliderPointers()` — entire class of stale pointer bugs eliminated.

### CollisionResult Pointer→Index Migration

**Root Cause:** `CollisionResult` stored raw `Collider*`. Shape destruction between `addResult()` and `doMerge()` made these stale.

**Fix:** `CollisionResult` now stores `size_t colliderIndexA/B`. Pointer computed at use time.

This aligned NOCUDA collision results with the CUDA path (`cuCollisionResult` already used integer indices).

### thread_local Vector Stale Data Fix

**Root Cause:** Collision lambdas used `thread_local std::vector<Aabb*> listA/listB` cleared at *end* of each call. Early returns (validation errors, `MaxCollisionCount` exceeded) left the vectors populated. Next invocation on the same thread would *append* stale entries.

**Fix:** Move `clear()` to *start* of each lambda. Simple, correct.

### enkiTS Re-entrancy Bug (`28b157a`)

This was the hardest bug. Root cause traced via log evidence:

```
[F:323 T:80216] addResult used sizes (1532,830) but doMerge has (511,208) idx=(224,576)
[F:323 T:80216] addResult used sizes (512,465) but doMerge has (511,208) idx=(408,10)
```

Multiple different source shape sizes in a single `doMerge` call — proved results from *different* body pairs were in the same buffer.

**Mechanism:**
1. Outer `hdt_parallel_for_each` processes body pairs
2. Each pair calls `processCollision` which uses `thread_local` buffers
3. Inside `processCollision`, inner `hdt_parallel_for_each` waits for workers
4. **enkiTS `WaitforTask` does work-stealing while waiting** — picks up outer queue tasks
5. Same thread re-entrantly calls `processCollision` for a *different* body pair
6. `thread_local` buffers corrupted by nested call

**Fix:** RAII re-entrancy guard. If re-entrant call detected, allocate heap buffers instead of reusing thread-local. Logs warning (limited to 10 per thread) when detected. Normal case: zero overhead.

### BUG-001: suspend() Race Condition

**Root Cause:** `suspend()` called `m_tasks.wait()` — but collision workers were spawned via the global enkiTS scheduler, *not* tracked by `AsyncTaskGroup`. "smp reset" during active collision = use-after-free.

**Fix:** `WorkerScope` RAII counter (`std::atomic<int> m_activeCollisionWorkers`). `suspend()` calls `waitForCollisionWorkers()` after `m_tasks.wait()`. The collision cancellation flag (`m_cancelCollisions`) allows workers to exit early.

### BUG-002: suspendSimulationUntilFinished() No Sync

`suspendSimulationUntilFinished()` set `m_isStasis = true` but didn't wait for in-flight async work. Papyrus callbacks that modify physics state could race with active collision workers.

**Fix:** Added `m_tasks.wait()` + `waitForCollisionWorkers()` before invoking the callback.

### BUG-003: FrameSyncEvent Early Return

Early return path in `FrameSyncEvent` didn't set `m_frameSyncComplete = true`. The `suspend()` caller spun until a 5-second timeout expired.

**Fix:** Set completion flag and notify in the early return path.

### Defense-in-Depth Collision Validation (3 layers)

1. **Dispatch loops**: Validate tree offset + numCollider < shape.size(), `continue` on failure
2. **addResult**: Validate computed index, `return false` (reject) on invalid — don't store bad indices
3. **doMerge**: Final bounds check with `continue` on any that slip through

Previously layer 2 logged the error but *stored the invalid index anyway*.

---

## Phase 5 — Expert Panel Review & TDD Hardening (2026-01-17)

Comprehensive architecture review (Bug Hunter, Security Specialist, Code Craftsman agents in parallel).

**Verdict:** APPROVED WITH CHANGES (7/10 — sound architecture, accumulated debt)

### 9 Bugs Fixed via TDD

**Wave 1 — Independent fixes:**

| Bug | Fix |
|-----|-----|
| BUG-003 | FrameSyncEvent signals completion on early return |
| M3 | `volatile bool m_initialized` → `std::atomic<bool>` |
| BUG-006 | Frame counter reset at `0xC0000000` (prevents overflow CTD at ~2.3 years) |
| BUG-007 | Division by zero guard in AVX constraint solver (epsilon check) |

**Wave 2 — Race condition chain (BUG-001/002):** Full `WorkerScope` + `waitForCollisionWorkers()` implementation.

**Wave 3 — Security hardening:**

| Issue | Fix |
|-------|-----|
| SEC-001 | Path traversal in save names — `isValidSaveName()` validation |
| SEC-002 | Unbounded XML resources — `MAX_HULL_POINTS=512`, `MAX_COLLIDE_LIST=64` |
| SEC-003 | Config bounds — `btClamped()` on `rotationSpeedLimit`, `maximumActiveSkeletons` |

**Test results after all fixes:** 7,847 assertions in 93 test cases — all passed. Both CUDA and NOCUDA builds.

---

## Phase 6 — Parallel Constraint Solver (2026-01-17)

**Commits:** `a00755a`, `7d2bdc3`

Enabled Bullet's built-in batched parallel solver that was previously bypassed entirely by the custom `GroupConstraintSolver`.

**How it works:**
1. `btConstraintSolverPoolMt` creates per-thread `btSequentialImpulseConstraintSolverMt` instances
2. When manifold count ≥ 8, `m_useBatching = true`
3. Graph coloring groups constraints into independent phases
4. Batches within each phase execute in parallel via enkiTS
5. No spinlocks (unlike the old `GroupConstraintSolver` approach)

**Dead code removed:** ~320 lines — `GroupConstraintSolver`, `SolverBodyMt`, `SolverTask` classes.

**Performance target:**
- Before: 14.3ms solver time at 100 entities (31.5% of frame)
- Target: <5ms via parallel batching

---

## Phase 7 — Highway SIMD Integration (2026-01-18)

**Commits:** `934853c`, `2a120d4`, `7b4022d`, `6b1ee9b`, `a114212`

Replaced compile-time ISA dispatch (build-time AVX2/AVX-512 selection) with Google Highway's runtime dispatch. A single binary now auto-selects the best SIMD path at startup.

**Runtime targets (auto-detected at launch):**
- SSE4.1 — baseline
- AVX2 — most CPUs (2013+)
- AVX3 — Intel Ice Lake+
- AVX3_ZEN4 — AMD Zen 4 (developer's CPU)

### What Was Highway-ified

| Component | File | Result |
|-----------|------|--------|
| Vertex skinning | `hdtHighwaySkinning.cpp` | Batch bone matrix transforms |
| AABB batch collision | `hdtHighwayAABB.cpp` | Per-vertex AABB computation |
| LCP dot product | `hdtHighwayLCP.cpp` | **6.77x speedup** (benchmarked) |
| LCP vector scale | `hdtHighwayLCP.cpp` | Element-wise scaling |
| Per-vertex AABB | `hdtHighwayPerVertex.cpp` | Replaces AVX-512 compile-time code |
| SoA constraint buffer | `hdtSoABuffer.cpp` | 64-byte aligned Structure-of-Arrays |

### Parallel AABB Update (`6b1ee9b`, `a114212`)

Two-phase parallel AABB with broadphase optimizations:

| Phase | Time | Notes |
|-------|------|-------|
| Compute (parallel) | 0.040ms | Fully parallelized via `btParallelFor` |
| Apply (sequential) | 0.692ms | DBVT tree mutations not thread-safe |
| ComputeOverlappingPairs | 0.536ms | Deferred collision pair detection |
| **Total** | **1.28ms** | vs 2.05ms before |

**Old vs New:**
- `UpdateAabbs`: 2.048ms → 0.748ms (-63%)
- Overall broadphase: 2.05ms → 1.28ms (-37%)
- Heavy load (100 entities): peaks went from 5.2ms → ~0.8ms (-85%)

### Velocity-Based AABB Skip (`a114212`)

Bodies with near-zero velocity skip AABB update entirely. Free performance on settled cloth.

### MSVC AVX-512 Fix

Highway disables AVX-512 on MSVC by default (`HWY_BROKEN_MSVC`). Fixed by `#define HWY_BROKEN_MSVC 0` for MSVC 19.40+ before any Highway headers. All Highway `.cpp` files compile with `/arch:AVX512` regardless of build config.

### Highway Constraint Solver — Investigated and Rejected (`ba07697`)

The Highway constraint solver (Phase 2 of the Highway workplan) was implemented and then disabled after investigation showed it was not viable:

- Friction constraint loops have `dynamic limit updates` that don't map to batch SIMD
- Typical friction batch size = 2 (`m_numFrictionDirections`) — SIMD overhead exceeds gain
- Contact/joint batches are larger but Bullet's existing `btSequentialImpulseConstraintSolverMt` parallelism already covers these
- Decision: leave constraint solver to Bullet's own batching, don't duplicate

---

## Phase 8 — CI Sanitizers (2026-01-20)

Added GitHub Actions workflows for:
- **Linux TSan** (Thread Sanitizer) — detects data races
- **Windows ASan** (Address Sanitizer) — detects memory safety bugs
- TSan suppressions for known false positives in Bullet Physics internals, enkiTS lock-free code, and Catch2

---

## Phase 9 — AVX Build Variant Elimination (2026-03-05 to 2026-03-06)

**Commits:** `5aa3c79` through `16f7e92`

The build matrix previously had an `AVX` dimension: `NoAVX`, `AVX`, `AVX2`, `AVX512`. This was:
- 4x more build configurations than needed
- Solved a problem Highway already solves (runtime dispatch)
- 112 vcxproj configuration entries → 16

**What was done:**
1. Stripped AVX variant configs from vcxproj (`112 → 16`)
2. Removed AVX variant entries from solution file
3. Replaced SSE4.1/AVX2 raw intrinsics in collision/shape files with scalar code (Highway handles these paths now)
4. Replaced AVX2 constraint solver with scalar (parallel solver handles this)
5. Removed `_AVX` suffix from all build configuration names
6. Updated CI matrix, justfile, and docs

**Result:** One binary with Highway runtime dispatch. No build variants for ISA.

---

## Cumulative Performance Summary

All numbers from Tracy profiler traces on a real game session. "Before" = upstream state at fork time.

| Metric | Before Fork | After | Change |
|--------|-------------|-------|--------|
| GlobalResultsSync | 3.8ms (17.4% frame) | eliminated | **-100%** |
| SyncPreviousCollisions | N/A | 256μs (0.3%) | replacement |
| UpdateAabbs (normal load) | 2.0ms | 0.75ms | **-63%** |
| UpdateAabbs (100 entities) | 5.2ms peak | ~0.8ms | **-85%** |
| CUDA batched collision | baseline | -32% frame | **measured** |
| LCP largeDot() | baseline | 6.77x faster | **benchmarked** |
| Constraint solver | single-threaded | parallel batched | qualitative |
| Build configurations | 112 (vcxproj) | 16 | **-86%** |
| Test coverage | 0 tests | 7,847 assertions / 93 cases | |
| Crash categories fixed | CTD on reset, stale pointers, re-entrancy | all addressed | |

---

## What's Still on the Table

Documented in `bullet_optimization_strategy_2026-01` memory:

| Opportunity | Estimated Gain | Complexity |
|-------------|---------------|------------|
| Remaining `hdtLCP.cpp` SIMD (`btSolveL1_*`) | 20-40% solver | Medium |
| Jolt Physics solver (replace Bullet solver only) | 50-100% solver | High |
| Full Jolt migration | 2-5x total | Very High |
| CUDA broadphase | significant | Very High |

**Current bottleneck**: `JointSolverLoop` is 74% of frame time at high NPC counts (Tracy data). Constraint solver is the remaining frontier.

---

*Generated 2026-03-07. Data from: Tracy profiler traces, git history, project memory files.*
