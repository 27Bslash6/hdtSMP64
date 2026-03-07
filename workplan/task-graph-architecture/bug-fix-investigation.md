# Bug Investigation: NOCUDA Physics Pipeline Crash - Stale Pointer Access

**Date**: 2026-01-10
**Severity**: CRITICAL
**Category**: Race Condition / Use-After-Free
**Status**: Identified, fix proposed

---

## Executive Summary

A race condition between `m_tasks.wait()` and parallel collision workers causes crashes during physics reset. The `suspend()` method only waits for the outer `AsyncTaskGroup` tasks, NOT the inner `hdt_parallel_for_each` workers spawned via the global enkiTS scheduler.

---

## Log Evidence

### Stale Pointer Errors (Frames 22-28)

```
[F:22 T:51080] doMerge: colliderA 0000028DDF96B580 out of range [...] - stale pointer! ownerA=colHeadBDO
[F:23 T:58548] doMerge: colliderA 0000028F8D7B8350 out of range [...] - stale pointer! ownerA=colWigMidBDO
[F:25 T:17392] doMerge: colliderA 0000028F8D818FC0 out of range [...] - stale pointer! ownerA=VirtualHead
[F:27 T:72196] doMerge: colliderB 0000028F90DE3640 out of range [...] - stale pointer! ownerB=BaLR
```

### Crash Sequence (Frame 28)

```
[F:28 T:63116] === SMP RESET: Starting full physics reset ===
[F:28 T:63116] SMP RESET: calling suspend()...
[F:28 T:63116] Physics suspend: loading=0, waiting for async tasks...
[F:28 T:47496] dispatchAllCollisionPairs: entering with 534 pairs  <-- Worker still running!
[F:28 T:7084] doMerge: colliderA ... stale pointer!  <-- Accessing freed memory
[F:28 T:44952] doMerge: colliderA ... stale pointer!  <-- Accessing freed memory
[F:28 T:63116] === HDT-SMP CRASH [Thread 63116] ===
Exception: Access violation reading address: 0x000002CDE123F2C8
Stack: enki::TaskScheduler::WaitforTask -> hdt::AsyncTaskGroup::wait -> hdt::SMPDebug_Execute
```

### Multiple Thread Crashes

Subsequent crashes at offset `0x839B6052` with garbage addresses:
- `0x00000000FFFFFFFF`
- `0x0000003FFFFFFFC0`
- `0x0000000000000002`
- `0x0000000000000037`

---

## Technical Root Cause

### Intent vs Implementation

**Intended Behavior:**
- `suspend()` should completely halt all physics work before returning
- No physics processing should occur during reset/load operations
- All collision processing should be complete before collision state is cleared

**Actual Implementation (Bug):**

```cpp
// hdtSkyrimPhysicsWorld.h:46-57
void suspend(bool loading = false)
{
    m_suspended = true;
    m_loading = loading;
    m_tasks.wait();  // <-- ONLY waits for AsyncTaskGroup tasks
}
```

The `m_tasks.wait()` waits for the `AsyncTaskGroup` which contains `doUpdate2ndStep` tasks. However, `doUpdate2ndStep` spawns **separate, independent** parallel workers via `hdt_parallel_for_each` that use the **global enkiTS scheduler**, NOT `m_tasks`.

### Execution Flow Analysis

**Normal Frame Flow:**
```
Frame N:
1. doUpdate() -> m_tasks.run(doUpdate2ndStep)     [queued to AsyncTaskGroup]
2. doUpdate2ndStep() -> stepSimulation()
3. stepSimulation() -> internalSingleStepSimulation()
4. internalSingleStepSimulation() -> performDiscreteCollisionDetection()
5. performDiscreteCollisionDetection() -> CollisionDispatcher::dispatchAllCollisionPairs()
6. dispatchAllCollisionPairs() -> hdt_parallel_for_each(m_pairs, processCollision)  [spawns enkiTS workers]
7. processCollision() -> MergeBuffer::doMerge()  [accesses colliders via pointers]
```

**Crash Scenario (Frame 28):**

```
Main Thread (T:63116):                          Worker Threads (T:47496, T:7084, T:44952):
============================                    ============================================
1. User triggers "smp reset"
2. suspend() called
3. m_suspended = true
4. m_tasks.wait() called                        [Workers from Frame 27 still running!]
   - Waits for doUpdate2ndStep                  - dispatchAllCollisionPairs() active
   - doUpdate2ndStep sees m_suspended=true      - hdt_parallel_for_each() in progress
   - doUpdate2ndStep returns early              - processCollision() executing
5. m_tasks.wait() returns                       - doMerge() accessing colliders
   (thinks all work is done)
6. resetSystems() called
7. clearCollisionState()                        - WORKERS STILL ACCESSING m_pairs!
8. reregisterAllBodies()
   - Bodies freed/reallocated                   - doMerge() accesses freed memory
9. CRASH: Access violation                      - "stale pointer" detected
```

### The Race Window

If the `m_suspended` check in `doUpdate2ndStep` passes and collision work begins, setting `m_suspended = true` afterwards does NOT stop the in-progress collision work:

```cpp
void SkyrimPhysicsWorld::doUpdate2ndStep(...)
{
    if (m_suspended || m_isStasis) {  // <-- Check happens HERE
        return;  // Early exit only if ALREADY suspended
    }
    // ... collision work proceeds if we get past this point
    // Setting m_suspended=true AFTER this point doesn't stop it
}
```

---

## Files Involved

| File | Line | Issue |
|------|------|-------|
| `hdtSkyrimPhysicsWorld.h` | 46-57 | `suspend()` only waits for `m_tasks`, not enkiTS workers |
| `hdtDispatcher.cpp` | 348-370 | No cancellation mechanism for parallel collision (NOCUDA path) |
| `hdtSkinnedMeshAlgorithm.cpp` | 558+ | `doMerge` accesses colliders without checking for reset |
| `hdtEnkiTSScheduler.h` | 128-214 | `hdt_parallel_for_each` uses global scheduler, not tracked by `m_tasks` |

---

## Proposed Fixes

### Option 1: Atomic Cancellation Flag (Recommended)

Add an atomic flag that collision workers check periodically.

**CollisionDispatcher additions:**
```cpp
class CollisionDispatcher : public btCollisionDispatcherMt
{
public:
    std::atomic<bool> m_cancelCollisions{false};

    void requestCollisionCancellation() { m_cancelCollisions.store(true, std::memory_order_release); }
    void clearCollisionCancellation() { m_cancelCollisions.store(false, std::memory_order_release); }
    bool isCollisionCancelled() const { return m_cancelCollisions.load(std::memory_order_acquire); }
};
```

**In dispatchAllCollisionPairs NOCUDA path:**
```cpp
hdt_parallel_for_each(m_pairs.begin(), m_pairs.end(),
    [&, this](const std::pair<SkinnedMeshBody*, SkinnedMeshBody*>& i) {
        if (isCollisionCancelled()) return;  // CHECK CANCELLATION

        if (i.first->m_shape->m_tree.collapseCollideL(&i.second->m_shape->m_tree))
            SkinnedMeshAlgorithm::processCollision(i.first, i.second, this);
    });
```

**In suspend():**
```cpp
void suspend(bool loading = false)
{
    // Set cancellation flag FIRST to stop in-flight collision workers
    if (m_dispatcher1) {
        static_cast<CollisionDispatcher*>(m_dispatcher1)->requestCollisionCancellation();
    }

    m_suspended = true;
    m_loading = loading;
    m_tasks.wait();

    // Brief pause for workers to exit (enkiTS is cooperative)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
```

### Option 2: Generation Counter

Add a generation counter that increments on reset. Workers capture generation at start and check before accessing shared state:

```cpp
std::atomic<uint32_t> m_generationCounter{0};

// Worker captures generation at start
uint32_t myGeneration = dispatcher->currentGeneration();
// ... later, before any memory access ...
if (dispatcher->currentGeneration() != myGeneration) return; // Reset happened
```

### Option 3: Full Scheduler Synchronization (Heavy-Handed)

Wait for ALL enkiTS tasks:
```cpp
EnkiTSScheduler::get().scheduler().WaitforAllAndShutdown();
EnkiTSScheduler::get().scheduler().Initialize();
```

**Drawback**: Kills and re-initializes entire scheduler.

---

## Bugs Summary

| # | Severity | Category | Location | Description |
|---|----------|----------|----------|-------------|
| 1 | CRITICAL | Race Condition | `hdtSkyrimPhysicsWorld::suspend()` | `m_tasks.wait()` doesn't wait for parallel collision workers |
| 2 | HIGH | Use-After-Free | `MergeBuffer::doMerge()` | Accesses collider pointers that may be freed during reset |
| 3 | MEDIUM | Missing Synchronization | `dispatchAllCollisionPairs` | No mechanism to cancel in-flight parallel work |

---

## Recommended Fix Priority

1. **Implement Option 1** (Atomic Cancellation Flag) - Low risk, targeted fix
2. The stale pointer validation in `doMerge` already exists as secondary defense (logs errors instead of crashing)
3. Consider Option 2 (Generation Counter) for additional safety in future versions

---

## Test Case for Verification

```cpp
// Test: Rapid reset during heavy collision processing
// 1. Load scene with many colliding physics objects (100+ pairs)
// 2. While physics actively running, trigger "smp reset" via console
// 3. Verify no crash occurs
// 4. Verify log shows "collision cancelled" instead of stale pointer errors
```

---

## Related Architecture

See `docs/architecture/` for full pipeline documentation:
- `02-FRAME-LIFECYCLE.md` - Async dispatch and sync points
- `03-COLLISION-PIPELINE.md` - NOCUDA collision processing
- `06-THREADING-EXPERT.md` - enkiTS parallelism patterns

---
---

# Bug Investigation #2: Loading-Time Stale Pointer - Cross-Thread Memory Visibility

**Date**: 2026-01-10
**Severity**: CRITICAL
**Category**: Memory Visibility / Threading
**Status**: Fix implemented, AWAITING IN-GAME TESTING
**Key Clue**: "THIS WORKED BEFORE THE MOVE TO ENKI"

---

## Executive Summary

During game loading, `colliderOffset` values set by `remapColliders()` on thread T:73596 are not visible to `refreshColliderPointers()` running on thread T:38268. The `colliderOffset` field is a plain `size_t` without atomics or memory fences, causing cross-thread visibility failures after the PPL→enkiTS migration.

---

## Log Evidence Timeline

### Phase 1: Shapes Built (Thread T:73596)
```
[20:09:37.xxx T:73596] finishBuild: shape=0x28DE0974CE0 (colWigMidBDO)
                       m_colliders.data()=0x8BC43520 size=1654
[20:09:37.xxx T:73596] remapColliders: sets colliderOffset for each tree node
```

### Phase 2: Systems Added (Thread T:38268) - 3 seconds later
```
[20:09:40.210 T:38268] World::addSkinnedMeshSystem: 11 systems with 94 shapes
[20:09:40.338 T:38268] refreshColliderPointers called for each shape
[20:09:40.338 T:38268] Shape::refreshColliderPointers: shape=0x28DE0974CE0
                       m_colliders.data()=0x8BC43520 size=1654 tree.cbuf=???
```

### Phase 3: Collision Detection - Stale Pointers (Frame 21+)
```
[F:21 T:xxxxx] doMerge: colliderA 0x8D818930 out of range
              [0x8BC43520, 0x8BC49CC0) - stale pointer!
              ownerA=colWigMidBDO ownerB=colHeadBDO
```

### Critical Observation
- Shape colWigMidBDO has `m_colliders` at `[0x8BC43520, 0x8BC49CC0)`
- But `cbuf` points to `0x8D818930` - which is **29MB away**
- Address `0x8D818930` is INSIDE colHeadBDO's `m_colliders` range at `0x8D815D80`
- This means `cbuf` points to the WRONG shape's colliders!

---

## Technical Root Cause

### The colliderOffset Field

```cpp
// hdtCollider.h - ColliderTree structure
class ColliderTree
{
    // ... other fields ...
    Collider* cbuf = nullptr;      // Plain pointer
    size_t colliderOffset = 0;     // Plain size_t - NO ATOMICS
    Aabb* aabb = nullptr;
    // ...
};
```

### The Problem Flow

**Thread A (T:73596) - finishBuild:**
```cpp
void PerVertexShape::finishBuild()
{
    m_tree.exportColliders(m_colliders);  // Fills colliders, sets cbuf as offset
    m_tree.remapColliders(m_colliders.data(), ...);  // Converts offset to pointer
    // colliderOffset = rawCbuf (the offset value) stored here
}
```

**Thread B (T:38268) - addSkinnedMeshSystem:**
```cpp
void SkinnedMeshWorld::addSkinnedMeshSystem(...)
{
    for (auto& mesh : system->meshes()) {
        mesh->m_shape->refreshColliderPointers();  // Reads colliderOffset
    }
}
```

### Memory Visibility Issue

Without a memory fence or atomic operation:
1. Thread A writes `colliderOffset = rawCbuf` to L1/L2 cache
2. Thread A may not flush to main memory immediately
3. Thread B reads `colliderOffset` from main memory (or its own stale cache line)
4. Thread B gets **uninitialized or stale value** (could be 0, garbage, or old pointer value)

### Why PPL Worked, enkiTS Doesn't

PPL (Parallel Patterns Library) uses Windows thread pool with stronger memory ordering guarantees:
- PPL task completion includes implicit memory fences
- PPL work-stealing has synchronization that flushes caches

enkiTS uses a lighter-weight cooperative scheduler:
- Minimal synchronization overhead (by design)
- No implicit memory fences between task completion and parent notification
- Relies on explicit atomic/fence usage for cross-thread visibility

---

## Evidence of Cross-Thread Access

From the log, threading pattern:
```
finishBuild runs on:           T:73596  (Physics worker)
addSkinnedMeshSystem runs on:  T:38268  (Game main thread)
refreshColliderPointers:       T:38268  (Same as addSkinnedMeshSystem)
```

The fact that `finishBuild` and `refreshColliderPointers` run on **different threads** with no synchronization between them is the root cause.

---

## Proposed Fix

### Option 1: Add Memory Fence After Setting colliderOffset (Minimal)

```cpp
void ColliderTree::remapColliders(Collider* start, Aabb* startAabb)
{
    colliders.swap(vectorA16<Collider>());
    size_t rawCbuf = (size_t)cbuf;

    colliderOffset = rawCbuf;  // Store offset
    std::atomic_thread_fence(std::memory_order_release);  // FENCE HERE

    cbuf = start + colliderOffset;
    aabb = startAabb + colliderOffset;

    for (auto& i : children)
        i.remapColliders(start, startAabb);
}

void ColliderTree::refreshColliderPointers(Collider* newBase, Aabb* newAabbBase)
{
    std::atomic_thread_fence(std::memory_order_acquire);  // FENCE HERE

    cbuf = newBase + colliderOffset;
    aabb = newAabbBase + colliderOffset;

    for (auto& i : children)
        i.refreshColliderPointers(newBase, newAabbBase);
}
```

### Option 2: Make colliderOffset Atomic (Cleaner)

```cpp
// hdtCollider.h
std::atomic<size_t> colliderOffset{0};

// In remapColliders:
colliderOffset.store(rawCbuf, std::memory_order_release);

// In refreshColliderPointers:
cbuf = newBase + colliderOffset.load(std::memory_order_acquire);
```

### Option 3: Force Same-Thread Execution (Heavy-Handed)

Ensure `finishBuild` and `refreshColliderPointers` always run on same thread by queuing work.

---

## Verification Steps

1. Add VERBOSE logging for colliderOffset values at both write and read points
2. Implement atomic fix
3. Build with `just profile V1_6_1170_NOCUDA_AVX2`
4. Test with save that triggers the crash
5. Verify no stale pointer errors in log

---

## Files to Modify

| File | Change |
|------|--------|
| `hdtSkinnedMesh/hdtCollider.h` | Change `size_t colliderOffset` to `std::atomic<size_t>` |
| `hdtSkinnedMesh/hdtCollider.cpp` | Use atomic load/store with acquire/release semantics |

---

## Test Case

```
1. Load game with physics-enabled character (hair, clothing, accessories)
2. Fast travel or load save with multiple physics systems
3. Verify Frame 21+ has no stale pointer errors
4. Verify collision works correctly (visual + no CTD)
```

---

## Implementation Log (2026-01-10)

### What Was Done

1. **Made `colliderOffset` atomic** in `hdtCollider.h:63`:
   ```cpp
   std::atomic<size_t> colliderOffset{0};
   ```

2. **Added copy/move semantics** to `ColliderTree` (lines 51-123) because `std::atomic` is not copyable and `ColliderTree` is stored in vectors.

3. **Updated `remapColliders()`** in `hdtCollider.cpp:244-248`:
   ```cpp
   colliderOffset.store(rawCbuf, std::memory_order_release);
   cbuf = start + rawCbuf;
   aabb = startAabb + rawCbuf;
   ```

4. **Updated `refreshColliderPointers()`** in `hdtCollider.cpp:256-258`:
   ```cpp
   const size_t offset = colliderOffset.load(std::memory_order_acquire);
   ```

5. **Updated diagnostic logging** in `hdtSkinnedMeshShape.h` to use `.load()` for atomic access.

### Build Status
- NOCUDA build: ✅ Compiles
- CUDA build: ✅ Compiles
- Unit tests: ✅ 467 assertions pass

### What Has NOT Been Done
- **IN-GAME TESTING** - The actual crash scenario has not been reproduced/verified fixed
- This fix is based on theory (cross-thread memory visibility) which may or may not be the root cause

### Previous Attempts That Failed
This is attempt #N at fixing stale cbuf pointers. Previous work included:
- Adding `refreshColliderPointers()` method
- Adding `colliderOffset` field to store index
- Adding diagnostic logging throughout the pipeline
- Multiple iterations of pointer refresh logic

### Risk Assessment
The atomic fix adds overhead to every ColliderTree copy/move operation. If the theory is wrong, we've added complexity without fixing the bug.

---
---

# Expert Panel Review (2026-01-10)

**Review Date**: 2026-01-10
**Reviewers**: bug-hunter-supreme, security-specialist, code-craftsman
**Subject**: Bug #1 and Bug #2 proposed fixes

---

## Panel Summary

| Expert | Critical | Major | Minor |
|--------|----------|-------|-------|
| Bug Hunter | 0 | 3 | 1 |
| Security | 2 | 2 | 3 |
| Code Craftsman | 1 | 6 | 2 |
| **Total** | **3** | **11** | **6** |

## Verdict: BLOCKED

The proposed fixes for Bug #1 are **incomplete and potentially dangerous**. The 10ms sleep is unanimously condemned as an anti-pattern. Bug #2 atomic fix is correct but has an ordering issue.

---

## Critical Issues (Must Fix Before Merge)

### 1. 10ms Sleep is NOT a Synchronization Mechanism

**All three experts flagged this as unacceptable.**

| Expert | Assessment |
|--------|------------|
| Bug Hunter | "10ms sleep is unreliable - workers might take longer" |
| Security | "Fixed-duration sleep is NEVER acceptable synchronization" (CWE-362, CWE-820) |
| Code Craftsman | "Indefensible garbage. Race condition fix via prayer." |

**Why it fails:**
- On a loaded system, 10ms may not be enough
- On a fast system, it wastes time
- Provides no correctness guarantee - just makes crash "probably less likely"
- DoS vector: rapid reset spam accumulates delays

**Required Fix**: Replace with proper worker tracking:
```cpp
std::atomic<int> m_activeCollisionWorkers{0};

void waitForCollisionWorkers() {
    while (m_activeCollisionWorkers.load(std::memory_order_acquire) > 0)
        std::this_thread::yield();
}
```

### 2. TOCTOU Race Window Still Exists

**CWE-416 (Use After Free), CWE-367 (TOCTOU)**

The proposed fix doesn't fully close the race. Between checking `m_suspended` and using physics objects, another thread can destroy those objects.

**Bug Hunter clarification**: The original analysis claimed `hdt_parallel_for_each` uses the global scheduler independently, but examination shows it's actually **synchronous** - it waits for completion. The real race is between:
1. `FrameEvent` arriving and `doUpdate()` starting BEFORE suspend completes
2. Worker passing the `m_suspended` check BEFORE flag is set

**Required Fix**: Epoch counter or reader-writer lock:
```cpp
std::atomic<uint64_t> m_epoch{0};

void suspend() {
    m_epoch++;  // Increment BEFORE setting suspended
    m_suspended = true;
    m_tasks.wait();
}

void doUpdate2ndStep(uint64_t capturedEpoch, ...) {
    if (capturedEpoch != m_epoch.load() || m_suspended) return;
    // ... rest of function
}
```

### 3. Atomic Store/Load Ordering Issue in Bug #2 Fix

**CWE-362, CWE-567 | Severity: MAJOR on x86-64, CRITICAL on ARM64**

The atomic store of `colliderOffset` happens BEFORE `cbuf` and `aabb` pointer updates:

```cpp
// CURRENT (problematic order):
colliderOffset.store(rawCbuf, std::memory_order_release);  // Line 246
cbuf = start + rawCbuf;   // Line 247 - AFTER the fence!
aabb = startAabb + rawCbuf;  // Line 248 - AFTER the fence!
```

The release fence only applies to writes BEFORE it. `cbuf` and `aabb` writes happen AFTER, so they're not synchronized.

**This masks on x86-64 (TSO memory model) but BREAKS on ARM64.**

**Required Fix**: Reorder so atomic store is truly last:
```cpp
cbuf = start + rawCbuf;
aabb = startAabb + rawCbuf;
colliderOffset.store(rawCbuf, std::memory_order_release);  // NOW LAST
```

---

## Major Issues (Should Fix)

| # | Issue | Source | Notes |
|---|-------|--------|-------|
| 1 | `static_cast<CollisionDispatcher*>` violates DIP | Code Craftsman | Use `dynamic_cast` or store concrete type |
| 2 | No RAII for cancellation flag lifecycle | Code Craftsman | Add `CollisionCancellationScope` guard |
| 3 | Missing `clearCollisionCancellation()` in `resume()` | Security | Flag stays set forever otherwise |
| 4 | Integer overflow in pointer arithmetic not validated | Security | Add bounds check before `newBase + offset` |
| 5 | `suspend()` knows too much about dispatcher internals | Code Craftsman | Encapsulate cancellation logic |
| 6 | No unit tests for cancellation behavior | Code Craftsman | Add stress tests |
| 7 | Missing threading model documentation | Code Craftsman | Document which threads run what |
| 8 | `CudaInterface::addCollisionPair` thread safety unverified | Bug Hunter | Audit for race conditions |
| 9 | Need ThreadSanitizer in CI | Bug Hunter | Catch future race conditions |
| 10 | Console `smp reset` has no rate limiting | Security | Add 2-second cooldown |
| 11 | Weak_ptr TOCTOU in `bodiesValid()` | Security | Lock both weak_ptrs atomically |

---

## Minor Issues

| # | Issue | Source |
|---|-------|--------|
| 1 | `CollisionDispatcher` taking on cancellation role (SRP) | Code Craftsman |
| 2 | Missing documentation for memory ordering rationale | Code Craftsman |
| 3 | Cancellation token pattern not used (C++20 style) | Code Craftsman |
| 4 | `hdt_parallel_for_each` non-random-access iterator fragility | Bug Hunter |
| 5 | ARM64 memory model needs expert review | Security |
| 6 | `m_delayedFuncs` access pattern needs audit | Bug Hunter |

---

## Positive Observations

1. **Bug #2 atomic approach is correct** - acquire/release semantics are appropriate (all experts agree)
2. **Extensive diagnostic logging** - helps identify exploitation attempts
3. **`m_pairs` race already fixed** - lock guard now covers all mutations
4. **Weak_ptr for lifecycle management** - sound pattern for preventing dangling pointers
5. **Tracy instrumentation** - aids performance anomaly detection
6. **Collision pair limit** - `CUDA_MAX_COLLISION_PAIRS = 2'000'000` prevents resource exhaustion

---

## Corrected Fix for Bug #1

Based on expert panel consensus:

```cpp
class CollisionDispatcher : public btCollisionDispatcherMt
{
public:
    std::atomic<bool> m_cancelCollisions{false};
    std::atomic<int> m_activeCollisionWorkers{0};

    void requestCollisionCancellation() {
        m_cancelCollisions.store(true, std::memory_order_release);
    }

    void clearCollisionCancellation() {
        m_cancelCollisions.store(false, std::memory_order_release);
    }

    bool isCollisionCancelled() const {
        return m_cancelCollisions.load(std::memory_order_acquire);
    }

    void waitForCollisionWorkers() {
        while (m_activeCollisionWorkers.load(std::memory_order_acquire) > 0) {
            std::this_thread::yield();
        }
    }

    // RAII guard for worker tracking
    struct WorkerScope {
        CollisionDispatcher* d;
        WorkerScope(CollisionDispatcher* d) : d(d) {
            d->m_activeCollisionWorkers.fetch_add(1, std::memory_order_acq_rel);
        }
        ~WorkerScope() {
            d->m_activeCollisionWorkers.fetch_sub(1, std::memory_order_acq_rel);
        }
    };
};

// In suspend():
void suspend(bool loading = false)
{
    // Use dynamic_cast for safety
    if (auto* cd = dynamic_cast<CollisionDispatcher*>(m_dispatcher1)) {
        cd->requestCollisionCancellation();
    }

    m_suspended = true;
    m_loading = loading;
    m_tasks.wait();

    // Wait for ALL collision workers - NO FIXED SLEEP
    if (auto* cd = dynamic_cast<CollisionDispatcher*>(m_dispatcher1)) {
        cd->waitForCollisionWorkers();
    }
}

// In resume():
void resume()
{
    if (auto* cd = dynamic_cast<CollisionDispatcher*>(m_dispatcher1)) {
        cd->clearCollisionCancellation();
    }
    m_suspended = false;
}

// In parallel collision loop:
hdt_parallel_for_each(m_pairs.begin(), m_pairs.end(),
    [&, this](const std::pair<SkinnedMeshBody*, SkinnedMeshBody*>& i) {
        WorkerScope scope(this);  // RAII tracking

        if (isCollisionCancelled()) return;  // Early exit

        if (i.first->m_shape->m_tree.collapseCollideL(&i.second->m_shape->m_tree))
            SkinnedMeshAlgorithm::processCollision(i.first, i.second, this);
    });
```

---

## Corrected Fix for Bug #2

Reorder the atomic store to be AFTER the pointer updates:

```cpp
void ColliderTree::remapColliders(Collider* start, Aabb* startAabb)
{
    colliders.swap(vectorA16<Collider>());
    size_t rawCbuf = (size_t)cbuf;

    // Update pointers FIRST
    cbuf = start + rawCbuf;
    aabb = startAabb + rawCbuf;

    // THEN store offset with release semantics (synchronizes ALL above writes)
    colliderOffset.store(rawCbuf, std::memory_order_release);

    for (auto& i : children)
        i.remapColliders(start, startAabb);
}
```

---

## Recommended Action Plan

### Blocking (Before Any Merge)
1. Remove the 10ms sleep - replace with worker counter
2. Fix atomic ordering in `hdtCollider.cpp`
3. Add `clearCollisionCancellation()` call in `resume()`

### High Priority (This Sprint)
4. Add RAII scope guard for cancellation
5. Add rate limiting to `smp reset` console command
6. Add ThreadSanitizer build to CI
7. Document threading model

### Medium Priority (Next Sprint)
8. Make `hdt_parallel_for_each` cancellation-aware at primitive level
9. Add stress tests for reset-during-collision scenarios
10. Audit `CudaInterface::addCollisionPair` thread safety

---

## Status Update

- **Bug #1**: Original fix REJECTED - 10ms sleep unacceptable. Corrected fix specified above.
- **Bug #2**: **SUPERSEDED BY OPTION C** - See below.

---
---

# Bug #2 Resolution: Option C Implementation (2026-01-10)

**Status**: IMPLEMENTED - AWAITING IN-GAME TESTING
**Approach**: Eliminate pointer storage entirely - store only offsets, compute pointers on-the-fly

---

## Why Previous Fixes Failed

Options A and B (atomic ordering, memory fences) attempted to fix cross-thread visibility of pointers. But the fundamental problem was deeper:

**Pointers themselves are the problem.** When ColliderTree nodes are copied (during vector reallocation, task scheduling, etc.), any stored pointer can become stale if it points to another shape's data.

The log evidence showed `cbuf` pointing 29MB away into a *different shape's* collider array - this isn't a visibility issue, it's a **cross-shape pointer contamination** issue caused by shallow copies.

---

## Option C: Index-Based Access (Implemented)

**Core Principle**: Never store pointers that can become stale. Store only indices/offsets. Compute actual pointers on-the-fly when needed.

### Changes Made

#### 1. ColliderTree Structure (hdtCollider.h)

**Before:**
```cpp
struct ColliderTree {
    Collider* cbuf = nullptr;           // POINTER - can become stale
    Aabb* aabb = nullptr;               // POINTER - can become stale
    std::atomic<size_t> colliderOffset; // Tried to sync with atomics
    // Custom copy/move to handle atomics...
};
```

**After:**
```cpp
struct ColliderTree {
    // NO POINTERS - eliminates cross-shape contamination entirely
    size_t colliderOffset = 0;  // Plain index into owning shape's vectors
    U32 numCollider = 0;

    // Default copy/move - offset is just a number, safe to copy
    ColliderTree(const ColliderTree&) = default;
    ColliderTree& operator=(const ColliderTree&) = default;

    // Helpers compute pointers on-the-fly
    Collider* getColliders(Collider* base) const { return base + colliderOffset; }
    Aabb* getAabbs(Aabb* base) const { return base + colliderOffset; }
};
```

#### 2. SkinnedMeshShape Base Accessors (hdtSkinnedMeshShape.h)

Added accessors to get base pointers for each shape:
```cpp
Collider* getColliderBase() { return m_colliders.data(); }
Aabb* getAabbBase() { return m_aabb.data(); }  // or m_aabb.get() for CUDA
```

#### 3. Collision Algorithm (hdtSkinnedMeshAlgorithm.cpp)

**Before:**
```cpp
auto abeg = a->aabb;    // Used stored pointer
auto acbuf = a->cbuf;   // Used stored pointer
```

**After:**
```cpp
// Compute pointers from base + offset (no stored pointers)
auto abeg = aabbBaseA + a->colliderOffset;
auto acbuf = colliderBaseA + a->colliderOffset;
```

#### 4. CUDA Code (hdtCudaInterface.cpp)

**Before:**
```cpp
*nodeData++ = {tree.aabb - m_tree->aabb, tree.numCollider};  // Pointer subtraction
collisionPair.addPair(a->cbuf - shape0->m_colliders.data(), ...);  // Pointer subtraction
```

**After:**
```cpp
*nodeData++ = {static_cast<unsigned int>(tree.colliderOffset), tree.numCollider};  // Direct offset
collisionPair.addPair(a->colliderOffset, b->colliderOffset, ...);  // Direct offset
```

#### 5. Tree Building (hdtCollider.cpp)

- `exportColliders()` now stores offset directly in `colliderOffset`
- `updateAabb()` now takes base pointer parameter
- Added `finalizeOffsets()` to clear temporary collider vectors
- Removed `remapColliders()` and `refreshColliderPointers()` entirely

---

## Files Modified

| File | Changes |
|------|---------|
| `hdtCollider.h` | Removed cbuf/aabb pointers, simplified to plain offset |
| `hdtCollider.cpp` | Updated updateAabb, exportColliders, added finalizeOffsets |
| `hdtSkinnedMeshShape.h` | Added getColliderBase/getAabbBase accessors |
| `hdtSkinnedMeshShape.cpp` | Updated finishBuild, internalUpdate |
| `hdtSkinnedMeshWorld.cpp` | Removed refreshColliderPointers calls |
| `hdtSkinnedMeshAlgorithm.cpp` | Updated all collision code to use base+offset |
| `hdtCudaInterface.cpp` | Updated to use colliderOffset directly |

---

## Why This Works

1. **No pointers to contaminate**: ColliderTree stores only `colliderOffset` (a size_t). When copied, it's just copying a number.

2. **Each access computes fresh pointer**: `shapeA->getColliderBase() + tree.colliderOffset` always yields correct pointer for *this* shape.

3. **Impossible to point to wrong shape**: The base pointer comes from the shape being processed, not from some stale cached value.

4. **Simpler code**: No atomics, no memory fences, no refresh functions. Default copy/move just works.

---

## Build Verification

```
✅ V1_6_1170_NOCUDA_AVX2 - Build succeeded
✅ V1_6_1170_CUDA_AVX2 - Build succeeded
✅ Unit tests - 467 assertions in 47 test cases pass
```

---

## Remaining Work

1. **IN-GAME TESTING** - Must verify CTD during loading is fixed
2. Bug #1 (suspend race condition) still needs the corrected fix with worker counter

---

## Lessons Learned

1. **Don't store pointers across thread boundaries** when the pointed-to data can move
2. **Indices/offsets are safer** than pointers for data that gets copied
3. **Simplest solution wins**: Removing pointer storage entirely is cleaner than trying to synchronize pointer updates
