# Expert Panel Architecture Review - January 2026

## Executive Summary

Comprehensive review of the hdtSMP64 physics pipeline by three specialist agents:
- **Bug Hunter Supreme**: Race conditions, memory safety, edge cases
- **Security Specialist**: Input validation, memory hooks, config handling
- **Code Craftsman**: SOLID principles, maintainability, modernization

| Expert | Critical | Major | Minor |
|--------|----------|-------|-------|
| Bug Hunter | 5 | 4 | 6 |
| Security Specialist | 2 | 3 | 5 |
| Code Craftsman | 2 | 5 | 4 |
| **Total (deduplicated)** | **7** | **10** | **12** |

**VERDICT**: APPROVED WITH CHANGES (7/10 - sound architecture, accumulated debt)

---

## Critical Issues (Must Fix Before Feature Work)

### BUG-001: Race Condition in suspend() - Collision Workers Not Tracked
**Severity**: CRITICAL | **Category**: Race Condition / Use-After-Free

The `suspend()` function waits only for `AsyncTaskGroup::m_tasks`, but parallel collision workers spawned via `hdt_parallel_for_each` in `dispatchAllCollisionPairs()` use the global enkiTS scheduler and are NOT tracked.

**Crash Scenario**:
1. Physics frame in progress, collision workers running
2. User triggers "smp reset" on console thread
3. `suspend()` waits for `m_tasks`, sets complete
4. `resetSystems()` destroys physics bodies
5. **CRASH**: Collision workers still running access freed memory

**Location**: `hdtSkyrimPhysicsWorld.h:49-88`, `hdtDispatcher.cpp:100-163`

**Fix**:
```cpp
// In CollisionDispatcher
std::atomic<int> m_activeCollisionWorkers{0};
std::atomic<bool> m_cancelCollisions{false};

void waitForCollisionWorkers() {
    while (m_activeCollisionWorkers.load(std::memory_order_acquire) > 0)
        std::this_thread::yield();
}

// In suspend() after m_frameSyncCV.wait():
static_cast<CollisionDispatcher*>(m_dispatcher1)->waitForCollisionWorkers();
```

---

### BUG-002: suspendSimulationUntilFinished() Has No Synchronization
**Severity**: CRITICAL | **Category**: Race Condition

Sets `m_isStasis = true` but does NOT wait for in-progress async work. Called from Papyrus functions that modify physics systems while workers may be active.

**Location**: `hdtSkyrimPhysicsWorld.cpp:171-185`

**Fix**:
```cpp
void SkyrimPhysicsWorld::suspendSimulationUntilFinished(std::function<void(void)> process)
{
    m_isStasis = true;
    m_tasks.wait();  // ADD THIS
    // Also wait for collision workers if implemented per BUG-001
    try { process(); }
    // ...
    m_isStasis = false;
}
```

---

### BUG-003: FrameSyncEvent Early Return Doesn't Signal Completion
**Severity**: CRITICAL | **Category**: Logic Error

**Location**: `hdtSkyrimPhysicsWorld.cpp:428-462`

**Fix**:
```cpp
void SkyrimPhysicsWorld::onEvent(const FrameSyncEvent& e)
{
    if (m_suspended) {
        // Still need to signal completion for any waiting suspend() call
        m_frameSyncComplete.store(true, std::memory_order_release);
        m_frameSyncCV.notify_all();
        return;
    }
    // ...
}
```

---

### C1: Preprocessor Soup in dispatchAllCollisionPairs()
**Severity**: CRITICAL | **Category**: Maintainability

250+ line function with interleaved `#ifdef CUDA` blocks. The non-CUDA path's closing brace is 270 lines from its opening.

**Location**: `hdtDispatcher.cpp:76-369`

**Fix**: Extract `dispatchCollisionPairs_CUDA()` and `dispatchCollisionPairs_NoCUDA()` as separate private methods. Share common code via helpers.

---

### C2: Magic Numbers in Solver
**Severity**: CRITICAL | **Category**: Undocumented

```cpp
if (iteration <= (maxIterations * 3 + 3) / 4)  // What does this mean?
```

**Location**: `hdtGroupConstraintSolver.cpp:458`

**Fix**:
```cpp
// After 75% of iterations, switch from combined task solving to separate
// contact/non-contact solving for better convergence
static constexpr float TASK_SPLIT_THRESHOLD = 0.75f;
const int splitIteration = static_cast<int>(maxIterations * TASK_SPLIT_THRESHOLD);
```

---

### SEC-001: Path Traversal in Save File Handling
**Severity**: HIGH | **CWE**: CWE-22

**Location**: `main.cpp:896-912`

**Fix**:
```cpp
bool isValidSaveName(const std::string& name) {
    return std::all_of(name.begin(), name.end(), [](char c) {
        return std::isalnum(c) || c == ' ' || c == '_' || c == '-';
    });
}
```

---

### SEC-004: Detours Hooks Without Offset Validation
**Severity**: HIGH | **CWE**: CWE-787

Hardcoded memory offsets without runtime signature validation.

**Location**: `Offsets.h`, `Hooks.cpp:357-367`

**Fix**: Add signature validation before hooking (pattern scan or known bytes check).

---

## Major Issues (Should Fix)

| ID | Issue | Location | Fix |
|----|-------|----------|-----|
| BUG-004 | m_pairs concurrent modification | `hdtDispatcher.cpp:100-160` | Two-pass: count then allocate |
| BUG-005 | CUDA 1-frame latency UAF | `hdtSkinnedMeshBody.cpp:111-127` | Verify sync in removePendingResultsFor |
| BUG-006 | Frame counter overflow | `hdtSkinnedMeshWorld.cpp:20-27` | Init m_lastUpdateFrame to UINT32_MAX |
| BUG-007 | Division by zero in AVX | `hdtGroupConstraintSolver.cpp:115` | Guard with epsilon check |
| M1 | God function setup | `hdtGroupConstraintSolver.cpp:241-356` | Extract 6 steps to methods |
| M2 | Silent exception swallow | `hdtSkinnedMeshWorld.cpp:326-346` | Set corruption flag |
| M3 | Volatile misuse | `hdtEnkiTSScheduler.h:71` | Use std::atomic<bool> |
| M4 | Random device abuse | `hdtGroupConstraintSolver.cpp:187-189` | Thread-local RNG |
| SEC-002 | Unbounded XML resources | `hdtSkyrimSystem.cpp:232-296` | Add MAX_BONES etc limits |
| SEC-003 | Config bounds missing | `config.cpp:89-134` | Apply console cmd bounds |

---

## Modernization Opportunities

### High-Value/Low-Effort

| Opportunity | Effort | Impact | Notes |
|-------------|--------|--------|-------|
| C++20 `std::jthread` | Low | Medium | Cleaner shutdown |
| C++20 `std::span` | Low | Medium | Replace ptr+size pairs |
| Highway SIMD | Medium | High | Cross-platform, auto-dispatch |
| Thread-local RNG | Low | Low-Medium | Performance improvement |
| fmt library | Low | Low | Better logging |

### Not Recommended

- **Jolt Physics**: Migration cost extreme, Bullet works fine
- **TaskFlow**: enkiTS is working well, no compelling reason to switch

---

## Positive Observations

1. **Tracy instrumentation is thorough** - Excellent profiling infrastructure
2. **enkiTS integration is clean** - Well-designed abstraction layer
3. **Four-layer architecture is sound** - Violations are local, not systemic
4. **Index-based collision system** - Recent move from pointers shows awareness
5. **Good use of btClamped()** - Many physics parameters properly bounded
6. **RAII patterns** - Smart pointers and resource management

---

## Implementation Plan

### Phase 1: Critical Fixes (1-2 days)
- [ ] BUG-001: Add collision worker tracking to CollisionDispatcher
- [ ] BUG-002: Add m_tasks.wait() to suspendSimulationUntilFinished
- [ ] BUG-003: Fix FrameSyncEvent early return signaling
- [ ] M3: Replace volatile with std::atomic in scheduler
- [ ] BUG-013: Initialize atomic bools explicitly

### Phase 2: Security Hardening (1 day)
- [ ] SEC-003: Add config bounds validation
- [ ] SEC-001: Sanitize save file paths
- [ ] SEC-002: Add XML resource limits

### Phase 3: Refactoring (2-3 days)
- [ ] C1: Split dispatchAllCollisionPairs into CUDA/NoCUDA methods
- [ ] C2: Document magic numbers with named constants
- [ ] M1: Extract solver setup steps
- [ ] M4: Thread-local RNG

### Phase 4: Testing (2 days)
- [ ] Unit tests for scheduler primitives
- [ ] Suspend/resume state machine tests
- [ ] CUDA/non-CUDA parity tests

---

## Verification

```bash
# Build both configs
just build V1_6_1170_CUDA_AVX2
just build V1_6_1170_NOCUDA_AVX2

# Run tests
just test

# Profile
just profile V1_6_1170_NOCUDA_AVX2
```

**In-game testing**:
- Rapid "smp reset" during combat (tests BUG-001/002/003)
- Save/load cycle with multiple NPCs (tests frame sync)
- Extended play session (tests frame counter)

---

*Review Date: 2026-01-11*
*Experts: Bug Hunter Supreme, Security Specialist, Code Craftsman*
*Full agent reports: ~/.claude/plans/splendid-jumping-parrot-agent-*.md*
