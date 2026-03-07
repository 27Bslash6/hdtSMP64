# Concurrency Testing Strategy for hdtSMP64

**Status**: Infrastructure Complete

**See also**: `docs/CONCURRENCY_TESTING.md` for usage guide

## Quick Start (Linux)

```bash
# Using justfile (recommended)
just tsan-quick          # Build + run TSan tests (skip stress/SIMD)
just tsan-sync           # TSan on sync tests only (fastest)
just asan-quick          # Build + run ASan tests
just helgrind-sync       # Helgrind on sync tests
just linux-ci            # Full sanitizer pipeline

# Extended stress testing
just tsan-stress                           # 30 seconds default
HDT_STRESS_DURATION_SEC=300 just tsan-stress  # 5 minutes

# Manual (if justfile doesn't work)
cmake --preset=tsan && cmake --build build/tsan
setarch $(uname -m) -R ./build/tsan/hdtSMP64_tests "[sync]" -s
```

**Note**: TSan requires `setarch -R` to disable ASLR. The justfile tasks handle this automatically.

---

## Problem Statement

hdtSMP64 has known threading bugs (BUG-001, BUG-002, BUG-003) causing crashes during "smp reset" and suspend/resume operations. The current test suite (~13 files, 3000 lines) has excellent coverage for isolated algorithms but critical gaps in concurrency testing.

---

## Current Test Suite Analysis

### Strengths
| Test File | Coverage | Quality |
|-----------|----------|---------|
| `test_thread_local_stale_data.cpp` | thread_local cleanup, 8-thread stress | Excellent |
| `test_aabb_simd.cpp` | SSE/AVX2 batch operations | Complete |
| `test_serialization.cpp` | RAII, double-free prevention | Complete |
| `test_dispatcher.cpp` | Collision pair filtering | Complete |
| `test_non_hookean.cpp` | Physics constraint math | Complete |

### Critical Gaps
- **No TSan/race detection** - Windows builds only
- **No AsyncTaskGroup tests** - suspend() behavior untested
- **No enkiTS scheduler tests** - parallel_for_each untested
- **No Bullet integration tests** - collision world isolated
- **No suspend/resume synchronization tests** - BUG-001/002/003 untested

---

## Research Findings

### Critical Discovery: TSan Unavailable on Windows

Thread Sanitizer (TSan) is **NOT supported** on Windows (neither MSVC nor Clang). This fundamentally shapes the strategy:

| Tool | Platform | Slowdown | Use Case |
|------|----------|----------|----------|
| **TSan** | Linux only | 5-15x | Race detection (CRITICAL) |
| **ASan** | All (MSVC) | 2x | Memory errors |
| **rr** | Linux only | 1.5x | Record/replay debugging |
| **Relacy** | All | N/A | Lock-free verification |
| **Helgrind** | Linux only | 20-50x | Alternative race detection |

### Key Technique: Deterministic Replay

**rr (record/replay debugger)** enables:
- Recording non-deterministic execution
- Reverse debugging to find root cause
- "Chaos mode" that randomizes scheduling to expose races
- **Limitation**: No CUDA support (NOCUDA builds only)

### enkiTS Testing Pattern

Current tests run ~100-1000 iterations. Research shows enkiTS stress tests should run for **minutes, not hundreds of iterations**:

```cpp
// Current: inadequate
for (int i = 0; i < 1000; i++) { ... }

// Better: time-based
auto end = steady_clock::now() + 30s;
while (steady_clock::now() < end) { ... }
```

---

## Implementation Plan

### Phase 1: Hybrid TSan Strategy (Priority: CRITICAL)

You have **WSL2 + native Ubuntu 24** - use each for what it does best:

| Environment | Use For | Why |
|-------------|---------|-----|
| **WSL2** | Quick iterative TSan runs | Same machine, instant feedback during dev |
| **Native Ubuntu** | Extended stress tests, rr debugging | Full kernel, no virtualization quirks, rr works properly |

#### WSL2 Setup (Quick Iteration)
```bash
# In WSL2 Ubuntu
sudo apt install clang-18 libc++-18-dev libc++abi-18-dev cmake ninja-build

# Build with TSan
mkdir build-tsan && cd build-tsan
cmake -DCMAKE_CXX_COMPILER=clang++-18 \
      -DCMAKE_CXX_FLAGS="-fsanitize=thread -g -stdlib=libc++" \
      -G Ninja ..
ninja

# Quick run during development
./hdtSMP64_tests 2>&1 | tee tsan.log
```

#### Native Ubuntu 24 Setup (Comprehensive Testing)
```bash
# Native Ubuntu - also install rr for record/replay debugging
sudo apt install clang-18 libc++-18-dev cmake ninja-build rr

# Build with TSan (same as WSL2)
mkdir build-tsan && cd build-tsan
cmake -DCMAKE_CXX_COMPILER=clang++-18 \
      -DCMAKE_CXX_FLAGS="-fsanitize=thread -g -stdlib=libc++" \
      -G Ninja ..
ninja

# Extended stress tests (run for minutes, not iterations)
./hdtSMP64_tests "[.stress]" --duration 300

# rr for debugging intermittent failures
rr record --chaos ./hdtSMP64_tests "[.stress]"
rr replay  # Reverse-debug to find root cause
```

**rr chaos mode** randomizes thread scheduling to expose races faster. Not available in WSL2.

#### CI Addition (after local validation)
```yaml
# .github/workflows/sanitizers.yml
linux-tsan:
  runs-on: ubuntu-latest
  steps:
    - uses: actions/checkout@v4
    - name: Build with TSan
      run: |
        cmake -DCMAKE_CXX_FLAGS="-fsanitize=thread -g" ..
        make -j$(nproc)
    - name: Run tests under TSan
      run: ./hdtSMP64_tests
```

**Files to modify:**
- `CMakeLists.txt` - Add TSan flag option for Linux builds
- Create `tsan.supp` - Suppression file for known false positives
- `.github/workflows/sanitizers.yml` - Add TSan job (after local works)

### Phase 2: Targeted Concurrency Tests (Priority: HIGH)

Create tests for the known race conditions.

#### Test 1: suspend() Worker Tracking (BUG-001)

```cpp
// tests/unit/test_suspend_race.cpp
TEST_CASE("suspend waits for collision workers") {
    MockPhysicsWorld world;
    std::atomic<bool> workerRunning{false};
    std::atomic<bool> workerAccessedAfterSuspend{false};

    // Start collision workers
    world.startCollisionProcessing([&]() {
        workerRunning = true;
        std::this_thread::sleep_for(100ms);
        if (world.isSuspended()) {
            workerAccessedAfterSuspend = true;  // BUG!
        }
        workerRunning = false;
    });

    // Trigger suspend while workers active
    std::this_thread::sleep_for(10ms);
    world.suspend();

    REQUIRE(!workerRunning);  // Must wait for completion
    REQUIRE(!workerAccessedAfterSuspend);
}
```

#### Test 2: FrameSyncEvent Early Return (BUG-003)

```cpp
TEST_CASE("FrameSyncEvent sets completion on early return") {
    MockPhysicsWorld world;
    world.setSuspended(true);

    auto start = steady_clock::now();
    world.frameSync();  // Should return immediately
    auto elapsed = steady_clock::now() - start;

    REQUIRE(elapsed < 100ms);  // NOT 5-second timeout
    REQUIRE(world.isFrameSyncComplete());
}
```

#### Test 3: Extended Stress Test

```cpp
TEST_CASE("concurrent suspend/resume stress test", "[.stress]") {
    MockPhysicsWorld world;
    std::atomic<int> suspendCount{0};
    std::atomic<int> accessAfterSuspend{0};

    auto runWorkers = [&](std::stop_token st) {
        while (!st.stop_requested()) {
            if (!world.isSuspended()) {
                world.processCollisions();
            }
        }
    };

    auto runSuspender = [&](std::stop_token st) {
        while (!st.stop_requested()) {
            world.suspend();
            suspendCount++;
            std::this_thread::sleep_for(1ms);
            world.resume();
        }
    };

    // Run for 30 seconds
    std::jthread workers[4];
    for (auto& w : workers) w = std::jthread(runWorkers);
    std::jthread suspender(runSuspender);

    std::this_thread::sleep_for(30s);
    // Stop threads...

    REQUIRE(accessAfterSuspend == 0);
    INFO("Completed " << suspendCount << " suspend/resume cycles");
}
```

**Files to create:**
- `tests/unit/test_suspend_race.cpp`
- `tests/unit/test_frame_sync.cpp`
- `tests/unit/test_async_task_group.cpp`

### Phase 3: Relacy Tests for Lock-Free Structures (Priority: MEDIUM)

ColliderTree uses atomic operations. Verify with Relacy:

```cpp
// tests/relacy/test_collider_tree.cpp
#include <relacy/relacy.hpp>

struct ColliderTreeTest : rl::test_suite<ColliderTreeTest, 4> {
    std::atomic<size_t> colliderOffset;
    std::vector<Collider> colliders;

    void thread(unsigned idx) {
        if (idx == 0) {
            // Writer: update colliders
            colliders.push_back(Collider{});
            colliderOffset.store(colliders.size() - 1,
                                 std::memory_order_release);
        } else {
            // Reader: access collider
            auto offset = colliderOffset.load(std::memory_order_acquire);
            if (offset < colliders.size()) {
                colliders[offset].access();  // No crash
            }
        }
    }
};
```

### Phase 4: Integration Test Infrastructure (Priority: MEDIUM)

Create a minimal Bullet world for collision testing:

```cpp
TEST_CASE("collision dispatch with concurrent updates") {
    btDefaultCollisionConfiguration config;
    btCollisionDispatcher dispatcher(&config);
    btDbvtBroadphase broadphase;
    btSequentialImpulseConstraintSolver solver;
    btDiscreteDynamicsWorld world(&dispatcher, &broadphase, &solver, &config);

    // Add bodies...
    // Step in parallel with modifications...
}
```

---

## Verification

### Local Testing
```bash
# Build and run with ASan (Windows)
just test  # Uses Debug config with ASan

# Run stress tests (tagged for exclusion from CI)
./hdtSMP64_tests "[.stress]" --duration 60
```

### CI Verification
1. Linux TSan job passes without races
2. Windows ASan job passes without memory errors
3. Stress tests run for extended duration on merge to main

### Manual In-Game Testing
Per expert panel: "rapid 'smp reset' during combat, save/load cycles"

---

## Files to Modify/Create

| File | Action |
|------|--------|
| `.github/workflows/sanitizers.yml` | Add Linux TSan job |
| `CMakeLists.txt` | Add `-fsanitize=thread` option |
| `tsan.supp` | Create suppression file |
| `tests/unit/test_suspend_race.cpp` | Create BUG-001 test |
| `tests/unit/test_frame_sync.cpp` | Create BUG-003 test |
| `tests/unit/test_async_task_group.cpp` | Create task group tests |
| `tests/relacy/test_collider_tree.cpp` | Create Relacy tests |
| `tests/integration/test_bullet_world.cpp` | Create integration tests |

---

## Priority Summary

| Phase | Environment | Focus | Status |
|-------|-------------|-------|--------|
| **1** | WSL2 | TSan setup + existing tests | DONE |
| **2** | WSL2 | BUG-001/002/003 test cases | DONE (test_suspend_sync.cpp) |
| **3** | Native Ubuntu | Extended stress (5+ min) + rr | DONE (stress-extended tag) |
| **4** | Both | CI pipeline | DONE (sanitizers.yml) |
| **5** | Both | Relacy + integration | TODO |

**Workflow**: Develop tests in WSL2 (fast iteration) → Validate on native Ubuntu (comprehensive) → Gate in CI

## What Was Implemented

### Infrastructure
- `CMakeLists.txt` - Added all 18 test files, TSan suppression file support
- `CMakePresets.json` - Already had `tsan` and `ci-tsan` presets
- `tsan.supp` - Suppression file for known issues (Bullet, enkiTS, Catch2)
- `.github/workflows/sanitizers.yml` - TSan job + extended stress job on main

### Test Coverage
- `test_suspend_sync.cpp` - 16 TEST_CASE blocks covering BUG-001/002/003/008
- `test_thread_local_stale_data.cpp` - 8-thread stress testing
- Extended stress test with configurable duration (`[.stress-extended]` tag)

### CI Integration
- TSan runs on every PR (uses suppression file)
- Extended 5-minute stress tests run on push to master only
