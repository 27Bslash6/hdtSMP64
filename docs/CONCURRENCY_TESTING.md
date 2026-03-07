# Concurrency Testing Guide

> Tools and techniques for testing thread safety in hdtSMP64.

---

## Table of Contents

- [Quick Reference](#quick-reference)
- [Known Threading Bugs](#known-threading-bugs)
- [Building with Sanitizers](#building-with-sanitizers)
- [ThreadSanitizer (TSan)](#threadsanitizer-tsan)
- [Helgrind](#helgrind-alternative-to-tsan)
- [Test Tags](#test-tags)
- [Extended Stress Tests](#extended-stress-tests)
- [rr (Record/Replay Debugger)](#rr-recordreplay-debugger)
- [CI Pipeline](#ci-pipeline)
- [Writing Thread-Safe Tests](#writing-thread-safe-tests)
- [Troubleshooting](#troubleshooting)

---

## Quick Reference

| Tool | What it catches | Platform | Overhead |
|------|-----------------|----------|----------|
| **TSan** | Data races, deadlocks | Linux only | 5-15x |
| **Helgrind** | Data races, lock order | Linux only | 20-50x |
| **ASan** | Memory errors | All | 2x |
| **UBSan** | Undefined behavior | All | 1.5x |
| **rr** | Record/replay debugging | Linux only | 1.5x |

## Known Threading Bugs

> [!WARNING]
> These bugs can cause crashes that are difficult to reproduce. See `expert_panel_review_2026-01` memory for full details.

| Bug | Description | Location | Status |
|-----|-------------|----------|--------|
| BUG-001 | suspend() doesn't wait for collision workers | `hdtSkyrimPhysicsWorld.h:49-88` | Open |
| BUG-002 | suspendSimulationUntilFinished() no sync | `hdtSkyrimPhysicsWorld.cpp:171-185` | Open |
| BUG-003 | FrameSyncEvent early return | `hdtSkyrimPhysicsWorld.cpp:428-462` | Open |

---

## Building with Sanitizers

> [!TIP]
> Start with ASan for memory errors, then move to TSan for race detection.

### Windows (ASan + UBSan only)

```powershell
# From project root
just sanitize          # ASan + UBSan combined
just sanitize-asan     # ASan only
just sanitize-ubsan    # UBSan only
```

### Linux (Full sanitizer support)

```bash
# ThreadSanitizer (race detection)
cmake --preset=tsan
cmake --build build/tsan
setarch $(uname -m) -R ./build/tsan/hdtSMP64_tests

# AddressSanitizer
cmake --preset=asan
cmake --build build/asan
./build/asan/hdtSMP64_tests

# ASan + UBSan combined
cmake --preset=asan-ubsan
cmake --build build/asan-ubsan
./build/asan-ubsan/hdtSMP64_tests
```

---

## ThreadSanitizer (TSan)

> [!IMPORTANT]
> TSan is the most effective tool for detecting data races. It requires Linux.

### Requirements

- Linux (native or Docker - WSL2 has issues)
- Clang 14+ or GCC 11+
- CPU with AVX2 support

### TSan Memory Mapping Issues

If you see `FATAL: ThreadSanitizer: unexpected memory mapping`, use:

```bash
# Per-process ASLR disable (safe, recommended)
setarch $(uname -m) -R ./build/tsan/hdtSMP64_tests

# Or in Docker (isolated environment)
docker run --rm -it --security-opt seccomp=unconfined \
  -v $(pwd):/code ubuntu:24.04 bash
```

### Suppression File

The `tsan.supp` file suppresses known false positives:

```bash
TSAN_OPTIONS="suppressions=tsan.supp" ./build/tsan/hdtSMP64_tests
```

Edit `tsan.supp` to add new suppressions. Format:
```
race:function_name
deadlock:mutex_name
```

---

## Helgrind (Alternative to TSan)

Helgrind is Valgrind's thread error detector. Slower but works when TSan doesn't.

> [!NOTE]
> Helgrind has 20-50x overhead compared to TSan's 5-15x. Use TSan when possible.

```bash
# Install
sudo apt install valgrind

# Run
valgrind --tool=helgrind ./build/debug/hdtSMP64_tests "[sync]"

# With more detail
valgrind --tool=helgrind --history-level=full ./build/debug/hdtSMP64_tests "[sync]"
```

---

## Test Tags

| Tag | Description | When to use |
|-----|-------------|-------------|
| `[sync]` | Synchronization tests | TSan runs |
| `[dispatcher]` | Collision dispatcher | TSan runs |
| `[thread]` | Threading tests | TSan runs |
| `[stress]` | Short stress tests | Regular CI |
| `[.stress-extended]` | Long stress tests (30s+) | Manual/merge only |
| `[.benchmark]` | Performance benchmarks | Manual only |
| `[aabb]` | SIMD tests | Skip with TSan |

### Running Specific Tests

```bash
# Only sync tests
./build/tsan/hdtSMP64_tests "[sync]"

# Exclude stress tests
./build/tsan/hdtSMP64_tests "~[.stress]" "~[.stress-extended]"

# Exclude SIMD (conflicts with TSan)
./build/tsan/hdtSMP64_tests "~[aabb]"
```

---

## Extended Stress Tests

For thorough race detection, run stress tests for minutes, not iterations:

```bash
# Default: 30 seconds
./build/tsan/hdtSMP64_tests "[.stress-extended]"

# Custom duration (5 minutes)
HDT_STRESS_DURATION_SEC=300 ./build/tsan/hdtSMP64_tests "[.stress-extended]"
```

---

## rr (Record/Replay Debugger)

rr records non-deterministic execution for replay debugging. Invaluable for intermittent races.

> [!TIP]
> Use `--chaos` mode to randomize thread scheduling and surface race conditions faster.

```bash
# Install
sudo apt install rr

# Record with chaos mode (randomizes scheduling)
rr record --chaos ./build/debug/hdtSMP64_tests "[sync]"

# Replay and debug
rr replay

# Inside rr, use gdb commands:
(rr) reverse-continue    # Run backwards
(rr) reverse-step        # Step backwards
(rr) when                # Show event number
```

> [!CAUTION]
> rr does not support CUDA. Use NOCUDA builds only.

---

## CI Pipeline

The GitHub Actions workflow runs sanitizers automatically:

| Job | Trigger | Duration |
|-----|---------|----------|
| ASan | Every PR | ~2 min |
| UBSan | Every PR | ~2 min |
| TSan | Every PR | ~3 min |
| ASan+UBSan | Every PR | ~3 min |
| Extended Stress | Push to master | ~10 min |

---

## Writing Thread-Safe Tests

<details>
<summary><strong>📚 Pattern: Mock with Atomics</strong></summary>

```cpp
class MockPhysicsWorld {
    std::atomic<bool> m_suspended{false};
    std::atomic<int> m_activeWorkers{0};

    void suspend() {
        m_suspended.store(true, std::memory_order_release);
        while (m_activeWorkers.load(std::memory_order_acquire) > 0)
            std::this_thread::yield();
    }
};
```

</details>

<details>
<summary><strong>📚 Pattern: RAII Worker Tracking</strong></summary>

```cpp
struct WorkerScope {
    std::atomic<int>& counter;
    WorkerScope(std::atomic<int>& c) : counter(c) {
        counter.fetch_add(1, std::memory_order_acq_rel);
    }
    ~WorkerScope() {
        counter.fetch_sub(1, std::memory_order_acq_rel);
    }
};
```

</details>

<details>
<summary><strong>📚 Pattern: Time-Based Stress Test</strong></summary>

```cpp
TEST_CASE("stress test", "[.stress-extended]") {
    const char* env = std::getenv("HDT_STRESS_DURATION_SEC");
    const int duration = env ? std::atoi(env) : 30;

    auto endTime = steady_clock::now() + seconds(duration);
    while (steady_clock::now() < endTime) {
        // Test operations
    }
}
```

</details>

---

## Troubleshooting

### TSan: "unexpected memory mapping"

```bash
# Use setarch to disable ASLR for the process
setarch $(uname -m) -R ./build/tsan/hdtSMP64_tests
```

### TSan: Segfault on startup

Likely TSan + AVX2 conflict. Exclude SIMD tests:

```bash
./build/tsan/hdtSMP64_tests "~[aabb]"
```

### Tests hang or take forever

Probably stuck in a stress test. Use tags to skip:

```bash
./build/debug/hdtSMP64_tests "~[.stress]" "~[.stress-extended]"
```

### No output until Ctrl-C

Output buffering. Use:

```bash
stdbuf -oL ./build/debug/hdtSMP64_tests
# Or
./build/debug/hdtSMP64_tests -s  # Show all assertions
```

---

<div align="center">

*For more architecture details, see [ARCHITECTURE.md](ARCHITECTURE.md)*

</div>
