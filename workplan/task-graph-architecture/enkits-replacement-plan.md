# enkiTS Full Replacement Plan

## Status: APPROVED - Ready for Implementation

## Decision Summary

**Original Goal:** Overlap CollisionDetection (8ms) with ConvertBodies+ConvertJoints (4ms)

**Expert Panel Finding:** Adding Taskflow or enkiTS alongside PPL causes thread pool over-subscription (2N threads for N cores). The only clean solution is full replacement.

**Final Decision:** Replace ALL PPL usage with enkiTS throughout the codebase.

## Why enkiTS Over Alternatives

| Option | Verdict | Reason |
|--------|---------|--------|
| Taskflow | REJECT | YAGNI - DAG features unused, same over-subscription problem |
| enkiTS | **ACCEPT** | Lower overhead, game-proven, clean replacement |
| Stay with PPL | REJECT | Can't add parallelism without over-subscription |
| Hybrid | REJECT | Two thread pools = worse than either alone |

### enkiTS Credentials
- Used in: Doom (2016), Doom Eternal, Rage 2, Saints Row
- Overhead: 0.5-1μs per task (vs 2-5μs for PPL)
- Single cpp file + headers
- Work-stealing scheduler

## Current PPL Usage Inventory

### Direct Usage (18 call sites)

| File | Pattern | Work Parallelized |
|------|---------|-------------------|
| hdtSkinnedMeshWorld.cpp:24 | `btSetTaskScheduler(btGetPPLTaskScheduler())` | Bullet scheduler init |
| hdtSkinnedMeshWorld.cpp:309-326 | `parallel_invoke(3 tasks)` | GPU sync, predict, update |
| hdtSkyrimPhysicsWorld.h:75 | `task_group` member | Async physics dispatch |
| hdtSkyrimPhysicsWorld.cpp:123-124 | `m_tasks.run()` | Async physics step |
| hdtSkyrimPhysicsWorld.cpp:399,415 | `m_tasks.wait()` | Wait for async |
| hdtDispatcher.cpp:93-96 | `parallel_for` | Pair filtering |
| hdtDispatcher.cpp:176-193 | `parallel_for_each` | CUDA object init |
| hdtDispatcher.cpp:206-207 | `parallel_for_each` | Bone updates |
| hdtDispatcher.cpp:242-251 | `parallel_for_each` | Tree updates |
| hdtDispatcher.cpp:255-264 | `parallel_for_each` | Fallback internal update |
| hdtDispatcher.cpp:280 | `parallel_for` | Collision pair gathering |
| hdtDispatcher.cpp:304-309 | `parallel_for_each` | Fallback collision |
| hdtDispatcher.cpp:341-355 | Multiple `parallel_for_each` | Non-CUDA updates |
| hdtGroupConstraintSolver.cpp:252 | `parallel_for_each` | Constraint group setup |
| hdtGroupConstraintSolver.cpp:458 | `parallel_for_each` | Solver iteration |
| hdtGroupConstraintSolver.cpp:467-470 | `parallel_for_each` (2x) | Split solver iteration |

### Include Sites (9 files)
- hdtSkinnedMeshWorld.cpp
- hdtSkyrimPhysicsWorld.cpp
- hdtDispatcher.h
- hdtSkinnedMeshSystem.h
- hdtSkinnedMeshShape.cpp
- hdtSkinnedMeshBody.cpp
- hdtCudaInterface.cpp
- hdtLCP.h
- LinearMath/btThreads.cpp

## Implementation Plan

### Phase 1: Foundation (Day 1)

1. Add enkiTS submodule:
```bash
git submodule add https://github.com/dougbinks/enkiTS.git external/enkiTS
```

2. Create `hdtSkinnedMesh/hdtEnkiTSScheduler.h`:
```cpp
#pragma once
#include <TaskScheduler.h>
#include <functional>

namespace hdt {

class EnkiTSScheduler {
    enki::TaskScheduler m_scheduler;

    EnkiTSScheduler() {
        m_scheduler.Initialize();
    }

public:
    static EnkiTSScheduler& get() {
        static EnkiTSScheduler instance;
        return instance;
    }

    enki::TaskScheduler& scheduler() { return m_scheduler; }

    void shutdown() { m_scheduler.WaitforAllAndShutdown(); }
};

// Drop-in replacement for concurrency::parallel_invoke
template<typename... Funcs>
void hdt_parallel_invoke(Funcs&&... funcs);

// Drop-in replacement for concurrency::parallel_for
template<typename Func>
void hdt_parallel_for(int begin, int end, Func&& func);

// Drop-in replacement for concurrency::parallel_for_each
template<typename Container, typename Func>
void hdt_parallel_for_each(Container& c, Func&& func);

// Drop-in replacement for concurrency::task_group
class AsyncTaskGroup {
    // Implementation wrapping ITaskSet
public:
    template<typename Func>
    void run(Func&& func);
    void wait();
};

} // namespace hdt
```

3. Create `btTaskSchedulerEnkiTS` in LinearMath/btThreads.cpp

4. Update vcxproj:
```xml
<AdditionalIncludeDirectories>
  $(SolutionDir)external\enkiTS\src;%(AdditionalIncludeDirectories)
</AdditionalIncludeDirectories>

<ClCompile Include="$(SolutionDir)external\enkiTS\src\TaskScheduler.cpp" />
```

### Phase 2: Non-Critical Paths (Day 1-2)

1. Replace hdtDispatcher.cpp parallel loops
2. Test with CUDA disabled to verify fallback paths

### Phase 3: Core Physics (Day 2)

1. Replace hdtSkinnedMeshWorld.cpp `parallel_invoke`
2. Replace hdtGroupConstraintSolver.cpp `parallel_for_each`
3. Switch Bullet scheduler:
```cpp
// Before
btSetTaskScheduler(btGetPPLTaskScheduler());

// After
btSetTaskScheduler(btGetEnkiTSTaskScheduler());
```
4. Profile with Tracy

### Phase 4: Async Frame (Day 2-3)

1. Replace SkyrimPhysicsWorld `task_group` with `AsyncTaskGroup`
2. Integration testing
3. Performance benchmarking

### Phase 5: Cleanup (Day 3)

1. Remove all `#include <ppl.h>` and `#include <ppltasks.h>`
2. Remove PPL scheduler from btThreads.cpp
3. Update CLAUDE.md and documentation

## Replacement Patterns

### parallel_invoke
```cpp
// PPL
concurrency::parallel_invoke(
    [this]() { syncPreviousCollisionResults(); },
    [this, timeStep]() { predictUnconstraintMotion(timeStep); },
    [this]() { for (auto& s : m_systems) s->internalUpdate(); }
);

// enkiTS
hdt_parallel_invoke(
    [this]() { syncPreviousCollisionResults(); },
    [this, timeStep]() { predictUnconstraintMotion(timeStep); },
    [this]() { for (auto& s : m_systems) s->internalUpdate(); }
);
```

### parallel_for
```cpp
// PPL
concurrency::parallel_for(0, size, [&](int i) {
    process(pairs[i]);
});

// enkiTS
hdt_parallel_for(0, size, [&](int i) {
    process(pairs[i]);
});
```

### parallel_for_each
```cpp
// PPL
concurrency::parallel_for_each(m_groups.begin(), m_groups.end(), [&](auto* g) {
    g->process();
});

// enkiTS
hdt_parallel_for_each(m_groups, [&](auto* g) {
    g->process();
});
```

## Risk Mitigation

| Risk | Severity | Mitigation |
|------|----------|------------|
| Nested parallelism deadlock | HIGH | Test GroupConstraintSolver early; may need task flattening |
| Performance regression | MEDIUM | Keep PPL fallback via `#ifdef USE_ENKITS` initially |
| Thread pool lifecycle | MEDIUM | Init at startup, shutdown on plugin unload |
| Build system issues | LOW | enkiTS is single cpp file |

## Rollback Plan

1. All work on `feature/enkits-replacement` branch
2. Conditional compilation with `#ifdef USE_ENKITS`
3. Can revert to PPL by changing one preprocessor define
4. Remove PPL codepaths after 2 weeks of stable testing

## Expected Results

| Metric | Before | After |
|--------|--------|-------|
| Thread pools | 2 (PPL + proposed) | 1 (enkiTS) |
| Per-task overhead | 2-5μs | 0.5-1μs |
| Physics frame time | Baseline | -5% to -15% |
| CPU utilization | Over-subscribed | Optimal |

## Statistics

| Metric | Value |
|--------|-------|
| Files to modify | 12 |
| Call sites to change | 18 |
| New code | ~250 LOC |
| Modified code | ~100 LOC |
| Removed code | ~150 LOC |
| Net change | ~+200 LOC |
| Effort | 2-3 days |

## Validation Checklist

- [ ] enkiTS submodule added and compiles
- [ ] hdtEnkiTSScheduler.h wrapper complete
- [ ] btTaskSchedulerEnkiTS implements btITaskScheduler
- [ ] All parallel_invoke sites converted
- [ ] All parallel_for sites converted
- [ ] All parallel_for_each sites converted
- [ ] task_group replaced with AsyncTaskGroup
- [ ] All PPL includes removed
- [ ] Tracy shows single thread pool
- [ ] No performance regression in benchmarks
- [ ] Solver output unchanged (determinism check)
- [ ] 12+ skeleton stress test passes
