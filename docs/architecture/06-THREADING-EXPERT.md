# Level 6: Threading & Parallelism Expert Guide

**Audience**: Performance engineers, developers adding new parallel code, debugging race conditions
**Prerequisites**: Level 0-5, deep understanding of concurrent programming

---

## Threading Architecture

hdtSMP64 uses **enkiTS** (Engine Kids Task Scheduler) for all CPU parallelism, replacing the previous Microsoft PPL implementation.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       THREADING ARCHITECTURE                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                      GAME MAIN THREAD                               │   │
│   │  ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐          │   │
│   │  │FrameEvt  │──►│ doUpdate │──►│ dispatch │──►│FrameSync │          │   │
│   │  └──────────┘   └──────────┘   └────┬─────┘   └────┬─────┘          │   │
│   └─────────────────────────────────────┼──────────────┼────────────────┘   │
│                                         │              │                    │
│                              async      │      wait    │                    │
│                                         ▼              │                    │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                   ENKITS TASK SCHEDULER                             │   │
│   │                                                                     │   │
│   │   ┌─────────────────────────────────────────────────────────────┐   │   │
│   │   │              SHARED WORKER THREAD POOL                       │   │   │
│   │   │         (N threads = hardware_concurrency)                  │   │   │
│   │   │                                                             │   │   │
│   │   │   ┌───────┐ ┌───────┐ ┌───────┐ ┌───────┐     ┌───────┐     │   │   │
│   │   │   │Worker0│ │Worker1│ │Worker2│ │Worker3│ ... │WorkerN│     │   │   │
│   │   │   └───┬───┘ └───┬───┘ └───┬───┘ └───┬───┘     └───┬───┘     │   │   │
│   │   │       │         │         │         │             │         │   │   │
│   │   │       └─────────┴────┬────┴─────────┴─────────────┘         │   │   │
│   │   │                      │                                       │   │   │
│   │   │           ┌──────────▼──────────┐                           │   │   │
│   │   │           │  WORK-STEALING QUEUE │                          │   │   │
│   │   │           │   (lock-free)        │                          │   │   │
│   │   │           └─────────────────────┘                           │   │   │
│   │   │                                                             │   │   │
│   │   │   Task Types:                                               │   │   │
│   │   │   ├── Physics step (doUpdate2ndStep)                        │   │   │
│   │   │   ├── Parallel for (hdt_parallel_for)                       │   │   │
│   │   │   ├── Parallel for_each (hdt_parallel_for_each)             │   │   │
│   │   │   └── Bullet solver (btParallelFor)                         │   │   │
│   │   └─────────────────────────────────────────────────────────────┘   │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Why enkiTS?

Previous implementation used Microsoft PPL (Parallel Patterns Library). Problems:

1. **Thread Pool Over-subscription**: PPL created its own thread pool, separate from Bullet's parallel solver, causing 2× threads competing for cores.

2. **Higher Overhead**: PPL task dispatch: 2-5μs per task; enkiTS: 0.5-1μs per task.

3. **Race Conditions**: PPL's slower dispatch exposed race conditions during save/load transitions.

enkiTS advantages:

- **Single Thread Pool**: Shared by hdtSMP64 AND Bullet's internal solver via `btTaskSchedulerEnkiTS`
- **Battle-Tested**: Used in DOOM, DOOM Eternal, Rage 2 (id Tech engine)
- **Work-Stealing**: Better load balancing than static partitioning
- **Lock-Free Dispatch**: Sub-microsecond overhead

## enkiTS Primitives

### hdt_parallel_for

Parallel loop with automatic chunking:

```cpp
// hdtEnkiTSScheduler.h:78-123
template <typename Func>
void hdt_parallel_for(int begin, int end, Func&& func)
{
    if (begin >= end) return;
    const int count = end - begin;

    // Sequential for trivial cases
    if (count <= 1) {
        for (int i = begin; i < end; ++i) func(i);
        return;
    }

    // Create TaskSet with automatic chunking
    struct ParallelForTask : public enki::ITaskSet {
        int m_begin;
        std::function<void(int)> m_func;

        ParallelForTask(int begin, int count, std::function<void(int)> func)
            : enki::ITaskSet(static_cast<uint32_t>(count))
            , m_begin(begin)
            , m_func(std::move(func))
        {
            // Chunk size = count / (num_threads * 4)
            m_MinRange = std::max(1u, count / (enki::GetNumHardwareThreads() * 4));
        }

        void ExecuteRange(enki::TaskSetPartition range, uint32_t) override {
            for (uint32_t i = range.start; i < range.end; ++i) {
                m_func(m_begin + static_cast<int>(i));
            }
        }
    };

    ParallelForTask task(begin, count, std::forward<Func>(func));
    EnkiTSScheduler::get().scheduler().AddTaskSetToPipe(&task);
    EnkiTSScheduler::get().scheduler().WaitforTask(&task);
}
```

**Location**: `hdtEnkiTSScheduler.h:78`

### hdt_parallel_for_each

Parallel iteration over containers:

```cpp
// hdtEnkiTSScheduler.h:128-221
template <typename Iterator, typename Func>
void hdt_parallel_for_each(Iterator begin, Iterator end, Func&& func)
{
    // For random-access iterators: use parallel_for directly
    if constexpr (std::is_same_v<iterator_category, std::random_access_iterator_tag>) {
        const auto count = std::distance(begin, end);
        // ... parallel_for implementation
    }
    else {
        // For non-random-access: copy to vector first
        std::vector<ValueType> items(begin, end);
        // ... parallel_for on vector
    }
}

// Container overload
template <typename Container, typename Func>
void hdt_parallel_for_each(Container& c, Func&& func) {
    hdt_parallel_for_each(std::begin(c), std::end(c), std::forward<Func>(func));
}
```

**Location**: `hdtEnkiTSScheduler.h:128`

### hdt_parallel_invoke

Run multiple functions in parallel:

```cpp
// hdtEnkiTSScheduler.h:268-326
template <typename Func1, typename Func2, typename Func3>
void hdt_parallel_invoke(Func1&& func1, Func2&& func2, Func3&& func3)
{
    detail::InvokeTask<Func1> task1(std::forward<Func1>(func1));
    detail::InvokeTask<Func2> task2(std::forward<Func2>(func2));
    detail::InvokeTask<Func3> task3(std::forward<Func3>(func3));

    auto& scheduler = EnkiTSScheduler::get().scheduler();
    scheduler.AddTaskSetToPipe(&task1);
    scheduler.AddTaskSetToPipe(&task2);
    scheduler.AddTaskSetToPipe(&task3);

    scheduler.WaitforTask(&task1);
    scheduler.WaitforTask(&task2);
    scheduler.WaitforTask(&task3);
}
```

**Location**: `hdtEnkiTSScheduler.h:268`

### AsyncTaskGroup

Async dispatch with deferred wait:

```cpp
// hdtEnkiTSScheduler.h:332-403
class AsyncTaskGroup
{
public:
    // Launch task without waiting
    template <typename Func>
    void run(Func&& func) {
        auto task = std::make_unique<AsyncTask>(std::forward<Func>(func));
        EnkiTSScheduler::get().scheduler().AddTaskSetToPipe(task.get());
        m_pendingTasks.push_back(std::move(task));
    }

    // Wait for all pending tasks
    void wait() {
        auto& scheduler = EnkiTSScheduler::get().scheduler();
        for (auto& task : m_pendingTasks) {
            scheduler.WaitforTask(task.get());
        }
        m_pendingTasks.clear();
    }

private:
    std::vector<std::unique_ptr<AsyncTask>> m_pendingTasks;
};
```

**Location**: `hdtEnkiTSScheduler.h:332`

## Bullet Integration

Bullet's parallel solver uses the same thread pool:

```cpp
// LinearMath/btThreads.cpp
class btTaskSchedulerEnkiTS : public btITaskScheduler
{
public:
    void parallelFor(int iBegin, int iEnd, int grainSize,
                     const btIParallelForBody& body) override
    {
        // Uses hdt::EnkiTSScheduler singleton
        hdt_parallel_for(iBegin, iEnd, [&](int i) {
            body.forLoop(i, i + 1);
        });
    }
};

// In SkinnedMeshWorld constructor:
btSetTaskScheduler(btGetEnkiTSTaskScheduler());
btSequentialImpulseConstraintSolverMt::s_allowNestedParallelForLoops = true;
```

**Location**: `LinearMath/btThreads.cpp`, `hdtSkinnedMeshWorld.cpp:32-37`

## Parallel Regions in NOCUDA Path

### 1. Physics Dispatch (Async)

```cpp
// hdtSkyrimPhysicsWorld.cpp:122-124
m_tasks.run([this, interval, tick, remainingTimeStep] {
    doUpdate2ndStep(interval, tick, remainingTimeStep);
});
```

This launches the entire physics step asynchronously, allowing the game to continue rendering.

### 2. System Internal Update

```cpp
// hdtSkinnedMeshWorld.cpp:442-448 (NOCUDA path)
{
    HDT_ZONE_SCOPED_N("SystemsInternalUpdate");
    for (int i = 0; i < m_systems.size(); ++i)
        m_systems[i]->internalUpdate();
}
```

Note: Sequential in NOCUDA. Could be parallelized, but systems are typically small in count.

### 3. Collision Pair Collection

```cpp
// hdtDispatcher.cpp:100-159 (NOCUDA path)
hdt_parallel_for(0, size, [&](int i) {
    auto& pair = pairs[i];
    // ... filter pairs
    if (needsCollision(...)) {
        HDT_LOCK_GUARD(l, lock);  // Lock for shared mutation
        bodies.insert(shape0);
        m_pairs.push_back({shape0, shape1});
        // ... collect shapes
    }
});
```

### 4. Internal Updates (Parallel)

```cpp
// hdtDispatcher.cpp:349-360 (NOCUDA path)
hdt_parallel_for_each(bodies, [](SkinnedMeshBody* shape) {
    shape->internalUpdate();
});

hdt_parallel_for_each(vertex_shapes, [](PerVertexShape* shape) {
    shape->internalUpdate();
});

hdt_parallel_for_each(triangle_shapes, [](PerTriangleShape* shape) {
    shape->internalUpdate();
});
```

### 5. Narrowphase Collision (Parallel)

```cpp
// hdtDispatcher.cpp:361-366 (NOCUDA path)
hdt_parallel_for_each(m_pairs, [&](auto& pair) {
    if (pair.first->m_shape->m_tree.collapseCollideL(&pair.second->m_shape->m_tree))
        SkinnedMeshAlgorithm::processCollision(pair.first, pair.second, this);
});
```

### 6. Constraint Solving (Bullet Internal)

Bullet's `btSequentialImpulseConstraintSolverMt` uses `btParallelFor` internally:

```
solveConstraints()
└── btConstraintSolverPoolMt::solveGroup()
    └── btParallelFor (via btTaskSchedulerEnkiTS)
        └── hdt_parallel_for (same thread pool)
```

## Synchronization Primitives

### SpinLock

Lightweight lock for short critical sections:

```cpp
// hdtPrefix.h
class SpinLock
{
    std::atomic_flag m_flag = ATOMIC_FLAG_INIT;

public:
    void lock() {
        while (m_flag.test_and_set(std::memory_order_acquire))
            ; // spin
    }

    void unlock() {
        m_flag.clear(std::memory_order_release);
    }
};

// Usage with RAII
#define HDT_LOCK_GUARD(name, mutex) std::lock_guard<decltype(mutex)> name(mutex)
```

### When to Use What

| Scenario | Primitive | Reason |
|----------|-----------|--------|
| Manifold allocation | SpinLock | Very short critical section |
| Physics state mutations | SpinLock | Low contention, short duration |
| Frame sync | AsyncTaskGroup::wait() | Block until completion |
| Parallel iteration | hdt_parallel_for | Automatic chunking |
| Collection iteration | hdt_parallel_for_each | Container-friendly |
| Independent tasks | hdt_parallel_invoke | Fixed number of tasks |

## Race Condition Patterns

### Pattern 1: Double-Checked Locking

Used for lazy initialization:

```cpp
// Example: Graph capture (conceptual)
if (m_graphCaptured && m_graphExec) {
    // Fast path: no lock needed
    launchGraph();
} else {
    std::lock_guard lock(s_graphCaptureMutex);
    // Re-check after acquiring lock
    if (!m_graphCaptured) {
        captureGraph();
        m_graphCaptured = true;
    }
    launchGraph();
}
```

### Pattern 2: Parallel Collection with Lock

```cpp
// hdtDispatcher.cpp:100-155
SpinLock lock;
std::unordered_set<SkinnedMeshBody*> bodies;

hdt_parallel_for(0, size, [&](int i) {
    // Read-only work (no lock needed)
    auto shape0 = getPair(i).first;
    auto shape1 = getPair(i).second;

    if (needsCollision(shape0, shape1)) {
        // Mutation requires lock
        HDT_LOCK_GUARD(l, lock);
        bodies.insert(shape0);
        bodies.insert(shape1);
    }
});
```

### Pattern 3: Async Dispatch with Sync Point

```cpp
// Main thread
void SkyrimPhysicsWorld::onEvent(const FrameEvent& e) {
    // Dispatch work asynchronously
    m_tasks.run([...] { doUpdate2ndStep(...); });
    // Return immediately - game continues rendering
}

void SkyrimPhysicsWorld::onEvent(const FrameSyncEvent& e) {
    // Sync point - wait for physics to complete
    m_tasks.wait();
}
```

## Common Pitfalls

### 1. Lock Scope Too Wide

```cpp
// BAD: Lock held during expensive work
std::lock_guard l(m_lock);
for (auto& body : bodies) {
    body->expensiveOperation();  // Other threads blocked!
}

// GOOD: Lock only for collection, release before work
std::vector<Body*> localCopy;
{
    std::lock_guard l(m_lock);
    localCopy = bodies;
}
for (auto& body : localCopy) {
    body->expensiveOperation();  // No lock held
}
```

### 2. Forgetting Nested Parallelism

```cpp
// This is OK in enkiTS due to work-stealing
hdt_parallel_for_each(systems, [&](System* sys) {
    // This nested parallel_for works correctly
    hdt_parallel_for_each(sys->bodies, [](Body* b) {
        b->update();
    });
});
```

### 3. Data Race on Container Resize

```cpp
// BAD: Vector may reallocate during parallel push_back
hdt_parallel_for(0, n, [&](int i) {
    results.push_back(compute(i));  // RACE!
});

// GOOD: Pre-allocate or use lock
results.resize(n);
hdt_parallel_for(0, n, [&](int i) {
    results[i] = compute(i);  // No resize
});
```

## Performance Monitoring

### Tracy Zones for Threading

| Zone | What It Measures |
|------|------------------|
| `hdt_parallel_for` | Parallel loop overhead |
| `hdt_parallel_for_each` | Parallel iteration overhead |
| `hdt_parallel_invoke` | Multi-function dispatch |
| `AsyncTaskGroup::run` | Async dispatch |
| `AsyncTaskGroup::wait` | Sync wait time |

### Metrics to Watch

```
┌─────────────────────────────────────────────────────────────────────┐
│                    THREADING METRICS                                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Thread Utilization:                                               │
│   ├── All cores busy during parallel regions?                       │
│   ├── Work-stealing visible in Tracy?                               │
│   └── Any threads spinning on locks?                                │
│                                                                     │
│   Overhead Analysis:                                                │
│   ├── Dispatch time (should be < 10μs per region)                   │
│   ├── Chunk size (too small = overhead, too large = imbalance)      │
│   └── Lock contention time                                          │
│                                                                     │
│   Scaling:                                                          │
│   ├── Does performance improve with more cores?                     │
│   ├── Is there a saturation point?                                  │
│   └── Amdahl's law: what % is sequential?                           │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Adding New Parallel Code

### Checklist

1. **Identify parallelizable work**: Independent iterations? No order dependency?
2. **Choose primitive**: `parallel_for` for index-based, `parallel_for_each` for containers
3. **Handle shared state**: Use lock or eliminate sharing
4. **Add Tracy instrumentation**: `HDT_ZONE_SCOPED_N("MyParallelRegion")`
5. **Test with thread sanitizer**: Catch data races early
6. **Profile**: Ensure parallel version is actually faster

### Template

```cpp
void MyNewParallelOperation(std::vector<Item>& items)
{
    HDT_ZONE_SCOPED_N("MyNewParallelOperation");
    HDT_ZONE_VALUE(static_cast<int64_t>(items.size()));

    // Option 1: No shared state
    hdt_parallel_for_each(items, [](Item& item) {
        item.computeIndependently();
    });

    // Option 2: Collect results
    SpinLock lock;
    std::vector<Result> results;
    results.reserve(items.size());

    hdt_parallel_for_each(items, [&](Item& item) {
        auto result = item.compute();
        HDT_LOCK_GUARD(l, lock);
        results.push_back(result);
    });

    // Option 3: Pre-sized output
    std::vector<Result> results(items.size());
    hdt_parallel_for(0, static_cast<int>(items.size()), [&](int i) {
        results[i] = items[i].compute();
    });
}
```

## Thread Safety Quick Reference

```
┌─────────────────────────────────────────────────────────────────────┐
│                    THREAD SAFETY QUICK REFERENCE                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   SAFE from any thread:                                             │
│   ✓ EnkiTSScheduler::get()                                          │
│   ✓ SkyrimPhysicsWorld::get()                                       │
│   ✓ Ref<T> operations (atomic refcount)                             │
│   ✓ Reading immutable config                                        │
│                                                                     │
│   REQUIRES m_lock:                                                  │
│   ⚠ m_systems modification                                          │
│   ⚠ addSkinnedMeshSystem / removeSkinnedMeshSystem                  │
│   ⚠ resetSystems                                                    │
│   ⚠ stepSimulation (acquired in doUpdate2ndStep)                    │
│                                                                     │
│   SINGLE-THREADED ONLY (main thread):                               │
│   ✗ onEvent(FrameEvent)                                             │
│   ✗ readTransform (reads game state)                                │
│   ✗ SKSE callbacks                                                  │
│                                                                     │
│   PARALLEL SAFE (within physics step):                              │
│   ✓ SkinnedMeshBody::internalUpdate (per-body)                      │
│   ✓ SkinnedMeshShape::internalUpdate (per-shape)                    │
│   ✓ collapseCollideL (per-pair)                                     │
│   ✓ Bullet solver iterations                                        │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

**Congratulations!** You've completed the architecture tutorial series. You now understand the hdtSMP64 physics pipeline from concept to implementation detail.

For further exploration:
- Run with Tracy profiler to see real-time execution
- Add instrumentation to areas you're investigating
- Trace through specific code paths with a debugger
