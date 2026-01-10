# Task Graph Architecture for Physics Pipeline

## Status: DECISION MADE - enkiTS Full Replacement

> **UPDATE:** Expert panel review concluded that adding a second task scheduler
> (Taskflow or enkiTS) alongside PPL causes thread over-subscription.
> The approved solution is **full replacement of PPL with enkiTS**.
> See [enkits-replacement-plan.md](enkits-replacement-plan.md) for implementation details.

## Problem Statement

Current physics pipeline uses manual `parallel_invoke` blocks with hardcoded dependency analysis. This is:
- Brittle: Changes require manual re-analysis of dependencies
- Suboptimal: Not all parallelism opportunities are exploited
- Hard to reason about: Dependencies scattered across code

## Current Pipeline (Sequential with Manual Parallelism)

```
Frame Start
    │
    ├─[ParallelFrameStart]─────────────────────────┐
    │   ├── GpuSync (wait for prev frame)          │
    │   ├── PredictMotion                          │ parallel_invoke
    │   └── SystemsInternalUpdate                  │
    │──────────────────────────────────────────────┘
    │
    ▼
CreatePredictiveContacts (sequential)
    │
    ▼
CollisionDetection ──────────────────────── 8.1ms
    │
    ▼
CalculateSimulationIslands (sequential)
    │
    ▼
SolveConstraints ────────────────────────── 10.4ms
    ├── ConvertBodies ─── 0.4ms  ← Could run earlier!
    ├── ConvertJoints ─── 3.6ms  ← Could run earlier!
    ├── ConvertContacts ─ 0.03ms ← NEEDS collision manifolds
    └── SolverIterations ─ 6.1ms
    │
    ▼
IntegrateTransforms
    │
    ▼
Frame End
```

## Dependency Analysis

| Operation | Inputs | Outputs | Can Parallel With |
|-----------|--------|---------|-------------------|
| GpuSync | prev frame GPU | collision results | PredictMotion, InternalUpdate |
| PredictMotion | bodies, gravity | interpolated transforms | GpuSync, InternalUpdate |
| InternalUpdate | game bones | shape AABBs | GpuSync, PredictMotion |
| UpdateAabbs | shapes | broadphase | - |
| CollisionDetection | AABBs | manifolds | ConvertBodies, ConvertJoints |
| ConvertBodies | bodies | solver bodies | CollisionDetection |
| ConvertJoints | constraints | solver constraints | CollisionDetection |
| ConvertContacts | **manifolds** | contact constraints | - (needs collision) |
| SolverIterate | all constraints | impulses | - |
| IntegrateTransforms | impulses | final transforms | - |

## Key Insight

**ConvertBodies + ConvertJoints (4ms) can run in parallel with CollisionDetection (8ms)**

Only ConvertContacts needs collision manifolds. This could save ~4ms per frame.

## Proposed Architecture: Task Graph

### Option 1: Taskflow (Recommended)

```cpp
#include <taskflow/taskflow.hpp>

class PhysicsTaskGraph {
    tf::Executor executor;
    tf::Taskflow taskflow;

public:
    void buildGraph() {
        // Declare tasks
        auto gpuSync = taskflow.emplace([this]{ syncGpu(); }).name("GpuSync");
        auto predict = taskflow.emplace([this]{ predictMotion(); }).name("PredictMotion");
        auto internal = taskflow.emplace([this]{ internalUpdate(); }).name("InternalUpdate");
        auto aabbs = taskflow.emplace([this]{ updateAabbs(); }).name("UpdateAabbs");
        auto collision = taskflow.emplace([this]{ collisionDetection(); }).name("Collision");
        auto bodies = taskflow.emplace([this]{ convertBodies(); }).name("ConvertBodies");
        auto joints = taskflow.emplace([this]{ convertJoints(); }).name("ConvertJoints");
        auto contacts = taskflow.emplace([this]{ convertContacts(); }).name("ConvertContacts");
        auto solve = taskflow.emplace([this]{ solverIterate(); }).name("SolverIterate");
        auto integrate = taskflow.emplace([this]{ integrateTransforms(); }).name("Integrate");

        // Declare dependencies (scheduler handles parallelism automatically)
        internal.precede(aabbs);
        aabbs.precede(collision);

        collision.precede(contacts);     // contacts needs manifolds
        bodies.precede(joints);          // joints needs body IDs
        joints.precede(solve);
        contacts.precede(solve);

        solve.precede(integrate);
    }

    void executeFrame() {
        executor.run(taskflow).wait();
    }
};
```

### Option 2: enkiTS (Minimal Overhead)

```cpp
#include "TaskScheduler.h"

enki::TaskScheduler g_TS;

struct CollisionTask : enki::ITaskSet {
    void ExecuteRange(enki::TaskSetPartition, uint32_t) override {
        collisionDetection();
    }
};

struct SolverPrepTask : enki::ITaskSet {
    void ExecuteRange(enki::TaskSetPartition, uint32_t) override {
        convertBodies();
        convertJoints();
    }
};

// Usage - manual dependency via WaitforTask
CollisionTask collisionTask;
SolverPrepTask solverPrepTask;

g_TS.AddTaskSetToPipe(&collisionTask);
g_TS.AddTaskSetToPipe(&solverPrepTask);  // Runs in parallel!

g_TS.WaitforTask(&collisionTask);
g_TS.WaitforTask(&solverPrepTask);

// Now both complete, can do contacts + solve
```

### Option 3: Custom btITaskScheduler with Dependencies

Extend Bullet's existing task scheduler interface:

```cpp
class btTaskSchedulerWithDeps : public btITaskScheduler {
    struct Task {
        std::function<void()> work;
        std::vector<Task*> dependencies;
        std::atomic<int> pendingDeps;
    };

    void submit(Task* task) {
        if (task->pendingDeps == 0) {
            // Ready to run
            enqueue(task);
        } else {
            // Wait for dependencies
            pendingTasks.push(task);
        }
    }

    void onTaskComplete(Task* task) {
        for (auto* dependent : task->dependents) {
            if (--dependent->pendingDeps == 0) {
                enqueue(dependent);
            }
        }
    }
};
```

## Expected Performance Impact

| Scenario | Current | With Task Graph | Savings |
|----------|---------|-----------------|---------|
| Dense (12 actors) | 20ms | 16ms | -20% |
| Medium (6 actors) | 12ms | 10ms | -17% |
| Light (2 actors) | 6ms | 5.5ms | -8% |

Primary savings from overlapping:
- ConvertBodies+Joints (4ms) with CollisionDetection (8ms)
- Estimated: hide 4ms of the 8ms collision cost

## Implementation Plan

### Phase 1: Proof of Concept (Low Risk)
1. Add Taskflow as header-only dependency
2. Create parallel wrapper for collision + solver prep
3. Measure impact without changing core Bullet code

### Phase 2: Full Integration
1. Replace manual parallel_invoke with task graph
2. Express all pipeline dependencies declaratively
3. Add instrumentation for task graph visualization

### Phase 3: Dynamic Optimization
1. Profile task execution times
2. Auto-tune grain sizes
3. Consider work-stealing between physics and game threads

## Library Comparison

| Feature | Taskflow | enkiTS | marl | PPL |
|---------|----------|--------|------|-----|
| Header-only | Yes | No (1 cpp) | No | N/A |
| DAG dependencies | Yes | Manual | Via fibers | No |
| MSVC support | Yes | Yes | Yes | Yes |
| Game-proven | Some | Many | SwiftShader | Yes |
| Overhead | Medium | Very Low | Medium | Low |
| Learning curve | Low | Very Low | Medium | Very Low |

## Recommendation

**REJECTED: Taskflow** - Expert panel found:
1. Thread over-subscription when mixed with PPL (2N threads for N cores)
2. YAGNI - DAG features unused for simple parallel+barrier pattern
3. Can't coexist with PPL without replacing all PPL usage

**APPROVED: Full enkiTS replacement** - See [enkits-replacement-plan.md](enkits-replacement-plan.md)
- Replaces ALL PPL usage (18 call sites, 12 files)
- Single thread pool = no over-subscription
- Lower overhead (0.5-1μs vs 2-5μs per task)
- Game-proven (Doom, Doom Eternal, Rage 2, Saints Row)
- 2-3 days implementation effort

## Files to Modify

1. `hdtSkinnedMeshWorld.cpp` - Replace internalSingleStepSimulation
2. `hdtGroupConstraintSolver.cpp` - Expose early conversion methods
3. New: `hdtPhysicsTaskGraph.h` - Task graph wrapper

## References

- Taskflow: https://taskflow.github.io
- enkiTS: https://github.com/dougbinks/enkiTS
- marl: https://github.com/google/marl
- Bullet btITaskScheduler: `LinearMath/btThreads.h`
