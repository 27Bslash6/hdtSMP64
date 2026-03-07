# Level 2: Frame Lifecycle & Threading Model

**Audience**: Developers working on timing, performance, or async code
**Prerequisites**: Level 0-1, understanding of multithreading concepts

---

## The Big Picture

hdtSMP64 runs physics **asynchronously** from the main game thread. This allows the game to continue rendering while physics calculates on worker threads.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           FRAME TIMELINE                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Frame N                                Frame N+1                          │
│   ──────────────────────────────────     ────────────────────               │
│                                                                             │
│   MAIN THREAD:                                                              │
│   ┌──────────┬─────────────────────┬──────────┬──────────────────────┐      │
│   │FrameEvt │  Game Logic/Render  │FrameSync │  Game Logic/Render   │      │
│   │ (start) │                     │ (wait)   │                      │      │
│   └────┬─────┴─────────────────────┴────┬─────┴──────────────────────┘      │
│        │                                │                                   │
│        │ Dispatch async                 │ Wait for completion               │
│        ▼                                ▼                                   │
│   WORKER THREAD:                                                            │
│   ─────┬─────────────────────────────────┐                                  │
│        │       doUpdate2ndStep()         │                                  │
│        │  ┌────────────────────────────┐ │                                  │
│        │  │ stepSimulation()           │ │                                  │
│        │  │  ├─ predictMotion          │ │                                  │
│        │  │  ├─ collisionDetection     │ │                                  │
│        │  │  ├─ solveConstraints       │ │                                  │
│        │  │  └─ integrate              │ │                                  │
│        │  └────────────────────────────┘ │                                  │
│        │                                 │                                  │
│   ─────┴─────────────────────────────────┘                                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Event Flow

### FrameEvent (Start of Frame)

Triggered by the game engine at the start of each frame.

```cpp
// hdtSkyrimPhysicsWorld.cpp:370-412
void SkyrimPhysicsWorld::onEvent(const FrameEvent& e)
{
    // 1. Handle pause/suspend state transitions
    if ((e.gamePaused || IsMenuManagerGamePaused(mm)) && !m_suspended) {
        suspend();
    }
    else if (!paused && m_suspended) {
        resume();
        return;  // Skip physics this frame after resume
    }

    // 2. Get time delta from game
    float interval = *(float*)(baseAddr + offset::GameStepTimer);

    // 3. Dispatch physics if conditions are met
    if (interval > FLT_EPSILON && !m_suspended && !m_isStasis && !m_systems.empty())
        doUpdate(interval);
}
```

**Location**: `hdtSkyrimPhysicsWorld.cpp:370`

### doUpdate (Time Accumulation)

Calculates substeps and dispatches async work.

```cpp
// hdtSkyrimPhysicsWorld.cpp:74-126
void SkyrimPhysicsWorld::doUpdate(float interval)
{
    // 1. Accumulate time since last computation
    m_accumulatedInterval += interval;

    // 2. Calculate tick (physics timestep)
    //    - Uses exponential average for stability
    //    - Clamped to min-fps (default 60Hz)
    m_averageInterval += (interval - m_averageInterval) * .125f;
    const auto tick = std::min(m_averageInterval, m_timeTick);

    // 3. Skip if too little time passed
    if (m_accumulatedInterval * 2.0f > tick) {
        // 4. Limit to maxSubSteps (default 4)
        const auto remainingTimeStep = std::min(m_accumulatedInterval, tick * m_maxSubSteps);

        // 5. Read bone transforms from game (SYNC)
        readTransform(remainingTimeStep);

        // 6. Dispatch physics work (ASYNC)
        m_tasks.run([this, interval, tick, remainingTimeStep] {
            doUpdate2ndStep(interval, tick, remainingTimeStep);
        });
    }
}
```

**Location**: `hdtSkyrimPhysicsWorld.cpp:74`

### doUpdate2ndStep (Async Physics)

Runs on a worker thread, performs the actual simulation.

```cpp
// hdtSkyrimPhysicsWorld.cpp:128-169
void SkyrimPhysicsWorld::doUpdate2ndStep(float interval, const float tick, const float remainingTimeStep)
{
    // 1. Early exit if suspended
    if (m_suspended || m_isStasis) return;

    // 2. Acquire lock (protects against concurrent modifications)
    std::lock_guard<decltype(m_lock)> l(m_lock);

    // 3. Update active state (which bodies are enabled)
    updateActiveState();

    // 4. Apply translation offset (numerical stability)
    auto offset = applyTranslationOffset();

    // 5. Step the simulation
    stepSimulation(remainingTimeStep, 0, tick);

    // 6. Restore offset
    restoreTranslationOffset(offset);
    m_accumulatedInterval = 0;

    // 7. Write results back to game
    writeTransform();
}
```

**Location**: `hdtSkyrimPhysicsWorld.cpp:128`

### FrameSyncEvent (End of Frame)

Wait point - ensures physics completes before next frame starts.

```cpp
// hdtSkyrimPhysicsWorld.cpp:414-438
void SkyrimPhysicsWorld::onEvent(const FrameSyncEvent& e)
{
    m_tasks.wait();  // Block until physics completes
}
```

**Location**: `hdtSkyrimPhysicsWorld.cpp:414`

## Timing Diagram: Normal Operation

```
Time ──────────────────────────────────────────────────────────────────────────►

     Frame N                          Frame N+1                    Frame N+2
     ├───────────────────────────────┤├─────────────────────────────┤

MAIN THREAD:
├────┬────────────────────────┬──────┤├────┬────────────────────────┬──────┤
│ FE │   GAME LOGIC/RENDER   │  FS  ││ FE │   GAME LOGIC/RENDER   │  FS  │
└──┬─┴───────────────────────┴──┬───┘└──┬─┴───────────────────────┴──┬───┘
   │                            │       │                            │
   │ dispatch                   │ wait  │ dispatch                   │ wait
   ▼                            ▼       ▼                            ▼

WORKER THREADS:
   ┌────────────────────────────┐       ┌────────────────────────────┐
   │   doUpdate2ndStep()        │       │   doUpdate2ndStep()        │
   │   ┌──────────────────────┐ │       │   ┌──────────────────────┐ │
   │   │  stepSimulation()    │ │       │   │  stepSimulation()    │ │
   │   └──────────────────────┘ │       │   └──────────────────────┘ │
   └────────────────────────────┘       └────────────────────────────┘

Legend:
  FE = FrameEvent (onEvent)
  FS = FrameSyncEvent (wait)
```

## Timing Diagram: Slow Physics

When physics takes longer than the frame, it overlaps into the next frame:

```
Time ──────────────────────────────────────────────────────────────────────────►

     Frame N              Frame N+1                    Frame N+2
     ├─────────────────┤├─────────────────────────────┤

MAIN THREAD:
├────┬──────────┬──────┤├────┬────────────────────────┬──────┤
│ FE │  RENDER  │  FS  ││ FE │   GAME LOGIC/RENDER   │  FS  │
└──┬─┴──────────┴──┬───┘└──┬─┴───────────────────────┴──┬───┘
   │               │       │                            │
   │               │ WAIT  │ dispatch                   │ wait
   ▼               ▼   ▼   ▼                            ▼

WORKER THREADS:
   ┌──────────────────────────┐ ┌────────────────────────────┐
   │   LONG PHYSICS STEP      │ │   doUpdate2ndStep()        │
   │   (takes 1.5 frames)     │ │                            │
   └──────────────────────────┘ └────────────────────────────┘
                           ▲
                           │
                    FrameSync BLOCKS here
                    until physics completes
```

## Substep Calculation

The physics runs at a fixed timestep (default 1/60s), but the game may run faster or slower:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    SUBSTEP CALCULATION                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Game FPS > min-fps (60):                                          │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ interval = 1/120s = 8.3ms                                   │   │
│   │ tick = averageInterval ≈ 8.3ms                              │   │
│   │ substeps = 1 (one small step)                               │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   Game FPS = min-fps (60):                                          │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ interval = 1/60s = 16.6ms                                   │   │
│   │ tick = min(averageInterval, 1/60s) = 16.6ms                 │   │
│   │ substeps = 1 (one standard step)                            │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   Game FPS < min-fps (30fps):                                       │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ interval = 1/30s = 33.3ms                                   │   │
│   │ tick = 16.6ms (clamped to min-fps)                          │   │
│   │ remainingTimeStep = min(33.3ms, 16.6ms * 4) = 33.3ms        │   │
│   │ substeps = 2 (two physics steps per frame)                  │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   Game FPS very low (15fps):                                        │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ interval = 1/15s = 66.6ms                                   │   │
│   │ tick = 16.6ms                                               │   │
│   │ remainingTimeStep = min(66.6ms, 16.6ms * 4) = 66.4ms        │   │
│   │ substeps = 4 (CLAMPED - physics runs at max speed)          │   │
│   │                                                             │   │
│   │ ⚠ Below 15fps, physics starts to slow down (jitter)        │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Suspend/Resume State Machine

Physics can be suspended during loading, menus, or explicit commands:

```
                    ┌─────────────────────────┐
                    │     RUNNING STATE       │
                    │   m_suspended = false   │
                    └───────────┬─────────────┘
                                │
        ┌───────────────────────┼───────────────────────┐
        │                       │                       │
        ▼                       ▼                       ▼
┌───────────────┐      ┌───────────────┐      ┌───────────────┐
│ Game Paused   │      │ PreLoadGame   │      │ smp reset cmd │
│ (Menu/Loading)│      │ SKSE message  │      │ (Console)     │
└───────┬───────┘      └───────┬───────┘      └───────┬───────┘
        │                      │                      │
        └──────────────────────┼──────────────────────┘
                               │
                               ▼
                    ┌─────────────────────────┐
                    │      suspend(loading)   │
                    │  1. m_tasks.wait()      │ ◄── Wait for async physics
                    │  2. m_suspended = true  │
                    │  3. m_loading = loading │
                    └───────────┬─────────────┘
                                │
                                ▼
                    ┌─────────────────────────┐
                    │    SUSPENDED STATE      │
                    │   m_suspended = true    │
                    │                         │
                    │  doUpdate2ndStep()      │
                    │  returns immediately    │
                    └───────────┬─────────────┘
                                │
                                │ (Game unpaused / Load complete)
                                ▼
                    ┌─────────────────────────┐
                    │       resume()          │
                    │  1. if m_loading:       │
                    │     resetSystems()      │ ◄── Re-read all bones
                    │     m_loading = false   │
                    │  2. m_suspended = false │
                    └───────────┬─────────────┘
                                │
                                │ (Skip doUpdate this frame)
                                ▼
                    ┌─────────────────────────┐
                    │     RUNNING STATE       │
                    │   m_suspended = false   │
                    └─────────────────────────┘
```

## Thread Safety

### Lock Hierarchy

```
┌─────────────────────────────────────────────────────────────────────┐
│                      LOCK HIERARCHY                                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   SkyrimPhysicsWorld::m_lock (SpinLock)                             │
│   ├── Protects: m_systems, physics state                            │
│   ├── Held during: doUpdate2ndStep, addSystem, removeSystem         │
│   └── NOT held during: readTransform (game thread only)             │
│                                                                     │
│   CollisionDispatcher::m_lock (SpinLock)                            │
│   ├── Protects: m_manifoldsPtr, manifold allocation                 │
│   ├── Held during: createManifold, clearAllManifold                 │
│   └── Fine-grained for collision processing                         │
│                                                                     │
│   Per-pair SpinLock (in dispatchAllCollisionPairs)                  │
│   ├── Protects: m_pairs vector during parallel collection           │
│   └── Very short-lived (just insertion)                             │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Critical Sections

| Operation | Lock | Thread |
|-----------|------|--------|
| `doUpdate` | None (main thread only) | Main |
| `readTransform` | None (game state) | Main |
| `doUpdate2ndStep` | `m_lock` | Worker |
| `stepSimulation` | `m_lock` (held) | Worker |
| `addSystem` | `m_lock` | Main |
| `removeSystem` | `m_lock` | Main |
| `resetSystems` | `m_lock` | Main |

## AsyncTaskGroup Implementation

The async dispatch uses enkiTS under the hood:

```cpp
// hdtEnkiTSScheduler.h:332-403
class AsyncTaskGroup
{
public:
    // Launch a task asynchronously
    template <typename Func>
    void run(Func&& func)
    {
        auto task = std::make_unique<AsyncTask>(std::forward<Func>(func));
        EnkiTSScheduler::get().scheduler().AddTaskSetToPipe(task.get());
        m_pendingTasks.push_back(std::move(task));
    }

    // Wait for all pending tasks to complete
    void wait()
    {
        auto& scheduler = EnkiTSScheduler::get().scheduler();
        for (auto& task : m_pendingTasks) {
            scheduler.WaitforTask(task.get());
        }
        m_pendingTasks.clear();
    }
};
```

**Location**: `hdtEnkiTSScheduler.h:332`

## Next Steps

- **[Level 3: Collision Pipeline](./03-COLLISION-PIPELINE.md)** - Deep dive into collision detection
- **[Level 4: Constraint Solving](./04-CONSTRAINT-SOLVING.md)** - How constraints are solved
