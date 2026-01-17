# Physics Mode Configuration Plan

> [!WARNING]
> **NEEDS REEVALUATION (Jan 2026)**
>
> This plan was written before the current deferred collision architecture was fully analyzed.
> Key findings from architecture review:
>
> 1. **CUDA builds already have deferred collisions** - hardcoded via `CUDA_DELAYED_COLLISIONS`
> 2. **The "realtime" use case is niche** - screenshot artists wanting same-frame results
> 3. **YAGNI concerns** - Engineering cost vs benefit questionable
> 4. **Current defaults work well** - `maxSubSteps=1` achieves the deferred behavior
>
> Before implementing, reconsider:
> - Is a config option actually needed, or is the compile-time CUDA/NOCUDA split sufficient?
> - Would users actually switch modes, or is this theoretical flexibility?
> - Does the tri-tri "higher accuracy" claim (line 16-20 hdtDispatcher.cpp) even matter?
>   (Spoiler: that comment is stale - ALL collision types are deferred in CUDA builds)

> *Configurable physics timing to eliminate cascading performance failures.*

## Table of Contents

- [Problem Statement](#problem-statement)
- [Solution](#solution-configurable-physics-mode)
- [Config Changes](#config-changes)
- [Recommended Configurations](#recommended-configurations)
- [Implementation](#implementation)
- [User Documentation](#user-documentation)
- [Testing Checklist](#testing-checklist)

---

## Problem Statement

> [!CAUTION]
> The current substep system (`maxSubSteps`) causes **cascading performance failures**:
>
> ```
> Low FPS → More substeps → More collision work → Even lower FPS → More substeps → 💀
> ```

With the recent move to **deferred physics** (async execution, 1 frame behind), multiple substeps provide no accuracy benefit - they're catching up to stale bone positions. The CPU cost is pure waste.

However, some users may prefer synchronized "realtime" physics for accuracy, accepting the performance tradeoffs.

## Solution: Configurable Physics Mode

Add a `physicsMode` config option with two modes:

| Mode | Substeps | Latency | Performance | Use Case |
|------|----------|---------|-------------|----------|
| **Deferred** (default) | 1 | 1 frame behind | Predictable, no cascading | Most users, VR, crowded scenes |
| **Realtime** | 1-N | Synchronized | Can cascade at low FPS | Accuracy purists, screenshots |

---

## Config Changes

### New Option in `<smp>` Section

```xml
<!--
  physicsMode: Controls physics timing behavior
  - "deferred" (default): Single physics step per frame, 1 frame behind game state.
    Best performance, no cascading slowdowns. Use maximumActiveSkeletons for perf tuning.
  - "realtime": Multiple substeps to stay synchronized with game state.
    More accurate but can cascade at low FPS. Requires careful skeleton management.
-->
<physicsMode>deferred</physicsMode>
```

### Interaction with Existing Options

| Option | Deferred Mode | Realtime Mode |
|--------|---------------|---------------|
| `maxSubSteps` | Ignored (always 1) | Used (1-N substeps) |
| `maximumActiveSkeletons` | Primary perf lever | Critical - keep low |
| `autoAdjustMaxSkeletons` | Recommended ON | Essential ON |
| `min-fps` | Sets physics tick rate | Sets tick + substep threshold |

---

## Recommended Configurations

> [!TIP]
> ### Performance (Deferred) - Default
> Recommended for most users, VR, and crowded scenes.

```xml
<smp>
  <physicsMode>deferred</physicsMode>
  <maximumActiveSkeletons>20</maximumActiveSkeletons>
  <autoAdjustMaxSkeletons>true</autoAdjustMaxSkeletons>
</smp>
<solver>
  <min-fps>60</min-fps>
  <!-- maxSubSteps ignored in deferred mode -->
</solver>
```

### Accuracy (Realtime) - For Enthusiasts

> [!WARNING]
> Realtime mode can cascade at low FPS. Keep `maximumActiveSkeletons` low.

```xml
<smp>
  <physicsMode>realtime</physicsMode>
  <maximumActiveSkeletons>5</maximumActiveSkeletons>
  <autoAdjustMaxSkeletons>true</autoAdjustMaxSkeletons>
</smp>
<solver>
  <min-fps>60</min-fps>
  <maxSubSteps>2</maxSubSteps>
</solver>
```

---

## Implementation

### Files to Modify

1. **hdtSkyrimPhysicsWorld.h** - Add enum and member
2. **hdtSkyrimPhysicsWorld.cpp** - Modify `doUpdate()` logic
3. **hdtSkinnedMeshWorld.cpp** - Modify `stepSimulation()`
4. **config.cpp** - Parse `physicsMode` option
5. **main.cpp** - Console command + stats display
6. **configs/configs.xml** - Add option with docs

### Code Changes

#### hdtSkyrimPhysicsWorld.h
```cpp
enum class PhysicsMode { Deferred, Realtime };

class SkyrimPhysicsWorld : ... {
    // ...
    PhysicsMode m_physicsMode = PhysicsMode::Deferred;
    // ...
};
```

#### hdtSkyrimPhysicsWorld.cpp - doUpdate()
```cpp
// Calculate max simulation time based on mode
float maxTime;
if (m_physicsMode == PhysicsMode::Deferred) {
    // Single step - no cascading, physics 1 frame behind
    maxTime = tick;
} else {
    // Realtime - multiple substeps allowed, can cascade
    maxTime = tick * m_maxSubSteps;
}
const auto remainingTimeStep = std::min(m_accumulatedInterval, maxTime);
```

#### hdtSkinnedMeshWorld.cpp - stepSimulation()
```cpp
int SkinnedMeshWorld::stepSimulation(btScalar remainingTimeStep, int maxSubSteps, btScalar fixedTimeStep)
{
    HDT_ZONE_SCOPED_N("StepSimulation");
    incrementFrame();
    applyGravity();
    if (hdt::SkyrimPhysicsWorld::get()->m_enableWind)
        applyWind();

    constexpr auto minPossiblePeriod = 1.0f / 300.0f;

    if (SkyrimPhysicsWorld::get()->m_physicsMode == PhysicsMode::Deferred) {
        // Single step only - no substep loop
        if (remainingTimeStep > minPossiblePeriod)
            internalSingleStepSimulation(remainingTimeStep);
    } else {
        // Realtime: legacy substep loop
        while (remainingTimeStep > fixedTimeStep) {
            internalSingleStepSimulation(fixedTimeStep);
            remainingTimeStep -= fixedTimeStep;
        }
        if (remainingTimeStep > minPossiblePeriod)
            internalSingleStepSimulation(remainingTimeStep);
    }

    clearForces();
    _bodies.clear();
    _shapes.clear();
    return 0;
}
```

#### config.cpp
```cpp
else if (tagEquals(tag, "physicsMode"))
{
    auto mode = reader.readString();
    if (mode == "realtime" || mode == "Realtime" || mode == "REALTIME")
        SkyrimPhysicsWorld::get()->m_physicsMode = PhysicsMode::Realtime;
    else
        SkyrimPhysicsWorld::get()->m_physicsMode = PhysicsMode::Deferred;
}
```

#### main.cpp - Console Commands
```cpp
// smp mode [deferred|realtime]
if (_strnicmp(buffer, "mode", MAX_PATH) == 0)
{
    auto world = SkyrimPhysicsWorld::get();
    if (buffer2[0] != '\0')
    {
        if (_strnicmp(buffer2, "deferred", MAX_PATH) == 0) {
            world->m_physicsMode = PhysicsMode::Deferred;
            Console_Print("[HDT-SMP] Physics mode: DEFERRED (single step, 1 frame behind)");
        } else if (_strnicmp(buffer2, "realtime", MAX_PATH) == 0) {
            world->m_physicsMode = PhysicsMode::Realtime;
            Console_Print("[HDT-SMP] Physics mode: REALTIME (substeps enabled, can cascade)");
        } else {
            Console_Print("[HDT-SMP] Unknown mode. Use: deferred, realtime");
        }
    }
    else
    {
        Console_Print("[HDT-SMP] Physics mode: %s",
            world->m_physicsMode == PhysicsMode::Deferred ? "DEFERRED" : "REALTIME");
    }
    return true;
}
```

<details>
<summary>Stats display integration</summary>

```cpp
// Update smp stats to show mode
Console_Print("  Mode: %s | MinFPS: %d | MaxSubSteps: %d%s",
    world->m_physicsMode == PhysicsMode::Deferred ? "DEFERRED" : "REALTIME",
    world->min_fps, world->m_maxSubSteps,
    world->m_physicsMode == PhysicsMode::Deferred ? " (ignored)" : "");
```

</details>

---

## User Documentation

### Deferred Mode (Default)

**How it works:** Physics runs asynchronously, one frame behind the game's bone positions. Each frame performs exactly one physics step, regardless of framerate.

**Advantages:**
- No cascading slowdowns - fixed cost per frame
- Handles crowded scenes (20+ NPCs) gracefully
- Predictable performance for VR
- Skeleton count is the intuitive performance lever

**Tradeoffs:**
- Physics is 1 frame behind (usually imperceptible)
- At very low FPS, physics may appear to slow down
- Slightly less precise collision at frame boundaries

**Best for:** Most users, performance-focused players, VR users, crowded scenes

### Realtime Mode

**How it works:** Physics attempts to stay synchronized with game state using multiple substeps when framerate drops below `min-fps`.

**Advantages:**
- Physics synchronized with current bone positions
- Traditional Bullet physics behavior
- More precise at varying framerates

**Tradeoffs:**
- **Can cascade:** Low FPS triggers more substeps → even lower FPS → death spiral
- Must carefully limit active skeletons (recommend 5 or fewer)
- Unpredictable performance in crowded scenes

**Best for:** Screenshot artists, physics accuracy enthusiasts, single-actor focus

---

## Migration Notes

- **Existing configs work unchanged** - defaults to Deferred mode
- **maxSubSteps still valid** - applies when using Realtime mode
- **Performance improvement automatic** - most users get better perf without config changes
- **MCM integration** - FSMP MCM can add dropdown for mode selection

---

## Testing Checklist

- [ ] Deferred: Single CollisionDetection in Tracy at all FPS levels
- [ ] Deferred: 20+ skeletons without cascading
- [ ] Deferred: Physics visually acceptable (1 frame lag not noticeable)
- [ ] Realtime: Substeps occur at low FPS (matches legacy)
- [ ] Realtime: Cascading occurs as expected
- [ ] Console: `smp mode deferred` / `smp mode realtime` work
- [ ] Console: `smp stats` shows current mode
- [ ] Config: `<physicsMode>` parsed correctly (case insensitive)
- [ ] Config: `smp reload` picks up mode changes
- [ ] Edge: Very high FPS (120+) - both modes similar
- [ ] Edge: Very low FPS (<20) - deferred stable, realtime cascades
- [ ] Edge: Loading screens handled correctly

---

<div align="center">

*For solver benchmarks, see [SOLVER_BENCHMARKS.md](SOLVER_BENCHMARKS.md)*

</div>
