# Taskflow Proof of Concept

## Goal
Validate task graph approach by overlapping CollisionDetection with ConvertBodies+ConvertJoints.

## Setup

### 1. Add Taskflow Header
```bash
# Option A: Git submodule
git submodule add https://github.com/taskflow/taskflow external/taskflow

# Option B: Just copy headers (header-only library)
# Copy taskflow/taskflow/ to hdtSMP64/external/taskflow/
```

### 2. Include Path
In `hdtSMP64.vcxproj`:
```xml
<AdditionalIncludeDirectories>...;$(ProjectDir)..\external\taskflow;...</AdditionalIncludeDirectories>
```

## Minimal Implementation

### hdtPhysicsTaskGraph.h
```cpp
#pragma once
#include <taskflow/taskflow.hpp>
#include "hdtSkinnedMeshWorld.h"

namespace hdt {

class PhysicsTaskGraph {
    tf::Executor m_executor;
    tf::Taskflow m_taskflow;

    // Cached task handles for the frame graph
    tf::Task m_collisionTask;
    tf::Task m_solverPrepTask;
    tf::Task m_contactsTask;

    SkinnedMeshWorld* m_world;
    btContactSolverInfo* m_solverInfo;

public:
    PhysicsTaskGraph(SkinnedMeshWorld* world) : m_world(world) {
        buildGraph();
    }

    void buildGraph() {
        // Phase that can run in parallel
        m_collisionTask = m_taskflow.emplace([this] {
            HDT_ZONE_SCOPED_N("TF_Collision");
            m_world->performDiscreteCollisionDetection();
        }).name("Collision");

        m_solverPrepTask = m_taskflow.emplace([this] {
            HDT_ZONE_SCOPED_N("TF_SolverPrep");
            // Early conversion - bodies + joints only
            m_world->getConstraintSolver().prepareEarlyConversion(
                m_world->getCollisionObjectArray(),
                m_world->getNumCollisionObjects(),
                m_world->getConstraintArray(),
                m_world->getNumConstraints(),
                *m_solverInfo);
        }).name("SolverPrep");

        // Contacts must wait for collision (needs manifolds)
        m_contactsTask = m_taskflow.emplace([this] {
            HDT_ZONE_SCOPED_N("TF_Contacts");
            // ConvertContacts happens in solveConstraints
        }).name("Contacts");

        // Dependencies
        m_collisionTask.precede(m_contactsTask);
        m_solverPrepTask.precede(m_contactsTask);
        // Both collision and solverPrep can run in parallel!
    }

    void executeParallelPhase(btContactSolverInfo& info) {
        m_solverInfo = &info;
        m_executor.run(m_taskflow).wait();
    }
};

} // namespace hdt
```

### Integration in hdtSkinnedMeshWorld.cpp
```cpp
void SkinnedMeshWorld::internalSingleStepSimulation(btScalar timeStep) {
    // ... existing ParallelFrameStart ...

    // Prepare solver info early
    getSolverInfo().m_timeStep = timeStep;

    // NEW: Use task graph for parallel collision + solver prep
    {
        HDT_ZONE_SCOPED_N("TaskGraphPhase");
        m_physicsTaskGraph->executeParallelPhase(getSolverInfo());
    }

    // Continue with islands and solving (contacts already converted)
    calculateSimulationIslands();
    solveConstraints(getSolverInfo());  // Now faster - bodies/joints already done

    // ... rest unchanged ...
}
```

## Validation Checklist

- [ ] Taskflow headers compile with MSVC
- [ ] Task graph executes without deadlock
- [ ] Tracy shows parallel execution of Collision + SolverPrep
- [ ] ConvertBodies/ConvertJoints show as "skipped" in solver
- [ ] Frame time reduced by ~4ms in dense scenes
- [ ] No physics behavior changes (same simulation results)

## Rollback Plan

If issues arise, simply:
1. Remove task graph call
2. Restore sequential collision → solve flow
3. Keep taskflow headers for future use

## Next Steps After POC

1. **Expand graph**: Add more operations (UpdateAabbs, IntegrateTransforms)
2. **Visualize**: Use `taskflow.dump(std::cout)` for DOT graph output
3. **Profile**: Compare task overhead vs parallelism gains
4. **Consider**: Replace all parallel_invoke with unified task graph
