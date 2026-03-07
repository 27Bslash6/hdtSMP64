# hdtSMP64 NOCUDA Physics Pipeline Tutorial

A progressive tutorial series for understanding the hdtSMP64 physics simulation pipeline (CPU-only/NOCUDA build).

## Document Index

| Level | Document | Audience | Focus |
|-------|----------|----------|-------|
| 0 | [Conceptual Overview](./00-CONCEPTUAL-OVERVIEW.md) | Beginners | What physics simulation is, why hdtSMP64 exists |
| 1 | [High-Level Pipeline](./01-HIGH-LEVEL-PIPELINE.md) | Intermediate | Code organization, layer architecture |
| 2 | [Frame Lifecycle](./02-FRAME-LIFECYCLE.md) | Intermediate | Frame timing, async dispatch, state machine |
| 3 | [Collision Pipeline](./03-COLLISION-PIPELINE.md) | Advanced | Broadphase, narrowphase, BVH traversal |
| 4 | [Constraint Solving](./04-CONSTRAINT-SOLVING.md) | Advanced | Solver algorithm, groups, scalar constraint resolution |
| 5 | [Data Structures](./05-DATA-STRUCTURES.md) | Advanced | Class hierarchy, memory layout, ownership |
| 6 | [Threading Expert](./06-THREADING-EXPERT.md) | Expert | enkiTS, parallelism patterns, race conditions |

## Reading Order

```
Beginner Path:           Full Path:                Expert Jump:
     │                        │                         │
     ▼                        ▼                         ▼
   Level 0              Level 0-1-2               Level 5-6
     │                        │                    (with reference
     ▼                        ▼                     to earlier)
   Level 1              Level 3-4-5
     │                        │
     ▼                        ▼
   (stop)               Level 6
```

## Quick Reference

### Key Source Files

| Component | Primary File | Header |
|-----------|--------------|--------|
| Physics World | `hdtSkyrimPhysicsWorld.cpp` | `.h` |
| Skinned Mesh World | `hdtSkinnedMesh/hdtSkinnedMeshWorld.cpp` | `.h` |
| Collision Dispatch | `hdtSkinnedMesh/hdtDispatcher.cpp` | `.h` |
| Constraint Solver | `hdtSkinnedMesh/hdtGroupConstraintSolver.cpp` | `.h` |
| Threading | `hdtSkinnedMesh/hdtEnkiTSScheduler.h` | (header-only) |

### Key Function Path (NOCUDA)

```
FrameEvent
└── SkyrimPhysicsWorld::onEvent()
    └── doUpdate(interval)
        ├── readTransform()
        └── AsyncTaskGroup::run() → doUpdate2ndStep()
            └── stepSimulation()
                └── internalSingleStepSimulation()
                    ├── SystemsInternalUpdate [sequential]
                    └── btDiscreteDynamicsWorldMt::internalSingleStepSimulation()
                        ├── predictUnconstraintMotion()
                        ├── performDiscreteCollisionDetection()
                        │   └── dispatchAllCollisionPairs()
                        │       ├── hdt_parallel_for (collect pairs)
                        │       ├── hdt_parallel_for_each (internal updates)
                        │       └── hdt_parallel_for_each (narrowphase)
                        ├── calculateSimulationIslands()
                        ├── solveConstraints()
                        │   └── btConstraintSolverPoolMt::solveGroup()
                        └── integrateTransforms()
```

### Parallel Regions

| Region | Location | Granularity |
|--------|----------|-------------|
| Pair collection | `hdtDispatcher.cpp:100` | Per-pair |
| Body internal update | `hdtDispatcher.cpp:351` | Per-body |
| Shape internal update | `hdtDispatcher.cpp:353-356` | Per-shape |
| Narrowphase collision | `hdtDispatcher.cpp:361` | Per-pair |
| Bullet solver | `btSequentialImpulseConstraintSolverMt` | Per-island |

## Contributing

When adding to these documents:

1. **Maintain accuracy**: Verify code paths with actual implementation
2. **Include line references**: `file.cpp:123` format for traceability
3. **Update diagrams**: Keep ASCII art in sync with code changes
4. **Progressive disclosure**: Each level should build on previous ones
