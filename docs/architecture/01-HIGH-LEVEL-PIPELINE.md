# Level 1: High-Level Pipeline Flow

**Audience**: Developers starting to read the code, intermediate level
**Prerequisites**: Level 0, basic C++ understanding

---

## Architecture Layers

hdtSMP64 is organized into distinct layers, each with specific responsibilities:

```
┌─────────────────────────────────────────────────────────────────────┐
│                       LAYER 1: SKSE PLUGIN                          │
│                                                                     │
│   main.cpp              Hooks.cpp            ActorManager.cpp       │
│   ┌──────────┐          ┌──────────┐         ┌──────────────┐       │
│   │ Plugin   │          │ Game     │         │ Track which  │       │
│   │ Entry    │          │ Function │         │ actors have  │       │
│   │ Point    │          │ Hooks    │         │ physics      │       │
│   └──────────┘          └──────────┘         └──────────────┘       │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    LAYER 2: SKYRIM PHYSICS                          │
│                                                                     │
│   SkyrimPhysicsWorld          SkyrimSystem           SkyrimBone     │
│   ┌──────────────┐           ┌──────────────┐       ┌───────────┐   │
│   │ Singleton    │           │ Per-skeleton │       │ Per-bone  │   │
│   │ world        │◄──────────│ physics      │◄──────│ physics   │   │
│   │ manager      │           │ system       │       │ state     │   │
│   └──────────────┘           └──────────────┘       └───────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                  LAYER 3: SKINNED MESH PHYSICS                      │
│                                                                     │
│   SkinnedMeshWorld        SkinnedMeshSystem       SkinnedMeshBody   │
│   ┌──────────────┐       ┌───────────────┐        ┌─────────────┐   │
│   │ Custom       │       │ Bone/body/    │        │ Collision   │   │
│   │ Bullet       │◄──────│ constraint    │◄───────│ geometry    │   │
│   │ world        │       │ container     │        │ for mesh    │   │
│   └──────────────┘       └───────────────┘        └─────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     LAYER 4: BULLET ENGINE                          │
│                                                                     │
│   btDiscreteDynamicsWorld    btRigidBody         btTypedConstraint  │
│   ┌──────────────────┐      ┌────────────┐       ┌──────────────┐   │
│   │ Core physics     │      │ Physics    │       │ Joint/spring │   │
│   │ simulation       │◄─────│ body       │◄──────│ definitions  │   │
│   └──────────────────┘      └────────────┘       └──────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Key Source Files

### Layer 1: SKSE Plugin

| File | Purpose |
|------|---------|
| `main.cpp` | SKSE plugin entry point, registers hooks and listeners |
| `Hooks.cpp` | Microsoft Detours hooks into game functions |
| `ActorManager.cpp` | Tracks actors with physics, manages skeleton activation |
| `config.cpp` | Runtime configuration loading |
| `XmlReader.cpp` | Physics XML file parsing |

### Layer 2: Skyrim-Specific Physics

| File | Purpose |
|------|---------|
| `hdtSkyrimPhysicsWorld.cpp` | Main physics world, handles game events |
| `hdtSkyrimSystem.cpp` | Per-skeleton physics system creator |
| `hdtSkyrimBone.cpp` | Bone physics with Skyrim-specific logic |
| `hdtSkyrimBody.cpp` | Collision body with tag-based filtering |

### Layer 3: Skinned Mesh Core

| File | Purpose |
|------|---------|
| `hdtSkinnedMesh/hdtSkinnedMeshWorld.cpp` | Custom Bullet world with skinned mesh support |
| `hdtSkinnedMesh/hdtSkinnedMeshSystem.cpp` | Container for bones/bodies/constraints |
| `hdtSkinnedMesh/hdtSkinnedMeshBody.cpp` | Physics body following bone transforms |
| `hdtSkinnedMesh/hdtDispatcher.cpp` | Collision dispatch with parallelization |
| `hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.cpp` | Narrow-phase collision detection |
| `hdtSkinnedMesh/hdtGroupConstraintSolver.cpp` | Grouped constraint solving |
| `hdtSkinnedMesh/hdtEnkiTSScheduler.h` | enkiTS parallel primitives wrapper |

## Sequence Diagram: Plugin Startup

```
┌──────────┐     ┌──────────┐     ┌─────────────────────┐     ┌────────────────┐
│  SKSE    │     │ main.cpp │     │ SkyrimPhysicsWorld  │     │ ActorManager   │
└────┬─────┘     └────┬─────┘     └──────────┬──────────┘     └───────┬────────┘
     │                │                       │                       │
     │ Load Plugin    │                       │                       │
     │───────────────►│                       │                       │
     │                │                       │                       │
     │                │ SKSEPlugin_Load()     │                       │
     │                │──────────────────────►│                       │
     │                │                       │                       │
     │                │                       │ Get singleton         │
     │                │                       │◄──────────────────────│
     │                │                       │                       │
     │                │ Register hooks        │                       │
     │                │───────────────────────┤                       │
     │                │                       │                       │
     │                │ Register event listeners                      │
     │                │───────────────────────┴───────────────────────►
     │                │                                               │
     │ Plugin Ready   │                                               │
     │◄───────────────│                                               │
```

## Sequence Diagram: Armor Attachment

When a character equips armor with physics:

```
┌──────────┐  ┌──────────┐  ┌─────────────────┐  ┌─────────────────────┐
│  Game    │  │ Hooks.cpp│  │  ActorManager   │  │ SkyrimPhysicsWorld  │
└────┬─────┘  └────┬─────┘  └───────┬─────────┘  └──────────┬──────────┘
     │             │                │                       │
     │ AttachArmor │                │                       │
     │────────────►│                │                       │
     │             │                │                       │
     │             │ onEvent(ArmorAttachEvent)              │
     │             │───────────────►│                       │
     │             │                │                       │
     │             │                │ Find/create skeleton  │
     │             │                │───────┐               │
     │             │                │       │               │
     │             │                │◄──────┘               │
     │             │                │                       │
     │             │                │ Load physics XML      │
     │             │                │───────┐               │
     │             │                │       │ XmlReader     │
     │             │                │◄──────┘               │
     │             │                │                       │
     │             │                │ Create SkyrimSystem   │
     │             │                │───────┐               │
     │             │                │       │               │
     │             │                │◄──────┘               │
     │             │                │                       │
     │             │                │ addSkinnedMeshSystem()│
     │             │                │──────────────────────►│
     │             │                │                       │
     │ Return      │                │                       │
     │◄────────────│                │                       │
```

## Sequence Diagram: Frame Update (NOCUDA)

The core physics loop that runs every frame:

```
┌──────────┐  ┌─────────────────────┐  ┌───────────────────┐  ┌────────────────┐
│  Game    │  │ SkyrimPhysicsWorld  │  │ SkinnedMeshWorld  │  │ enkiTS Workers │
└────┬─────┘  └──────────┬──────────┘  └─────────┬─────────┘  └───────┬────────┘
     │                   │                       │                    │
     │ FrameEvent        │                       │                    │
     │──────────────────►│                       │                    │
     │                   │                       │                    │
     │                   │ Check pause/suspend   │                    │
     │                   │───────┐               │                    │
     │                   │◄──────┘               │                    │
     │                   │                       │                    │
     │                   │ doUpdate(interval)    │                    │
     │                   │───────────────────────┤                    │
     │                   │                       │                    │
     │                   │ readTransform()       │                    │
     │                   │       │               │                    │
     │                   │◄──────┘               │                    │
     │                   │                       │                    │
     │                   │ Dispatch async task   │                    │
     │                   │─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─►│
     │                   │                       │                    │
     │ Return (async)    │                       │   doUpdate2ndStep  │
     │◄──────────────────│                       │◄───────────────────│
     │                   │                       │                    │
     │                   │                       │ stepSimulation()   │
     │                   │                       │───────┐            │
     │                   │                       │       │            │
     │ (Game continues)  │                       │       ▼            │
     │                   │                       │ Physics runs       │
     │                   │                       │ on worker thread   │
     │                   │                       │       │            │
     │                   │                       │◄──────┘            │
     │                   │                       │                    │
     │ FrameSyncEvent    │                       │ writeTransform()   │
     │──────────────────►│                       │◄──────────────────►│
     │                   │                       │                    │
     │                   │ m_tasks.wait()        │                    │
     │                   │◄──────────────────────┴────────────────────│
     │                   │                       │                    │
     │ Frame Complete    │                       │                    │
     │◄──────────────────│                       │                    │
```

## Data Flow Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                          DATA FLOW                                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   GAME STATE                                                        │
│   ┌─────────────────────────────────────────────────────────┐       │
│   │  NiNode skeletal hierarchy (bone transforms)            │       │
│   │  NiPoint3 wind vector                                   │       │
│   │  float gameTime                                         │       │
│   └───────────────────────────┬─────────────────────────────┘       │
│                               │                                     │
│                               ▼ readTransform()                     │
│   PHYSICS STATE                                                     │
│   ┌─────────────────────────────────────────────────────────┐       │
│   │  btRigidBody positions/velocities                       │       │
│   │  btTypedConstraint joint states                         │       │
│   │  btPersistentManifold collision contacts               │        │
│   └───────────────────────────┬─────────────────────────────┘       │
│                               │                                     │
│                               ▼ stepSimulation()                    │
│   SIMULATION                                                        │
│   ┌─────────────────────────────────────────────────────────┐       │
│   │  1. Predict motion (apply forces/damping)               │       │
│   │  2. Collision detection (find intersections)            │       │
│   │  3. Constraint solving (apply joints/springs)           │       │
│   │  4. Integration (update positions)                      │       │
│   └───────────────────────────┬─────────────────────────────┘       │
│                               │                                     │
│                               ▼ writeTransform()                    │
│   GAME STATE (updated)                                              │
│   ┌─────────────────────────────────────────────────────────┐       │
│   │  NiNode skeletal hierarchy (updated transforms)         │       │
│   └─────────────────────────────────────────────────────────┘       │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Key Function Call Path

For a single physics frame in NOCUDA mode:

```
SkyrimPhysicsWorld::onEvent(FrameEvent)
    │
    ├── Check game state (paused, loading, suspended)
    │
    └── doUpdate(interval)
            │
            ├── Calculate tick and substeps
            │
            ├── readTransform(remainingTimeStep)
            │       │
            │       └── For each SkyrimSystem:
            │               └── Read bone matrices from game
            │
            └── AsyncTaskGroup::run() → doUpdate2ndStep()
                    │
                    ├── updateActiveState()
                    │
                    ├── applyTranslationOffset()
                    │
                    └── stepSimulation(timeStep, 0, tick)
                            │
                            ├── applyGravity()
                            ├── applyWind() [if enabled]
                            │
                            └── For each substep:
                                    └── internalSingleStepSimulation(timeStep)
                                            │
                                            ├── SystemsInternalUpdate [parallel]
                                            │       └── Update bone transforms
                                            │
                                            └── btDiscreteDynamicsWorldMt::internalSingleStepSimulation()
                                                    ├── predictUnconstraintMotion()
                                                    ├── performDiscreteCollisionDetection()
                                                    ├── calculateSimulationIslands()
                                                    ├── solveConstraints()
                                                    └── integrateTransforms()
```

## Next Steps

- **[Level 2: Frame Lifecycle](./02-FRAME-LIFECYCLE.md)** - Deep dive into frame timing and threading
- **[Level 3: Collision Pipeline](./03-COLLISION-PIPELINE.md)** - How collision detection works
