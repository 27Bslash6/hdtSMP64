# Level 5: Data Structures Reference

**Audience**: Developers modifying core systems, debugging, or extending functionality
**Prerequisites**: Level 0-4, solid C++ understanding

---

## Class Hierarchy Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         CLASS HIERARCHY                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Bullet Engine                      hdtSMP64 Custom                        │
│   ═══════════════                    ════════════════                       │
│                                                                             │
│   btDiscreteDynamicsWorld            SkyrimPhysicsWorld                     │
│         ▲                                   ▲                               │
│         │                                   │                               │
│   btDiscreteDynamicsWorldMt ◄────── SkinnedMeshWorld                        │
│                                            │                                │
│                                            ├──► SkinnedMeshSystem[]         │
│                                            │          │                     │
│   btRigidBody                              │          ├──► SkinnedMeshBone[]│
│        ▲                                   │          ├──► SkinnedMeshBody[]│
│        │                                   │          └──► Constraint[]     │
│   SkinnedMeshBone::m_rig                   │                                │
│                                            │                                │
│   btCollisionObject                        │                                │
│        ▲                                   │                                │
│        │                                   │                                │
│   SkinnedMeshBody ─────────────────────────┘                                │
│        │                                                                    │
│        └──► SkinnedMeshShape                                                │
│                    ▲                                                        │
│            ┌───────┴───────┐                                                │
│            │               │                                                │
│      PerVertexShape   PerTriangleShape                                      │
│                                                                             │
│   btTypedConstraint                                                         │
│        ▲                                                                    │
│        ├── GenericConstraint                                                │
│        ├── StiffSpringConstraint                                            │
│        ├── ConeTwistConstraint                                              │
│        └── BoneScaleConstraint                                              │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Core Classes

### SkyrimPhysicsWorld

The singleton physics world, integrates with Skyrim's event system.

```cpp
// hdtSkyrimPhysicsWorld.h
class SkyrimPhysicsWorld : public SkinnedMeshWorld,
                           public BSTEventSink<SKSECameraEvent>
{
public:
    static SkyrimPhysicsWorld* get();  // Singleton access

    // Event handlers
    void onEvent(const FrameEvent& e);       // Called at frame start
    void onEvent(const FrameSyncEvent& e);   // Called at frame end
    void onEvent(const ShutdownEvent& e);    // Called on game exit

    // Simulation control
    void doUpdate(float interval);
    void doUpdate2ndStep(float interval, float tick, float remainingTimeStep);

    // State management
    void suspend();
    void resume();
    void resetSystems();

private:
    // Threading
    AsyncTaskGroup m_tasks;             // Async physics dispatch
    SpinLock m_lock;                    // Protects physics state
    std::atomic<bool> m_suspended;      // Pause state
    std::atomic<bool> m_isStasis;       // Temporary pause

    // Timing
    float m_accumulatedInterval;        // Time since last physics step
    float m_averageInterval;            // Exponential average frame time
    float m_timeTick;                   // Physics timestep (1/min_fps)
    int m_maxSubSteps;                  // Max physics steps per frame

    // Environment
    btVector3 m_windSpeed;              // Current wind velocity
    bool m_enableWind;                  // Wind enabled flag
    float m_windStrength;               // Wind force multiplier
};
```

**Location**: `hdtSkyrimPhysicsWorld.h`

### SkinnedMeshWorld

Base class providing skinned mesh physics support on top of Bullet.

```cpp
// hdtSkinnedMeshWorld.h
class SkinnedMeshWorld : public btDiscreteDynamicsWorldMt
{
public:
    // System management
    void addSkinnedMeshSystem(SkinnedMeshSystem* system);
    void removeSkinnedMeshSystem(SkinnedMeshSystem* system);

    // Physics step
    int stepSimulation(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep);
    void internalSingleStepSimulation(btScalar timeStep);

    // Overridden Bullet methods
    void performDiscreteCollisionDetection() override;
    void solveConstraints(btContactSolverInfo& solverInfo) override;
    void integrateTransforms(btScalar timeStep) override;

protected:
    std::vector<Ref<SkinnedMeshSystem>> m_systems;

    // Solvers
    btConstraintSolverPoolMt* m_solverPool;
    GroupConstraintSolver m_constraintSolver;

    // Frame counter for dirty flag optimization
    static uint32_t s_currentFrame;

    // Wind vector (SIMD aligned)
    __m128 m_windSpeed;
};
```

**Location**: `hdtSkinnedMeshWorld.h`

### SkinnedMeshSystem

Container for all physics objects in a single physics definition (from XML).

```cpp
// hdtSkinnedMeshSystem.h
class SkinnedMeshSystem : public RefObject
{
public:
    // Transform sync
    void readTransform(float timeStep);   // Game → Physics
    void writeTransform();                 // Physics → Game
    void internalUpdate();                 // Update shapes/AABBs

    // Components
    std::vector<Ref<SkinnedMeshBone>> m_bones;
    std::vector<Ref<SkinnedMeshBody>> m_meshes;
    std::vector<Ref<GenericConstraint>> m_constraints;
    std::vector<Ref<ConstraintGroup>> m_constraintGroups;

    // State
    SkinnedMeshWorld* m_world;
    bool m_initialized;
    bool block_resetting;
};
```

**Location**: `hdtSkinnedMeshSystem.h`

### SkinnedMeshBone

A physics-enabled bone that can be driven by animation or simulation.

```cpp
// hdtSkinnedMeshBone.h
class SkinnedMeshBone : public RefObject
{
public:
    void readTransform(float timeStep);    // Read from game skeleton
    void writeTransform();                  // Write to game skeleton
    void internalUpdate();                  // Update world transform

    btRigidBody m_rig;                     // The Bullet rigid body

    // Properties
    float m_gravityFactor;                 // Per-bone gravity scale
    float m_windFactor;                    // Per-bone wind scale
    float m_marginMultiplier;              // Collision margin scale

    // Collision filtering
    std::vector<SkinnedMeshBone*> m_canCollideWithBones;
    std::vector<SkinnedMeshBone*> m_noCollideWithBones;
};
```

**Location**: `hdtSkinnedMeshBone.h`

### SkinnedMeshBody

A collision body that follows skeletal animation, used for collision detection.

```cpp
// hdtSkinnedMeshBody.h
class SkinnedMeshBody : public btCollisionObject, public RefObject
{
public:
    void internalUpdate();                 // Update vertex positions
    bool canCollideWith(const SkinnedMeshBody* body) const;
    bool isBoundingSphereCollided(SkinnedMeshBody* rhs);

    // Shape
    Ref<SkinnedMeshShape> m_shape;         // Collision geometry

    // Skinning data
    struct SkinnedBone {
        btMatrix4x3T vertexToBone;         // Transform matrix
        BoundingSphere localBoundingSphere;
        BoundingSphere worldBoundingSphere;
        SkinnedMeshBone* ptr;
        float weightThreshold;
        bool isKinematic;
    };
    std::vector<SkinnedBone> m_skinnedBones;
    std::vector<Bone> m_bones;             // Bone transforms for GPU
    std::vector<Vertex> m_vertices;        // Vertex data
    std::vector<VertexPos> m_vpos;         // Computed positions

    // Collision filtering
    std::vector<IDStr> m_tags;
    std::unordered_set<IDStr> m_canCollideWithTags;
    std::unordered_set<IDStr> m_noCollideWithTags;

    // State
    bool m_isKinematic;
    uint32_t m_lastUpdateFrame;            // Dirty flag
};
```

**Location**: `hdtSkinnedMeshBody.h`

### SkinnedMeshShape

Base class for collision shapes attached to skinned meshes.

```cpp
// hdtSkinnedMeshShape.h
class SkinnedMeshShape : public RefObject
{
public:
    virtual void internalUpdate() = 0;     // Update AABBs

    // BVH tree for spatial acceleration
    CubeTree m_tree;

    // Collider data
    std::vector<Collider> m_colliders;

    // Shape type queries
    virtual PerVertexShape* asPerVertexShape() { return nullptr; }
    virtual PerTriangleShape* asPerTriangleShape() { return nullptr; }
};

class PerVertexShape : public SkinnedMeshShape
{
    // Sphere colliders centered on vertices
    void internalUpdate() override;
};

class PerTriangleShape : public SkinnedMeshShape
{
    // Triangle colliders for mesh surfaces
    PerVertexShape* m_verticesCollision;   // For vertex-vertex tests
    void internalUpdate() override;
};
```

**Location**: `hdtSkinnedMeshShape.h`

### CollisionDispatcher

Custom collision dispatcher with parallel processing support.

```cpp
// hdtDispatcher.h
class CollisionDispatcher : public btCollisionDispatcherMt
{
public:
    void dispatchAllCollisionPairs(
        btOverlappingPairCache* pairCache,
        const btDispatcherInfo& dispatchInfo,
        btDispatcher* dispatcher) override;

    void clearAllManifold();
    void clearCollisionState();            // Reset for load/save

    bool needsCollision(const btCollisionObject* body0,
                       const btCollisionObject* body1);

private:
    SpinLock m_lock;                       // Protects manifold pool

    // Working data (cleared each frame)
    std::vector<std::pair<SkinnedMeshBody*, SkinnedMeshBody*>> m_pairs;
};
```

**Location**: `hdtDispatcher.h`

## Memory Layout

### Vertex Data

```
┌─────────────────────────────────────────────────────────────────────┐
│                    VERTEX MEMORY LAYOUT                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   struct Vertex (per-vertex input):                                 │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ float x, y, z          │ 12 bytes │ Local position          │   │
│   │ uint8_t boneIndices[4] │  4 bytes │ Up to 4 bone influences │   │
│   │ float boneWeights[4]   │ 16 bytes │ Weight per bone         │   │
│   │ float margin           │  4 bytes │ Collision margin        │   │
│   └─────────────────────────────────────────────────────────────┘   │
│   Total: 36 bytes per vertex                                        │
│                                                                     │
│   struct VertexPos (computed output):                               │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ __m128 position        │ 16 bytes │ World position (SIMD)   │   │
│   └─────────────────────────────────────────────────────────────┘   │
│   Total: 16 bytes per vertex                                        │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Collider Data

```
┌─────────────────────────────────────────────────────────────────────┐
│                    COLLIDER MEMORY LAYOUT                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   struct Collider (per-collider):                                   │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ Type type              │  4 bytes │ Sphere/Capsule/Triangle │   │
│   │ uint32_t indices[4]    │ 16 bytes │ Vertex indices          │   │
│   │ float margin           │  4 bytes │ Collision margin        │   │
│   │ void* userData         │  8 bytes │ Additional data ptr     │   │
│   └─────────────────────────────────────────────────────────────┘   │
│   Total: 32 bytes per collider                                      │
│                                                                     │
│   struct Aabb (bounding box):                                       │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ __m128 m_min           │ 16 bytes │ Min corner (SIMD)       │   │
│   │ __m128 m_max           │ 16 bytes │ Max corner (SIMD)       │   │
│   └─────────────────────────────────────────────────────────────┘   │
│   Total: 32 bytes per AABB                                          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### BVH Tree Node

```
┌─────────────────────────────────────────────────────────────────────┐
│                    BVH TREE NODE LAYOUT                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   struct CubeTree::Node:                                            │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ Aabb aabb              │ 32 bytes │ Bounding box            │   │
│   │ int32_t left           │  4 bytes │ Left child index        │   │
│   │ int32_t right          │  4 bytes │ Right child index       │   │
│   │ int32_t colliderIdx    │  4 bytes │ Leaf collider index     │   │
│   │ uint32_t flags         │  4 bytes │ Node properties         │   │
│   └─────────────────────────────────────────────────────────────┘   │
│   Total: 48 bytes per node                                          │
│                                                                     │
│   Tree structure (example with 8 leaves):                           │
│                                                                     │
│                    [0] Root                                         │
│                   /         \                                       │
│               [1]            [2]                                    │
│              /    \         /    \                                  │
│           [3]     [4]    [5]     [6]                                │
│          /  \    /  \   /  \    /  \                                │
│        L0  L1  L2  L3 L4  L5  L6  L7                                │
│                                                                     │
│   Leaf nodes: colliderIdx >= 0                                      │
│   Internal nodes: left, right >= 0, colliderIdx = -1                │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Object Ownership

```
┌─────────────────────────────────────────────────────────────────────┐
│                    OWNERSHIP HIERARCHY                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   SkyrimPhysicsWorld (singleton)                                    │
│   └──► owns: m_systems (vector<Ref<SkinnedMeshSystem>>)             │
│                                                                     │
│   SkinnedMeshSystem (reference counted)                             │
│   ├──► owns: m_bones (vector<Ref<SkinnedMeshBone>>)                 │
│   ├──► owns: m_meshes (vector<Ref<SkinnedMeshBody>>)                │
│   ├──► owns: m_constraints (vector<Ref<GenericConstraint>>)         │
│   └──► owns: m_constraintGroups (vector<Ref<ConstraintGroup>>)      │
│                                                                     │
│   SkinnedMeshBody (reference counted)                               │
│   ├──► owns: m_shape (Ref<SkinnedMeshShape>)                        │
│   ├──► owns: m_vertices (vector<Vertex>)                            │
│   └──► refs: m_skinnedBones[].ptr → SkinnedMeshBone*                │
│                                                                     │
│   SkinnedMeshShape (reference counted)                              │
│   ├──► owns: m_colliders (vector<Collider>)                         │
│   └──► owns: m_tree (CubeTree)                                      │
│                                                                     │
│   Ref<T> = intrusive reference-counted smart pointer                │
│   RefObject = base class providing reference count                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Thread Safety Annotations

```
┌─────────────────────────────────────────────────────────────────────┐
│                    THREAD SAFETY MATRIX                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Class                    Thread-Safe?    Protected By             │
│   ─────────────────────────────────────────────────────────────────│
│   SkyrimPhysicsWorld       Partial         m_lock for mutations     │
│   SkinnedMeshSystem        No              Caller must lock         │
│   SkinnedMeshBone          No              Caller must lock         │
│   SkinnedMeshBody          No              Caller must lock         │
│   CollisionDispatcher      Partial         m_lock for manifolds     │
│   EnkiTSScheduler          Yes             Internal synchronization │
│   Ref<T>                   Yes             Atomic refcount          │
│                                                                     │
│   Safe to call from any thread:                                     │
│   - SkyrimPhysicsWorld::get()                                       │
│   - EnkiTSScheduler::get()                                          │
│   - Ref<T> copy/assign/destroy                                      │
│                                                                     │
│   Must be called from main thread:                                  │
│   - onEvent(FrameEvent)                                             │
│   - addSkinnedMeshSystem/removeSkinnedMeshSystem                    │
│   - resetSystems                                                    │
│                                                                     │
│   Called from worker threads (during physics step):                 │
│   - internalUpdate() on bodies/shapes                               │
│   - collapseCollideL() tree traversal                               │
│   - constraint solving                                              │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Next Steps

- **[Level 6: Threading Guide](./06-THREADING-EXPERT.md)** - Advanced parallelism patterns
