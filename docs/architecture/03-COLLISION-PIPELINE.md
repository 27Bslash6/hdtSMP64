# Level 3: Collision Detection Pipeline (NOCUDA)

**Audience**: Developers working on collision, optimization, or debugging physics issues
**Prerequisites**: Level 0-2, understanding of spatial data structures

---

## Overview

Collision detection determines which physics bodies are intersecting and generates contact manifolds for the constraint solver. This is the most computationally expensive part of the physics pipeline.

```
┌─────────────────────────────────────────────────────────────────────┐
│                    COLLISION DETECTION PHASES                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   PHASE 1: BROADPHASE                                               │
│   ┌─────────────────────────────────────────────────────────┐       │
│   │ Quick AABB overlap test to find potential pairs          │       │
│   │ Uses: btDbvtBroadphase (dynamic bounding volume tree)   │       │
│   │ Output: Overlapping pair cache                          │       │
│   └─────────────────────────────────────────────────────────┘       │
│                         │                                           │
│                         ▼                                           │
│   PHASE 2: INTERNAL UPDATES                                         │
│   ┌─────────────────────────────────────────────────────────┐       │
│   │ Update vertex positions and AABBs for skinned meshes    │       │
│   │ Runs in PARALLEL via enkiTS                             │       │
│   │ Output: Updated per-vertex AABBs                        │       │
│   └─────────────────────────────────────────────────────────┘       │
│                         │                                           │
│                         ▼                                           │
│   PHASE 3: NARROWPHASE                                              │
│   ┌─────────────────────────────────────────────────────────┐       │
│   │ Precise collision detection between shape pairs          │       │
│   │ Uses: BVH tree traversal + primitive tests              │       │
│   │ Output: Contact manifolds with normals/depths           │       │
│   └─────────────────────────────────────────────────────────┘       │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Detailed Sequence Diagram

```
┌───────────────────┐ ┌───────────────────┐ ┌─────────────────────┐ ┌───────────────────┐
│ btDynamicsWorldMt │ │CollisionDispatcher│ │ SkinnedMeshBody     │ │SkinnedMeshAlgorithm│
└─────────┬─────────┘ └─────────┬─────────┘ └──────────┬──────────┘ └─────────┬─────────┘
          │                     │                      │                      │
          │ performDiscreteCollisionDetection()        │                      │
          │─────────────────────┤                      │                      │
          │                     │                      │                      │
          │                     │ dispatchAllCollisionPairs()                 │
          │                     │───────────────────────┤                     │
          │                     │                      │                      │
          │                     │  ╔════════════════════════════════════════╗ │
          │                     │  ║ PHASE 1: Collect Pairs (PARALLEL)     ║ │
          │                     │  ╚════════════════════════════════════════╝ │
          │                     │                      │                      │
          │                     │  hdt_parallel_for(overlappingPairs)         │
          │                     │  ├─► needsCollision(shape0, shape1)?        │
          │                     │  ├─► isBoundingSphereCollided()?           │
          │                     │  └─► Add to m_pairs (with lock)            │
          │                     │                      │                      │
          │                     │  ╔════════════════════════════════════════╗ │
          │                     │  ║ PHASE 2: Internal Updates (PARALLEL)  ║ │
          │                     │  ╚════════════════════════════════════════╝ │
          │                     │                      │                      │
          │                     │  hdt_parallel_for_each(bodies)              │
          │                     │──────────────────────►│                     │
          │                     │                      │ internalUpdate()     │
          │                     │                      │ (vertex skinning)    │
          │                     │                      │                      │
          │                     │  hdt_parallel_for_each(vertex_shapes)       │
          │                     │──────────────────────►│                     │
          │                     │                      │ internalUpdate()     │
          │                     │                      │ (AABB calculation)   │
          │                     │                      │                      │
          │                     │  hdt_parallel_for_each(triangle_shapes)     │
          │                     │──────────────────────►│                     │
          │                     │                      │ internalUpdate()     │
          │                     │                      │                      │
          │                     │  ╔════════════════════════════════════════╗ │
          │                     │  ║ PHASE 3: Process Collisions (PARALLEL)║ │
          │                     │  ╚════════════════════════════════════════╝ │
          │                     │                      │                      │
          │                     │  hdt_parallel_for_each(m_pairs)             │
          │                     │  ├─► collapseCollideL() (BVH traversal)    │
          │                     │  │                     │                    │
          │                     │  └───────────────────────────────────────────►
          │                     │                      │  processCollision()  │
          │                     │                      │  └─► checkCollide()  │
          │                     │                      │  └─► addResult()     │
          │                     │                      │                      │
          │                     │  m_pairs.clear()     │                      │
          │◄────────────────────│                      │                      │
```

## Phase 1: Pair Collection

The broadphase (Bullet's `btDbvtBroadphase`) maintains an overlapping pair cache. We iterate through these pairs to find which ones need collision detection.

```cpp
// hdtDispatcher.cpp:94-159 (NOCUDA path)
void CollisionDispatcher::dispatchAllCollisionPairs(...)
{
    auto size = pairCache->getNumOverlappingPairs();
    auto pairs = pairCache->getOverlappingPairArrayPtr();

    // Thread-safe containers for collecting work
    SpinLock lock;
    std::unordered_set<SkinnedMeshBody*> bodies;
    std::unordered_set<PerVertexShape*> vertex_shapes;
    std::unordered_set<PerTriangleShape*> triangle_shapes;

    // PARALLEL: Iterate all overlapping pairs
    hdt_parallel_for(0, size, [&](int i) {
        auto& pair = pairs[i];

        auto shape0 = dynamic_cast<SkinnedMeshBody*>(pair.m_pProxy0->m_clientObject);
        auto shape1 = dynamic_cast<SkinnedMeshBody*>(pair.m_pProxy1->m_clientObject);

        if (shape0 || shape1) {
            // Check if collision is needed and bounding spheres overlap
            if (hdt::needsCollision(shape0, shape1) &&
                shape0->isBoundingSphereCollided(shape1))
            {
                HDT_LOCK_GUARD(l, lock);  // Thread-safe insertion

                bodies.insert(shape0);
                bodies.insert(shape1);
                m_pairs.push_back({shape0, shape1});

                // Collect shapes for internal update
                auto a = shape0->m_shape->asPerTriangleShape();
                auto b = shape1->m_shape->asPerTriangleShape();
                if (a) triangle_shapes.insert(a);
                else   vertex_shapes.insert(shape0->m_shape->asPerVertexShape());
                // ... similar for b
            }
        }
    });
}
```

**Location**: `hdtDispatcher.cpp:94`

### Collision Filtering

The `needsCollision` function implements tag-based filtering:

```cpp
// hdtDispatcher.cpp:45-54
bool needsCollision(const SkinnedMeshBody* shape0, const SkinnedMeshBody* shape1)
{
    // Same body can't collide with itself
    if (!shape0 || !shape1 || shape0 == shape1)
        return false;

    // Kinematic bodies don't collide with each other
    if (shape0->m_isKinematic && shape1->m_isKinematic)
        return false;

    // Check tag-based collision rules
    return shape0->canCollideWith(shape1) && shape1->canCollideWith(shape0);
}
```

**Location**: `hdtDispatcher.cpp:45`

## Phase 2: Internal Updates

Before narrowphase, we need to update vertex positions based on bone transforms and recalculate AABBs.

```
┌─────────────────────────────────────────────────────────────────────┐
│                    INTERNAL UPDATE PIPELINE                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   FOR EACH SkinnedMeshBody:                                         │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │  1. Read bone transforms from game skeleton                 │   │
│   │  2. Compute skinning matrix per vertex                      │   │
│   │  3. Transform vertex positions to world space               │   │
│   │  4. Update bounding sphere AABB                             │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   FOR EACH PerVertexShape:                                          │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │  1. For each collider (sphere around vertex):               │   │
│   │     - Compute world position from vertex + offset           │   │
│   │     - Calculate AABB from sphere center + radius            │   │
│   │  2. Update BVH tree leaf AABBs                              │   │
│   │  3. Propagate AABBs up the tree                             │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   FOR EACH PerTriangleShape:                                        │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │  1. For each triangle collider:                             │   │
│   │     - Get world positions of 3 vertices                     │   │
│   │     - Calculate AABB enclosing triangle + margin            │   │
│   │  2. Update BVH tree leaf AABBs                              │   │
│   │  3. Propagate AABBs up the tree                             │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

```cpp
// hdtDispatcher.cpp:349-360 (NOCUDA path)
// Phase 2 execution
hdt_parallel_for_each(bodies.begin(), bodies.end(),
    [](SkinnedMeshBody* shape) { shape->internalUpdate(); });

hdt_parallel_for_each(vertex_shapes.begin(), vertex_shapes.end(),
    [](PerVertexShape* shape) { shape->internalUpdate(); });

hdt_parallel_for_each(triangle_shapes.begin(), triangle_shapes.end(),
    [](PerTriangleShape* shape) { shape->internalUpdate(); });

// Update bullet shape AABBs
for (auto body : bodies) {
    body->m_bulletShape.m_aabb = body->m_shape->m_tree.aabbAll;
}
```

**Location**: `hdtDispatcher.cpp:349`

## Phase 3: Narrowphase Collision

For each pair that passed broadphase, we do precise collision detection using BVH tree traversal.

```cpp
// hdtDispatcher.cpp:361-366 (NOCUDA path)
hdt_parallel_for_each(m_pairs.begin(), m_pairs.end(),
    [&, this](const std::pair<SkinnedMeshBody*, SkinnedMeshBody*>& i) {
        if (i.first->m_shape->m_tree.collapseCollideL(&i.second->m_shape->m_tree))
            SkinnedMeshAlgorithm::processCollision(i.first, i.second, this);
    });
```

**Location**: `hdtDispatcher.cpp:361`

### BVH Tree Traversal

The `collapseCollideL` function uses a stack-based tree traversal:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    BVH TREE TRAVERSAL                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Tree A (left)              Tree B (right)                         │
│        ○                          ○                                 │
│       /│\                        /│\                                │
│      ○ ○ ○                      ○ ○ ○                               │
│     /│ │ │\                    /│ │ │\                              │
│    ● ● ● ● ●                  ● ● ● ● ●                             │
│    (leaves)                   (leaves)                              │
│                                                                     │
│   Algorithm:                                                        │
│   1. Start with (root_A, root_B) on stack                           │
│   2. Pop pair from stack                                            │
│   3. If AABBs don't overlap → skip                                  │
│   4. If both are leaves:                                            │
│      → Add to collision check queue                                 │
│   5. Otherwise:                                                     │
│      → Split larger node                                            │
│      → Push child pairs onto stack                                  │
│   6. Repeat until stack empty                                       │
│                                                                     │
│   Complexity: O(n log n) average, O(n²) worst case                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Collision Check Types

Different shape combinations use different collision algorithms:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    COLLISION CHECK TYPES                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Vertex-Vertex (sphere-sphere):                                    │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │     ○       ○                                               │   │
│   │    ( )     ( )   distance < r1 + r2 → collision             │   │
│   │     ○       ○                                               │   │
│   │   ←r1→   ←r2→                                               │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   Vertex-Triangle (sphere-triangle):                                │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │        ○        Find closest point on triangle              │   │
│   │       ( )       Check if distance < radius                  │   │
│   │      /│\│\                                                  │   │
│   │     ─────▽      Special cases: inside, edge, vertex         │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   Triangle-Triangle:                                                │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │     /\          Check edge-edge intersections               │   │
│   │    /  \         Check vertex-in-triangle                    │   │
│   │   /────\─\      Generate contact manifold                   │   │
│   │      /\   \                                                 │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Contact Manifold Generation

When collision is detected, we generate a contact manifold:

```cpp
// hdtSkinnedMeshAlgorithm.cpp (conceptual)
struct ContactResult {
    btVector3 positionWorldOnA;  // Contact point on shape A
    btVector3 positionWorldOnB;  // Contact point on shape B
    btVector3 normalWorldOnB;    // Contact normal (from A to B)
    btScalar depth;              // Penetration depth
};

// Results are merged into btPersistentManifold
void MergeBuffer::apply(CollisionDispatcher* dispatcher)
{
    for (auto& result : results) {
        btPersistentManifold* manifold = dispatcher->getNewManifold(...);
        manifold->addManifoldPoint(btManifoldPoint(
            result.positionWorldOnA,
            result.positionWorldOnB,
            result.normalWorldOnB,
            result.depth
        ));
    }
}
```

## Performance Characteristics

### Parallelization Points

| Phase | Parallelized? | Granularity |
|-------|---------------|-------------|
| Pair collection | Yes | Per-pair |
| Body internal update | Yes | Per-body |
| Shape internal update | Yes | Per-shape |
| Narrowphase collision | Yes | Per-pair |
| Manifold creation | Sequential (lock) | N/A |

### Bottlenecks

```
┌─────────────────────────────────────────────────────────────────────┐
│                    TYPICAL FRAME BREAKDOWN                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Phase 1: Pair Collection          ████░░░░░░░░░░░░░ ~15%         │
│   Phase 2: Internal Updates         ████████░░░░░░░░░ ~35%         │
│   Phase 3: Narrowphase              ████████████████░ ~50%         │
│                                                                     │
│   Within Narrowphase:                                               │
│   ├── BVH traversal                 ████████░░░░░░░░░ ~40%         │
│   ├── Primitive tests               ████░░░░░░░░░░░░░ ~20%         │
│   └── Manifold creation             ████████░░░░░░░░░ ~40%         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Scaling Behavior

```
Bodies    Pairs     Collision Time    Notes
──────────────────────────────────────────────────────────
10        ~50       0.5ms             Light load
20        ~200      1.5ms             Moderate
50        ~1000     5ms               Heavy (many NPCs)
100       ~4000     15ms+             Extreme (may exceed budget)

Collision grows as O(n²) in worst case (all bodies collide)
But culling keeps it closer to O(n log n) for typical scenes
```

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                    COLLISION DATA FLOW                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   INPUT:                                                            │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ btOverlappingPairCache                                      │   │
│   │ └── Array of (btBroadphaseProxy*, btBroadphaseProxy*)       │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                         │                                           │
│                         ▼                                           │
│   FILTER:                                                           │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ needsCollision() - Tag filtering                            │   │
│   │ isBoundingSphereCollided() - Quick sphere test              │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                         │                                           │
│                         ▼                                           │
│   WORKING SET:                                                      │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ m_pairs: vector<pair<SkinnedMeshBody*, SkinnedMeshBody*>>   │   │
│   │ bodies: set<SkinnedMeshBody*>                               │   │
│   │ vertex_shapes: set<PerVertexShape*>                         │   │
│   │ triangle_shapes: set<PerTriangleShape*>                     │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                         │                                           │
│                         ▼                                           │
│   UPDATE:                                                           │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ body->internalUpdate() - Vertex skinning                    │   │
│   │ shape->internalUpdate() - AABB calculation                  │   │
│   │ tree.propagate() - BVH update                               │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                         │                                           │
│                         ▼                                           │
│   DETECT:                                                           │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ collapseCollideL() - BVH traversal                          │   │
│   │ checkCollide() - Primitive intersection                     │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                         │                                           │
│                         ▼                                           │
│   OUTPUT:                                                           │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ btPersistentManifold array                                  │   │
│   │ └── Contact points, normals, depths                         │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Next Steps

- **[Level 4: Constraint Solving](./04-CONSTRAINT-SOLVING.md)** - How constraints are solved
- **[Level 5: Data Structures](./05-DATA-STRUCTURES.md)** - Key classes and relationships
