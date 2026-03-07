# hdtSMP64 Ecosystem Libraries & Optimization Techniques

**Analysis Date:** 2026-01-07
**Purpose:** Identify external libraries, tools, and techniques to improve hdtSMP64 performance

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Physics Engine Alternatives](#physics-engine-alternatives)
3. [Algorithm Modernization (XPBD)](#algorithm-modernization-xpbd)
4. [SIMD Optimization Libraries](#simd-optimization-libraries)
5. [Parallel Runtime Alternatives](#parallel-runtime-alternatives)
6. [GPU Compute Options](#gpu-compute-options)
7. [Memory Allocators](#memory-allocators)
8. [Profiling Tools](#profiling-tools)
9. [Math Libraries](#math-libraries)
10. [Data-Oriented Design (ECS)](#data-oriented-design-ecs)
11. [Implementation Recommendations](#implementation-recommendations)

---

## Executive Summary

### Top Recommendations by Impact

| Priority | Recommendation | Expected Impact | Effort |
|----------|---------------|-----------------|--------|
| 1 | **Google Highway SIMD** | 20-40% vertex skinning speedup | Medium |
| 2 | **Tracy Profiler** | Identify actual bottlenecks | Low |
| 3 | **mimalloc allocator** | 10-20% allocation speedup | Low |
| 4 | **XPBD algorithm** | Better stability, fewer iterations | High |
| 5 | **oneTBB scheduler** | Better multi-core scaling | Medium |
| 6 | **Eigen for math** | Faster matrix operations | Medium |

### Quick Wins vs Strategic Changes

```
┌─────────────────────────────────────────────────────────────────────┐
│                    EFFORT vs IMPACT MATRIX                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  HIGH    │ XPBD Algorithm    │ PhysX 5 Migration │                 │
│  IMPACT  │ Highway SIMD      │ Jolt Physics      │                 │
│          │ oneTBB            │                   │                 │
│          ├───────────────────┼───────────────────┤                 │
│          │ Tracy Profiler    │ Eigen Math        │                 │
│  LOW     │ mimalloc          │ ECS Refactor      │                 │
│  IMPACT  │                   │                   │                 │
│          └───────────────────┴───────────────────┘                 │
│               LOW EFFORT          HIGH EFFORT                       │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Physics Engine Alternatives

### Option 1: NVIDIA PhysX 5

**Status:** Production-ready, actively maintained
**License:** BSD 3-Clause (open source since 2018)

#### Advantages
- Native GPU acceleration via CUDA (same as current hdtSMP64)
- **FEM-based soft body** and **Position-Based Dynamics (PBD)** for cloth
- Former NVIDIA Flex library integrated (cloth, inflatables, fluids)
- Battle-tested in AAA games
- Excellent Skyrim engine compatibility (same CUDA dependency)

#### Cloth-Specific Features
```
PhysX 5.5 Cloth Features:
├── GPU-exclusive soft body simulation
├── Signed Distance Field (SDF) collisions
├── FEM cloth elements (experimental)
├── Self-collision handling
└── Configurable CPU/GPU execution paths
```

#### Integration Complexity
- **High** - Would require significant rewrite of constraint system
- Bullet Physics API is quite different from PhysX
- May offer better out-of-box cloth simulation quality

#### Recommendation
**Not recommended for hdtSMP64** - Too invasive. Better to optimize existing Bullet-based system.

---

### Option 2: Jolt Physics

**Status:** Production-ready (Horizon Forbidden West, other AAA titles)
**License:** MIT

#### Performance vs Bullet
| Benchmark | Jolt | Bullet | Improvement |
|-----------|------|--------|-------------|
| Cube stress test (rigidbodies at 60fps) | ~3100 | ~900-950 | **3.3x** |
| General throughput | Baseline | -131% | Jolt wins |
| Mobile optimization | Excellent | Good | Jolt wins |

#### Critical Limitation
**No soft body support** - Jolt excels at rigid bodies but does not implement cloth/soft body simulation. This makes it **unsuitable for hdtSMP64's core use case**.

#### Recommendation
**Not recommended** - Lacks soft body features essential for cloth/hair physics.

---

### Option 3: Stay with Bullet (Recommended)

**Rationale:**
- Mature soft body implementation already integrated
- Known performance characteristics
- Optimization effort better spent on parallel/SIMD improvements
- Community familiarity with Bullet-based physics XMLs

**Optimization Path:**
Focus on algorithmic improvements (XPBD) and infrastructure optimizations (SIMD, threading, allocators) rather than engine replacement.

---

## Algorithm Modernization (XPBD)

### Current: Position-Based Dynamics (PBD)

hdtSMP64 currently uses traditional constraint-based physics via Bullet's sequential impulse solver.

### Proposed: Extended Position-Based Dynamics (XPBD)

**Paper:** Macklin et al., 2016 - "XPBD: Position-Based Simulation of Compliant Constrained Dynamics"

#### Why XPBD?

| Property | Traditional PBD | XPBD |
|----------|----------------|------|
| Iteration dependence | Stiffness varies with iterations | **Independent** |
| Time-step stability | Sensitive | **Robust** |
| Compliance control | Difficult | **Direct physical parameters** |
| Convergence | Slow for stiff constraints | **Faster** |

#### XPBD Algorithm Core

```cpp
// XPBD Simulation Loop (pseudocode)
void xpbd_step(float dt, int iterations) {
    // 1. Predict positions
    for (auto& particle : particles) {
        particle.velocity += dt * gravity;
        particle.predicted = particle.position + dt * particle.velocity;
    }

    // 2. Initialize Lagrange multipliers
    for (auto& constraint : constraints) {
        constraint.lambda = 0.0f;
    }

    // 3. Iterative constraint solving
    for (int iter = 0; iter < iterations; iter++) {
        for (auto& c : constraints) {
            float C = c.evaluate(particles);           // Constraint value
            vec3 gradC = c.gradient(particles);        // Constraint gradient

            // XPBD compliance term (α = 1/(stiffness * dt²))
            float alpha = c.compliance / (dt * dt);

            // Lagrange multiplier update
            float denom = dot(gradC, invMass * gradC) + alpha;
            float dlambda = (-C - alpha * c.lambda) / denom;
            c.lambda += dlambda;

            // Position correction
            for (auto& p : c.particles) {
                p.predicted += p.invMass * dlambda * gradC;
            }
        }
    }

    // 4. Update velocities and positions
    for (auto& particle : particles) {
        particle.velocity = (particle.predicted - particle.position) / dt;
        particle.position = particle.predicted;
    }
}
```

#### Implementation Resources

| Resource | Type | URL |
|----------|------|-----|
| Original Paper | PDF | http://mmacklin.com/xpbd.pdf |
| C++/OpenGL Implementation | GitHub | https://github.com/frederic-hallein/xpbd-softbody-implementation |
| PositionBasedDynamics Library | GitHub | https://github.com/InteractiveComputerGraphics/PositionBasedDynamics |
| Tutorial | Blog | https://carmencincotti.com/2022-08-08/xpbd-extended-position-based-dynamics/ |

#### Migration Strategy

```
Phase 1: Prototype XPBD solver alongside existing Bullet solver
Phase 2: Validate cloth behavior matches or exceeds current quality
Phase 3: Gradual migration of constraint types
Phase 4: Remove Bullet dependency for soft body (keep for collision)
```

#### Expected Benefits
- **50% fewer iterations** for equivalent stiffness
- **Better stability** at variable frame rates
- **Physically meaningful parameters** (compliance instead of iteration count)
- **GPU-friendly** - constraint solving is embarrassingly parallel

---

## SIMD Optimization Libraries

### Current State

hdtSMP64 uses manual SSE/AVX intrinsics:
```cpp
// Current: Manual intrinsics (hdtSkinnedMeshBody.cpp)
auto p = v.m_skinPos.get128();
auto w = _mm_load_ps(v.m_weight);
auto flg = _mm_movemask_ps(_mm_cmplt_ps(_mm_set_ps1(FLT_EPSILON), w));
auto posMargin = calcVertexState(p, m_bones[v.getBoneIdx(0)], setAll0(w));
```

### Recommended: Google Highway

**Repository:** https://github.com/google/highway
**License:** Apache 2.0

#### Advantages over Manual Intrinsics

| Aspect | Manual SSE/AVX | Google Highway |
|--------|---------------|----------------|
| Portability | Per-ISA code paths | Single source, all targets |
| Runtime dispatch | Manual implementation | Built-in |
| Code maintenance | High (duplicated logic) | Low (single implementation) |
| Performance | Baseline | **Matches or exceeds** |
| AVX-512 support | Manual, complex | Automatic |
| Future ISA (SVE, etc.) | Requires rewrite | Automatic |

#### Highway Example (Vertex Skinning)

```cpp
#include "hwy/highway.h"

namespace hn = hwy::HWY_NAMESPACE;

void skinVertices_highway(
    const Vertex* vertices,
    const Bone* bones,
    VertexPos* output,
    size_t count)
{
    const hn::ScalableTag<float> d;
    const size_t N = hn::Lanes(d);

    size_t i = 0;
    for (; i + N <= count; i += N) {
        // Load N vertices at once
        auto skinPos_x = hn::LoadU(d, &vertices[i].m_skinPos.x);
        auto skinPos_y = hn::LoadU(d, &vertices[i].m_skinPos.y);
        auto skinPos_z = hn::LoadU(d, &vertices[i].m_skinPos.z);
        auto weights = hn::LoadU(d, &vertices[i].m_weight[0]);

        // Transform by bone matrices...
        // (vectorized across N vertices simultaneously)

        hn::StoreU(result_x, d, &output[i].x);
        hn::StoreU(result_y, d, &output[i].y);
        hn::StoreU(result_z, d, &output[i].z);
    }

    // Handle remainder with scalar code
    for (; i < count; ++i) { /* ... */ }
}
```

#### Integration Steps

1. Add Highway via vcpkg: `vcpkg install highway`
2. Replace hot path intrinsics with Highway equivalents
3. Enable runtime dispatch for optimal ISA selection
4. Benchmark against baseline

#### Alternative: xsimd

**Repository:** https://github.com/xtensor-stack/xsimd
**Pros:** Header-only, similar abstraction level
**Cons:** Less active development, smaller community

---

## Parallel Runtime Alternatives

### Current: Microsoft PPL (Parallel Patterns Library)

```cpp
// Current usage
#include <ppl.h>
concurrency::parallel_for_each(m_tasks.begin(), m_tasks.end(), ...);
```

### Alternative: Intel oneTBB

**Repository:** https://github.com/oneapi-src/oneTBB
**License:** Apache 2.0

#### Why Consider oneTBB?

| Feature | PPL | oneTBB |
|---------|-----|--------|
| Platform | Windows only | Cross-platform |
| NUMA awareness | Basic | **Advanced** |
| Work stealing | Yes | **Optimized** |
| High core count (32+) | Degrades | **Scales well** |
| Task arena control | Limited | **Fine-grained** |
| Flow graphs | No | **Yes** |

#### oneTBB Advantages for Physics

1. **Better scaling** on high-core-count CPUs (Ryzen 9, Threadripper)
2. **Task arenas** for isolating physics from game threads
3. **Flow graphs** for expressing constraint dependencies
4. **Parallel reduce** optimized for reductions (COM calculation)

#### Migration Example

```cpp
// PPL (current)
concurrency::parallel_for(0, size, [&](int i) {
    // work
});

// oneTBB (proposed)
#include <tbb/parallel_for.h>
tbb::parallel_for(tbb::blocked_range<int>(0, size),
    [&](const tbb::blocked_range<int>& r) {
        for (int i = r.begin(); i < r.end(); ++i) {
            // work
        }
    });
```

#### Flow Graph for Physics Pipeline

```cpp
#include <tbb/flow_graph.h>

tbb::flow::graph g;

// Define physics pipeline as flow graph
tbb::flow::function_node<void, void> read_transforms(g, 1, read_transform_func);
tbb::flow::function_node<void, void> collision_detect(g, tbb::flow::unlimited, collision_func);
tbb::flow::function_node<void, void> solve_constraints(g, 1, solve_func);
tbb::flow::function_node<void, void> integrate(g, tbb::flow::unlimited, integrate_func);

// Connect nodes
tbb::flow::make_edge(read_transforms, collision_detect);
tbb::flow::make_edge(collision_detect, solve_constraints);
tbb::flow::make_edge(solve_constraints, integrate);
```

#### Recommendation

**Medium priority** - PPL is adequate for current workloads. Consider oneTBB if:
- Targeting high-core-count systems (16+ cores)
- Need cross-platform support (Linux builds)
- Want flow graph for complex dependency management

---

## GPU Compute Options

### Current: CUDA

hdtSMP64 already uses CUDA effectively for:
- Vertex skinning (`kernelBodyUpdate`)
- AABB computation (`kernelPerVertexUpdate`, `kernelPerTriangleUpdate`)
- Collision detection (`kernelCollision`)

### CUDA vs Alternatives

| Technology | Performance | Portability | Recommendation |
|-----------|-------------|-------------|----------------|
| **CUDA** | Best on NVIDIA | NVIDIA only | **Keep using** |
| DirectCompute | Good | Windows + all GPUs | Not worth switching |
| Vulkan Compute | Good | Cross-platform | Future consideration |
| OpenCL | Moderate | All platforms | Declining support |

### CUDA Optimization Opportunities

#### 1. CUDA Graphs

Reduce kernel launch overhead by capturing command sequences:

```cpp
cudaGraph_t graph;
cudaGraphExec_t graphExec;

// Capture kernel sequence
cudaStreamBeginCapture(stream, cudaStreamCaptureModeGlobal);
    kernelBodyUpdate<<<grid1, block1, 0, stream>>>(...);
    kernelPerVertexUpdate<<<grid2, block2, 0, stream>>>(...);
    kernelBoundingBoxReduce<<<grid3, block3, 0, stream>>>(...);
cudaStreamEndCapture(stream, &graph);

cudaGraphInstantiate(&graphExec, graph, nullptr, nullptr, 0);

// Execute captured graph (much lower overhead)
cudaGraphLaunch(graphExec, stream);
```

**Expected improvement:** 10-30% reduction in GPU overhead for small workloads

#### 2. Cooperative Groups

Better synchronization primitives for constraint solving:

```cpp
#include <cooperative_groups.h>
namespace cg = cooperative_groups;

__global__ void solveConstraints(...) {
    auto grid = cg::this_grid();
    auto block = cg::this_thread_block();

    // Iteration with grid-wide sync
    for (int iter = 0; iter < iterations; iter++) {
        // Solve constraints
        solveIteration<<<...>>>();

        // Grid-wide synchronization (requires cooperative launch)
        grid.sync();
    }
}
```

#### 3. Multi-Stream Parallelism

Already identified in OPTIMIZATION_REPORT.md - process multiple bodies concurrently.

---

## Memory Allocators

### Current: Default CRT Allocator

The codebase uses standard `new`/`delete` and Bullet's internal allocators.

### Recommended: mimalloc

**Repository:** https://github.com/microsoft/mimalloc
**License:** MIT

#### Why mimalloc?

| Allocator | Multi-threaded Performance | Memory Overhead | Integration Effort |
|-----------|---------------------------|-----------------|-------------------|
| CRT (default) | Baseline | Baseline | N/A |
| **mimalloc** | **+15-20%** | Slightly higher | **Very low** |
| jemalloc | Variable (+/-) | Lower | Low |
| tcmalloc | Good for large allocs | Higher | Low |

#### Integration (Global Override)

```cpp
// Option 1: Link mimalloc-override.lib (Windows)
// Automatically replaces all malloc/free/new/delete

// Option 2: Explicit initialization
#include <mimalloc.h>

// In DllMain or plugin init:
mi_option_set(mi_option_eager_commit, 1);  // Good for physics
mi_option_set(mi_option_large_os_pages, 1); // If available
```

#### Bullet-Specific Integration

```cpp
// Custom Bullet allocator using mimalloc
#include <mimalloc.h>

void* btMimallocAlloc(size_t size) {
    return mi_malloc(size);
}

void btMimallocFree(void* ptr) {
    mi_free(ptr);
}

// Register with Bullet
btAlignedAllocSetCustom(btMimallocAlloc, btMimallocFree);
```

#### Expected Impact
- **10-20% faster** allocation-heavy operations
- Reduced fragmentation over long play sessions
- Better cache locality for small objects

---

## Profiling Tools

### Recommended: Tracy Profiler

**Repository:** https://github.com/wolfpld/tracy
**License:** BSD 3-Clause

#### Why Tracy?

| Feature | Tracy | VTune | Superluminal |
|---------|-------|-------|--------------|
| Cost | **Free** | Free/Pro | Paid |
| Real-time view | **Yes** | No | Yes |
| Frame analysis | **Excellent** | Basic | Good |
| GPU profiling | **D3D11/12/Vulkan** | Intel GPUs | Partial |
| Memory tracking | **Yes** | Yes | Limited |
| Lock contention | **Yes** | Yes | Yes |
| Integration effort | **Low** | Medium | Low |

#### Integration

```cpp
#include <tracy/Tracy.hpp>

// Mark zones (hot functions)
void SkinnedMeshBody::internalUpdate() {
    ZoneScoped;  // Automatic function naming

    // Or with custom name
    ZoneScopedN("VertexSkinning");

    for (int idx = 0; idx < size; ++idx) {
        ZoneScopedN("SingleVertex");
        // ...
    }
}

// Track frame boundaries
void SkyrimPhysicsWorld::doUpdate() {
    FrameMark;  // Mark frame end

    ZoneScoped;
    // ...
}

// Memory tracking
void* ptr = mi_malloc(size);
TracyAlloc(ptr, size);
// ...
TracyFree(ptr);
```

#### CUDA Integration

```cpp
#include <tracy/TracyC.h>

// Create GPU context
TracyCuContext cuCtx;
TracyCuContextCreate(&cuCtx, cudaStream);

// Mark GPU zones
{
    TracyCuZone(cuCtx, "KernelBodyUpdate");
    kernelBodyUpdate<<<grid, block, 0, stream>>>(...);
}
```

#### Setup Steps

1. Clone Tracy: `git clone https://github.com/wolfpld/tracy`
2. Build profiler GUI: `cmake --build tracy/profiler`
3. Add Tracy to hdtSMP64:
   - Include `tracy/public/tracy/Tracy.hpp`
   - Link `TracyClient.cpp` or use header-only mode
   - Define `TRACY_ENABLE` for release profiling builds
4. Run profiler, connect to game

---

## Math Libraries

### Current: Bullet's btVector3/btMatrix

Custom SSE-optimized types tightly coupled to Bullet.

### Alternative: Eigen

**Repository:** https://gitlab.com/libeigen/eigen
**License:** MPL2/BSD

#### Performance Comparison

| Operation | btVector3/btMatrix | Eigen (Fixed) | Improvement |
|-----------|-------------------|---------------|-------------|
| 4x4 * vec4 | Good (SSE) | **Better (auto-vectorize)** | ~10-20% |
| 3x3 * vec3 | Good | Similar | - |
| Batch operations | Manual SIMD | **Auto-optimized** | Varies |

#### When to Use Eigen

**Good candidates:**
- Vertex skinning (batch matrix transforms)
- Bone hierarchy computations
- Any non-Bullet math code

**Keep btVector3/btMatrix for:**
- Direct Bullet API interactions
- Constraint solver internals

#### Integration Pattern

```cpp
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Convert Bullet <-> Eigen at boundaries
inline Eigen::Vector3f toEigen(const btVector3& v) {
    return Eigen::Vector3f(v.x(), v.y(), v.z());
}

inline btVector3 toBullet(const Eigen::Vector3f& v) {
    return btVector3(v.x(), v.y(), v.z());
}

// Use Eigen for batch operations
void transformVerticesBatch(
    const Eigen::Matrix4f* boneMatrices,
    const Eigen::Vector4f* vertices,
    Eigen::Vector4f* output,
    size_t count)
{
    // Eigen auto-vectorizes this
    for (size_t i = 0; i < count; ++i) {
        output[i] = boneMatrices[boneIdx[i]] * vertices[i];
    }
}
```

---

## Data-Oriented Design (ECS)

### Concept

Entity Component System (ECS) architectures improve cache utilization by storing components contiguously rather than in object hierarchies.

### Recommended: EnTT

**Repository:** https://github.com/skypjack/entt
**License:** MIT

#### ECS for Physics (Conceptual)

```cpp
#include <entt/entt.hpp>

// Components
struct Position { float x, y, z; };
struct Velocity { float x, y, z; };
struct BoneWeight { float weights[4]; int boneIndices[4]; };
struct SkinnedVertex { /* ... */ };

// System: Vertex Skinning
void skinningSystem(entt::registry& registry, const BoneMatrices& bones) {
    auto view = registry.view<SkinnedVertex, BoneWeight, Position>();

    // Process all skinned vertices (cache-friendly iteration)
    view.each([&](auto& vertex, auto& weights, auto& pos) {
        // Transform vertex by weighted bone matrices
        pos = transformByBones(vertex, weights, bones);
    });
}

// System: Integration
void integrationSystem(entt::registry& registry, float dt) {
    auto view = registry.view<Position, Velocity>();

    // SIMD-friendly contiguous iteration
    view.each([dt](auto& pos, auto& vel) {
        pos.x += vel.x * dt;
        pos.y += vel.y * dt;
        pos.z += vel.z * dt;
    });
}
```

#### Applicability to hdtSMP64

**Caution:** Full ECS refactor would be a **major architectural change**.

**Partial adoption opportunities:**
- Vertex data storage (SoA layout without full ECS)
- Bone transform caching
- Constraint organization

#### Recommendation

**Low priority** for hdtSMP64 - The performance gains don't justify the refactoring effort for a mature codebase. Focus on SIMD and parallelization improvements instead.

---

## Implementation Recommendations

### Phase 1: Profiling & Quick Wins (1 week)

```
1. Integrate Tracy profiler
   └── Identify actual vs. assumed bottlenecks

2. Add mimalloc
   └── Global override, minimal code changes

3. Enable /O2 /GL /LTCG optimizations
   └── Verify not already enabled
```

### Phase 2: SIMD Modernization (2-3 weeks)

```
1. Prototype Highway for vertex skinning
   ├── Benchmark against current SSE code
   └── If beneficial, migrate hot paths

2. Evaluate Eigen for matrix operations
   └── Benchmark batch transforms
```

### Phase 3: Threading Improvements (2 weeks)

```
1. Parallelize identified sequential loops
   └── (See OPTIMIZATION_REPORT.md)

2. Evaluate oneTBB
   ├── If targeting high-core systems
   └── Or need better NUMA handling
```

### Phase 4: Algorithmic Improvements (4-6 weeks)

```
1. Prototype XPBD constraint solver
   ├── Start with distance constraints
   ├── Add bending constraints
   └── Benchmark iterations vs quality

2. If XPBD successful:
   ├── GPU implementation
   └── Gradual migration
```

### Decision Tree

```
                    ┌─────────────────────┐
                    │ Performance issue?  │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │ Profile with Tracy  │
                    └──────────┬──────────┘
                               │
              ┌────────────────┼────────────────┐
              │                │                │
    ┌─────────▼────────┐ ┌────▼─────┐ ┌───────▼────────┐
    │ CPU-bound?       │ │GPU-bound?│ │ Memory-bound?  │
    └─────────┬────────┘ └────┬─────┘ └───────┬────────┘
              │               │               │
    ┌─────────▼────────┐     │      ┌────────▼────────┐
    │ Try:             │     │      │ Try:            │
    │ - Highway SIMD   │     │      │ - mimalloc      │
    │ - Parallelization│     │      │ - SoA layout    │
    │ - oneTBB         │     │      │ - Prefetching   │
    └──────────────────┘     │      └─────────────────┘
                             │
              ┌──────────────▼─────────────┐
              │ Try:                       │
              │ - CUDA Graphs              │
              │ - Multi-stream             │
              │ - Kernel fusion            │
              └────────────────────────────┘
```

---

## Summary Table

| Library/Tool | Purpose | Impact | Effort | Priority |
|-------------|---------|--------|--------|----------|
| **Tracy** | Profiling | Essential | Low | **P0** |
| **mimalloc** | Allocation | 10-20% | Low | **P1** |
| **Highway** | SIMD abstraction | 20-40% | Medium | **P1** |
| **oneTBB** | Threading | 10-30% | Medium | P2 |
| **Eigen** | Math operations | 10-20% | Medium | P2 |
| **XPBD** | Algorithm | 30-50% | High | P2 |
| **CUDA Graphs** | GPU overhead | 10-30% | Medium | P2 |
| PhysX 5 | Engine replacement | ? | Very High | Not recommended |
| Jolt | Engine replacement | N/A | High | Not recommended |
| EnTT/ECS | Architecture | ? | Very High | Not recommended |

---

## References

1. Google Highway: https://github.com/google/highway
2. Intel oneTBB: https://github.com/oneapi-src/oneTBB
3. Tracy Profiler: https://github.com/wolfpld/tracy
4. mimalloc: https://github.com/microsoft/mimalloc
5. Eigen: https://eigen.tuxfamily.org
6. XPBD Paper: http://mmacklin.com/xpbd.pdf
7. PositionBasedDynamics: https://github.com/InteractiveComputerGraphics/PositionBasedDynamics
8. EnTT: https://github.com/skypjack/entt
9. PhysX 5: https://github.com/NVIDIA-Omniverse/PhysX
10. Jolt Physics: https://github.com/jrouwe/JoltPhysics

---

*Report generated from ecosystem research and performance analysis*
