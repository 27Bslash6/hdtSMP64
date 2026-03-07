# Phase 6: XPBD GPU Constraint Solver

**Branch:** TBD (future work)
**Status:** ROADMAP - Long-term
**Goal:** Replace Bullet CPU solver with GPU-native XPBD for "infinite" scaling
**Effort:** 8-12 weeks
**Expected Gain:** 100+ actors at 60 FPS, 500+ actors at 30 FPS

---

## Executive Summary

For scaling beyond ~50 actors, the CPU constraint solver (Bullet's Gauss-Seidel) becomes the bottleneck. XPBD (Extended Position-Based Dynamics) is the industry-standard solution for GPU-accelerated cloth/hair physics, used by Unreal Engine Chaos Cloth, Unity Cloth, and PhysX 5.

**This is a major architectural change and should only be undertaken after Phase 3 proves insufficient for scaling requirements.**

---

## Why XPBD?

### Bullet's Limitation

Bullet uses Sequential Impulse / Gauss-Seidel constraint solving:

```
for each iteration:
    for each constraint:  // SEQUENTIAL - can't parallelize!
        solve(constraint)
        update_velocities()
```

This is inherently serial - each constraint solve affects the next. GPU parallelization is impossible without fundamental algorithmic changes.

### XPBD's Advantage

XPBD (Müller et al., 2020) uses position-based constraints with compliance:

```
for each iteration:
    parallel_for each constraint:  // PARALLEL - GPU-native!
        compute_position_correction()

    parallel_for each particle:    // PARALLEL
        apply_corrections()
```

Each constraint computes independently within an iteration. Perfect for GPU.

### Key Properties

| Property | Bullet (Gauss-Seidel) | XPBD |
|----------|----------------------|------|
| Parallelism | Sequential | Massively parallel |
| Convergence | Iteration-dependent | Material-independent |
| Stiffness | Limited by iterations | True compliance |
| GPU suitability | Poor | Excellent |
| Scaling | O(n) serial | O(n/cores) parallel |

---

## Architecture Overview

### Current Pipeline (Bullet)

```
Collision Detection → Manifold Generation → Sequential Solve → Integration
                                                    ↑
                                              CPU bottleneck
```

### Target Pipeline (XPBD)

```
Collision Detection → Constraint Generation → Parallel XPBD → Integration
        ↓                     ↓                    ↓              ↓
       GPU                   GPU                  GPU            GPU
```

### Data Flow

```
Frame N:
  [Bone Transforms]     ← Upload from game (CPU → GPU)
         ↓
  [Vertex Skinning]     ← GPU kernel
         ↓
  [Collision Detection] ← GPU kernel (already implemented)
         ↓
  [Constraint Setup]    ← GPU kernel (new)
         ↓
  [XPBD Iterations]     ← GPU kernel (new, runs N times)
         ↓
  [Position Integration]← GPU kernel (new)
         ↓
  [Final Positions]     ← Download for rendering (GPU → CPU)
```

---

## XPBD Algorithm

### Core Loop

```cpp
// Per-frame XPBD simulation
void xpbd_step(float dt, int iterations) {
    // 1. Predict positions (explicit Euler)
    parallel_for(particles) {
        p.velocity += dt * gravity;
        p.predicted = p.position + dt * p.velocity;
    }

    // 2. Generate collision constraints
    generate_collision_constraints();

    // 3. Iterative constraint projection
    for (int iter = 0; iter < iterations; iter++) {
        // Solve all constraints in parallel
        parallel_for(constraints) {
            solve_constraint(c);
        }
    }

    // 4. Update velocities from position change
    parallel_for(particles) {
        p.velocity = (p.predicted - p.position) / dt;
        p.position = p.predicted;
    }
}
```

### Constraint Types

| Type | Description | Cloth/Hair Use |
|------|-------------|----------------|
| Distance | Maintains edge length | Primary structure |
| Bending | Resists angle change | Cloth stiffness |
| Collision | Prevents penetration | Character collision |
| Attachment | Pins to bone | Root attachment |
| Volume | Preserves volume | Hair thickness |

### Compliance Parameter

XPBD's key innovation - stiffness is material property, not iteration artifact:

```cpp
// Compliance α = 1/stiffness (inverse stiffness)
// Lower α = stiffer constraint

float alpha = compliance / (dt * dt);  // Time-scaled compliance
float delta_lambda = (-C - alpha * lambda) / (w + alpha);
```

This means:
- Same material behavior regardless of iteration count
- More iterations = more accuracy, not more stiffness
- Physical units (Pa⁻¹) instead of magic numbers

---

## Implementation Plan

### Task 1: Particle System (2 weeks)
**New file:** `hdtXPBDParticle.h/cpp/cu`

Create GPU-resident particle representation:

```cpp
struct XPBDParticle {
    float3 position;
    float3 predicted;
    float3 velocity;
    float invMass;        // 0 = fixed/kinematic
    int constraintCount;  // For averaging corrections
};

class XPBDParticleSystem {
    CudaBuffer<XPBDParticle> m_particles;
    CudaBuffer<float3> m_corrections;  // Accumulated per iteration

    void predict(float dt, cudaStream_t stream);
    void integrate(float dt, cudaStream_t stream);
};
```

### Task 2: Constraint System (3 weeks)
**New file:** `hdtXPBDConstraint.h/cpp/cu`

GPU constraint representation and solving:

```cpp
struct XPBDDistanceConstraint {
    int p0, p1;           // Particle indices
    float restLength;
    float compliance;
};

struct XPBDCollisionConstraint {
    int particle;
    float3 contactPoint;
    float3 contactNormal;
    float penetration;
};

__global__ void solveDistanceConstraints(
    XPBDDistanceConstraint* constraints,
    int count,
    XPBDParticle* particles,
    float dt
);

__global__ void solveCollisionConstraints(
    XPBDCollisionConstraint* constraints,
    int count,
    XPBDParticle* particles,
    float dt
);
```

### Task 3: Graph Coloring (2 weeks)
**Integrated into constraint system**

Constraints sharing particles must be solved in separate groups to avoid race conditions:

```cpp
class ConstraintGraph {
    // Constraints grouped by color (no shared particles within group)
    std::vector<std::vector<int>> m_colorGroups;

    void color();  // Graph coloring algorithm

    void solve(cudaStream_t stream) {
        for (auto& group : m_colorGroups) {
            // All constraints in group can run in parallel
            solveGroup<<<...>>>(group);
            // Sync between groups
            cudaStreamSynchronize(stream);
        }
    }
};
```

### Task 4: Collision Integration (2 weeks)
**Modify:** `hdtCudaCollision.cu`

Convert collision manifolds to XPBD constraints:

```cpp
__global__ void manifoldsToConstraints(
    CollisionManifold* manifolds,
    int count,
    XPBDCollisionConstraint* constraints,
    int* constraintCount
);
```

### Task 5: Material Migration (1 week)
**Modify:** `hdtSkinnedMeshSystem.cpp`, XML parsing

Map Bullet material parameters to XPBD compliance:

```cpp
// Bullet: stiffness (arbitrary units, iteration-dependent)
// XPBD: compliance (inverse stiffness, physical units)

float bulletToXPBDCompliance(float bulletStiffness, int iterations) {
    // Empirical mapping - tune for visual equivalence
    return 1.0f / (bulletStiffness * iterations * iterations);
}
```

### Task 6: Fallback Path (1 week)
**Modify:** `config.cpp`, `hdtSkinnedMeshWorld.cpp`

Keep Bullet solver as fallback:

```xml
<config>
    <solver type="xpbd" iterations="10" />  <!-- or "bullet" -->
</config>
```

```cpp
if (config.solverType == "xpbd") {
    m_solver = std::make_unique<XPBDSolver>();
} else {
    m_solver = std::make_unique<BulletSolver>();
}
```

---

## Performance Expectations

### Constraint Scaling

| Actors | Constraints | Bullet (CPU) | XPBD (GPU) |
|--------|-------------|--------------|------------|
| 10 | ~5K | 2.8ms | 0.3ms |
| 50 | ~25K | 14ms | 0.8ms |
| 100 | ~50K | 28ms | 1.5ms |
| 500 | ~250K | 140ms | 5ms |

### Memory Requirements

| Component | Per-Actor | 100 Actors |
|-----------|-----------|------------|
| Particles | ~100KB | 10MB |
| Constraints | ~200KB | 20MB |
| Graph coloring | ~50KB | 5MB |
| **Total GPU** | ~350KB | **35MB** |

### Iteration Tuning

| Iterations | Quality | Cost | Use Case |
|------------|---------|------|----------|
| 4 | Low | 0.5x | Distant actors |
| 8 | Medium | 1x | Normal |
| 16 | High | 2x | Close-up |
| 32 | Ultra | 4x | Cinematics |

---

## Risk Analysis

### Risk 1: Visual Behavior Change
**Probability:** High
**Impact:** Medium
**Mitigation:**
- Extensive A/B comparison testing
- Tune compliance parameters to match Bullet behavior
- Keep Bullet fallback for mod compatibility

### Risk 2: Constraint Graph Coloring Overhead
**Probability:** Medium
**Impact:** Medium
**Mitigation:**
- Pre-compute graph coloring (static constraints)
- Dynamic constraints use conservative grouping
- Profile color group count vs parallelism

### Risk 3: Numerical Stability
**Probability:** Low
**Impact:** High
**Mitigation:**
- Use double precision for compliance accumulation
- Clamp position corrections
- Add position damping for stability

### Risk 4: Memory Pressure
**Probability:** Medium
**Impact:** Medium
**Mitigation:**
- Pool constraint buffers
- Stream constraint generation
- LOD reduces constraint count for distant actors

---

## Dependencies

### Prerequisites
- **Phase 3 complete:** Double-buffer sync elimination
- **Profiling:** Confirm Bullet solver is the bottleneck
- **Scaling test:** Verify 50+ actors needed

### External References
- [XPBD Paper (Müller 2016)](https://matthias-research.github.io/pages/publications/XPBD.pdf)
- [Small Steps in Physics Simulation (Müller 2019)](https://matthias-research.github.io/pages/publications/smallsteps.pdf)
- [Parallel XPBD (Fratarcangeli 2016)](https://www.cs.utah.edu/~ladislav/fratarcangeli16parallel/fratarcangeli16parallel.html)

---

## Success Criteria

### Performance
- [ ] 100 actors under 16ms (60 FPS viable)
- [ ] 500 actors under 33ms (30 FPS viable)
- [ ] Linear scaling with actor count

### Quality
- [ ] Visual parity with Bullet solver (A/B test)
- [ ] No new stability issues (no explosions)
- [ ] Material parameters map cleanly

### Maintainability
- [ ] Clean solver abstraction (swappable)
- [ ] Bullet fallback preserved
- [ ] Tracy instrumentation complete

---

## Decision Gate

**Do not start Phase 6 until:**

1. Phase 3 is complete and validated
2. Profiling shows Bullet solver is >30% of frame time
3. User requirement confirms 100+ actor scaling needed
4. Team has bandwidth for 8-12 week effort

Phase 3's 50-actor capacity may be sufficient for most use cases. XPBD is the "infinity" solution, not the default path.

---

## Future Extensions (Phase 7+)

### Multigrid XPBD (MGPBD)
- Hierarchical constraint solving
- Better global convergence
- Required for very large cloth sheets

### Substepping
- Multiple physics steps per render frame
- Better stability for stiff materials
- Trade performance for quality

### GPU-Resident Everything
- Bone transforms computed on GPU
- Only download final render positions
- Eliminates all PCIe except upload
