# Level 4: Constraint Solving Deep Dive

**Audience**: Advanced developers, performance optimization, physics debugging
**Prerequisites**: Level 0-3, understanding of linear algebra and physics

---

## Overview

After collision detection generates contact manifolds, the constraint solver adjusts body positions and velocities to satisfy all constraints (joints, contacts, springs) simultaneously.

```
┌─────────────────────────────────────────────────────────────────────┐
│                    CONSTRAINT SOLVING PIPELINE                      │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   INPUT: Contact manifolds + Joint constraints                      │
│                         │                                           │
│                         ▼                                           │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │  PHASE 1: Setup                                             │   │
│   │  - Convert constraints to solver format                     │   │
│   │  - Build constraint rows (Jacobians)                        │   │
│   │  - Prepare solver bodies                                    │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                         │                                           │
│                         ▼                                           │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │  PHASE 2: Iterative Solving                                 │   │
│   │  - For each iteration (default 10):                         │   │
│   │    - For each constraint row:                               │   │
│   │      - Apply impulse to satisfy constraint                  │   │
│   │      - Clamp impulse to limits                              │   │
│   │  - Group constraints solved together (MLCP optional)        │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                         │                                           │
│                         ▼                                           │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │  PHASE 3: Finish                                            │   │
│   │  - Write solved velocities back to bodies                   │   │
│   │  - Clear temporary solver data                              │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                         │                                           │
│                         ▼                                           │
│   OUTPUT: Updated body velocities                                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Solver Architecture

hdtSMP64 uses a hybrid solver architecture with two levels:

```
┌─────────────────────────────────────────────────────────────────────┐
│                      SOLVER HIERARCHY                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Level 1: Bullet's btConstraintSolverPoolMt                        │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │  - Parallel solver pool (uses enkiTS via btTaskScheduler)   │   │
│   │  - Handles contact constraints from collision detection     │   │
│   │  - Processes simulation islands in parallel                 │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   Level 2: GroupConstraintSolver (hdtSMP64 custom)                  │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │  - Solves constraint groups (joint chains)                  │   │
│   │  - Uses Sequential Impulse (SI) or MLCP                     │   │
│   │  - Scalar constraint resolution (Highway handles SIMD)      │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Sequence Diagram: solveConstraints

```
┌─────────────────────┐ ┌────────────────────────┐ ┌─────────────────────────┐
│ SkinnedMeshWorld    │ │ btConstraintSolverPoolMt│ │ GroupConstraintSolver   │
└──────────┬──────────┘ └───────────┬────────────┘ └────────────┬────────────┘
           │                        │                           │
           │ solveConstraints()     │                           │
           │────────────────────────┤                           │
           │                        │                           │
           │ prepareSolve()         │                           │
           │───────────────────────►│                           │
           │                        │                           │
           │                        │ Initialize thread pool    │
           │                        │───────┐                   │
           │                        │◄──────┘                   │
           │                        │                           │
           │ Collect constraint groups                          │
           │────────────────────────┤                           │
           │                        │                           │
           │ solveGroup()           │                           │
           │───────────────────────►│                           │
           │                        │                           │
           │                        │ For each simulation island:
           │                        │  ├─► convertBodies()      │
           │                        │  ├─► convertJoints()      │
           │                        │  └─► For each iteration:  │
           │                        │       └─► solveIteration()│
           │                        │                           │
           │ For each constraint group:                         │
           │─────────────────────────────────────────────────────►
           │                        │                           │
           │                        │                           │ solveGroup()
           │                        │                           │───────┐
           │                        │                           │       │
           │                        │                           │  Setup │
           │                        │                           │  Iterate
           │                        │                           │  Finish
           │                        │                           │◄──────┘
           │                        │                           │
           │ allSolved()            │                           │
           │───────────────────────►│                           │
           │                        │                           │
           │ clearAllManifold()     │                           │
           │◄───────────────────────│                           │
```

## Constraint Types

### 1. Contact Constraints (from collision)

Generated by collision detection, prevent interpenetration:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    CONTACT CONSTRAINT                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Body A          Contact Normal          Body B                    │
│     ○─────────────────►│◄───────────────────○                       │
│                        │                                            │
│   Constraint: relative velocity along normal ≥ 0                    │
│   (bodies can separate but not interpenetrate)                      │
│                                                                     │
│   Jacobian row:                                                     │
│   J = [-n, -r_A × n, n, r_B × n]                                    │
│                                                                     │
│   where:                                                            │
│     n = contact normal                                              │
│     r_A = contact point relative to body A center                   │
│     r_B = contact point relative to body B center                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 2. Joint Constraints (from XML definition)

Defined in physics XML files, maintain skeletal structure:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    BALL SOCKET CONSTRAINT                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│       Body A                        Body B                          │
│         ○──────────┬───────────────○                                │
│                    │                                                │
│                  Pivot                                              │
│                                                                     │
│   Constraint: pivot point in A frame = pivot point in B frame      │
│   (3 position constraints)                                          │
│                                                                     │
│   Jacobian rows:                                                    │
│   J_x = [1, 0, 0, r_A×[1,0,0], -1, 0, 0, -r_B×[1,0,0]]              │
│   J_y = [0, 1, 0, r_A×[0,1,0],  0,-1, 0, -r_B×[0,1,0]]              │
│   J_z = [0, 0, 1, r_A×[0,0,1],  0, 0,-1, -r_B×[0,0,1]]              │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                    STIFF SPRING CONSTRAINT                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│       Body A         spring          Body B                         │
│         ○─────────╱╲╱╲╱╲╱╲──────────○                               │
│                                                                     │
│   Constraint: distance = rest length (with stiffness/damping)       │
│   Applies force proportional to stretch                             │
│                                                                     │
│   Force = -k * (distance - restLength) - d * relativeVelocity       │
│                                                                     │
│   where:                                                            │
│     k = spring stiffness                                            │
│     d = damping coefficient                                         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Constraint Groups

hdtSMP64 groups related constraints for more efficient solving:

```cpp
// hdtConstraintGroup.h (conceptual)
class ConstraintGroup {
    std::vector<Ref<GenericConstraint>> m_constraints;
    static int MaxIterations;      // Default: 4
    static bool EnableMLCP;        // Default: false

    void solveGroup() {
        // Setup phase
        for (auto& c : m_constraints) {
            c->setupSolverData();
        }

        // Iterative solving
        for (int iter = 0; iter < MaxIterations; ++iter) {
            for (auto& c : m_constraints) {
                c->solveSingleIteration();
            }
        }
    }
};
```

### Group Structure Example

A typical hair physics setup:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    CONSTRAINT GROUP: HAIR_STRAND_01                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Head (kinematic)                                                  │
│       ○                                                             │
│       │ ←─ Ball socket + angular limit                              │
│       ○ Hair01                                                      │
│       │ ←─ Stiff spring + damping                                   │
│       ○ Hair02                                                      │
│       │ ←─ Stiff spring + damping                                   │
│       ○ Hair03                                                      │
│       │ ←─ Stiff spring + damping                                   │
│       ○ Hair04 (tip)                                                │
│                                                                     │
│   Group contains 4 joint constraints + 4 spring constraints         │
│   Solved together for stability (prevents jitter)                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Sequential Impulse Algorithm

The core solving algorithm:

```
┌─────────────────────────────────────────────────────────────────────┐
│              SEQUENTIAL IMPULSE ALGORITHM                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   FOR iteration = 1 to numIterations:                               │
│       FOR each constraint row c:                                    │
│                                                                     │
│           1. Compute relative velocity at constraint point          │
│              v_rel = J * V  (where V = body velocities)             │
│                                                                     │
│           2. Compute constraint error                               │
│              error = c.rhs - v_rel                                  │
│                                                                     │
│           3. Compute impulse magnitude                              │
│              λ = error / c.diagonal                                 │
│                                                                     │
│           4. Clamp accumulated impulse                              │
│              old_λ = c.accumulatedImpulse                           │
│              c.accumulatedImpulse = clamp(old_λ + λ, lo, hi)        │
│              Δλ = c.accumulatedImpulse - old_λ                      │
│                                                                     │
│           5. Apply impulse to bodies                                │
│              V_A += M_A⁻¹ * J_A^T * Δλ                               │
│              V_B += M_B⁻¹ * J_B^T * Δλ                               │
│                                                                     │
│   END FOR                                                           │
│                                                                     │
│   Note: Each iteration brings solution closer to equilibrium        │
│   More iterations = more accurate but more expensive                │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Scalar Fallback Solver

`hdtGroupConstraintSolver.cpp` contains two scalar constraint row resolvers that handle
low constraint counts (below the Highway batch threshold, where SIMD throughput is irrelevant):

```cpp
// hdtGroupConstraintSolver.cpp
// Scalar resolver — both limits (previously gResolveSingleConstraintRowGeneric_avx256)
static btScalar gResolveSingleConstraintRowGeneric(btSolverBody& body1, btSolverBody& body2,
                                                   const btSolverConstraint& c)
{
    const btScalar deltaVel1Dotn =
        c.m_contactNormal1.dot(body1.internalGetDeltaLinearVelocity()) +
        c.m_relpos1CrossNormal.dot(body1.internalGetDeltaAngularVelocity());
    // ... impulse clamping and velocity update via btVector3 methods
}
```

The AVX2 intrinsics (`_mm256_*`) that were previously in these functions were removed as part
of the Highway SIMD refactor. The hot path (>= batch threshold) is handled by the Highway
batch solver in `hdtHighwaySolverBridge.h`, which provides runtime-dispatched SIMD
(SSE4 → AVX2 → AVX-512) without requiring a per-ISA build variant.

**Location**: `hdtGroupConstraintSolver.cpp`

## Solver Configuration

Tunable parameters in `configs.xml`:

```xml
<configs>
    <!-- Bullet solver iterations (default 10) -->
    <solver-iterations>10</solver-iterations>

    <!-- GroupConstraintSolver iterations (default 4) -->
    <group-iterations>4</group-iterations>

    <!-- Enable MLCP for grouped constraints -->
    <mlcp-enabled>false</mlcp-enabled>

    <!-- Error reduction parameter (0-1) -->
    <erp>0.2</erp>
</configs>
```

| Parameter | Effect | Trade-off |
|-----------|--------|-----------|
| `solver-iterations` | More = more accurate | More CPU time |
| `group-iterations` | More = stiffer joints | More CPU time |
| `mlcp-enabled` | Better accuracy for complex chains | Significant CPU cost |
| `erp` | Higher = faster correction | Can cause oscillation |

## Performance Characteristics

```
┌─────────────────────────────────────────────────────────────────────┐
│                    SOLVER TIMING BREAKDOWN                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   prepareSolve()                ██░░░░░░░░░░░░░░░░░ ~10%            │
│   convertBodies/Joints          ████░░░░░░░░░░░░░░░ ~20%            │
│   solveGroup (Bullet)           ████████░░░░░░░░░░░ ~40%            │
│   solveGroup (ConstraintGroups) ██████░░░░░░░░░░░░░ ~25%            │
│   allSolved + cleanup           █░░░░░░░░░░░░░░░░░░ ~5%             │
│                                                                     │
│   Scaling:                                                          │
│   - O(n * k * i) where n=constraints, k=iterations, i=groups        │
│   - Parallel across simulation islands                              │
│   - Group solving is sequential within each group                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Debug Information

Tracy zones for solver profiling:

| Zone Name | What It Measures |
|-----------|------------------|
| `SolveConstraints` | Total solver time |
| `PrepareSolve` | Setup and initialization |
| `SolveGroup` | Per-island solving |
| `AllSolved` | Finalization |
| `Manifolds` | Number of contact manifolds (Tracy plot) |
| `Constraints` | Number of joint constraints (Tracy plot) |
| `ConstraintGroups` | Number of groups (Tracy plot) |

## Common Issues

### 1. Jittery Physics

**Symptoms**: Bodies vibrate or oscillate unnaturally
**Causes**: Insufficient iterations, high ERP, conflicting constraints
**Solutions**:
- Increase `solver-iterations`
- Decrease `erp`
- Check for over-constrained systems

### 2. Soft/Stretchy Joints

**Symptoms**: Joints appear to stretch under load
**Causes**: Insufficient iterations for constraint satisfaction
**Solutions**:
- Increase `group-iterations`
- Enable MLCP for critical chains
- Reduce constraint count if possible

### 3. Explosion/Instability

**Symptoms**: Bodies fly off to infinity
**Causes**: Numerical instability, extreme velocities
**Solutions**:
- Check velocity clamping in `integrateTransforms`
- Reduce timestep (increase `min-fps`)
- Add damping to constraints

## Next Steps

- **[Level 5: Data Structures](./05-DATA-STRUCTURES.md)** - Key classes and relationships
- **[Level 6: Threading Guide](./06-THREADING-EXPERT.md)** - Advanced parallelism
