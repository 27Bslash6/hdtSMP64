# Constraint Solver Overhaul Plan

> *Investigation findings and future optimization roadmap for the constraint solver.*

## Table of Contents

- [Executive Summary](#executive-summary)
- [Current Architecture](#current-architecture)
- [Performance Profile](#performance-profile-from-traces)
- [Why GroupConstraintSolver Crashes](#why-groupconstraintsolver-crashes)
- [Solver Dispatch Flow](#solver-dispatch-flow-bullet-physics)
- [Optimization Opportunities](#optimization-opportunities)
- [Recommended Overhaul Steps](#recommended-overhaul-steps)
- [Files to Modify](#files-to-modify)
- [Key Code Locations](#key-code-locations)
- [References](#references)
- [Notes from Investigation](#notes-from-investigation)

---

## Executive Summary

> [!WARNING]
> The constraint solver has dead code (`GroupConstraintSolver`), unused parallelism, and architectural issues. This document captures investigation findings for a future overhaul.

---

## Current Architecture

### What's Actually Used

```
hdtSkinnedMeshWorld::solveConstraints()
    └── m_solverPool->solveGroup()           // btConstraintSolverPoolMt
            └── ThreadSolver->solver->solveGroup()  // btSequentialImpulseConstraintSolver
                    ├── solveGroupCacheFriendlySetup()   // 42% of solver time!
                    ├── solveGroupCacheFriendlyIterations()
                    │       └── solveSingleIteration() x N  // 10 iterations default
                    └── solveGroupCacheFriendlyFinish()
```

### What's Dead Code

```
GroupConstraintSolver (hdtGroupConstraintSolver.cpp)
    ├── m_groups populated but NEVER USED for solving
    ├── ConstraintGroup::setup() and iteration() - NEVER CALLED
    ├── SolverTask classes with locking - NEVER USED
    └── AVX-256 optimized constraint resolution - NEVER USED
```

---

## Performance Profile (from traces)

| Zone | % Time | Mean (ms) | Notes |
|------|--------|-----------|-------|
| **SolverSetup** | 7.31% | 3.34ms | convertBodies + convertJoints + convertContacts |
| **SolverIterations** | 9.82% | 4.49ms | 10 Gauss-Seidel iterations |
| **SolverFinish** | 0.31% | 0.14ms | writeback to bodies |
| **TOTAL** | 17.44% | 7.97ms | Per physics step |

**Key insight:** Setup is 42% of solver time - abnormally high.

---

## Why GroupConstraintSolver Crashes

### Bug 1: Race Conditions in Group Setup
```cpp
// hdtGroupConstraintSolver.cpp:249
concurrency::parallel_for_each(m_groups.begin(), m_groups.end(), [&](ConstraintGroup* i)
{
    i->setup(&m_tmpSolverBodyPool, infoGlobal);   // RACE: multiple groups zero same bodies
    i->iteration(bodies, numBodies, infoGlobal);  // RACE: multiple groups modify same bodies
});
```

Multiple ConstraintGroups share bodies in `m_tmpSolverBodyPool`. Parallel execution corrupts solver state.

### Bug 2: Fundamentally Broken Design

Each `ConstraintGroup` has its **OWN** constraint pools:
```cpp
// hdtConstraintGroup.cpp:107
m_tmpSolverNonContactConstraintPool.resizeNoInitialize(totalNumRows);
```

But `GroupConstraintSolver` also has pools from `Base::solveGroupCacheFriendlySetup()`. These don't share state correctly. The ConstraintGroup system was started but never finished.

### Bug 3: Never Tested Code Path

The original code always used `m_solverPool`, never `m_constraintSolver`. GroupConstraintSolver was dead code from day one.

---

## Solver Dispatch Flow (Bullet Physics)

### btConstraintSolverPoolMt (Current - Works)
```cpp
ThreadSolver* ts = getAndLockThreadSolver();  // Get solver from pool
ts->solver->solveGroup(...);                   // Each island gets isolated solver
ts->mutex.unlock();
```
- **Island-level parallelism**: Different islands solved in parallel
- **Within-island**: Sequential Gauss-Seidel (no parallelism)

### btSequentialImpulseConstraintSolverMt (Available - Not Used)
- **Batched constraints**: Groups constraints by body connectivity
- **Phase-based parallelism**: Batches in same phase solved in parallel
- **Uses btParallelSum**: For parallel iteration

### btSimulationIslandManagerMt Dispatch Logic
```cpp
if (island->manifoldArray.size() >= s_minimumContactManifoldsForBatching) {
    solveIsland(m_solverMt, *island, ...);  // Use parallel solver
} else {
    btParallelFor(..., dispatcher);          // Use pool of sequential solvers
}
```
Threshold is 250 manifolds - hdtSMP islands are smaller, so parallel solver is never used.

---

## Optimization Opportunities

### 1. SolverSetup Optimization (HIGH PRIORITY)
Setup is 42% of solver time. Investigate:
- `convertBodies()` - Can bodies be cached across frames?
- `convertJoints()` - Are constraints rebuilt unnecessarily?
- `convertContacts()` - Contact caching?

### 2. Enable btSequentialImpulseConstraintSolverMt (MEDIUM)
Replace btSequentialImpulseConstraintSolver in pool with Mt version:
```cpp
// btDiscreteDynamicsWorldMt.cpp:93
btConstraintSolver* solver = new btSequentialImpulseConstraintSolverMt();  // Instead of non-Mt
```
This gives within-island parallelism via batching.

### 3. Reduce Iterations (LOW - Quality Tradeoff)
```xml
<numIterations>10</numIterations>  <!-- Current default, range 4-128 -->
```
For cloth/hair, 4-6 might be sufficient.

### 4. SIMD Upgrades (MEDIUM)
- Current: SSE2/SSE4.1+FMA3 in Bullet, AVX-256 in GroupConstraintSolver (unused)
- Opportunity: AVX-512 for constraint resolution
- Consider: Google Highway for portable SIMD

### 5. Graph Coloring (COMPLEX - Future)
Color constraints so same-color constraints don't share bodies. Solve each color in parallel.

---

## Recommended Overhaul Steps

### Phase 1: Cleanup (1-2 hours)
1. Remove or disable `GroupConstraintSolver` and `ConstraintGroup` - it's broken dead code
2. Remove `m_constraintSolver` member from `SkinnedMeshWorld`
3. Remove `m_groups` population code

### Phase 2: Profile Setup (2-4 hours)
1. Add granular Tracy zones to `convertBodies`, `convertJoints`, `convertContacts`
2. Identify the actual bottleneck in setup
3. Consider caching solver bodies across frames

### Phase 3: Enable Proper Parallelism (4-8 hours)
1. Replace solver instances in pool with `btSequentialImpulseConstraintSolverMt`
2. Or: Lower `s_minimumContactManifoldsForBatching` threshold
3. Test and profile

### Phase 4: SIMD Optimization (8-16 hours)
1. Evaluate Google Highway integration
2. Add AVX-512 constraint resolution path
3. Benchmark on various CPUs

---

## Files to Modify

| File | Current State | Action |
|------|---------------|--------|
| `hdtGroupConstraintSolver.cpp/h` | Dead code with bugs | DELETE or gut |
| `hdtConstraintGroup.cpp/h` | Dead code | DELETE or gut |
| `hdtSkinnedMeshWorld.cpp` | Uses m_solverPool | Remove m_constraintSolver, m_groups |
| `hdtSkinnedMeshWorld.h` | Has GroupConstraintSolver member | Remove |
| `btSequentialImpulseConstraintSolver.cpp` | Has Tracy zones (keep) | Keep instrumentation |
| `btDiscreteDynamicsWorldMt.cpp` | Creates btSequentialImpulseConstraintSolver | Consider using Mt version |

---

## Key Code Locations

### Constraint Resolution (Hot Path)
```cpp
// btSequentialImpulseConstraintSolver.cpp:46
static btScalar gResolveSingleConstraintRowGeneric_scalar_reference(
    btSolverBody& bodyA, btSolverBody& bodyB, const btSolverConstraint& c)
```

### SIMD Variants
- `gResolveSingleConstraintRowGeneric_sse2` - SSE2 version
- `gResolveSingleConstraintRowGeneric_sse4_1_fma3` - SSE4.1+FMA3 version
- `gResolveSingleConstraintRowGeneric_avx256` - AVX-256 (in GroupConstraintSolver, unused)

### Solver Configuration
```cpp
// config.cpp:30-31
SkyrimPhysicsWorld::get()->getSolverInfo().m_numIterations = btClamped(reader.readInt(), 4, 128);
```

---

## References

- Bullet Physics source: `BulletDynamics/ConstraintSolver/`
- Gauss-Seidel PGS solver: Standard iterative constraint solver
- btBatchedConstraints: Bullet's constraint batching for parallelism
- Google Highway: https://github.com/google/highway

---

## Notes from Investigation

1. **AABB check in broadphase was redundant** - Bullet's btDbvtBroadphase already does this. Removed.

2. **Tracy instrumentation added** to btSequentialImpulseConstraintSolver - SolverSetup, SolverIterations, SolverFinish zones.

3. **Tag filtering** rejects 72% of collision pairs in `needsCollision()` - this is working well.

4. **Island sizes are small** (<250 manifolds) - this is why btSimulationIslandManagerMt never uses the parallel solver path.

---

*Last updated: Investigation session Jan 2025*
