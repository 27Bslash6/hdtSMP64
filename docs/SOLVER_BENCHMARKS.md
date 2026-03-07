# Solver Performance Benchmarks

> *Benchmark results from `test_solver_performance.cpp` on optimized Release configuration.*

## Table of Contents

- [Executive Summary](#executive-summary)
- [Detailed Results](#detailed-results)
- [Configuration Preset Benchmarks](#configuration-preset-benchmarks)
- [Recommendations by Use Case](#recommendations-by-use-case)
- [Future Optimization Opportunities](#future-optimization-opportunities)
- [Methodology](#methodology)

---

## Executive Summary

| Finding | Impact | Recommendation |
|---------|--------|----------------|
| Gauss-Seidel scales O(n²) per iteration | Large systems expensive | Keep constraint groups small |
| Chain propagation is **the** problem | Long hair needs many iterations | Consider XPBD or sub-stepping |
| Warm starting gives ~10x improvement | Free performance boost | Implement if not already |
| 4 iterations ≈ 99.99% accuracy | Diminishing returns after | 4-8 iterations optimal |

> [!IMPORTANT]
> **Key Takeaway**: 4-8 iterations is the sweet spot for most use cases. Beyond 8 provides negligible accuracy gains.

## Detailed Results

### 1. Gauss-Seidel Scaling (4 iterations)

| System Size | Description | Time (μs) | Time/Constraint |
|-------------|-------------|-----------|-----------------|
| 10 | Single bone chain | 0.37 | 0.037 μs |
| 25 | Short hair strand | 1.28 | 0.051 μs |
| 50 | Hair strand | 3.69 | 0.074 μs |
| 100 | Cloth section | 13.4 | 0.134 μs |
| 200 | Full cloth | 55.1 | 0.276 μs |

**Scaling:** O(n²) confirmed. 20x size increase → 148x time increase.

### 2. Iteration Count vs Accuracy (50 constraints)

| Iterations | Time (μs) | Max Error | RMS Error | Quality |
|------------|-----------|-----------|-----------|---------|
| 1 | 0.94 | 8.7% | 3.1% | Poor |
| 2 | 1.86 | 0.5% | 0.2% | Acceptable |
| 4 | 3.74 | 0.003% | 0.0008% | Good |
| 8 | 7.35 | ~0% | ~0% | Excellent |
| 16 | 14.7 | ~0% | ~0% | Overkill |
| 32 | 29.7 | ~0% | ~0% | Wasteful |

**Conclusion:** 4 iterations is the sweet spot. Beyond 8 is pointless for typical constraint systems.

### 3. Chain Propagation Problem (Critical for Hair)

> [!WARNING]
> This is the **fundamental limitation** of Gauss-Seidel for hair physics.

| Chain Length | Iterations | Tip Value | Propagation |
|--------------|------------|-----------|-------------|
| 5 bones | 1 | 0.013 | 100% (all 5) |
| 5 bones | 16 | 0.029 | 100% |
| 10 bones | 1 | 0.0001 | 50% (5/10) |
| 10 bones | 16 | 0.0009 | 60% (6/10) |
| 20 bones | 16 | 8.8e-7 | 30% (6/20) |
| 40 bones | 16 | 6.2e-13 | 15% (6/40) |
| 80 bones | 16 | 5.0e-26 | 7.5% (6/80) |

**The Problem:** An impulse at the root barely reaches the tip. For long hair chains, the tip essentially doesn't know the root moved.

**Why:** Gauss-Seidel propagates information one constraint per iteration. For an 80-bone chain, you'd need ~80 iterations for full propagation.

**Solutions:**
1. More iterations (expensive, doesn't scale)
2. Sub-stepping (update contacts, solve again)
3. XPBD (position-based, better propagation)
4. Hierarchical solvers (multi-grid approaches)

### 4. Warm Starting Benefit

| Start Type | 4 Iterations Error | Improvement |
|------------|-------------------|-------------|
| Cold (from zero) | 5.6e-6 | baseline |
| Warm (90% of converged) | 5.6e-7 | **9.9x better** |

> [!TIP]
> **Conclusion:** Reusing previous frame's solution is essentially free and dramatically improves accuracy.

---

## Configuration Preset Benchmarks

Real-world performance measurements from `test_bullet_solver_configs.cpp` using synthetic Gauss-Seidel/MLCP solvers.

### Iteration Scaling (25-bone Hair Chain)

| Iterations | Time (μs) | Error | Time/Iter |
|------------|-----------|-------|-----------|
| 2 | 0.55 | 0.359 | 0.275 |
| 4 | 1.13 | 0.293 | 0.283 |
| 6 | 1.55 | 0.243 | 0.259 |
| 8 | 2.06 | 0.206 | 0.257 |
| 10 | 2.56 | 0.178 | 0.256 |
| 16 | 4.06 | 0.125 | 0.254 |
| 32 | 8.18 | 0.067 | 0.256 |

**Key Finding:** Linear time scaling, diminishing accuracy returns. 8-10 iterations is the practical sweet spot.

### Iteration Scaling (8×8 Cloth Grid, 112 constraints)

| Iterations | Time (μs) | Error | Time/Iter |
|------------|-----------|-------|-----------|
| 2 | 1.32 | 0.140 | 0.661 |
| 4 | 2.58 | 0.071 | 0.644 |
| 8 | 5.02 | 0.033 | 0.628 |
| 16 | 10.13 | 0.013 | 0.633 |
| 32 | 19.94 | 0.004 | 0.623 |

**Key Finding:** Cloth converges better than chains at same iteration count due to mesh connectivity.

### MLCP vs Sequential Impulse

| Iterations | Solver | Hair Time (μs) | Hair Error | Cloth Time | Cloth Err |
|------------|--------|---------------|------------|------------|-----------|
| 4 | SI | 1.05 | 0.293 | 2.58 | 0.071 |
| 4 | MLCP | 1.26 | 0.239 | 2.79 | 0.061 |
| 8 | SI | 2.06 | 0.206 | 5.02 | 0.033 |
| 8 | MLCP | 2.30 | 0.154 | 5.51 | 0.026 |
| 16 | SI | 4.03 | 0.125 | 10.13 | 0.013 |
| 16 | MLCP | 4.39 | 0.086 | 11.04 | 0.010 |

**Key Finding:** MLCP costs ~10-20% more time but gives ~20-30% better accuracy. Worth it for quality presets.

### groupIterations Effectiveness (16 Total Iterations)

| NumIter | GroupIter | Hair Time | Hair Error | Cloth Time | Cloth Err |
|---------|-----------|-----------|------------|------------|-----------|
| 16 | 1 | 4.04 | 0.125 | 10.71 | 0.013 |
| 8 | 2 | 4.04 | 0.125 | 10.04 | 0.013 |
| 4 | 4 | 4.00 | 0.125 | 10.52 | 0.013 |
| 2 | 8 | 3.94 | 0.125 | 10.08 | 0.013 |

**Key Finding:** For pure solving (no contact updates), distribution doesn't matter. Same accuracy, same time.

### Realistic Game Scenarios

**5 NPCs with 20-bone hair (100 bones, 95 constraints total):**

| Preset | Time (μs) | 60fps % | Error |
|--------|-----------|---------|-------|
| Minimum (4×1) | 4.1 | 0.025% | 0.155 |
| Low (6×2) | 11.7 | 0.070% | 0.077 |
| Default (10×2+MLCP) | 21.7 | 0.130% | 0.033 |
| Balanced (10×4) | 38.4 | 0.230% | 0.025 |
| Quality (16×4+MLCP) | 68.0 | 0.408% | 0.009 |
| Maximum (16×8+MLCP) | 133.9 | 0.803% | 0.003 |

**Mixed: 3 NPCs hair + 1 cloth cape:**

| Preset | Time (μs) | 60fps % |
|--------|-----------|---------|
| Minimum | 3.0 | 0.018% |
| Low | 8.7 | 0.052% |
| Default | 16.4 | 0.098% |
| Balanced | 28.5 | 0.171% |
| Quality | 50.3 | 0.302% |
| Maximum | 100.9 | 0.605% |

### NPC Scaling (Default config: 10×2 SI)

| NPCs | Bones | Constraints | Time (μs) | μs/NPC | 60fps % |
|------|-------|-------------|-----------|--------|---------|
| 1 | 20 | 19 | 4.0 | 4.04 | 0.02% |
| 2 | 40 | 38 | 7.8 | 3.90 | 0.05% |
| 5 | 100 | 95 | 19.4 | 3.89 | 0.12% |
| 10 | 200 | 190 | 38.9 | 3.89 | 0.23% |
| 20 | 400 | 380 | 77.3 | 3.87 | 0.46% |

**Key Finding:** Near-perfect linear scaling. Physics stays well under 1% of frame budget even with 20 NPCs.

---

## Recommendations by Use Case

### Hair Physics (Chain Constraints)
```xml
<!-- Current defaults are suboptimal for long hair -->
<groupIterations>4</groupIterations>   <!-- Increase to 8 for long hair -->
<groupEnableMLCP>false</groupEnableMLCP>  <!-- MLCP doesn't help chains -->
```

Consider:
- Breaking long hair into multiple constraint groups
- Implementing warm starting if not present
- Sub-stepping (TGS-style) for better propagation

### Cloth Physics (Mesh Constraints)
```xml
<!-- More connected, less chain-like -->
<groupIterations>4</groupIterations>   <!-- Usually sufficient -->
<groupEnableMLCP>true</groupEnableMLCP>  <!-- May help with rigid patches -->
```

### Performance-Critical (Many NPCs)
```xml
<groupIterations>2</groupIterations>   <!-- Minimum viable -->
<groupEnableMLCP>false</groupEnableMLCP>  <!-- Skip expensive solver -->
<autoAdjustMaxSkeletons>true</autoAdjustMaxSkeletons>
```

## Future Optimization Opportunities

### High Impact
1. **Warm Starting** - If not implemented, add it. ~10x accuracy improvement for free.
2. **Constraint Group Sizing** - Keep groups under 50 constraints for O(n²) reasons.
3. **XPBD for Soft Bodies** - Modern approach, iteration-independent stiffness.

### Medium Impact
4. **TGS Sub-stepping** - Update contacts locally between substeps.
5. **Hierarchical Solving** - Multi-grid for very large cloth.

### Low Priority
6. **Eigen Integration** - Only if reformulating entire solver.
7. **GPU Solver** - XPBD parallelizes well; current solver less so.

## Test Commands

```bash
# Run all solver benchmarks
just bench-solver

# Run quick timing summary
just bench-summary

# Build benchmark configuration (AVX2 optimized)
just bench
```

## Methodology

- Synthetic constraint matrices (diagonally dominant, sparse)
- Tridiagonal matrices for chain tests (realistic hair model)
- 100 measured runs, 10 warmup runs
- High-resolution timer (std::chrono::high_resolution_clock)
- Benchmark configuration: MSVC /O2, AVX2, WPO enabled

---

<div align="center">

*For GPU physics architecture, see [GPU_PHYSICS_ARCHITECTURE.md](GPU_PHYSICS_ARCHITECTURE.md)*

</div>
