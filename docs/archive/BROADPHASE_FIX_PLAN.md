# Fix O(n²) Collision Pair Broadphase

## Status: INVESTIGATED - Body AABB Check Redundant

After implementation and profiling, the body-level AABB check was found to be redundant with Bullet's btDbvtBroadphase and has been **disabled**. See "Investigation Results" section below.

## Problem Summary

The collision pipeline has O(n²) scaling because `isBoundingSphereCollided()` at `hdtSkinnedMeshBody.cpp:365-402` is **disabled** (always returns `true`). This means ALL pairs from Bullet's broadphase pass through to `m_pairs`, and the expensive `collapseCollideL()` tree traversal runs for every pair.

## Root Cause

```cpp
// hdtSkinnedMeshBody.cpp:365-402
bool SkinnedMeshBody::isBoundingSphereCollided(SkinnedMeshBody* rhs)
{
    if (canCollideWith(rhs) && rhs->canCollideWith(this))
    {
        return true;  // ← ALWAYS TRUE - filtering disabled!
        // ... 30 lines of commented-out sphere checking code ...
    }
    return false;
}
```

## Solution: Add Body AABB Check

Add a simple AABB overlap test using existing `m_bulletShape.m_aabb` data. This is O(1) per pair and rejects non-overlapping bodies immediately.

## Files to Modify

| File | Change |
|------|--------|
| `hdtSMP64/hdtSkinnedMesh/hdtSkinnedMeshBody.cpp` | Fix `isBoundingSphereCollided()` |
| `hdtSMP64/hdtSkinnedMesh/hdtSkinnedMeshBody.h` | No changes needed |

## Implementation

### Step 1: Replace isBoundingSphereCollided()

**File:** `hdtSMP64/hdtSkinnedMesh/hdtSkinnedMeshBody.cpp` (lines 365-402)

Replace the disabled function with a proper AABB check:

```cpp
bool SkinnedMeshBody::isBoundingSphereCollided(SkinnedMeshBody* rhs)
{
    // Fast rejection: body AABBs must overlap
    // Note: canCollideWith() already checked in needsCollision() at call site
    return m_bulletShape.m_aabb.collideWith(rhs->m_bulletShape.m_aabb);
}
```

This:
- Uses existing `m_bulletShape.m_aabb` (already updated per-frame in `internalUpdate()`)
- Is O(1) per pair
- Removes redundant `canCollideWith()` check (already done in `needsCollision()`)
- Properly rejects pairs where body AABBs don't overlap

### Step 2: Verify AABB Update Path

Confirm that `m_bulletShape.m_aabb` is correctly updated before collision dispatch:

1. `SkinnedMeshBody::internalUpdate()` (line 143-234) updates vertex positions
2. Calls `m_shape->internalUpdate()` which updates collider AABBs
3. Line 231: `m_bulletShape.m_aabb = m_shape->m_tree.aabbAll` syncs the body AABB

**This path is already correct** - no changes needed.

## Expected Impact

| Metric | Before | After |
|--------|--------|-------|
| Pairs passed to m_pairs | ALL from Bullet | Only AABB-overlapping |
| Check cost per pair | O(1) trivial | O(1) AABB test |
| Expected pair reduction | 0% | 50-80% for sparse scenes |
| Complexity | O(n²) pairs × O(tree) | O(n²) pairs × O(1) reject + O(k) × O(tree) |

## Verification

1. **Build:** `just build V1_6_1170_NOCUDA_AVX2`

2. **Profile:** `just profile V1_6_1170_NOCUDA_AVX2` then use Tracy to compare:
   - `DispatchCollisionPairs` zone timing
   - Tracy zone values for filtering stages (see Instrumentation below)

3. **Test in-game:**
   - Load save with multiple NPCs (5+) with physics hair/cloth
   - Use `smp timing 200` console command
   - Compare mean frame times before/after

## Instrumentation

Added Tracy zone values to track filtering stages in `hdtDispatcher.cpp`:

| Zone Value | Meaning |
|------------|---------|
| `hdtBodies` | Pairs with at least one SkinnedMeshBody |
| `needsCollision` | Pairs that passed tag filtering |
| `aabbPass` | Pairs that passed AABB overlap test |

**Rejection calculation:**
- Tag filter rejections: `hdtBodies - needsCollision`
- AABB rejections: `needsCollision - aabbPass`
- Total rejections: `hdtBodies - aabbPass`

### Investigation Results (Final)

Tested with 12 dense NPCs using Tracy instrumentation:

| Stage | Count | Rejection Rate |
|-------|-------|----------------|
| Bullet broadphase | 600 | - |
| Tag filtering (`needsCollision`) | 170 | 72% |
| AABB check | 169 | <1% |

**Key Finding:** Body-level AABB check is redundant with Bullet's btDbvtBroadphase. Pairs that reach `isBoundingSphereCollided()` have already passed Bullet's proxy AABB test. Our body AABB is nearly identical data, so we reject almost nothing while adding ~2% overhead.

**Conclusion:** The O(n²) cost is NOT from pair filtering - it's from processing the ~170 pairs that pass all filters. Each pair runs `collapseCollideL()` tree traversal. The body AABB check has been **disabled** with explanatory comments in `hdtSkinnedMeshBody.cpp:365-376`.

**Next optimization targets:**
1. `collapseCollideL()` tree traversal optimization
2. Collider-level early rejection within tree traversal
3. Constraint solver chain propagation (separate O(n²) issue)

## Risk Assessment

**Risk: LOW**

- Uses existing, already-maintained AABB data
- Single function replacement
- No changes to collision detection logic
- No changes to constraint solving
- Easy to revert if issues arise

## Future Enhancements (If Needed)

If this doesn't provide sufficient improvement:

1. **SIMD batching:** Use `Aabb::collideWith2()` to test 2 pairs simultaneously
2. **Spatial hash grid:** Pre-filter bodies into grid cells before Bullet's broadphase
3. **Re-enable bone-level sphere check:** For `m_useBoundingSphere` bodies only

But start with the simple AABB fix first - it's likely sufficient.
