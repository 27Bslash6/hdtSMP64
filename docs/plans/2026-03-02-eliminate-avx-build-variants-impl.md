# Eliminate AVX Build Variants: Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace all remaining raw SIMD intrinsics with scalar/Highway equivalents, then collapse 112 build configurations to 18 by removing the AVX dimension.

**Architecture:** Three files still use raw intrinsics that prevent a single-config build. `hdtGroupConstraintSolver.cpp` is the only actual AVX2 blocker (uses `_mm256_*`). The other two (`hdtLCP.cpp`, `hdtSkinnedMeshAlgorithm.cpp`) use SSE4.1 which doesn't block anything but should be cleaned up for consistency. After all three are cleaned, a Python script strips AVX configs from the vcxproj, then the sln/CI/justfile are updated.

**Tech Stack:** C++17, Bullet Physics (`btVector3`, `btSolverBody`, `btSolverConstraint`), MSBuild, Python 3 (xml.etree.ElementTree for vcxproj edit), Highway SIMD (already integrated)

**Design doc:** `docs/plans/2026-03-02-eliminate-avx-build-variants.md`

---

## Background: What These Functions Actually Do

**`hdtGroupConstraintSolver.cpp`** — the scalar path for Bullet's constraint solver (called per-constraint when batch count < 64). Uses AVX2 to pack body1+body2 vectors into 256-bit registers for dual dot products. The Highway batch solver (`hdtHighwaySolverBridge.h`) already handles the hot path (>64 constraints). This scalar fallback can safely become plain C++.

**`hdtLCP.cpp`** — Cholesky factorization for the MLCP solver. Uses SSE4.1 `_mm_dp_ps` (dot product) and `_mm_hadd_ps`. The `highway::largeDot()` and `highway::vectorScale()` helpers already replaced the main hot spots; these are the remaining matrix factorization loops.

**`hdtSkinnedMeshAlgorithm.cpp`** — per-collision-pair geometry (cross products, dot products, triangle intersection). Uses SSE4.1 on `btVector3::get128()` data. `btVector3` already has scalar `.dot()`, `.cross()`, `.length()`, `.normalize()` — use them directly.

**Build commands** (run from repo root in Git Bash):
```bash
# Build
powershell.exe -NoProfile -Command "& 'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/MSBuild/Current/Bin/amd64/MSBuild.exe' 'hdtSMP64.sln' '-p:Configuration=V1_6_1170_NOCUDA_AVX2' '-p:Platform=x64' '-v:m' '-p:SolutionDir=$(pwd)/'"

# Build tests
powershell.exe -NoProfile -Command "& 'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/MSBuild/Current/Bin/amd64/MSBuild.exe' 'tests/hdtSMP64_tests.vcxproj' '-p:Configuration=Release' '-p:Platform=x64' '-v:m'"

# Run tests
./tests/x64/tests/Release/hdtSMP64_tests.exe --reporter console
```

---

## Task 1: Scalar-ify hdtGroupConstraintSolver.cpp

This is the AVX2 blocker. Validate that replacing the dual-body packing with plain scalar
works before touching the build system.

**Files:**
- Modify: `hdtSMP64/hdtSkinnedMesh/hdtGroupConstraintSolver.cpp`

### Step 1: Verify the build currently works

```bash
powershell.exe -NoProfile -Command "& 'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/MSBuild/Current/Bin/amd64/MSBuild.exe' 'hdtSMP64.sln' '-p:Configuration=V1_6_1170_NOCUDA_AVX2' '-p:Platform=x64' '-v:m' '-p:SolutionDir=$(pwd)/'"
```

Expected: Build succeeds. This is your baseline.

### Step 2: Read the full file

Read `hdtSMP64/hdtSkinnedMesh/hdtGroupConstraintSolver.cpp` in full before editing.

### Step 3: Replace the file contents

The two AVX2 functions, their helpers, and macros all get replaced. The scalar versions
are mathematically identical — just two sequential body operations instead of one packed
8-wide operation.

Delete everything from line 1 through the closing brace of `GroupConstraintSolver::GroupConstraintSolver()`
and replace with:

```cpp
#include "hdtGroupConstraintSolver.h"
#include "LinearMath/btScalar.h"

namespace hdt
{
	// Scalar constraint row resolver (generic — both limits)
	// Previously gResolveSingleConstraintRowGeneric_avx256.
	// The Highway batch solver (hdtHighwaySolverBridge.h) handles the hot path (>= batch threshold).
	// This scalar path handles low constraint counts where SIMD throughput is irrelevant.
	static btScalar gResolveSingleConstraintRowGeneric(btSolverBody& body1, btSolverBody& body2,
	                                                    const btSolverConstraint& c)
	{
		const btScalar deltaVel1Dotn =
			c.m_contactNormal1.dot(body1.internalGetDeltaLinearVelocity()) +
			c.m_relpos1CrossNormal.dot(body1.internalGetDeltaAngularVelocity());
		const btScalar deltaVel2Dotn =
			c.m_contactNormal2.dot(body2.internalGetDeltaLinearVelocity()) +
			c.m_relpos2CrossNormal.dot(body2.internalGetDeltaAngularVelocity());

		btScalar deltaImpulse = c.m_rhs - btScalar(c.m_appliedImpulse) * c.m_cfm;
		deltaImpulse -= (deltaVel1Dotn + deltaVel2Dotn) * c.m_jacDiagABInv;

		const btScalar sum = btScalar(c.m_appliedImpulse) + deltaImpulse;
		const btScalar appliedImpulse = btClamped(sum, c.m_lowerLimit, c.m_upperLimit);
		deltaImpulse = appliedImpulse - btScalar(c.m_appliedImpulse);
		c.m_appliedImpulse = appliedImpulse;

		body1.internalGetDeltaLinearVelocity()  += c.m_contactNormal1 * body1.internalGetInvMass() * deltaImpulse;
		body1.internalGetDeltaAngularVelocity() += c.m_angularComponentA * deltaImpulse;
		body2.internalGetDeltaLinearVelocity()  += c.m_contactNormal2 * body2.internalGetInvMass() * deltaImpulse;
		body2.internalGetDeltaAngularVelocity() += c.m_angularComponentB * deltaImpulse;

		const btScalar jacInv = c.m_jacDiagABInv;
		return (btFabs(jacInv) > SIMD_EPSILON) ? (deltaImpulse / jacInv) : btScalar(0);
	}

	// Scalar constraint row resolver (lower limit only — contact constraints)
	// Previously gResolveSingleConstraintRowLowerLimit_avx256.
	static btScalar gResolveSingleConstraintRowLowerLimit(btSolverBody& body1, btSolverBody& body2,
	                                                       const btSolverConstraint& c)
	{
		const btScalar deltaVel1Dotn =
			c.m_contactNormal1.dot(body1.internalGetDeltaLinearVelocity()) +
			c.m_relpos1CrossNormal.dot(body1.internalGetDeltaAngularVelocity());
		const btScalar deltaVel2Dotn =
			c.m_contactNormal2.dot(body2.internalGetDeltaLinearVelocity()) +
			c.m_relpos2CrossNormal.dot(body2.internalGetDeltaAngularVelocity());

		btScalar deltaImpulse = c.m_rhs - btScalar(c.m_appliedImpulse) * c.m_cfm;
		deltaImpulse -= (deltaVel1Dotn + deltaVel2Dotn) * c.m_jacDiagABInv;

		const btScalar sum = btScalar(c.m_appliedImpulse) + deltaImpulse;
		const btScalar appliedImpulse = btMax(sum, c.m_lowerLimit);
		deltaImpulse = appliedImpulse - btScalar(c.m_appliedImpulse);
		c.m_appliedImpulse = appliedImpulse;

		body1.internalGetDeltaLinearVelocity()  += c.m_contactNormal1 * body1.internalGetInvMass() * deltaImpulse;
		body1.internalGetDeltaAngularVelocity() += c.m_angularComponentA * deltaImpulse;
		body2.internalGetDeltaLinearVelocity()  += c.m_contactNormal2 * body2.internalGetInvMass() * deltaImpulse;
		body2.internalGetDeltaAngularVelocity() += c.m_angularComponentB * deltaImpulse;

		const btScalar jacInv = c.m_jacDiagABInv;
		return (btFabs(jacInv) > SIMD_EPSILON) ? (deltaImpulse / jacInv) : btScalar(0);
	}

	btSingleConstraintRowSolver GroupConstraintSolver::getResolveSingleConstraintRowGenericAVX()
	{
		return gResolveSingleConstraintRowGeneric;
	}

	btSingleConstraintRowSolver GroupConstraintSolver::getResolveSingleConstraintRowLowerLimitAVX()
	{
		return gResolveSingleConstraintRowLowerLimit;
	}

	GroupConstraintSolver::GroupConstraintSolver()
	{
		m_resolveSingleConstraintRowGeneric    = gResolveSingleConstraintRowGeneric;
		m_resolveSingleConstraintRowLowerLimit = gResolveSingleConstraintRowLowerLimit;
	}
} // namespace hdt
```

Note: The original constructor checked `CPU_FEATURE_FMA3 && CPU_FEATURE_SSE4_1` before
enabling the AVX2 path. With scalar, no CPU feature check is needed — scalar always works.
The Highway batch solver already handles the real hot path.

Also remove the `#include <immintrin.h>` if present at the top of the file.

### Step 4: Build and verify no AVX2 intrinsics remain

```bash
powershell.exe -NoProfile -Command "& 'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/MSBuild/Current/Bin/amd64/MSBuild.exe' 'hdtSMP64.sln' '-p:Configuration=V1_6_1170_NOCUDA_AVX2' '-p:Platform=x64' '-v:m' '-p:SolutionDir=$(pwd)/'"
```

Expected: Build succeeds.

Then verify no AVX2 intrinsics remain:
```bash
grep -n "_mm256\|__m256" hdtSMP64/hdtSkinnedMesh/hdtGroupConstraintSolver.cpp
```

Expected: No output.

### Step 5: Run tests

```bash
powershell.exe -NoProfile -Command "& 'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/MSBuild/Current/Bin/amd64/MSBuild.exe' 'tests/hdtSMP64_tests.vcxproj' '-p:Configuration=Release' '-p:Platform=x64' '-v:m'"
./tests/x64/tests/Release/hdtSMP64_tests.exe --reporter console
```

Expected: All tests pass.

### Step 6: Commit

```bash
git add hdtSMP64/hdtSkinnedMesh/hdtGroupConstraintSolver.cpp
git commit -m "refactor: replace AVX2 constraint solver with scalar

The scalar path handles constraints below the Highway batch threshold (64).
SIMD throughput is irrelevant at those counts; the Highway batch bridge
owns the hot path. Removes the only AVX2 dependency from non-Highway code."
```

---

## Task 2: Clean up hdtSkinnedMeshAlgorithm.cpp

Replace SSE4.1 geometry intrinsics with `btVector3` methods. These aren't blocking anything
(SSE4.1 doesn't require /arch:AVX2) but they're inconsistent with the rest of the codebase.

**Files:**
- Modify: `hdtSMP64/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.cpp`

### Step 1: Read the file around the intrinsics

Read `hdtSMP64/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.cpp` lines 160–290 to see the full
`cross_product` and `checkCollide` implementations.

### Step 2: Replace cross_product with btVector3 cross

Find:
```cpp
inline __m128 cross_product(__m128 const& vec0, __m128 const& vec1)
{
    __m128 tmp0 = _mm_shuffle_ps(vec0, vec0, _MM_SHUFFLE(3, 0, 2, 1));
    __m128 tmp1 = _mm_shuffle_ps(vec1, vec1, _MM_SHUFFLE(3, 1, 0, 2));
    __m128 tmp2 = _mm_mul_ps(tmp0, vec1);
    __m128 tmp3 = _mm_mul_ps(tmp0, tmp1);
    __m128 tmp4 = _mm_shuffle_ps(tmp2, tmp2, _MM_SHUFFLE(3, 0, 2, 1));
    return _mm_sub_ps(tmp3, tmp4);
}
```

Replace with: **delete the function entirely.** The callers use `btVector3`, and `btVector3`
already has `.cross()`. Update the call sites to use `btVector3::cross()` instead of
calling `cross_product(a.get128(), b.get128())`.

### Step 3: Replace _mm_dp_ps, _mm_sqrt_ps, _mm_div_ps with btVector3 methods

For each `_mm_dp_ps(a, b, mask)` computing a 3D dot product (mask `0x77` or `0x7f`):
```cpp
// Old: _mm_cvtss_f32(_mm_dp_ps(vec_a, vec_b, 0x77))
// New:
btVector3 a_bt(...), b_bt(...);
btScalar result = a_bt.dot(b_bt);
```

For normalize:
```cpp
// Old: auto len = _mm_sqrt_ps(_mm_dp_ps(raw_normal, raw_normal, 0x77));
//      auto normal = _mm_div_ps(raw_normal, len);
// New:
btVector3 normal = raw_normal_bt;
btScalar len = normal.length();
if (len < FLT_EPSILON) { /* handle degenerate */ }
normal /= len;
```

The `SP0`, `SP1` structs in this file have `pos()` returning `btTransform` and `.get128()`
on inner vectors. Work at the `btVector3` level throughout.

### Step 4: Remove immintrin.h include if present

Check the top of the file for `#include <immintrin.h>` or `#include <nmmintrin.h>` and remove.

### Step 5: Build

```bash
powershell.exe -NoProfile -Command "& 'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/MSBuild/Current/Bin/amd64/MSBuild.exe' 'hdtSMP64.sln' '-p:Configuration=V1_6_1170_NOCUDA_AVX2' '-p:Platform=x64' '-v:m' '-p:SolutionDir=$(pwd)/'"
```

Expected: Build succeeds.

### Step 6: Verify no SSE intrinsics remain in this file

```bash
grep -n "_mm_\|__m128\|_mm256\|__m256" hdtSMP64/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.cpp
```

Expected: No output.

### Step 7: Run tests

```bash
./tests/x64/tests/Release/hdtSMP64_tests.exe --reporter console
```

Expected: All tests pass.

### Step 8: Commit

```bash
git add hdtSMP64/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.cpp
git commit -m "refactor: replace SSE4.1 geometry intrinsics with btVector3 methods

cross_product, checkCollide, and triangle intersection ops now use
btVector3::cross(), dot(), length(), normalize() directly."
```

---

## Task 3: Clean up hdtLCP.cpp

Replace remaining SSE4.1 matrix solver intrinsics. These are in the Cholesky factorization
loops. `largeDot` and `vectorScale` were already migrated to `hdtHighwayLCP.cpp`; these
are the remaining lower-level matrix ops.

**Files:**
- Modify: `hdtSMP64/hdtSkinnedMesh/hdtLCP.cpp`

### Step 1: Read the file to map all intrinsic sites

Read `hdtSMP64/hdtSkinnedMesh/hdtLCP.cpp` in full. There are roughly 4 functions with intrinsics:
- A dot product helper (lines ~36–52)
- `factorMatrixPD_1_2()` style functions (lines ~95–170)
- A matrix scaling loop (lines ~220–235)
- A 4-row accumulation (lines ~374–390)

### Step 2: Replace the SSE4.1 dot product with scalar

The pattern `_mm_loadu_ps` + `_mm_dp_ps` + `_mm_hadd_ps` is computing a sum of dot products
over a float array. Replace with a simple scalar loop:

```cpp
// Old SSE4.1 dot over array (4 floats at a time):
__m128 xmm0 = _mm_setzero_ps();
for (...) {
    __m128 xmm1 = _mm_loadu_ps(a);
    __m128 xmm2 = _mm_loadu_ps(b);
    xmm0 = _mm_add_ps(xmm0, _mm_mul_ps(xmm1, xmm2));
    ...
}
xmm0 = _mm_hadd_ps(xmm0, xmm0);
xmm0 = _mm_hadd_ps(xmm0, xmm0);
float sum = _mm_cvtss_f32(xmm0);

// New scalar:
float sum = 0.0f;
for (int i = 0; i < n; i++) sum += a[i] * b[i];
```

Note: `largeDot()` in `hdtHighwayLCP.cpp` is already the Highway-accelerated version of this.
If the LCP code is calling its own inline version, replace it with a call to `highway::largeDot()`
from `hdtHighwayLCP.h`.

### Step 3: Replace _mm_dp_ps (dot product of 4 floats) pattern

```cpp
// Old: _mm_cvtss_f32(_mm_dp_ps(p1, q1, 0xf1))
// Computes dot of 4 floats and stores result in lane 0.
// New (where p1, q1 are float[4]):
float dp = p1[0]*q1[0] + p1[1]*q1[1] + p1[2]*q1[2] + p1[3]*q1[3];
// Or if they're btVector3:
float dp = vec_p.dot(vec_q);
```

### Step 4: Replace load/store/multiply/add with scalar

```cpp
// Old:
__m128 p1 = _mm_loadu_ps(ell);
__m128 dd = _mm_loadu_ps(dee);
// ... multiply, store
_mm_storeu_ps(ell, q1);

// New:
for (int k = 0; k < 4; k++) {
    float val = ell[k] * dee[k]; // or whatever the operation is
    ell[k] = val;
}
```

Work through each intrinsic block systematically. The compiler will auto-vectorize the
scalar loops; perf will be equivalent to SSE4.1 in practice.

### Step 5: Build

```bash
powershell.exe -NoProfile -Command "& 'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/MSBuild/Current/Bin/amd64/MSBuild.exe' 'hdtSMP64.sln' '-p:Configuration=V1_6_1170_NOCUDA_AVX2' '-p:Platform=x64' '-v:m' '-p:SolutionDir=$(pwd)/'"
```

Expected: Build succeeds.

### Step 6: Verify no SSE intrinsics remain in this file

```bash
grep -n "_mm_\|__m128\|_mm256\|__m256" hdtSMP64/hdtSkinnedMesh/hdtLCP.cpp
```

Expected: No output.

### Step 7: Run tests — pay special attention to solver tests

```bash
./tests/x64/tests/Release/hdtSMP64_tests.exe --reporter console
```

Expected: All tests pass, especially `test_bullet_solver_configs` which exercises the LCP solver.

### Step 8: Verify zero intrinsics across all non-Highway files

```bash
grep -rn "_mm256\|__m256\|_mm512\|__m512\|_mm_dp_ps\|_mm_hadd_ps" \
  hdtSMP64/hdtSkinnedMesh/ \
  --include="*.cpp" --include="*.h" \
  | grep -v "hdtHighway\|hdtSoA\|BulletCollision\|BulletDynamics\|LinearMath"
```

Expected: No output. This is the gate check — zero raw SIMD in non-Highway files.

### Step 9: Commit

```bash
git add hdtSMP64/hdtSkinnedMesh/hdtLCP.cpp
git commit -m "refactor: replace SSE4.1 LCP solver intrinsics with scalar

Cholesky factorization loops now use plain C++. The compiler auto-vectorizes
the scalar code; hot paths are already handled by highway::largeDot and
highway::vectorScale in hdtHighwayLCP.cpp."
```

---

## Task 4: Strip AVX configs from hdtSMP64.vcxproj (script)

The vcxproj is 10k+ lines — editing by hand is a liability. Write a Python script to
surgically remove the 94 AVX-variant configurations.

**Files:**
- Create: `scripts/strip_avx_configs.py`
- Modify: `hdtSMP64/hdtSMP64.vcxproj`

### Step 1: Write the script

Create `scripts/strip_avx_configs.py`:

```python
#!/usr/bin/env python3
"""
Remove AVX build variant configurations from hdtSMP64.vcxproj.

Keeps: {VERSION}_CUDA, {VERSION}_NOCUDA, {VERSION}_CUDA_DEBUG, {VERSION}_NOCUDA_DEBUG
Removes: anything with _NoAVX, _AVX_, _AVX2_, _AVX512_ in the configuration name
Also sets global EnableEnhancedInstructionSet to NotSet for kept configs.
"""

import xml.etree.ElementTree as ET
import re
import sys
import shutil
from pathlib import Path

NS = "http://schemas.microsoft.com/developer/msbuild/2003"
ET.register_namespace("", NS)

def tag(name):
    return f"{{{NS}}}{name}"

def is_avx_config(config_name):
    """Return True if this config should be removed (has an AVX variant suffix)."""
    # Match _NoAVX, _AVX_, _AVX2_, _AVX512_ anywhere in name
    return bool(re.search(r'_(NoAVX|AVX_|AVX2|AVX512)', config_name))

def get_config_name(condition):
    """Extract config name from Condition attribute like '$(Configuration)|$(Platform)'=='FOO|x64'"""
    m = re.search(r"'=='\s*'([^|]+)\|", condition)
    if not m:
        m = re.search(r"Include=\"([^|]+)\|", condition)
    return m.group(1) if m else None

def strip_avx(vcxproj_path):
    tree = ET.parse(vcxproj_path)
    root = tree.getroot()

    removed_configs = set()
    elements_to_remove = []

    # Pass 1: collect ProjectConfiguration entries to remove
    for ig in root.findall(f".//{tag('ItemGroup')}"):
        for pc in ig.findall(tag("ProjectConfiguration")):
            include = pc.get("Include", "")
            config_name = include.split("|")[0]
            if is_avx_config(config_name):
                elements_to_remove.append((ig, pc))
                removed_configs.add(config_name)

    # Pass 2: collect PropertyGroup and ItemDefinitionGroup with matching Condition
    for elem in root:
        cond = elem.get("Condition", "")
        config_name = get_config_name(cond)
        if config_name and config_name in removed_configs:
            elements_to_remove.append((root, elem))
        # Also check nested children
        for child in elem:
            child_cond = child.get("Condition", "")
            child_config = get_config_name(child_cond)
            if child_config and child_config in removed_configs:
                elements_to_remove.append((elem, child))

    # Remove collected elements
    for parent, child in elements_to_remove:
        try:
            parent.remove(child)
        except ValueError:
            pass  # already removed

    # Pass 3: set EnableEnhancedInstructionSet to NotSet for all remaining non-Highway ClCompile
    for eis in root.findall(f".//{tag('EnableEnhancedInstructionSet')}"):
        # Check if this is inside a Highway file override (parent ClCompile has Include)
        # Highway files intentionally use AVX512 — don't touch those
        parent_compile = eis.find("..")  # not directly available in ElementTree
        # Safe approach: only reset if value is AVX/AVX2/AVX512 but not inside
        # a per-file ClCompile (those have an Include attribute on the ClCompile parent).
        # We'll reset all non-per-file ones.
        current = eis.text or ""
        if current in ("AdvancedVectorExtensions", "AdvancedVectorExtensions2",
                       "AdvancedVectorExtensions512"):
            eis.text = "NotSet"

    print(f"Removed {len(removed_configs)} configurations:")
    for c in sorted(removed_configs):
        print(f"  - {c}")

    # Backup original
    backup = Path(vcxproj_path).with_suffix(".vcxproj.bak")
    shutil.copy2(vcxproj_path, backup)
    print(f"Backup saved to {backup}")

    tree.write(vcxproj_path, encoding="utf-8", xml_declaration=True)
    print(f"Written: {vcxproj_path}")

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "hdtSMP64/hdtSMP64.vcxproj"
    strip_avx(path)
```

### Step 2: Run the script (dry-run first — count configs before)

```bash
# Count configs before
grep -c "ProjectConfiguration Include" hdtSMP64/hdtSMP64.vcxproj

# Run
python3 scripts/strip_avx_configs.py hdtSMP64/hdtSMP64.vcxproj

# Count configs after
grep -c "ProjectConfiguration Include" hdtSMP64/hdtSMP64.vcxproj
```

Expected before: 112. Expected after: 18.

### Step 3: Verify the Highway per-file AVX512 overrides are preserved

```bash
grep -A2 "hdtHighway" hdtSMP64/hdtSMP64.vcxproj | grep "AVX512"
```

Expected: still shows `AdvancedVectorExtensions512` for the Highway files.

### Step 4: Build with a kept config to verify vcxproj is valid

```bash
powershell.exe -NoProfile -Command "& 'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/MSBuild/Current/Bin/amd64/MSBuild.exe' 'hdtSMP64.sln' '-p:Configuration=V1_6_1170_NOCUDA' '-p:Platform=x64' '-v:m' '-p:SolutionDir=$(pwd)/'"
```

If this fails because `.sln` still references old configs (expected), continue to Task 5.
If it fails for another reason, investigate before proceeding.

### Step 5: Commit

```bash
git add scripts/strip_avx_configs.py hdtSMP64/hdtSMP64.vcxproj
git rm hdtSMP64/hdtSMP64.vcxproj.bak 2>/dev/null || true
git commit -m "build: strip AVX variant configs from vcxproj (112 → 18)

Script-driven removal of NoAVX/AVX/AVX2/AVX512 build configurations.
Highway SIMD provides runtime dispatch; compile-time AVX variants are obsolete.
CUDA/NOCUDA split retained — nvcc is a build-time dependency."
```

---

## Task 5: Update hdtSMP64.sln

The solution file lists all configurations. Remove the 94 deleted ones.

**Files:**
- Modify: `hdtSMP64/hdtSMP64.sln`

### Step 1: Check what's in the .sln

```bash
grep -c "V1_6_1170_NOCUDA_AVX2\|V1_6_1170_NOCUDA_NoAVX" hdtSMP64.sln
```

If count > 0, proceed. If 0, the .sln may already be clean (unlikely).

### Step 2: Write a script to clean the .sln

Create `scripts/strip_avx_sln.py`:

```python
#!/usr/bin/env python3
"""Remove AVX build variant lines from hdtSMP64.sln."""

import re
import sys
import shutil
from pathlib import Path

def is_avx_config(line):
    return bool(re.search(r'_(NoAVX|AVX_|AVX2|AVX512)', line))

def strip_avx_sln(sln_path):
    path = Path(sln_path)
    lines = path.read_text(encoding="utf-8-sig").splitlines(keepends=True)

    kept = []
    removed = 0
    for line in lines:
        if is_avx_config(line):
            removed += 1
        else:
            kept.append(line)

    shutil.copy2(sln_path, path.with_suffix(".sln.bak"))
    path.write_text("".join(kept), encoding="utf-8")
    print(f"Removed {removed} lines from {sln_path}")

if __name__ == "__main__":
    strip_avx_sln(sys.argv[1] if len(sys.argv) > 1 else "hdtSMP64.sln")
```

### Step 3: Run it

```bash
python3 scripts/strip_avx_sln.py hdtSMP64.sln
```

### Step 4: Build to verify .sln is valid

```bash
powershell.exe -NoProfile -Command "& 'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/MSBuild/Current/Bin/amd64/MSBuild.exe' 'hdtSMP64.sln' '-p:Configuration=V1_6_1170_NOCUDA' '-p:Platform=x64' '-v:m' '-p:SolutionDir=$(pwd)/'"
```

Expected: Build succeeds.

### Step 5: Run tests

```bash
./tests/x64/tests/Release/hdtSMP64_tests.exe --reporter console
```

Expected: All pass.

### Step 6: Commit

```bash
git add hdtSMP64.sln scripts/strip_avx_sln.py
git rm hdtSMP64.sln.bak 2>/dev/null || true
git commit -m "build: remove AVX variant entries from solution file"
```

---

## Task 6: Update build.yml, justfile, and CLAUDE.md

**Files:**
- Modify: `.github/workflows/build.yml`
- Modify: `justfile`
- Modify: `CLAUDE.md`

### Step 1: Update build.yml

In `.github/workflows/build.yml`, the `build` job has a matrix with an `avx` dimension.
Simplify it:

Find the `strategy.matrix.include` section:
```yaml
    strategy:
      fail-fast: false
      matrix:
        include:
          # Anniversary Edition 1.6.1170 (primary target)
          - version: V1_6_1170
            cuda: NOCUDA
            avx: NoAVX
          - version: V1_6_1170
            cuda: NOCUDA
            avx: AVX2
```

Replace with:
```yaml
    strategy:
      fail-fast: false
      matrix:
        include:
          # Anniversary Edition 1.6.1170 (primary target)
          - version: V1_6_1170
            cuda: NOCUDA
```

Then find and update all references to `${{ matrix.avx }}` in the job steps:

- Job name: `Build ${{ matrix.version }}_${{ matrix.cuda }}_${{ matrix.avx }}`
  → `Build ${{ matrix.version }}_${{ matrix.cuda }}`

- Cache path: `x64/${{ matrix.version }}_${{ matrix.cuda }}_${{ matrix.avx }}`
  → `x64/${{ matrix.version }}_${{ matrix.cuda }}`

- Cache key: `build-${{ matrix.version }}-${{ matrix.cuda }}-${{ matrix.avx }}-...`
  → `build-${{ matrix.version }}-${{ matrix.cuda }}-...`

- Build step: `/p:Configuration=${{ matrix.version }}_${{ matrix.cuda }}_${{ matrix.avx }}`
  → `/p:Configuration=${{ matrix.version }}_${{ matrix.cuda }}`

- Artifact name: `hdtSMP64-${{ matrix.version }}-${{ matrix.cuda }}-${{ matrix.avx }}`
  → `hdtSMP64-${{ matrix.version }}-${{ matrix.cuda }}`

- Artifact path: `x64/${{ matrix.version }}_${{ matrix.cuda }}_${{ matrix.avx }}/hdtSMP64.dll`
  → `x64/${{ matrix.version }}_${{ matrix.cuda }}/hdtSMP64.dll`

### Step 2: Update justfile

In `justfile`:

```just
# Before:
default_config := "V1_6_1170_NOCUDA_AVX2"

# After:
default_config := "V1_6_1170_NOCUDA"
```

Find the `build-all-nocuda` recipe and replace:
```just
# Before:
build-all-nocuda:
    just build V1_6_1170_NOCUDA_NoAVX
    just build V1_6_1170_NOCUDA_AVX
    just build V1_6_1170_NOCUDA_AVX2
    just build V1_6_1170_NOCUDA_AVX512

# After:
build-all-nocuda:
    just build V1_6_1170_NOCUDA
```

Same pattern for `build-all-cuda`. Update the `configs` recipe to remove all AVX variant
entries and show the new clean list.

### Step 3: Update CLAUDE.md build config section

In `CLAUDE.md`, under "Build Configurations", update:

The **AVX Options** section should be removed or replaced with a note:
```markdown
**SIMD:** Highway SIMD provides automatic runtime dispatch (SSE4 → AVX2 → AVX512).
No separate build variants needed — one binary supports all CPUs.
```

Update example config names: `V1_6_659_CUDA_AVX2` → `V1_6_659_CUDA`.

### Step 4: Commit

```bash
git add .github/workflows/build.yml justfile CLAUDE.md
git commit -m "build: remove avx dimension from CI matrix, justfile, and docs

Single config per version/cuda variant. Highway handles runtime SIMD dispatch."
```

---

## Task 7: Final Verification

### Step 1: Full build verification — both CUDA and NOCUDA

```bash
# NOCUDA
powershell.exe -NoProfile -Command "& 'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/MSBuild/Current/Bin/amd64/MSBuild.exe' 'hdtSMP64.sln' '-p:Configuration=V1_6_1170_NOCUDA' '-p:Platform=x64' '-v:m' '-p:SolutionDir=$(pwd)/'"
```

Expected: Build succeeds.

### Step 2: Full test run

```bash
powershell.exe -NoProfile -Command "& 'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/MSBuild/Current/Bin/amd64/MSBuild.exe' 'tests/hdtSMP64_tests.vcxproj' '-p:Configuration=Release' '-p:Platform=x64' '-v:m'"
./tests/x64/tests/Release/hdtSMP64_tests.exe --reporter console
```

Expected: All tests pass.

### Step 3: Zero intrinsics check across entire codebase

```bash
grep -rn "_mm256\|__m256\|_mm512\|__m512\|_mm_dp_ps\|_mm_hadd_ps\|_mm_shuffle_ps" \
  hdtSMP64/hdtSkinnedMesh/ \
  --include="*.cpp" --include="*.h" \
  | grep -v "hdtHighway\|hdtSoA\|BulletCollision\|BulletDynamics\|LinearMath"
```

Expected: No output.

### Step 4: Config count verification

```bash
grep -c "ProjectConfiguration Include" hdtSMP64/hdtSMP64.vcxproj
```

Expected: 18 (or fewer if DEBUG configs were also trimmed).

### Step 5: Final commit if any fixups needed, then tag

```bash
git log --oneline -8
```

Review that commits tell a clean story. No fixup commits needed if previous tasks were
done correctly.

---

## Success Criteria Checklist

- [ ] Zero `_mm256`/`_mm512` intrinsics in non-Highway files
- [ ] Zero `_mm_dp_ps`/`_mm_hadd_ps`/`_mm_shuffle_ps` in non-Highway files
- [ ] All unit tests pass
- [ ] Build succeeds for `V1_6_1170_NOCUDA`
- [ ] 112 → 18 configurations in vcxproj
- [ ] CI build.yml updated (no `avx` matrix dimension)
- [ ] `justfile` default is `V1_6_1170_NOCUDA`
- [ ] `CLAUDE.md` docs updated
