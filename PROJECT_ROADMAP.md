# hdtSMP64 Project Enhancement Roadmap

**Generated:** 2026-01-07
**Purpose:** Identify gaps and improvement opportunities beyond performance optimization

---

## Current State Summary

| Area | Status | Gap Level |
|------|--------|-----------|
| Performance optimization | Documented (OPTIMIZATION_REPORT.md) | Low |
| Bug fixes | Documented (BUG_FIXES.md) | Low |
| Ecosystem research | Documented (ECOSYSTEM_RECOMMENDATIONS.md) | Low |
| Automated testing | **None** | Critical |
| CI/CD pipeline | **PR linting only** | High |
| Documentation | **Minimal** | High |
| Static analysis | **None** | Medium |
| Build reproducibility | **Manual deps** | Medium |
| Debug/diagnostics | **Basic logging** | Medium |

---

## Gap Analysis

### 1. Testing Infrastructure (Critical Gap)

**Current State:** Zero automated tests

**Impact:**
- Regressions go unnoticed until user reports
- Refactoring is risky
- No confidence in bug fix correctness

**Recommended Actions:**

#### 1.1 Unit Test Framework

```
Recommended: Catch2 or Google Test

tests/
├── unit/
│   ├── test_skinnedmeshbody.cpp
│   ├── test_constraintsolver.cpp
│   ├── test_actormanager.cpp
│   └── test_config.cpp
├── integration/
│   └── test_physics_system.cpp
└── CMakeLists.txt
```

**Priority test targets:**
| Component | Why |
|-----------|-----|
| `SkinnedMeshBody::internalUpdate()` | Hot path, easy to regress |
| `GroupConstraintSolver` | Complex parallel code |
| `ActorManager::setSkeletonsActive()` | Complex sorting/selection |
| XML config parsing | User-facing, error-prone |
| Collision filtering | Bug-002 was in this area |

#### 1.2 Physics Simulation Tests

```cpp
// Example: Determinism test
TEST_CASE("Physics simulation is deterministic") {
    auto world = createTestWorld();
    auto system = loadTestPhysicsSystem("cloth_basic.xml");

    // Run simulation
    std::vector<btVector3> positions1;
    for (int i = 0; i < 100; i++) {
        world->stepSimulation(1.0f/60.0f);
    }
    capturePositions(system, positions1);

    // Reset and run again
    system->resetTransformsToOriginal();
    std::vector<btVector3> positions2;
    for (int i = 0; i < 100; i++) {
        world->stepSimulation(1.0f/60.0f);
    }
    capturePositions(system, positions2);

    REQUIRE(positions1 == positions2);
}
```

#### 1.3 Benchmark Tests

```cpp
// Performance regression detection
BENCHMARK("Vertex skinning 10k vertices") {
    auto body = createTestBody(10000);
    return body->internalUpdate();
};

BENCHMARK("Constraint solving 1000 constraints") {
    auto solver = createTestSolver(1000);
    return solver->solveSingleIteration(...);
};
```

---

### 2. CI/CD Pipeline (High Gap)

**Current State:** Only PR title linting

**Missing:**
- Automated builds for all configurations
- Test execution
- Static analysis
- Release automation

**Recommended GitHub Actions Workflow:**

```yaml
# .github/workflows/build.yml
name: Build and Test

on:
  push:
    branches: [master]
  pull_request:
    branches: [master]

jobs:
  build:
    strategy:
      matrix:
        config:
          - { version: "SE", cuda: "NOCUDA", avx: "NoAVX" }
          - { version: "SE", cuda: "NOCUDA", avx: "AVX2" }
          - { version: "V1_6_659", cuda: "CUDA", avx: "AVX2" }
          - { version: "VR", cuda: "NOCUDA", avx: "AVX" }

    runs-on: windows-latest

    steps:
      - uses: actions/checkout@v4
        with:
          submodules: recursive

      - name: Setup MSVC
        uses: microsoft/setup-msbuild@v1

      - name: Setup CUDA
        if: matrix.config.cuda == 'CUDA'
        uses: Jimver/cuda-toolkit@v0.2.11
        with:
          cuda: '11.6.0'

      - name: Build
        run: |
          msbuild hdtSMP64.sln /p:Configuration=${{ matrix.config.version }}_${{ matrix.config.cuda }}_${{ matrix.config.avx }} /p:Platform=x64

      - name: Run Tests
        run: |
          ./build/tests/hdtSMP64_tests.exe

      - name: Upload Artifact
        uses: actions/upload-artifact@v3
        with:
          name: hdtSMP64-${{ matrix.config.version }}-${{ matrix.config.cuda }}-${{ matrix.config.avx }}
          path: build/*.dll

  static-analysis:
    runs-on: windows-latest
    steps:
      - uses: actions/checkout@v4

      - name: Run cppcheck
        run: |
          choco install cppcheck
          cppcheck --enable=all --error-exitcode=1 hdtSMP64/

      - name: Run clang-tidy
        run: |
          # clang-tidy checks
```

---

### 3. Documentation (High Gap)

**Current State:** Only CLAUDE.md and README.md

**Missing:**
- API documentation
- Architecture overview
- Contributor guide
- Physics XML schema documentation
- Troubleshooting guide

**Recommended Documentation Structure:**

```
docs/
├── README.md                    # Main docs index
├── ARCHITECTURE.md              # System architecture overview
├── CONTRIBUTING.md              # How to contribute
├── API/
│   ├── SkinnedMeshSystem.md
│   ├── ConstraintTypes.md
│   └── ConfigOptions.md
├── PHYSICS_XML_SCHEMA.md        # XML format documentation
├── TROUBLESHOOTING.md           # Common issues and solutions
├── PERFORMANCE_TUNING.md        # User-facing perf guide
└── DEVELOPMENT.md               # Dev environment setup
```

#### 3.1 Architecture Documentation

```markdown
# hdtSMP64 Architecture

## Component Overview

┌─────────────────────────────────────────────────────────────────┐
│                        SKSE Plugin Layer                        │
├─────────────────────────────────────────────────────────────────┤
│  Hooks.cpp          │  main.cpp           │  ActorManager.cpp  │
│  (Game events)      │  (Plugin entry)     │  (Actor tracking)  │
└─────────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Physics World Layer                         │
├─────────────────────────────────────────────────────────────────┤
│  SkyrimPhysicsWorld │  SkyrimSystem       │  SkyrimBone        │
│  (World management) │  (Per-actor system) │  (Bone physics)    │
└─────────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Skinned Mesh Layer                            │
├─────────────────────────────────────────────────────────────────┤
│  SkinnedMeshWorld   │  SkinnedMeshSystem  │  SkinnedMeshBody   │
│  (Custom Bullet)    │  (Constraint groups)│  (Collision mesh)  │
└─────────────────────────────────────────────────────────────────┘
                               │
                    ┌──────────┴──────────┐
                    ▼                     ▼
        ┌───────────────────┐   ┌───────────────────┐
        │   CPU Backend     │   │   CUDA Backend    │
        │  (SSE/AVX SIMD)   │   │  (GPU compute)    │
        └───────────────────┘   └───────────────────┘
```

#### 3.2 Physics XML Schema Documentation

```markdown
# Physics XML Schema

## Root Element: `<system>`

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | string | required | System identifier |
| `version` | int | 1 | Schema version |

## Bone Definition: `<bone>`

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | string | required | NiNode name to attach to |
| `mass` | float | 1.0 | Bone mass (kg) |
| `inertia` | vec3 | auto | Inertia tensor |
| `centerOfMass` | vec3 | (0,0,0) | Local COM offset |
| `linearDamping` | float | 0.1 | Linear velocity damping |
| `angularDamping` | float | 0.1 | Angular velocity damping |
| `margin` | float | 0.1 | Collision margin |

### Example

```xml
<system name="hair_physics">
  <bone name="hair_01" mass="0.5" linearDamping="0.2">
    <shape type="sphere" radius="0.1"/>
  </bone>

  <constraint type="ballsocket" bodyA="head" bodyB="hair_01">
    <pivot>(0, 0, -5)</pivot>
  </constraint>
</system>
```

---

### 4. Static Analysis Integration (Medium Gap)

**Current State:** None

**Recommended Tools:**

| Tool | Purpose | Integration |
|------|---------|-------------|
| **cppcheck** | General static analysis | CI + pre-commit |
| **clang-tidy** | Modern C++ linting | CI + IDE |
| **PVS-Studio** | Deep analysis (commercial) | Optional CI |
| **Address Sanitizer** | Runtime memory checking | Debug builds |
| **Thread Sanitizer** | Race condition detection | Debug builds |

#### 4.1 clang-tidy Configuration

```yaml
# .clang-tidy
Checks: >
  -*,
  bugprone-*,
  -bugprone-easily-swappable-parameters,
  cppcoreguidelines-*,
  -cppcoreguidelines-pro-type-reinterpret-cast,
  -cppcoreguidelines-pro-bounds-pointer-arithmetic,
  modernize-*,
  -modernize-use-trailing-return-type,
  performance-*,
  readability-*,
  -readability-magic-numbers

WarningsAsErrors: >
  bugprone-use-after-move,
  bugprone-dangling-handle

CheckOptions:
  - key: readability-identifier-naming.ClassCase
    value: CamelCase
  - key: readability-identifier-naming.FunctionCase
    value: camelBack
```

#### 4.2 Sanitizer Build Configuration

```xml
<!-- In .vcxproj for Debug builds -->
<PropertyGroup Condition="'$(Configuration)'=='Debug'">
  <EnableAddressSanitizer>true</EnableAddressSanitizer>
</PropertyGroup>
```

---

### 5. Build Reproducibility (Medium Gap)

**Current State:** Manual dependency management

**Issues:**
- SKSE, Bullet, Detours versions not locked
- Build instructions scattered
- Hard to onboard new contributors

**Recommended: vcpkg Manifest**

```json
// vcpkg.json
{
  "name": "hdtsmp64",
  "version": "1.50.4",
  "dependencies": [
    {
      "name": "bullet3",
      "version>=": "3.25"
    },
    {
      "name": "detours",
      "version>=": "4.0.1"
    }
  ],
  "overrides": [
    { "name": "bullet3", "version": "3.25" }
  ]
}
```

**Alternative: Git Submodules (Current)**

Formalize with version pinning:
```bash
# .gitmodules
[submodule "bullet3"]
    path = external/bullet3
    url = https://github.com/bulletphysics/bullet3.git
    branch = 3.25

[submodule "detours"]
    path = external/detours
    url = https://github.com/microsoft/Detours.git
    tag = v4.0.1
```

---

### 6. Debug/Diagnostic Features (Medium Gap)

**Current State:** Basic `_MESSAGE`/`_WARNING` logging

**Missing:**
- Runtime performance metrics
- Visual debug rendering
- Config hot-reload
- Physics state inspection

#### 6.1 Runtime Metrics System

```cpp
// Proposed: hdtMetrics.h
class PhysicsMetrics {
public:
    static PhysicsMetrics& instance();

    // Per-frame metrics
    struct FrameMetrics {
        float totalTime;
        float collisionTime;
        float solverTime;
        float skinningTime;
        int activeSkeletons;
        int totalConstraints;
        int collisionPairs;
    };

    void beginFrame();
    void endFrame();
    void logCollision(float time);
    void logSolver(float time);

    // Rolling averages
    FrameMetrics getAverageMetrics(int frames = 60) const;

    // Console command: "smp metrics"
    void printToConsole() const;

private:
    std::deque<FrameMetrics> m_history;
    FrameMetrics m_current;
};
```

#### 6.2 Visual Debug Rendering

```cpp
// Proposed: hdtDebugRenderer.h
class DebugRenderer {
public:
    void setEnabled(bool enabled);

    // Render options
    void showBones(bool show);
    void showConstraints(bool show);
    void showCollisionShapes(bool show);
    void showAABBs(bool show);
    void showVelocities(bool show);

    // Called from frame event
    void render(NiCamera* camera);

private:
    void drawBone(const SkinnedMeshBone* bone);
    void drawConstraint(const btTypedConstraint* constraint);
    void drawCollisionShape(const SkinnedMeshBody* body);
};
```

#### 6.3 Config Hot-Reload

```cpp
// Proposed addition to config.cpp
class ConfigWatcher {
public:
    void start();
    void stop();

    // Check for changes each frame
    void update();

private:
    std::filesystem::file_time_type m_lastModified;
    std::thread m_watchThread;
    std::atomic<bool> m_configChanged{false};
};

// In frame event:
if (ConfigWatcher::instance().hasChanged()) {
    loadConfig();  // Reload without restart
    _MESSAGE("Config reloaded");
}
```

---

### 7. Code Modernization Opportunities

**Current:** Mixed C++11/14/17 patterns

**Modernization candidates:**

| Pattern | Current | Modern Alternative |
|---------|---------|-------------------|
| Raw loops | `for (int i = 0; ...)` | Range-based for, algorithms |
| Raw pointers in containers | `std::vector<T*>` | `std::vector<Ref<T>>` |
| Manual RAII | `new`/`delete` pairs | `std::unique_ptr` |
| Callback functions | Function pointers | `std::function` or templates |
| String formatting | `sprintf_s` | `std::format` (C++20) |
| Optional values | Sentinel values | `std::optional` |

#### Example Modernization

```cpp
// Before (hdtSkinnedMeshSystem.cpp:15-21)
void SkinnedMeshSystem::readTransform(float timeStep)
{
    for (int i = 0; i < m_bones.size(); ++i)
        m_bones[i]->readTransform(timeStep);
    // ...
}

// After
void SkinnedMeshSystem::readTransform(float timeStep)
{
    for (auto& bone : m_bones)
        bone->readTransform(timeStep);
    // ...
}
```

---

### 8. Error Handling Improvements

**Current State:** 17 try/catch blocks, mostly around XML parsing

**Issues:**
- Silent failures in many places
- No structured error reporting
- Crashes instead of graceful degradation

**Recommended:**

#### 8.1 Result Type for Fallible Operations

```cpp
// Proposed: hdtResult.h
template<typename T, typename E = std::string>
class Result {
public:
    static Result Ok(T value);
    static Result Err(E error);

    bool isOk() const;
    bool isErr() const;

    T& value();
    E& error();

    // Monadic operations
    template<typename F>
    auto map(F&& f) -> Result<decltype(f(std::declval<T>())), E>;

    template<typename F>
    auto andThen(F&& f) -> decltype(f(std::declval<T>()));
};

// Usage
Result<Ref<SkyrimSystem>, std::string> SkyrimSystemCreator::createSystem(...) {
    auto xmlResult = parseXml(path);
    if (xmlResult.isErr())
        return Result::Err("Failed to parse: " + xmlResult.error());

    // ...
    return Result::Ok(system);
}
```

#### 8.2 Graceful Degradation

```cpp
// Instead of crashing on CUDA init failure:
if (!CudaInterface::instance()->hasCuda()) {
    _WARNING("CUDA unavailable, falling back to CPU");
    // Continue with CPU path
}

// Instead of crashing on bad physics XML:
auto result = SkyrimSystemCreator().createSystem(...);
if (result.isErr()) {
    _ERROR("Failed to create physics system: %s", result.error().c_str());
    _ERROR("Armor will have no physics");
    // Don't add broken system, but don't crash
}
```

---

## Priority Matrix

| Enhancement | Impact | Effort | Priority |
|-------------|--------|--------|----------|
| Unit tests for hot paths | High | Medium | **P0** |
| CI build pipeline | High | Medium | **P0** |
| Architecture docs | Medium | Low | **P1** |
| clang-tidy integration | Medium | Low | **P1** |
| Address Sanitizer builds | High | Low | **P1** |
| Physics XML schema docs | Medium | Medium | **P2** |
| Runtime metrics | Medium | Medium | **P2** |
| Config hot-reload | Low | Medium | **P3** |
| Visual debug renderer | Low | High | **P3** |
| Result type error handling | Medium | High | **P3** |

---

## Quick Start Checklist

### This Week
- [ ] Add Catch2 test framework
- [ ] Write tests for BUG-002 fix (collision filtering)
- [ ] Set up basic GitHub Actions build workflow
- [ ] Enable Address Sanitizer in debug builds

### This Month
- [ ] Achieve 50% test coverage of hot paths
- [ ] Add cppcheck to CI
- [ ] Write ARCHITECTURE.md
- [ ] Document physics XML schema

### This Quarter
- [ ] Full CI/CD with release automation
- [ ] Runtime metrics system
- [ ] Contributor documentation
- [ ] Config hot-reload

---

## Conclusion

The hdtSMP64 project has solid core functionality but lacks the supporting infrastructure expected of a mature open-source project. The highest-impact improvements are:

1. **Automated testing** - Prevents regressions, enables confident refactoring
2. **CI/CD pipeline** - Catches issues early, automates releases
3. **Documentation** - Reduces contributor friction, helps users

These investments will compound over time, making future development faster and more reliable.

---

*Generated by Claude Code project analysis*
