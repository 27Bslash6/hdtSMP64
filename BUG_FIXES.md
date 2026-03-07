# hdtSMP64 Memory Bug Fixes

This document details memory-related bugs identified in the hdtSMP64 codebase through static analysis, along with recommended fixes.

---

## Critical Severity

### BUG-001: Memory Leak in Serializer::ReadData

**File:** `hdtSMP64/hdtSerialization.h`
**Lines:** 97-106
**Type:** Memory Leak
**Impact:** Memory leaks on every save game load. Accumulates over time.

#### Current Code
```cpp
template<class _Storage_t, class _Stream_t>
inline void Serializer<_Storage_t, _Stream_t>::ReadData(SKSESerializationInterface* intfc, UInt32 length)
{
    char* data_block = new char[length];
    intfc->ReadRecordData(data_block, length);
    std::string s_data(data_block, length);
    //_MESSAGE("Reading Data: %s", s_data.c_str());
    _Stream_t _stream; _stream << s_data;
    this->Deserialize(_stream);
}
```

#### Problem
`data_block` is allocated with `new char[length]` but never deallocated.

#### Fixed Code
```cpp
template<class _Storage_t, class _Stream_t>
inline void Serializer<_Storage_t, _Stream_t>::ReadData(SKSESerializationInterface* intfc, UInt32 length)
{
    char* data_block = new char[length];
    intfc->ReadRecordData(data_block, length);
    std::string s_data(data_block, length);
    delete[] data_block;  // FIX: Free allocated memory
    //_MESSAGE("Reading Data: %s", s_data.c_str());
    _Stream_t _stream; _stream << s_data;
    this->Deserialize(_stream);
}
```

#### Alternative Fix (RAII)
```cpp
template<class _Storage_t, class _Stream_t>
inline void Serializer<_Storage_t, _Stream_t>::ReadData(SKSESerializationInterface* intfc, UInt32 length)
{
    std::unique_ptr<char[]> data_block(new char[length]);
    intfc->ReadRecordData(data_block.get(), length);
    std::string s_data(data_block.get(), length);
    _Stream_t _stream; _stream << s_data;
    this->Deserialize(_stream);
}
```

---

## High Severity

### BUG-002: Copy-Paste Bug in CollisionDispatcher::needsCollision

**File:** `hdtSMP64/hdtSkinnedMesh/hdtDispatcher.cpp`
**Lines:** 60-61
**Type:** Logic Error / Incorrect Variable
**Impact:** Incorrect collision detection between bones, potential crashes or physics glitches.

#### Current Code
```cpp
auto rb0 = static_cast<SkinnedMeshBone*>(body0->getUserPointer());
auto rb1 = static_cast<SkinnedMeshBone*>(body0->getUserPointer());  // BUG: body0 should be body1

return rb0->canCollideWith(rb1) && rb1->canCollideWith(rb0);
```

#### Problem
Both `rb0` and `rb1` are assigned from `body0`. This is clearly a copy-paste error where `body1` should be used for `rb1`.

#### Fixed Code
```cpp
auto rb0 = static_cast<SkinnedMeshBone*>(body0->getUserPointer());
auto rb1 = static_cast<SkinnedMeshBone*>(body1->getUserPointer());  // FIX: Use body1

return rb0->canCollideWith(rb1) && rb1->canCollideWith(rb0);
```

---

### BUG-003: Raw Pointers in Bone Collision Lists

**File:** `hdtSMP64/hdtSkinnedMesh/hdtSkinnedMeshBody.h`
**Lines:** 104-105
**Type:** Dangling Pointer Risk
**Impact:** Potential crash if bones are destroyed while still referenced.

#### Current Code
```cpp
std::vector<SkinnedMeshBone*> m_canCollideWithBones;
std::vector<SkinnedMeshBone*> m_noCollideWithBones;
```

#### Problem
Raw pointers are stored without ownership semantics. If a `SkinnedMeshBone` is destroyed while still in these vectors, subsequent access causes undefined behavior.

#### Recommended Fix
```cpp
std::vector<Ref<SkinnedMeshBone>> m_canCollideWithBones;
std::vector<Ref<SkinnedMeshBone>> m_noCollideWithBones;
```

#### Alternative
If reference counting overhead is a concern, document that bones must outlive bodies and add validation in debug builds:
```cpp
#ifdef _DEBUG
bool validateBoneReferences() const;
#endif
```

---

### BUG-004: Global Serializer List Never Cleaned

**File:** `hdtSMP64/hdtSerialization.h`
**Lines:** 13, 69-70
**Type:** Memory Leak / Dangling Pointer
**Impact:** If serializers are created dynamically, list accumulates invalid pointers.

#### Current Code
```cpp
extern std::vector<SerializerBase*> g_SerializerList;

// In constructor:
Serializer() {
    g_SerializerList.push_back(this);
};

// Destructor does NOT remove from list:
~Serializer() {};
```

#### Problem
Serializers register themselves but never unregister.

#### Fixed Code
```cpp
Serializer() {
    g_SerializerList.push_back(this);
};

~Serializer() {
    auto it = std::find(g_SerializerList.begin(), g_SerializerList.end(), this);
    if (it != g_SerializerList.end()) {
        g_SerializerList.erase(it);
    }
};
```

---

## Medium Severity

### BUG-005: CUDA Buffer Pool Not Thread Safe

**File:** `hdtSMP64/hdtSkinnedMesh/hdtCudaInterface.cpp`
**Lines:** 236-240
**Type:** Race Condition
**Impact:** Data corruption or crashes when multiple threads access CUDA buffer pool.

#### Current Code
```cpp
// FIXME: Not thread safe
static CudaBufferPool* instance()
{
    return &s_pools[cuGetDevice()];
}
```

#### Problem
The `s_pools` map access is not protected. Multiple threads calling `instance()` simultaneously could cause race conditions.

#### Fixed Code
```cpp
static CudaBufferPool* instance()
{
    static std::mutex s_mutex;
    std::lock_guard<std::mutex> lock(s_mutex);
    return &s_pools[cuGetDevice()];
}
```

#### Better Fix (C++11 thread-safe static initialization)
```cpp
static CudaBufferPool* instance()
{
    // Thread-safe in C++11+
    thread_local CudaBufferPool* cached = nullptr;
    thread_local int cachedDevice = -1;

    int device = cuGetDevice();
    if (cachedDevice != device) {
        static std::mutex s_mutex;
        std::lock_guard<std::mutex> lock(s_mutex);
        cached = &s_pools[device];
        cachedDevice = device;
    }
    return cached;
}
```

---

### BUG-006: Detached Weather Thread Without Lifecycle Management

**File:** `hdtSMP64/main.cpp`
**Lines:** 685-689
**Type:** Use-After-Free Risk
**Impact:** Potential crash on game exit if thread accesses freed resources.

#### Current Code
```cpp
if (hdt::SkyrimPhysicsWorld::get()->m_enableWind) {
    _MESSAGE("Wind enabled");
    std::thread t(hdt::WeatherCheck);
    t.detach();
}
```

#### Problem
The thread is detached without any mechanism to stop it on shutdown. If the DLL is unloaded while the thread is running, it will crash.

#### Fixed Code
```cpp
// At file scope or in a manager class:
static std::atomic<bool> g_weatherThreadRunning{false};
static std::thread g_weatherThread;

// In initialization:
if (hdt::SkyrimPhysicsWorld::get()->m_enableWind) {
    _MESSAGE("Wind enabled");
    g_weatherThreadRunning = true;
    g_weatherThread = std::thread([]() {
        while (g_weatherThreadRunning) {
            hdt::WeatherCheckOnce();  // Modified to do one check
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
}

// In shutdown handler:
void onShutdown() {
    g_weatherThreadRunning = false;
    if (g_weatherThread.joinable()) {
        g_weatherThread.join();
    }
}
```

---

## Low Severity

### BUG-007: MergeBuffer::release() Double-Free Risk

**File:** `hdtSMP64/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.h`
**Line:** 94
**Type:** Double-Free Risk
**Impact:** Crash if `release()` called twice.

#### Current Code
```cpp
void release() { if (buffer) delete[] buffer; }
```

#### Problem
After deletion, `buffer` is not set to nullptr. A second call to `release()` would double-free.

#### Fixed Code
```cpp
void release() {
    if (buffer) {
        delete[] buffer;
        buffer = nullptr;
    }
}
```

---

### BUG-008: Raw Heap Allocation Not Exception-Safe

**File:** `hdtSMP64/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.cpp`
**Lines:** 770, 786
**Type:** Memory Leak on Exception
**Impact:** Memory leak if exception thrown between allocation and deallocation.

#### Current Code
```cpp
auto collision = new CollisionResult[MaxCollisionCount];
// ... processing that could throw ...
delete[] collision;
```

#### Problem
If an exception is thrown between `new` and `delete[]`, the memory leaks.

#### Fixed Code
```cpp
auto collision = std::make_unique<CollisionResult[]>(MaxCollisionCount);
// ... processing ...
// No explicit delete needed - RAII handles cleanup
```

Or keep raw pointer but use scope guard:
```cpp
auto collision = new CollisionResult[MaxCollisionCount];
auto guard = finally([collision]() { delete[] collision; });
// ... processing ...
```

---

## Design Recommendations

### 1. Standardize Smart Pointer Usage

The codebase mixes `Ref<T>`, `NiPointer<T>`, `std::shared_ptr<T>`, and raw pointers. Consider:
- Use `Ref<T>` for all plugin-owned objects
- Use `NiPointer<T>` only for game engine objects
- Avoid raw pointers in containers

### 2. Add Weak Reference Support

The `Ref<T>` template lacks weak reference support. For breaking circular references:
```cpp
template<typename T>
class WeakRef {
    T* m_ptr;
    RefCounter* m_counter;
public:
    Ref<T> lock() const;
    bool expired() const;
};
```

### 3. Enable Address Sanitizer in Debug Builds

Add to debug build configurations:
```xml
<AdditionalOptions>/fsanitize=address %(AdditionalOptions)</AdditionalOptions>
```

### 4. Memory Leak Detection CI

Consider adding DrMemory or Visual Studio memory profiler runs to CI/CD pipeline.

---

## Testing Checklist

- [ ] Load multiple save games and monitor memory with Task Manager
- [ ] Enable/disable CUDA multiple times during gameplay
- [ ] Enter/exit areas with many NPCs repeatedly
- [ ] Open/close RaceMenu multiple times
- [ ] Fast travel between distant locations
- [ ] Exit game cleanly and check for crash in wind thread

---

## Version History

| Date | Author | Changes |
|------|--------|---------|
| 2026-01-07 | Claude Code Analysis | Initial bug identification |
