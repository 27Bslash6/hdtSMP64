# Plan: ImGui Debug Overlay for hdtSMP64

> *Toggleable debug overlay showing real-time physics performance stats.*

## Table of Contents

- [Summary](#summary)
- [Data to Display](#data-to-display)
- [Implementation Approach](#implementation-approach-imgui--d3d11)
- [Implementation Steps](#implementation-steps)
- [Verification](#verification)
- [Files to Modify](#files-to-modify)
- [Notes](#notes)

---

## Summary

Add toggleable debug overlay showing physics performance stats via `smp overlay` console command.

## Data to Display

| Metric | Description |
|--------|-------------|
| Physics status | ACTIVE/DISABLED |
| Main thread time | Time spent on main thread (ms) |
| Async step time | Async physics step duration (ms) |
| Total SMP time | Combined time + theoretical max FPS |
| Skeleton count | active / max / tracked |
| CUDA status | GPU acceleration state (if applicable) |

---

## Implementation Approach: ImGui + D3D11

### New Files
| File | Purpose |
|------|---------|
| `hdtSMP64/hdtImGui.h` | Wrapper header with conditional macros |
| `hdtSMP64/hdtImGui.cpp` | Present hook, ImGui init, render loop |
| `hdtSMP64/hdtOverlay.h` | Overlay window declaration |
| `hdtSMP64/hdtOverlay.cpp` | UI drawing code |
| `hdtSMP64/ImGui.props` | VS PropertySheet for opt-in builds |
| `external/imgui/` | Git submodule |

### Key Integration Points

> [!NOTE]
> Hook into existing Detours transaction for minimal code changes.

1. **Hooks.cpp** - Add `hookPresent()` within existing Detours transaction
2. **main.cpp** - Add `smp overlay` case to SMPDebug_Execute (~line 475)
3. **hdtSkyrimPhysicsWorld.cpp** - Call `updateOverlayMetrics()` after `m_tasks.wait()` in FrameSyncEvent

### Present Hook Strategy
- Get swap chain from `BSRenderManager::GetSingleton()->swapChain`
- Hook vtable index 8 (Present) via Detours
- Lazy-init ImGui on first Present call
- Render before calling original Present

### Thread Safety

> [!WARNING]
> Physics runs async; can't read metrics directly from render thread.

**Solution**: Atomic snapshot struct (`OverlayMetrics`) updated in FrameSyncEvent after `m_tasks.wait()`

### Build Configuration

| Build Type | Overlay | Overhead |
|------------|---------|----------|
| Default | Disabled | Zero |
| Debug | Enabled via `ImGui.props` | Minimal |

> [!TIP]
> Pattern follows existing `Tracy.props` for consistency.

---

## Implementation Steps

### Step 1: Add ImGui submodule
```bash
git submodule add https://github.com/ocornut/imgui.git external/imgui
cd external/imgui && git checkout v1.91.6
```

### Step 2: Create ImGui.props
Property sheet defining include paths, preprocessor, and ImGui source files.

### Step 3: Create hdtImGui.h/cpp
- Conditional compilation wrapper (like hdtTracy.h)
- Present hook function
- ImGui initialization using BSRenderManager's D3D11 device/context
- Render loop calling overlay draw

### Step 4: Create hdtOverlay.h/cpp
- `OverlayMetrics` atomic struct for thread-safe data
- `updateOverlayMetrics()` function called from main thread
- `OverlayWindow::draw()` ImGui UI code

### Step 5: Integrate hooks
- Add `hookPresent()`/`unhookPresent()` to Hooks.cpp
- Add "overlay" command to main.cpp
- Add metrics update call to hdtSkyrimPhysicsWorld.cpp

### Step 6: Update vcxproj
- Add new source files
- Add conditional import of ImGui.props

---

## Verification
1. Build with `just build` (default - no overlay, should compile cleanly)
2. Build with ImGui.props imported (verify overlay code compiles)
3. In-game: `smp overlay` toggles window
4. Verify metrics update in real-time
5. Test alt-tab / resolution change doesn't crash (RTV recreation)

---

## Files to Modify
- `hdtSMP64/Hooks.cpp` - hookPresent in transaction
- `hdtSMP64/main.cpp` - "overlay" command case
- `hdtSMP64/hdtSkyrimPhysicsWorld.cpp` - metrics update call
- `hdtSMP64/hdtSMP64.vcxproj` - new files + conditional import
- `.gitmodules` - imgui submodule

---

## Notes

> [!IMPORTANT]
> VR builds require special handling - wrap with `#ifndef SKYRIMVR` (different render path).

- No input hooking needed - overlay is display-only
- Follows Tracy integration pattern for consistency

---

<div align="center">

*For profiling without overlay, see Tracy integration in [ARCHITECTURE.md](ARCHITECTURE.md)*

</div>
