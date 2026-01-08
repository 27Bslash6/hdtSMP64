#pragma once

// hdtTracy.h - Tracy profiler integration for hdtSMP64
//
// Usage:
//   Define HDT_TRACY_ENABLE in preprocessor definitions to enable profiling.
//   When disabled, all macros become no-ops with zero overhead.
//
// Macros:
//   HDT_ZONE_SCOPED          - Profile current function
//   HDT_ZONE_SCOPED_N(name)  - Profile with custom name
//   HDT_FRAME_MARK           - Mark frame boundary
//   HDT_FRAME_MARK_N(name)   - Mark named frame boundary
//   HDT_PLOT(name, value)    - Plot a value over time
//
// Build with Tracy:
//   1. Add HDT_TRACY_ENABLE to preprocessor definitions
//   2. Include $(SolutionDir)external\tracy\public in include paths
//   3. Link with ws2_32.lib, dbghelp.lib

#ifdef HDT_TRACY_ENABLE

// Tracy requires TRACY_ENABLE to be defined before including
#ifndef TRACY_ENABLE
#define TRACY_ENABLE
#endif

// Optional: Enable call stack capture (higher overhead but more detail)
// #define TRACY_CALLSTACK 16

// Optional: Enable frame image capture
// #define TRACY_NO_FRAME_IMAGE

// Save and undefine CUDA macro - conflicts with Tracy's GpuContextType::CUDA enum
#ifdef CUDA
#pragma push_macro("CUDA")
#undef CUDA
#define HDT_RESTORE_CUDA
#endif

#include <tracy/Tracy.hpp>

// Restore CUDA macro
#ifdef HDT_RESTORE_CUDA
#pragma pop_macro("CUDA")
#undef HDT_RESTORE_CUDA
#endif

// Zone profiling macros
#define HDT_ZONE_SCOPED           ZoneScoped
#define HDT_ZONE_SCOPED_N(name)   ZoneScopedN(name)
#define HDT_ZONE_TEXT(text, len)  ZoneText(text, len)
#define HDT_ZONE_VALUE(value)     ZoneValue(value)

// Frame markers
#define HDT_FRAME_MARK            FrameMark
#define HDT_FRAME_MARK_N(name)    FrameMarkNamed(name)
#define HDT_FRAME_MARK_START(name) FrameMarkStart(name)
#define HDT_FRAME_MARK_END(name)   FrameMarkEnd(name)

// Value plotting
#define HDT_PLOT(name, value)     TracyPlot(name, value)
#define HDT_PLOT_CONFIG(name, type, step, fill, color) \
    TracyPlotConfig(name, type, step, fill, color)

// Memory tracking (optional - use with custom allocators)
#define HDT_ALLOC(ptr, size)      TracyAlloc(ptr, size)
#define HDT_FREE(ptr)             TracyFree(ptr)

// Lock profiling
#define HDT_LOCKABLE(type, var)   TracyLockable(type, var)
#define HDT_LOCKABLE_N(type, var, name) TracyLockableN(type, var, name)

// Message logging
#define HDT_MESSAGE(text, len)    TracyMessage(text, len)
#define HDT_MESSAGE_L(text)       TracyMessageL(text)

// Connection status
#define HDT_IS_CONNECTED          TracyIsConnected

#else // HDT_TRACY_ENABLE not defined

// No-op versions - zero overhead when profiling disabled
#define HDT_ZONE_SCOPED           (void)0
#define HDT_ZONE_SCOPED_N(name)   (void)0
#define HDT_ZONE_TEXT(text, len)  (void)0
#define HDT_ZONE_VALUE(value)     (void)0

#define HDT_FRAME_MARK            (void)0
#define HDT_FRAME_MARK_N(name)    (void)0
#define HDT_FRAME_MARK_START(name) (void)0
#define HDT_FRAME_MARK_END(name)   (void)0

#define HDT_PLOT(name, value)     (void)0
#define HDT_PLOT_CONFIG(name, type, step, fill, color) (void)0

#define HDT_ALLOC(ptr, size)      (void)0
#define HDT_FREE(ptr)             (void)0

#define HDT_LOCKABLE(type, var)   type var
#define HDT_LOCKABLE_N(type, var, name) type var

#define HDT_MESSAGE(text, len)    (void)0
#define HDT_MESSAGE_L(text)       (void)0

#define HDT_IS_CONNECTED          false

#endif // HDT_TRACY_ENABLE
