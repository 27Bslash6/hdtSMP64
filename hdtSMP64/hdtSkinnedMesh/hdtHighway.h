#pragma once

// Google Highway SIMD Abstraction Layer for hdtSMP64
// Provides portable SIMD operations that auto-dispatch to SSE4/AVX2/AVX-512
//
// Usage:
//   #include "hdtHighway.h"
//
//   // In implementation file (.cpp):
//   #undef HWY_TARGET_INCLUDE
//   #define HWY_TARGET_INCLUDE "myfile.cpp"
//   #include <hwy/foreach_target.h>
//   #include <hwy/highway.h>
//
//   HWY_BEFORE_NAMESPACE();
//   namespace hdt {
//   namespace HWY_NAMESPACE {
//       void MyBatchFunction(...) { /* Highway code */ }
//   }
//   }
//   HWY_AFTER_NAMESPACE();
//
//   // Export with dynamic dispatch:
//   #if HWY_ONCE
//   namespace hdt {
//       HWY_EXPORT(MyBatchFunction);
//       void CallMyBatchFunction(...) {
//           HWY_DYNAMIC_DISPATCH(MyBatchFunction)(...);
//       }
//   }
//   #endif

#ifndef HDT_HIGHWAY_H
#define HDT_HIGHWAY_H

// MSVC-specific configuration
#ifdef _MSC_VER
// Disable overly aggressive MSVC warnings for Highway headers
#pragma warning(push)
#pragma warning(disable : 4244) // conversion, possible loss of data
#pragma warning(disable : 4267) // conversion from size_t
#pragma warning(disable : 4324) // structure was padded due to alignment
#pragma warning(disable : 4505) // unreferenced local function removed
#endif

// Highway disables AVX-512 on MSVC by default due to old compiler bugs.
// VS2022 17.10+ (MSVC 19.40+) has fixed these issues, so we re-enable.
// See: https://github.com/Mysticial/Flops/issues/16
#if defined(_MSC_VER) && (_MSC_VER >= 1940)
#define HWY_BROKEN_MSVC 0
#endif

// Target configuration - enable all x86 SIMD we care about
// Highway will auto-detect and dispatch at runtime
#ifndef HWY_DISABLED_TARGETS
// Keep EMU128 and SCALAR as fallbacks, but prefer native SIMD
#define HWY_DISABLED_TARGETS 0
#endif

// Baseline target selection - compile SSE4.1 as baseline for modern CPUs
#ifndef HWY_BASELINE_TARGETS
#define HWY_BASELINE_TARGETS HWY_SSE4
#endif

// Include Highway core
#include <hwy/highway.h>

// Namespace alias for convenience
namespace hn = hwy::HWY_NAMESPACE;

// Common type tags used throughout hdtSMP64
namespace hdt
{
	namespace highway
	{

		// Scalable float vector (4 lanes on SSE, 8 on AVX2, 16 on AVX-512)
		using FloatTag = hn::ScalableTag<float>;

		// Fixed-width tags for specific use cases
		using Float4Tag = hn::CappedTag<float, 4>; // Always 4 lanes (SSE-width)
		using Float8Tag = hn::CappedTag<float, 8>; // Up to 8 lanes (AVX2-width)

		// Integer tags for indices
		using Int32Tag = hn::ScalableTag<int32_t>;
		using UInt32Tag = hn::ScalableTag<uint32_t>;

		// Get lane count at runtime
		inline size_t GetFloatLanes()
		{
			return hn::Lanes(FloatTag());
		}

	} // namespace highway
} // namespace hdt

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif // HDT_HIGHWAY_H
