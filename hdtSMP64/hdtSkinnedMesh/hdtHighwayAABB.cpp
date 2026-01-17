// Highway-accelerated batch AABB collision detection
//
// This file uses Highway's multi-target compilation pattern to generate
// optimized code for SSE4, AVX2, and AVX-512 at compile time, with runtime
// dispatch to the best available target.
//
// The algorithm matches Aabb::collideWith() exactly:
//   collision = !( any(ref.max < cand.min) || any(cand.max < ref.min) )
// where "any" is true if any of X,Y,Z components satisfy the condition.

// clang-format off
// Highway multi-target include pattern - order is critical

// Highway disables AVX-512 on MSVC by default due to old compiler bugs.
// VS2022 17.10+ (MSVC 19.40+) has fixed these issues, so we re-enable.
// Must be defined BEFORE any Highway headers.
#if defined(_MSC_VER) && (_MSC_VER >= 1940)
#define HWY_BROKEN_MSVC 0
#endif

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "hdtHighwayAABB.cpp"
#include <hwy/foreach_target.h>  // Re-includes this file for each target
#include <hwy/highway.h>
// clang-format on

#include "hdtAABB.h"
#include "hdtHighway.h"
#include "hdtTracy.h"

#include <cstring>

HWY_BEFORE_NAMESPACE();

namespace hdt
{
	namespace HWY_NAMESPACE
	{

		namespace hn = hwy::HWY_NAMESPACE;

		// Extract X,Y,Z components from an __m128 AABB bound
		// Note: __m128 layout is [x, y, z, w] where w is unused for AABB
		HWY_INLINE void ExtractXYZ(const __m128& v, float& x, float& y, float& z)
		{
			// Use aligned array to avoid potential issues with direct member access
			alignas(16) float tmp[4];
			_mm_store_ps(tmp, v);
			x = tmp[0];
			y = tmp[1];
			z = tmp[2];
		}

		// Batch collision implementation - processes AABBs in SIMD-width batches
		//
		// Strategy: Since AABBs are AoS (each AABB is a pair of __m128), we process
		// by gathering X components, Y components, Z components separately for N AABBs,
		// then perform the collision test in parallel.
		//
		// The collision test for a single AABB pair is:
		//   separated = (ref.max.x < cand.min.x) || (cand.max.x < ref.min.x) ||
		//               (ref.max.y < cand.min.y) || (cand.max.y < ref.min.y) ||
		//               (ref.max.z < cand.min.z) || (cand.max.z < ref.min.z)
		//   collision = !separated
		//
		// In SIMD, for N candidates simultaneously:
		//   Load: candMinX[N], candMinY[N], candMinZ[N], candMaxX[N], candMaxY[N], candMaxZ[N]
		//   Compare: sepX = (refMaxX < candMinX) | (candMaxX < refMinX)
		//            sepY = (refMaxY < candMinY) | (candMaxY < refMinY)
		//            sepZ = (refMaxZ < candMinZ) | (candMaxZ < refMinZ)
		//   Result: separated = sepX | sepY | sepZ
		//           collision = !separated (i.e., mask bit NOT set)
		size_t batchCollideWithImpl(const Aabb& ref, const Aabb* candidates, size_t count, uint64_t* resultBits)
		{
			HDT_ZONE_SCOPED_N("Highway::batchCollideWith");

			if (count == 0)
				return 0;

			const hn::ScalableTag<float> d;
			const size_t N = hn::Lanes(d);

			// Extract reference AABB components
			float refMinX, refMinY, refMinZ;
			float refMaxX, refMaxY, refMaxZ;
			ExtractXYZ(ref.m_min, refMinX, refMinY, refMinZ);
			ExtractXYZ(ref.m_max, refMaxX, refMaxY, refMaxZ);

			// Broadcast reference bounds to SIMD registers
			const auto vRefMinX = hn::Set(d, refMinX);
			const auto vRefMinY = hn::Set(d, refMinY);
			const auto vRefMinZ = hn::Set(d, refMinZ);
			const auto vRefMaxX = hn::Set(d, refMaxX);
			const auto vRefMaxY = hn::Set(d, refMaxY);
			const auto vRefMaxZ = hn::Set(d, refMaxZ);

			size_t totalCollisions = 0;
			size_t i = 0;

			// Temporary arrays for gathering AABB components
			// Use stack allocation with max reasonable batch size (AVX-512 = 16 floats)
			alignas(64) float candMinX[16];
			alignas(64) float candMinY[16];
			alignas(64) float candMinZ[16];
			alignas(64) float candMaxX[16];
			alignas(64) float candMaxY[16];
			alignas(64) float candMaxZ[16];

			// For storing the separation result to extract mask
			alignas(64) float separatedArr[16];

			// Process full SIMD-width batches
			// Note: N is typically 4 (SSE4), 8 (AVX2), or 16 (AVX-512)
			const size_t batchSize = (N <= 16) ? N : 16; // Cap at 16 for stack arrays

			for (; i + batchSize <= count; i += batchSize) {
				// Gather candidate AABB components into SoA layout
				for (size_t j = 0; j < batchSize; ++j) {
					const Aabb& cand = candidates[i + j];
					ExtractXYZ(cand.m_min, candMinX[j], candMinY[j], candMinZ[j]);
					ExtractXYZ(cand.m_max, candMaxX[j], candMaxY[j], candMaxZ[j]);
				}

				// Load candidate bounds into SIMD registers
				const auto vCandMinX = hn::Load(d, candMinX);
				const auto vCandMinY = hn::Load(d, candMinY);
				const auto vCandMinZ = hn::Load(d, candMinZ);
				const auto vCandMaxX = hn::Load(d, candMaxX);
				const auto vCandMaxY = hn::Load(d, candMaxY);
				const auto vCandMaxZ = hn::Load(d, candMaxZ);

				// Compute separation on each axis
				// sepAxis = (refMax < candMin) | (candMax < refMin)
				// Using Lt (less than) to match original _mm_cmplt_ps behavior

				// X axis separation
				const auto sepX1 = hn::Lt(vRefMaxX, vCandMinX); // refMax.x < candMin.x
				const auto sepX2 = hn::Lt(vCandMaxX, vRefMinX); // candMax.x < refMin.x
				const auto sepX = hn::Or(sepX1, sepX2);

				// Y axis separation
				const auto sepY1 = hn::Lt(vRefMaxY, vCandMinY);
				const auto sepY2 = hn::Lt(vCandMaxY, vRefMinY);
				const auto sepY = hn::Or(sepY1, sepY2);

				// Z axis separation
				const auto sepZ1 = hn::Lt(vRefMaxZ, vCandMinZ);
				const auto sepZ2 = hn::Lt(vCandMaxZ, vRefMinZ);
				const auto sepZ = hn::Or(sepZ1, sepZ2);

				// Combined separation: any axis separated means no collision
				const auto separated = hn::Or(hn::Or(sepX, sepY), sepZ);

				// Convert mask to float values: 1.0 if separated, 0.0 if collision
				// We use IfThenElse to convert the mask to numeric values
				const auto one = hn::Set(d, 1.0f);
				const auto zero = hn::Zero(d);
				const auto separatedFloat = hn::IfThenElse(separated, one, zero);

				// Store to memory to extract the mask
				hn::Store(separatedFloat, d, separatedArr);

				// Build collision bitmask: bit set if NOT separated (i.e., collision)
				uint64_t mask = 0;
				for (size_t j = 0; j < batchSize; ++j) {
					if (separatedArr[j] == 0.0f) {
						mask |= (1ULL << j);
						++totalCollisions;
					}
				}

				// Store collision bits in result array
				const size_t wordIdx = i / 64;
				const size_t bitOffset = i % 64;
				if (bitOffset + batchSize <= 64) {
					// Entire mask fits in current word
					resultBits[wordIdx] |= (mask << bitOffset);
				}
				else {
					// Mask crosses word boundary
					resultBits[wordIdx] |= (mask << bitOffset);
					resultBits[wordIdx + 1] |= (mask >> (64 - bitOffset));
				}
			}

			// Scalar remainder - process remaining AABBs one at a time
			// Uses the existing optimized SSE implementation in Aabb::collideWith
			for (; i < count; ++i) {
				if (ref.collideWith(candidates[i])) {
					const size_t wordIdx = i / 64;
					const size_t bitIdx = i % 64;
					resultBits[wordIdx] |= (1ULL << bitIdx);
					++totalCollisions;
				}
			}

			return totalCollisions;
		}

		// Alternative implementation for small batches
		// Avoids gather overhead by using direct scalar calls
		size_t batchCollideWithSmallImpl(const Aabb& ref, const Aabb* candidates, size_t count, uint64_t* resultBits)
		{
			size_t totalCollisions = 0;

			for (size_t i = 0; i < count; ++i) {
				if (ref.collideWith(candidates[i])) {
					const size_t wordIdx = i / 64;
					const size_t bitIdx = i % 64;
					resultBits[wordIdx] |= (1ULL << bitIdx);
					++totalCollisions;
				}
			}

			return totalCollisions;
		}

		const char* getSimdTargetNameImpl()
		{
			return hwy::TargetName(HWY_TARGET);
		}

	} // namespace HWY_NAMESPACE
} // namespace hdt

HWY_AFTER_NAMESPACE();

// Dynamic dispatch export - compiled only once
#if HWY_ONCE

namespace hdt
{
	namespace highway
	{

		// Export function pointers for dynamic dispatch
		HWY_EXPORT(batchCollideWithImpl);
		HWY_EXPORT(batchCollideWithSmallImpl);
		HWY_EXPORT(getSimdTargetNameImpl);

		// Threshold below which scalar path is likely faster due to gather overhead
		// The gather loop has significant overhead; benchmark shows ~12 candidates
		// is the crossover point on typical hardware.
		static constexpr size_t SMALL_BATCH_THRESHOLD = 12;

		size_t batchCollideWith(const Aabb& ref, const Aabb* candidates, size_t count, uint64_t* resultBits)
		{
			if (count == 0)
				return 0;

			// For small batches, the overhead of gather may exceed SIMD benefit
			if (count < SMALL_BATCH_THRESHOLD) {
				return HWY_DYNAMIC_DISPATCH(batchCollideWithSmallImpl)(ref, candidates, count, resultBits);
			}

			return HWY_DYNAMIC_DISPATCH(batchCollideWithImpl)(ref, candidates, count, resultBits);
		}

		const char* getSimdTargetName()
		{
			return HWY_DYNAMIC_DISPATCH(getSimdTargetNameImpl)();
		}

	} // namespace highway
} // namespace hdt

#endif // HWY_ONCE
