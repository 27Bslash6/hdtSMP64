#pragma once

// Highway-accelerated batch AABB collision detection
//
// Provides portable SIMD-accelerated collision testing that auto-dispatches
// to SSE4/AVX2/AVX-512 at runtime via Google Highway.
//
// Usage:
//   uint64_t resultBits[2] = {0};  // 128 bits max
//   size_t hits = highway::batchCollideWith(refAABB, candidateArray, count, resultBits);
//   // resultBits[i/64] & (1ULL << (i%64)) is set if candidates[i] collides

#include <cstddef>
#include <cstdint>

namespace hdt
{
	struct Aabb; // Forward declaration (defined in hdtAABB.h)

	namespace highway
	{
		// Test reference AABB against N candidate AABBs using SIMD batch processing.
		//
		// For each candidate[i], tests collision against ref using the same logic as
		// Aabb::collideWith() - results are bit-identical to the scalar version.
		//
		// Parameters:
		//   ref         - Reference AABB to test against all candidates
		//   candidates  - Array of candidate AABBs (must be at least count elements)
		//   count       - Number of candidates to test (max 128 currently)
		//   resultBits  - Output bitmask array: bit i set if candidates[i] collides with ref
		//                 Must have at least ceil(count/64) uint64_t elements
		//                 Caller is responsible for zero-initializing before call
		//
		// Returns: Total number of collisions detected
		//
		// Thread safety: Fully reentrant, no shared state
		// Memory: No dynamic allocation, stack-only
		size_t batchCollideWith(const Aabb& ref, const Aabb* candidates, size_t count, uint64_t* resultBits);

		// Query the detected SIMD capability for diagnostic purposes
		// Returns a human-readable string like "AVX2", "AVX-512", "SSE4", etc.
		const char* getSimdTargetName();

	} // namespace highway
} // namespace hdt
