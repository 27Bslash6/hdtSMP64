#pragma once

#include <cstddef> // for size_t

// Highway SIMD Batch Skinning for hdtSMP64
//
// Provides batch vertex skinning using Google Highway for automatic
// SIMD dispatch (SSE4/AVX2/AVX-512). Processes N vertices per iteration
// where N = vector lane count.
//
// Designed to produce bit-identical results to the scalar SSE implementation
// in hdtSkinnedMeshBody.cpp::calcVertexState().

namespace hdt
{
	// Forward declarations - avoid pulling in heavy headers
	class SoAVertexBuffer;
	struct Bone;
	struct VertexPos;

	namespace highway
	{
		// Batch skin vertices using Highway SIMD with runtime dispatch.
		//
		// Transforms vertices from skin space to world space using bone
		// matrices and weights. Output includes position (xyz) and margin
		// multiplier (w).
		//
		// Parameters:
		//   soa    - SoA vertex buffer with positions, weights, bone indices
		//   bones  - Array of bone transforms (indexed by soa->boneIndices)
		//   output - Output VertexPos array (must be pre-allocated, count elements)
		//   count  - Number of vertices to process
		//
		// Thread Safety:
		//   Safe to call from multiple threads with disjoint output ranges.
		//   Input data (soa, bones) is read-only.
		//
		// Performance:
		//   Processes N vertices per SIMD iteration (N = Lanes(ScalableTag<float>))
		//   Remainder vertices processed with scalar fallback.
		void batchSkinVertices(const SoAVertexBuffer* soa, const Bone* bones, VertexPos* output, size_t count);

	} // namespace highway
} // namespace hdt
