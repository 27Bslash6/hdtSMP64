// Highway SIMD Batch Skinning Implementation
//
// Uses Highway multi-target pattern for automatic SIMD dispatch.
// Compiles multiple versions (SSE4, AVX2, AVX-512) and selects at runtime.

// clang-format off
// Highway multi-target include pattern - order is critical

// Highway disables AVX-512 on MSVC by default due to old compiler bugs.
// VS2022 17.10+ (MSVC 19.40+) has fixed these issues, so we re-enable.
// Must be defined BEFORE any Highway headers.
#if defined(_MSC_VER) && (_MSC_VER >= 1940)
#define HWY_BROKEN_MSVC 0
#endif

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "hdtHighwaySkinning.cpp"
#include <hwy/foreach_target.h>  // Re-includes this file for each target
#include <hwy/highway.h>
// clang-format on

#include "hdtHighwaySkinning.h"

#include "hdtBone.h"
#include "hdtHighway.h"
#include "hdtSoABuffer.h"
#include "hdtTracy.h"
#include "hdtVertex.h"

HWY_BEFORE_NAMESPACE();
namespace hdt
{
	namespace HWY_NAMESPACE
	{

		namespace hn = hwy::HWY_NAMESPACE;

		// Transform a single vertex by a bone matrix (scalar fallback)
		// Matches calcVertexState() from hdtSkinnedMeshBody.cpp exactly
		HWY_INLINE void transformVertexScalar(float px, float py, float pz, const Bone& bone, float weight, float& outX,
											  float& outY, float& outZ, float& outM)
		{
			// Column-major matrix multiply: col[0]*x + col[1]*y + col[2]*z + col[3]
			const btVector3* col = bone.m_vertexToWorld.m_col;

			float rx = col[0][0] * px + col[1][0] * py + col[2][0] * pz + col[3][0];
			float ry = col[0][1] * px + col[1][1] * py + col[2][1] * pz + col[3][1];
			float rz = col[0][2] * px + col[1][2] * py + col[2][2] * pz + col[3][2];

			// Get margin multiplier from correct location based on build
#ifdef CUDA
			float margin = col[3][3];
#else
			float margin = bone.m_maginMultipler;
#endif

			// Accumulate weighted result
			outX += weight * rx;
			outY += weight * ry;
			outZ += weight * rz;
			outM += weight * margin;
		}

		// Batch skin vertices using Highway SIMD
		// Processes N vertices per iteration where N = Lanes(d)
		void batchSkinVerticesImpl(const SoAVertexBuffer* soa, const Bone* bones, VertexPos* output, size_t count)
		{
			if (count == 0 || soa == nullptr || bones == nullptr || output == nullptr)
				return;

			const hn::ScalableTag<float> d;
			const size_t N = hn::Lanes(d);

			// SoA input pointers
			const float* HWY_RESTRICT posX = soa->posX();
			const float* HWY_RESTRICT posY = soa->posY();
			const float* HWY_RESTRICT posZ = soa->posZ();
			const float* HWY_RESTRICT weights = soa->weights();	  // 4 weights per vertex
			const U32* HWY_RESTRICT boneIdx = soa->boneIndices(); // 4 indices per vertex

			// Epsilon for weight comparison
			constexpr float WEIGHT_EPSILON = 1.192092896e-07f; // FLT_EPSILON
			const auto vEpsilon = hn::Set(d, WEIGHT_EPSILON);
			const auto vZero = hn::Zero(d);

			size_t i = 0;

			// Main vectorized loop - process N vertices per iteration
			// NOTE: We can't easily vectorize across bones due to gather requirements,
			// so we vectorize across vertices and process each bone contribution serially.
			// This is still beneficial because we can do N matrix multiplies in parallel.
			for (; i + N <= count; i += N) {
				// Load N vertex positions (contiguous in SoA layout)
				auto vPosX = hn::Load(d, posX + i);
				auto vPosY = hn::Load(d, posY + i);
				auto vPosZ = hn::Load(d, posZ + i);

				// Initialize accumulators
				auto accX = vZero;
				auto accY = vZero;
				auto accZ = vZero;
				auto accM = vZero;

				// Process 4 bone contributions per vertex
				// Weight layout is: [v0_w0, v0_w1, v0_w2, v0_w3, v1_w0, v1_w1, v1_w2, v1_w3, ...]
				// Bone index layout matches weight layout
				for (int boneSlot = 0; boneSlot < 4; ++boneSlot) {
					// Gather weights for this bone slot across N vertices
					// weights[i*4 + boneSlot] for vertex i
					// We need to load with stride 4
					auto vWeight = vZero;
					for (size_t lane = 0; lane < N; ++lane) {
						size_t vertIdx = i + lane;
						float w = weights[vertIdx * 4 + boneSlot];
						// Use InsertLane for portable lane insertion
						vWeight = hn::InsertLane(vWeight, lane, w);
					}

					// Skip this bone slot if all weights are below epsilon
					auto weightMask = hn::Gt(vWeight, vEpsilon);
					if (hn::AllFalse(d, weightMask)) {
						// First bone always processed, others can be skipped
						if (boneSlot > 0)
							continue;
					}

					// For each lane, gather bone transform data and compute contribution
					// This is where Highway's gather would be ideal, but bone matrices
					// are complex structures. We fall back to scalar gather + vector compute.

					// Temporary arrays for gathered bone data (12 floats per bone: 4 columns x 3 rows)
					HWY_ALIGN float boneCol0X[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
					HWY_ALIGN float boneCol0Y[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
					HWY_ALIGN float boneCol0Z[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
					HWY_ALIGN float boneCol1X[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
					HWY_ALIGN float boneCol1Y[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
					HWY_ALIGN float boneCol1Z[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
					HWY_ALIGN float boneCol2X[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
					HWY_ALIGN float boneCol2Y[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
					HWY_ALIGN float boneCol2Z[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
					HWY_ALIGN float boneCol3X[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
					HWY_ALIGN float boneCol3Y[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
					HWY_ALIGN float boneCol3Z[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
					HWY_ALIGN float boneMargin[HWY_MAX_LANES_D(hn::ScalableTag<float>)];

					// Gather bone data for N vertices
					for (size_t lane = 0; lane < N; ++lane) {
						size_t vertIdx = i + lane;
						U32 bidx = boneIdx[vertIdx * 4 + boneSlot];
						const Bone& bone = bones[bidx];
						const btVector3* col = bone.m_vertexToWorld.m_col;

						boneCol0X[lane] = col[0][0];
						boneCol0Y[lane] = col[0][1];
						boneCol0Z[lane] = col[0][2];
						boneCol1X[lane] = col[1][0];
						boneCol1Y[lane] = col[1][1];
						boneCol1Z[lane] = col[1][2];
						boneCol2X[lane] = col[2][0];
						boneCol2Y[lane] = col[2][1];
						boneCol2Z[lane] = col[2][2];
						boneCol3X[lane] = col[3][0];
						boneCol3Y[lane] = col[3][1];
						boneCol3Z[lane] = col[3][2];

#ifdef CUDA
						boneMargin[lane] = col[3][3];
#else
						boneMargin[lane] = bone.m_maginMultipler;
#endif
					}

					// Load gathered bone data into vectors
					auto vCol0X = hn::Load(d, boneCol0X);
					auto vCol0Y = hn::Load(d, boneCol0Y);
					auto vCol0Z = hn::Load(d, boneCol0Z);
					auto vCol1X = hn::Load(d, boneCol1X);
					auto vCol1Y = hn::Load(d, boneCol1Y);
					auto vCol1Z = hn::Load(d, boneCol1Z);
					auto vCol2X = hn::Load(d, boneCol2X);
					auto vCol2Y = hn::Load(d, boneCol2Y);
					auto vCol2Z = hn::Load(d, boneCol2Z);
					auto vCol3X = hn::Load(d, boneCol3X);
					auto vCol3Y = hn::Load(d, boneCol3Y);
					auto vCol3Z = hn::Load(d, boneCol3Z);
					auto vMargin = hn::Load(d, boneMargin);

					// Column-major matrix multiply: col[0]*x + col[1]*y + col[2]*z + col[3]
					// Result X = col0.x*px + col1.x*py + col2.x*pz + col3.x
					auto resX = hn::MulAdd(vCol0X, vPosX, hn::MulAdd(vCol1X, vPosY, hn::MulAdd(vCol2X, vPosZ, vCol3X)));
					auto resY = hn::MulAdd(vCol0Y, vPosX, hn::MulAdd(vCol1Y, vPosY, hn::MulAdd(vCol2Y, vPosZ, vCol3Y)));
					auto resZ = hn::MulAdd(vCol0Z, vPosX, hn::MulAdd(vCol1Z, vPosY, hn::MulAdd(vCol2Z, vPosZ, vCol3Z)));

					// Multiply by weight and accumulate
					// Use IfThenElse to handle zero weights for boneSlot > 0
					if (boneSlot == 0) {
						// First bone always contributes
						accX = hn::MulAdd(vWeight, resX, accX);
						accY = hn::MulAdd(vWeight, resY, accY);
						accZ = hn::MulAdd(vWeight, resZ, accZ);
						accM = hn::MulAdd(vWeight, vMargin, accM);
					}
					else {
						// Conditional accumulation for bones 1-3
						auto contrib_x = hn::Mul(vWeight, resX);
						auto contrib_y = hn::Mul(vWeight, resY);
						auto contrib_z = hn::Mul(vWeight, resZ);
						auto contrib_m = hn::Mul(vWeight, vMargin);

						accX = hn::IfThenElse(weightMask, hn::Add(accX, contrib_x), accX);
						accY = hn::IfThenElse(weightMask, hn::Add(accY, contrib_y), accY);
						accZ = hn::IfThenElse(weightMask, hn::Add(accZ, contrib_z), accZ);
						accM = hn::IfThenElse(weightMask, hn::Add(accM, contrib_m), accM);
					}
				}

				// Store results to AoS VertexPos output
				// VertexPos is [x, y, z, margin] per vertex
				// Need to transpose from SoA accumulators to AoS output
				HWY_ALIGN float outX[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
				HWY_ALIGN float outY[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
				HWY_ALIGN float outZ[HWY_MAX_LANES_D(hn::ScalableTag<float>)];
				HWY_ALIGN float outM[HWY_MAX_LANES_D(hn::ScalableTag<float>)];

				hn::Store(accX, d, outX);
				hn::Store(accY, d, outY);
				hn::Store(accZ, d, outZ);
				hn::Store(accM, d, outM);

				for (size_t lane = 0; lane < N; ++lane) {
					// Write directly to VertexPos::m_data
					// Using _mm_setr_ps since we're on x86 with SSE4 baseline
					__m128 result = _mm_setr_ps(outX[lane], outY[lane], outZ[lane], outM[lane]);
					_mm_store_ps(reinterpret_cast<float*>(&output[i + lane].m_data), result);
				}
			}

			// Scalar remainder loop for tail elements
			for (; i < count; ++i) {
				float px = posX[i];
				float py = posY[i];
				float pz = posZ[i];

				float accX = 0.0f;
				float accY = 0.0f;
				float accZ = 0.0f;
				float accM = 0.0f;

				// Process 4 bone contributions
				for (int boneSlot = 0; boneSlot < 4; ++boneSlot) {
					float w = weights[i * 4 + boneSlot];

					// First bone always processed, others conditional
					if (boneSlot > 0 && w <= WEIGHT_EPSILON)
						continue;

					U32 bidx = boneIdx[i * 4 + boneSlot];
					const Bone& bone = bones[bidx];

					transformVertexScalar(px, py, pz, bone, w, accX, accY, accZ, accM);
				}

				__m128 result = _mm_setr_ps(accX, accY, accZ, accM);
				_mm_store_ps(reinterpret_cast<float*>(&output[i].m_data), result);
			}
		}

	} // namespace HWY_NAMESPACE
} // namespace hdt
HWY_AFTER_NAMESPACE();

// Export with dynamic dispatch
#if HWY_ONCE

namespace hdt
{
	namespace highway
	{

		HWY_EXPORT(batchSkinVerticesImpl);

		void batchSkinVertices(const SoAVertexBuffer* soa, const Bone* bones, VertexPos* output, size_t count)
		{
			HDT_ZONE_SCOPED_N("Highway::batchSkinVertices");
			HWY_DYNAMIC_DISPATCH(batchSkinVerticesImpl)(soa, bones, output, count);
		}

	} // namespace highway
} // namespace hdt

#endif // HWY_ONCE
