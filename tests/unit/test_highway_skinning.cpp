#include "../include/catch.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

// Highway skinning tests
// Tests are organized into:
// 1. Standalone math validation (always enabled)
// 2. Integration tests with hdtSMP64 library (disabled in standalone builds)

#define HIGHWAY_SKINNING_TESTS_ENABLED 0

// =============================================================================
// Standalone skinning math tests - validate the algorithm without Highway
// =============================================================================

namespace test
{

	// Simplified bone transform (4x3 matrix as used in hdtSMP64)
	struct BoneTransform
	{
		float m[4][3]; // Column-major: m[col][row], affine 4x3 matrix

		// Apply transform to a point
		void transformPoint(float x, float y, float z, float& outX, float& outY, float& outZ) const
		{
			outX = m[0][0] * x + m[1][0] * y + m[2][0] * z + m[3][0];
			outY = m[0][1] * x + m[1][1] * y + m[2][1] * z + m[3][1];
			outZ = m[0][2] * x + m[1][2] * y + m[2][2] * z + m[3][2];
		}
	};

	// Create identity bone transform
	static BoneTransform identityBone()
	{
		BoneTransform b;
		std::memset(&b, 0, sizeof(b));
		b.m[0][0] = 1.0f; // X column
		b.m[1][1] = 1.0f; // Y column
		b.m[2][2] = 1.0f; // Z column
		return b;
	}

	// Create translation bone transform
	static BoneTransform translationBone(float tx, float ty, float tz)
	{
		BoneTransform b = identityBone();
		b.m[3][0] = tx;
		b.m[3][1] = ty;
		b.m[3][2] = tz;
		return b;
	}

	// Create scale bone transform
	static BoneTransform scaleBone(float sx, float sy, float sz)
	{
		BoneTransform b;
		std::memset(&b, 0, sizeof(b));
		b.m[0][0] = sx;
		b.m[1][1] = sy;
		b.m[2][2] = sz;
		return b;
	}

	// Simplified vertex with bone weights
	struct TestVertex
	{
		float x, y, z;			 // Position
		float weights[4];		 // Bone weights (should sum to 1.0)
		uint32_t boneIndices[4]; // Bone indices
	};

	// Perform weighted skinning (scalar reference implementation)
	static void skinVertex(const TestVertex& v, const BoneTransform* bones, float& outX, float& outY, float& outZ)
	{
		outX = outY = outZ = 0.0f;

		for (int i = 0; i < 4; i++) {
			if (v.weights[i] <= 0.0f)
				continue;

			float px, py, pz;
			bones[v.boneIndices[i]].transformPoint(v.x, v.y, v.z, px, py, pz);

			outX += px * v.weights[i];
			outY += py * v.weights[i];
			outZ += pz * v.weights[i];
		}
	}

} // namespace test

using namespace test;

TEST_CASE("Skinning math - identity transform", "[skinning][math]")
{
	BoneTransform bones[1] = {identityBone()};
	TestVertex v = {10.0f, 20.0f, 30.0f, {1.0f, 0.0f, 0.0f, 0.0f}, {0, 0, 0, 0}};

	float outX, outY, outZ;
	skinVertex(v, bones, outX, outY, outZ);

	REQUIRE(outX == Approx(10.0f));
	REQUIRE(outY == Approx(20.0f));
	REQUIRE(outZ == Approx(30.0f));
}

TEST_CASE("Skinning math - translation transform", "[skinning][math]")
{
	BoneTransform bones[1] = {translationBone(5.0f, -3.0f, 2.0f)};
	TestVertex v = {10.0f, 20.0f, 30.0f, {1.0f, 0.0f, 0.0f, 0.0f}, {0, 0, 0, 0}};

	float outX, outY, outZ;
	skinVertex(v, bones, outX, outY, outZ);

	REQUIRE(outX == Approx(15.0f));
	REQUIRE(outY == Approx(17.0f));
	REQUIRE(outZ == Approx(32.0f));
}

TEST_CASE("Skinning math - scale transform", "[skinning][math]")
{
	BoneTransform bones[1] = {scaleBone(2.0f, 0.5f, 1.0f)};
	TestVertex v = {10.0f, 20.0f, 30.0f, {1.0f, 0.0f, 0.0f, 0.0f}, {0, 0, 0, 0}};

	float outX, outY, outZ;
	skinVertex(v, bones, outX, outY, outZ);

	REQUIRE(outX == Approx(20.0f));
	REQUIRE(outY == Approx(10.0f));
	REQUIRE(outZ == Approx(30.0f));
}

TEST_CASE("Skinning math - weighted blend of two bones", "[skinning][math]")
{
	BoneTransform bones[2] = {translationBone(10.0f, 0.0f, 0.0f), translationBone(0.0f, 10.0f, 0.0f)};
	TestVertex v = {0.0f, 0.0f, 0.0f, {0.5f, 0.5f, 0.0f, 0.0f}, {0, 1, 0, 0}};

	float outX, outY, outZ;
	skinVertex(v, bones, outX, outY, outZ);

	// 50% of (10,0,0) + 50% of (0,10,0) = (5,5,0)
	REQUIRE(outX == Approx(5.0f));
	REQUIRE(outY == Approx(5.0f));
	REQUIRE(outZ == Approx(0.0f));
}

TEST_CASE("Skinning math - four bone blend", "[skinning][math]")
{
	BoneTransform bones[4] = {translationBone(4.0f, 0.0f, 0.0f), translationBone(0.0f, 4.0f, 0.0f),
							  translationBone(0.0f, 0.0f, 4.0f), translationBone(-4.0f, -4.0f, -4.0f)};
	// Equal weights: 25% each
	TestVertex v = {0.0f, 0.0f, 0.0f, {0.25f, 0.25f, 0.25f, 0.25f}, {0, 1, 2, 3}};

	float outX, outY, outZ;
	skinVertex(v, bones, outX, outY, outZ);

	// (4,0,0)*0.25 + (0,4,0)*0.25 + (0,0,4)*0.25 + (-4,-4,-4)*0.25 = (0,0,0)
	REQUIRE(outX == Approx(0.0f).margin(1e-5f));
	REQUIRE(outY == Approx(0.0f).margin(1e-5f));
	REQUIRE(outZ == Approx(0.0f).margin(1e-5f));
}

TEST_CASE("Skinning math - zero weight bones ignored", "[skinning][math]")
{
	BoneTransform bones[4] = {
		translationBone(10.0f, 0.0f, 0.0f), translationBone(0.0f, 100.0f, 0.0f), // Should be ignored
		translationBone(0.0f, 0.0f, 100.0f),									 // Should be ignored
		translationBone(-100.0f, 0.0f, 0.0f)									 // Should be ignored
	};
	TestVertex v = {0.0f, 0.0f, 0.0f, {1.0f, 0.0f, 0.0f, 0.0f}, {0, 1, 2, 3}};

	float outX, outY, outZ;
	skinVertex(v, bones, outX, outY, outZ);

	REQUIRE(outX == Approx(10.0f));
	REQUIRE(outY == Approx(0.0f));
	REQUIRE(outZ == Approx(0.0f));
}

TEST_CASE("Skinning math - batch consistency", "[skinning][math]")
{
	// Test that processing multiple vertices gives consistent results
	BoneTransform bones[2] = {translationBone(1.0f, 2.0f, 3.0f), scaleBone(2.0f, 2.0f, 2.0f)};

	TestVertex vertices[5] = {
		{1.0f, 0.0f, 0.0f, {1.0f, 0.0f, 0.0f, 0.0f}, {0, 0, 0, 0}},
		{0.0f, 1.0f, 0.0f, {0.0f, 1.0f, 0.0f, 0.0f}, {0, 1, 0, 0}},
		{0.0f, 0.0f, 1.0f, {0.5f, 0.5f, 0.0f, 0.0f}, {0, 1, 0, 0}},
		{1.0f, 1.0f, 1.0f, {0.25f, 0.75f, 0.0f, 0.0f}, {0, 1, 0, 0}},
		{5.0f, 5.0f, 5.0f, {0.0f, 1.0f, 0.0f, 0.0f}, {0, 1, 0, 0}},
	};

	float results[5][3];
	for (int i = 0; i < 5; i++) {
		skinVertex(vertices[i], bones, results[i][0], results[i][1], results[i][2]);
	}

	// Vertex 0: identity at (1,0,0) + translation = (2,2,3)
	REQUIRE(results[0][0] == Approx(2.0f));
	REQUIRE(results[0][1] == Approx(2.0f));
	REQUIRE(results[0][2] == Approx(3.0f));

	// Vertex 1: (0,1,0) scaled by 2 = (0,2,0)
	REQUIRE(results[1][0] == Approx(0.0f));
	REQUIRE(results[1][1] == Approx(2.0f));
	REQUIRE(results[1][2] == Approx(0.0f));

	// Vertex 4: (5,5,5) scaled by 2 = (10,10,10)
	REQUIRE(results[4][0] == Approx(10.0f));
	REQUIRE(results[4][1] == Approx(10.0f));
	REQUIRE(results[4][2] == Approx(10.0f));
}

// =============================================================================
// SoA buffer layout tests - validate Structure-of-Arrays data organization
// =============================================================================

TEST_CASE("SoA buffer layout validation", "[skinning][soa]")
{
	// Test that SoA (Structure of Arrays) organization is correct
	// This mirrors the layout in hdtSoABuffer

	SECTION("Separate coordinate arrays")
	{
		std::vector<float> posX = {1.0f, 2.0f, 3.0f, 4.0f};
		std::vector<float> posY = {5.0f, 6.0f, 7.0f, 8.0f};
		std::vector<float> posZ = {9.0f, 10.0f, 11.0f, 12.0f};

		// Verify independent access
		REQUIRE(posX[2] == 3.0f);
		REQUIRE(posY[2] == 7.0f);
		REQUIRE(posZ[2] == 11.0f);
	}

	SECTION("Interleaved weights (4 per vertex)")
	{
		// Weights stored as: w0_v0, w1_v0, w2_v0, w3_v0, w0_v1, w1_v1, ...
		std::vector<float> weights = {
			1.0f,  0.0f,  0.0f,	 0.0f,	// Vertex 0
			0.5f,  0.5f,  0.0f,	 0.0f,	// Vertex 1
			0.25f, 0.25f, 0.25f, 0.25f, // Vertex 2
		};

		// Access weight i of vertex v: weights[v * 4 + i]
		REQUIRE(weights[0 * 4 + 0] == 1.0f);  // v0, w0
		REQUIRE(weights[1 * 4 + 1] == 0.5f);  // v1, w1
		REQUIRE(weights[2 * 4 + 3] == 0.25f); // v2, w3
	}

	SECTION("Bone indices (4 per vertex)")
	{
		std::vector<uint32_t> indices = {
			0, 0, 0, 0, // Vertex 0: all bone 0
			0, 1, 0, 0, // Vertex 1: bones 0 and 1
			0, 1, 2, 3, // Vertex 2: bones 0-3
		};

		REQUIRE(indices[0 * 4 + 0] == 0);
		REQUIRE(indices[1 * 4 + 1] == 1);
		REQUIRE(indices[2 * 4 + 3] == 3);
	}
}

// =============================================================================
// Highway skinning integration tests (disabled in standalone builds)
// =============================================================================
#if HIGHWAY_SKINNING_TESTS_ENABLED

namespace hdt
{
	class SoAVertexBuffer;
	struct Bone;
	struct VertexPos;

	namespace highway
	{
		void batchSkinVertices(const SoAVertexBuffer* soaBuffer, const Bone* bones, VertexPos* output, size_t count);
		const char* getSimdTargetName();
	} // namespace highway
} // namespace hdt

TEST_CASE("Highway::batchSkinVertices matches scalar", "[skinning][highway]")
{
	// This test would compare Highway SIMD results with scalar reference
	// Requires linking against full hdtSMP64 library
	REQUIRE(true); // Placeholder
}

TEST_CASE("Highway::getSimdTargetName returns valid string", "[skinning][highway]")
{
	const char* name = hdt::highway::getSimdTargetName();
	REQUIRE(name != nullptr);
	REQUIRE(strlen(name) > 0);
	INFO("Detected SIMD target: " << name);
}

#endif // HIGHWAY_SKINNING_TESTS_ENABLED
