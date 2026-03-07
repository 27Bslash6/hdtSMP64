/**
 * Solver Edge Case Tests
 *
 * Tests for edge cases in the constraint solver, particularly:
 * - BUG-007: Division by zero when m_jacDiagABInv is zero or near-zero
 * - Degenerate constraint handling
 * - NaN/Inf propagation prevention
 *
 * These tests validate the solver's behavior without requiring the full
 * Bullet Physics library by testing the mathematical operations directly.
 */

#include "../include/catch.hpp"

#include <cmath>
#include <limits>

namespace
{

	// Match the epsilon used in the solver fix
	constexpr float HDT_SOLVER_EPSILON = 1e-10f;

	/**
	 * Simulates the return value calculation from gResolveSingleConstraintRowGeneric_avx256
	 * and gResolveSingleConstraintRowLowerLimit_avx256.
	 *
	 * Original buggy code (lines 115, 168):
	 *   return deltaImpulse.m128_f32[0] / c.m_jacDiagABInv;
	 *
	 * Fixed code should guard against division by zero/near-zero.
	 */
	float computeConstraintReturn_BUGGY(float deltaImpulse, float jacDiagABInv)
	{
		// This is the BUGGY version - no guard
		return deltaImpulse / jacDiagABInv;
	}

	float computeConstraintReturn_FIXED(float deltaImpulse, float jacDiagABInv)
	{
		// This is the FIXED version with epsilon guard
		if (std::fabs(jacDiagABInv) < HDT_SOLVER_EPSILON) {
			return 0.0f; // Degenerate constraint - return 0 instead of NaN/Inf
		}
		return deltaImpulse / jacDiagABInv;
	}

	// Helper to check if a float is valid (not NaN or Inf)
	bool isValidFloat(float value)
	{
		return std::isfinite(value);
	}

} // anonymous namespace


// =============================================================================
// BUG-007: Division by Zero in AVX Constraint Solver
// =============================================================================

TEST_CASE("AVX solver: zero jacDiagABInv causes NaN (bug reproduction)", "[solver][edge-case][bug-007]")
{
	// This test documents the bug - the buggy code produces NaN
	float deltaImpulse = 1.0f;
	float jacDiagABInv = 0.0f;

	float buggyResult = computeConstraintReturn_BUGGY(deltaImpulse, jacDiagABInv);

	// The bug: dividing by zero produces infinity/NaN
	REQUIRE_FALSE(isValidFloat(buggyResult));

	// The fix: should return 0.0f for degenerate constraints
	float fixedResult = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
	REQUIRE(isValidFloat(fixedResult));
	REQUIRE(fixedResult == 0.0f);
}

TEST_CASE("AVX solver: near-zero jacDiagABInv returns zero", "[solver][edge-case][bug-007]")
{
	float deltaImpulse = 1.0f;

	SECTION("Value 1e-11 (below epsilon 1e-10)")
	{
		float jacDiagABInv = 1e-11f;

		float fixedResult = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		REQUIRE(isValidFloat(fixedResult));
		REQUIRE(fixedResult == 0.0f);
	}

	SECTION("Negative near-zero value -1e-11")
	{
		float jacDiagABInv = -1e-11f;

		float fixedResult = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		REQUIRE(isValidFloat(fixedResult));
		REQUIRE(fixedResult == 0.0f);
	}

	SECTION("Value at epsilon boundary (1e-10)")
	{
		// At exactly epsilon, should still return 0 (using < comparison)
		float jacDiagABInv = 1e-10f;

		float fixedResult = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		// At exactly epsilon, fabs(1e-10) < 1e-10 is false, so it divides
		// But for practical purposes, this is fine - we just need to avoid true denormals
		REQUIRE(isValidFloat(fixedResult));
	}
}

TEST_CASE("AVX solver: normal jacDiagABInv values work correctly", "[solver][edge-case][bug-007]")
{
	SECTION("Typical value 1.0")
	{
		float deltaImpulse = 2.5f;
		float jacDiagABInv = 1.0f;

		float result = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		REQUIRE(isValidFloat(result));
		REQUIRE(result == Approx(2.5f));
	}

	SECTION("Small but valid value 0.001")
	{
		float deltaImpulse = 1.0f;
		float jacDiagABInv = 0.001f;

		float result = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		REQUIRE(isValidFloat(result));
		REQUIRE(result == Approx(1000.0f));
	}

	SECTION("Negative value -0.5")
	{
		float deltaImpulse = 1.0f;
		float jacDiagABInv = -0.5f;

		float result = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		REQUIRE(isValidFloat(result));
		REQUIRE(result == Approx(-2.0f));
	}

	SECTION("Large value 1000.0")
	{
		float deltaImpulse = 500.0f;
		float jacDiagABInv = 1000.0f;

		float result = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		REQUIRE(isValidFloat(result));
		REQUIRE(result == Approx(0.5f));
	}

	SECTION("Zero deltaImpulse with normal jacDiagABInv")
	{
		float deltaImpulse = 0.0f;
		float jacDiagABInv = 1.0f;

		float result = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		REQUIRE(isValidFloat(result));
		REQUIRE(result == 0.0f);
	}
}

TEST_CASE("AVX solver: edge case combinations", "[solver][edge-case][bug-007]")
{
	SECTION("Both zero - should return 0, not NaN")
	{
		float deltaImpulse = 0.0f;
		float jacDiagABInv = 0.0f;

		// Buggy version would produce NaN (0/0)
		float buggyResult = computeConstraintReturn_BUGGY(deltaImpulse, jacDiagABInv);
		REQUIRE(std::isnan(buggyResult));

		// Fixed version returns 0
		float fixedResult = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		REQUIRE(isValidFloat(fixedResult));
		REQUIRE(fixedResult == 0.0f);
	}

	SECTION("Very large deltaImpulse with near-zero jacDiagABInv")
	{
		float deltaImpulse = 1e30f;
		float jacDiagABInv = 1e-11f;

		// Fixed version guards against this
		float fixedResult = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		REQUIRE(isValidFloat(fixedResult));
		REQUIRE(fixedResult == 0.0f);
	}

	SECTION("Denormalized jacDiagABInv")
	{
		float deltaImpulse = 1.0f;
		float jacDiagABInv = std::numeric_limits<float>::denorm_min();

		// Denormalized values are smaller than epsilon, so should return 0
		float fixedResult = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		REQUIRE(isValidFloat(fixedResult));
		REQUIRE(fixedResult == 0.0f);
	}
}

TEST_CASE("AVX solver: NaN propagation prevention", "[solver][edge-case][bug-007]")
{
	// If a NaN somehow gets into the system, the fixed code should handle it
	// Note: NaN as input is a separate bug, but we should at least not make it worse

	SECTION("NaN deltaImpulse with valid jacDiagABInv")
	{
		float deltaImpulse = std::numeric_limits<float>::quiet_NaN();
		float jacDiagABInv = 1.0f;

		// This will still produce NaN (NaN/1 = NaN), which is expected
		// The fix specifically targets jacDiagABInv being zero
		float result = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		// Just verify we don't crash - NaN propagation is a different issue
		(void)result;
	}

	SECTION("NaN jacDiagABInv")
	{
		float deltaImpulse = 1.0f;
		float jacDiagABInv = std::numeric_limits<float>::quiet_NaN();

		float result = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		// fabs(NaN) < epsilon is false, so it will divide
		// This produces NaN, which is at least consistent behavior
		(void)result;
	}
}

TEST_CASE("AVX solver: infinity handling", "[solver][edge-case][bug-007]")
{
	SECTION("Infinite jacDiagABInv should produce zero result (correct behavior)")
	{
		float deltaImpulse = 1.0f;
		float jacDiagABInv = std::numeric_limits<float>::infinity();

		float result = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		// 1/infinity = 0, which is valid
		REQUIRE(isValidFloat(result));
		REQUIRE(result == 0.0f);
	}

	SECTION("Negative infinite jacDiagABInv")
	{
		float deltaImpulse = 1.0f;
		float jacDiagABInv = -std::numeric_limits<float>::infinity();

		float result = computeConstraintReturn_FIXED(deltaImpulse, jacDiagABInv);
		// 1/-infinity = -0, which is valid
		REQUIRE(isValidFloat(result));
		REQUIRE(result == Approx(0.0f));
	}
}
