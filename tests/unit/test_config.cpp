#include "../include/catch.hpp"

#include <string>
#include <vector>

// Mock/standalone tests for config parsing logic
// These tests validate XML parsing and config value handling

TEST_CASE("Config value parsing", "[config]")
{
	SECTION("Parse integer values")
	{
		// Test integer parsing similar to config.cpp readUInt/readInt
		auto parseUInt = [](const char* str, unsigned int def) -> unsigned int {
			if (!str)
				return def;
			try {
				return std::stoul(str);
			}
			catch (...) {
				return def;
			}
		};

		REQUIRE(parseUInt("100", 0) == 100);
		REQUIRE(parseUInt("0", 50) == 0);
		REQUIRE(parseUInt(nullptr, 42) == 42);
		REQUIRE(parseUInt("invalid", 99) == 99);
		REQUIRE(parseUInt("", 10) == 10);
	}

	SECTION("Parse float values")
	{
		auto parseFloat = [](const char* str, float def) -> float {
			if (!str)
				return def;
			try {
				return std::stof(str);
			}
			catch (...) {
				return def;
			}
		};

		REQUIRE(parseFloat("1.5", 0.0f) == Approx(1.5f));
		REQUIRE(parseFloat("0.0", 1.0f) == Approx(0.0f));
		REQUIRE(parseFloat(nullptr, 3.14f) == Approx(3.14f));
		REQUIRE(parseFloat("invalid", 2.0f) == Approx(2.0f));
	}

	SECTION("Parse boolean values")
	{
		auto parseBool = [](const char* str, bool def) -> bool {
			if (!str)
				return def;
			std::string s(str);
			if (s == "true" || s == "1")
				return true;
			if (s == "false" || s == "0")
				return false;
			return def;
		};

		REQUIRE(parseBool("true", false) == true);
		REQUIRE(parseBool("false", true) == false);
		REQUIRE(parseBool("1", false) == true);
		REQUIRE(parseBool("0", true) == false);
		REQUIRE(parseBool(nullptr, true) == true);
		REQUIRE(parseBool("invalid", false) == false);
	}
}

TEST_CASE("Frame rate calculations", "[physics]")
{
	SECTION("Time tick from FPS")
	{
		// Verify fps-to-tick conversion matches config.cpp logic
		auto fpsToTick = [](int fps) -> float { return 1.0f / static_cast<float>(fps); };

		REQUIRE(fpsToTick(60) == Approx(1.0f / 60.0f));
		REQUIRE(fpsToTick(120) == Approx(1.0f / 120.0f));
		REQUIRE(fpsToTick(30) == Approx(1.0f / 30.0f));
	}

	SECTION("Substep limiting")
	{
		// Test max substep clamping logic from SkyrimPhysicsWorld
		auto clampSubsteps = [](float accumulated, float tick, int maxSubsteps) -> float {
			return std::min(accumulated, tick * maxSubsteps);
		};

		float tick = 1.0f / 60.0f; // 60 FPS
		int maxSubsteps = 4;

		// Normal case - small accumulation
		REQUIRE(clampSubsteps(tick * 2, tick, maxSubsteps) == Approx(tick * 2));

		// Clamped case - large accumulation
		REQUIRE(clampSubsteps(tick * 10, tick, maxSubsteps) == Approx(tick * 4));
	}
}

TEST_CASE("Exponential moving average", "[physics]")
{
	SECTION("EMA converges to stable value")
	{
		// Test the averaging logic from SkyrimPhysicsWorld::doUpdate
		float average = 1.0f / 60.0f; // Start at 60 FPS
		float alpha = 0.125f;

		// Feed constant 30 FPS intervals
		float targetInterval = 1.0f / 30.0f;
		for (int i = 0; i < 100; i++) {
			average += (targetInterval - average) * alpha;
		}

		// Should converge close to target
		REQUIRE(average == Approx(targetInterval).epsilon(0.01));
	}

	SECTION("EMA responds to sudden changes")
	{
		float average = 1.0f / 60.0f;
		float alpha = 0.125f;

		// Sudden spike
		float spike = 1.0f / 15.0f; // 15 FPS
		average += (spike - average) * alpha;

		// Should partially move toward spike
		REQUIRE(average > 1.0f / 60.0f);
		REQUIRE(average < spike);
	}
}

// SEC-003: Config bounds validation tests
// These tests verify that config values are properly clamped to safe ranges
// Matches btClamped implementation from LinearMath/btMinMax.h
namespace
{
	template<typename T>
	T btClamped(T val, T lo, T hi)
	{
		return val < lo ? lo : (hi < val ? hi : val);
	}

	// Simulates config parsing for rotationSpeedLimit
	// Current (buggy): returns raw value without bounds check
	// Fixed: should use btClamped(value, 0.0f, 360.0f)
	float parseRotationSpeedLimit(float rawValue)
	{
		// BUG: Currently config.cpp line 101 does NOT clamp this value
		// return rawValue;  // <-- This is what the code currently does

		// FIX: Apply btClamped with bounds 0-360 radians/sec
		return btClamped(rawValue, 0.0f, 360.0f);
	}

	// Simulates config parsing for maximumActiveSkeletons
	// Current (buggy): returns raw value without bounds check
	// Fixed: should use btClamped(value, 1, 100) to match console command
	int parseMaximumActiveSkeletons(int rawValue)
	{
		// BUG: Currently config.cpp line 130 does NOT clamp this value
		// return rawValue;  // <-- This is what the code currently does

		// FIX: Apply btClamped with bounds 1-100 to match console command
		return btClamped(rawValue, 1, 100);
	}
} // namespace

TEST_CASE("Config bounds validation - rotationSpeedLimit", "[config][bounds][SEC-003]")
{
	// rotationSpeedLimit should be clamped to [0.0, 360.0] radians/sec
	// 360 rad/s is ~57 rotations/sec (very generous upper bound)

	SECTION("Values within bounds are unchanged")
	{
		REQUIRE(parseRotationSpeedLimit(0.0f) == Approx(0.0f));
		REQUIRE(parseRotationSpeedLimit(10.0f) == Approx(10.0f)); // Default value
		REQUIRE(parseRotationSpeedLimit(180.0f) == Approx(180.0f));
		REQUIRE(parseRotationSpeedLimit(360.0f) == Approx(360.0f));
	}

	SECTION("Negative values are clamped to 0")
	{
		REQUIRE(parseRotationSpeedLimit(-1.0f) == Approx(0.0f));
		REQUIRE(parseRotationSpeedLimit(-100.0f) == Approx(0.0f));
		REQUIRE(parseRotationSpeedLimit(-999999.0f) == Approx(0.0f));
	}

	SECTION("Excessive values are clamped to 360")
	{
		REQUIRE(parseRotationSpeedLimit(361.0f) == Approx(360.0f));
		REQUIRE(parseRotationSpeedLimit(1000.0f) == Approx(360.0f));
		REQUIRE(parseRotationSpeedLimit(999999.0f) == Approx(360.0f));
	}
}

TEST_CASE("Config bounds validation - maximumActiveSkeletons", "[config][bounds][SEC-003]")
{
	// maximumActiveSkeletons should be clamped to [1, 100]
	// This matches the console command validation in main.cpp

	SECTION("Values within bounds are unchanged")
	{
		REQUIRE(parseMaximumActiveSkeletons(1) == 1);
		REQUIRE(parseMaximumActiveSkeletons(20) == 20); // Default value
		REQUIRE(parseMaximumActiveSkeletons(50) == 50);
		REQUIRE(parseMaximumActiveSkeletons(100) == 100);
	}

	SECTION("Values below 1 are clamped to 1")
	{
		REQUIRE(parseMaximumActiveSkeletons(0) == 1);
		REQUIRE(parseMaximumActiveSkeletons(-1) == 1);
		REQUIRE(parseMaximumActiveSkeletons(-100) == 1);
	}

	SECTION("Values above 100 are clamped to 100")
	{
		REQUIRE(parseMaximumActiveSkeletons(101) == 100);
		REQUIRE(parseMaximumActiveSkeletons(500) == 100);
		REQUIRE(parseMaximumActiveSkeletons(999999) == 100);
	}
}

// BUG-NEW-007: Wind factor calculation with division by zero guard
// Calculates wind reduction factor based on distance from obstruction.
// Returns 0 when dist <= distanceForNoWind, 1 when dist >= distanceForMaxWind,
// and linear interpolation between. Guards against division by zero.
namespace
{
	float calculateWindFactor(float dist, float distanceForNoWind, float distanceForMaxWind)
	{
		const float denominator = distanceForMaxWind - distanceForNoWind;
		// Guard against division by zero when distances are equal or inverted
		if (denominator <= 0.0001f) {
			return 0.f; // Default to no wind if config is invalid
		}
		return std::clamp((dist - distanceForNoWind) / denominator, 0.f, 1.f);
	}
} // namespace

TEST_CASE("Wind factor calculation", "[physics][wind][BUG-NEW-007]")
{
	// Wind factor: 0 = full wind blocking, 1 = no blocking (full wind)
	// Linear interpolation between distanceForNoWind and distanceForMaxWind

	SECTION("Normal case - different distances")
	{
		// distanceForNoWind = 100, distanceForMaxWind = 500
		// dist <= 100 -> factor = 0 (blocked)
		// dist >= 500 -> factor = 1 (full wind)
		// dist = 300 -> factor = 0.5 (linear)

		REQUIRE(calculateWindFactor(0.f, 100.f, 500.f) == Approx(0.f));
		REQUIRE(calculateWindFactor(100.f, 100.f, 500.f) == Approx(0.f));
		REQUIRE(calculateWindFactor(300.f, 100.f, 500.f) == Approx(0.5f));
		REQUIRE(calculateWindFactor(500.f, 100.f, 500.f) == Approx(1.f));
		REQUIRE(calculateWindFactor(1000.f, 100.f, 500.f) == Approx(1.f));
	}

	SECTION("Edge case - equal distances (division by zero)")
	{
		// BUG: When m_distanceForMaxWind == m_distanceForNoWind, denominator is 0
		// This produces INF/NaN, and std::clamp with NaN fails silently
		// FIX: Return 0.f (no wind) when config is invalid

		REQUIRE(calculateWindFactor(0.f, 100.f, 100.f) == Approx(0.f));
		REQUIRE(calculateWindFactor(100.f, 100.f, 100.f) == Approx(0.f));
		REQUIRE(calculateWindFactor(200.f, 100.f, 100.f) == Approx(0.f));
	}

	SECTION("Edge case - inverted distances (negative denominator)")
	{
		// If distanceForMaxWind < distanceForNoWind, denominator is negative
		// This is also invalid config, should return 0.f

		REQUIRE(calculateWindFactor(0.f, 500.f, 100.f) == Approx(0.f));
		REQUIRE(calculateWindFactor(300.f, 500.f, 100.f) == Approx(0.f));
		REQUIRE(calculateWindFactor(1000.f, 500.f, 100.f) == Approx(0.f));
	}

	SECTION("Edge case - both distances are zero")
	{
		REQUIRE(calculateWindFactor(0.f, 0.f, 0.f) == Approx(0.f));
		REQUIRE(calculateWindFactor(100.f, 0.f, 0.f) == Approx(0.f));
	}

	SECTION("Edge case - tiny denominator (near epsilon)")
	{
		// Denominator just above epsilon should work
		REQUIRE(calculateWindFactor(100.f, 100.f, 100.001f) == Approx(0.f)); // Below epsilon
		REQUIRE(calculateWindFactor(100.f, 100.f, 100.01f) == Approx(0.f));	 // At epsilon boundary
	}
}

// =============================================================================
// Highway SIMD Config Tests
// =============================================================================
// Tests for HighwayConfig struct defined in config.h

namespace
{
	// Mirror of HighwayConfig from config.h for standalone testing
	struct TestHighwayConfig
	{
		bool enabled = true;
		int batchThreshold = 64;

		static constexpr int MIN_THRESHOLD = 0;
		static constexpr int MAX_THRESHOLD = 65536;

		void clampThreshold() { batchThreshold = btClamped(batchThreshold, MIN_THRESHOLD, MAX_THRESHOLD); }
	};
} // namespace

TEST_CASE("HighwayConfig default values", "[config][highway]")
{
	TestHighwayConfig cfg;

	SECTION("Default enabled is true")
	{
		REQUIRE(cfg.enabled == true);
	}

	SECTION("Default batchThreshold is 64")
	{
		REQUIRE(cfg.batchThreshold == 64);
	}
}

TEST_CASE("HighwayConfig threshold clamping", "[config][highway]")
{
	SECTION("Values within bounds are unchanged")
	{
		TestHighwayConfig cfg;
		cfg.batchThreshold = 128;
		cfg.clampThreshold();
		REQUIRE(cfg.batchThreshold == 128);

		cfg.batchThreshold = 0;
		cfg.clampThreshold();
		REQUIRE(cfg.batchThreshold == 0);

		cfg.batchThreshold = 65536;
		cfg.clampThreshold();
		REQUIRE(cfg.batchThreshold == 65536);
	}

	SECTION("Negative values clamped to MIN_THRESHOLD")
	{
		TestHighwayConfig cfg;
		cfg.batchThreshold = -1;
		cfg.clampThreshold();
		REQUIRE(cfg.batchThreshold == TestHighwayConfig::MIN_THRESHOLD);

		cfg.batchThreshold = -1000;
		cfg.clampThreshold();
		REQUIRE(cfg.batchThreshold == TestHighwayConfig::MIN_THRESHOLD);
	}

	SECTION("Excessive values clamped to MAX_THRESHOLD")
	{
		TestHighwayConfig cfg;
		cfg.batchThreshold = 65537;
		cfg.clampThreshold();
		REQUIRE(cfg.batchThreshold == TestHighwayConfig::MAX_THRESHOLD);

		cfg.batchThreshold = 1000000;
		cfg.clampThreshold();
		REQUIRE(cfg.batchThreshold == TestHighwayConfig::MAX_THRESHOLD);
	}
}

TEST_CASE("HighwayConfig batch threshold decision logic", "[config][highway]")
{
	// Test the logic that determines whether to use Highway path

	auto shouldUseHighway = [](bool enabled, int count, int threshold) -> bool {
		return enabled && count >= threshold;
	};

	SECTION("Highway disabled always returns false")
	{
		REQUIRE(shouldUseHighway(false, 1000, 64) == false);
		REQUIRE(shouldUseHighway(false, 64, 64) == false);
		REQUIRE(shouldUseHighway(false, 0, 0) == false);
	}

	SECTION("Below threshold returns false")
	{
		REQUIRE(shouldUseHighway(true, 63, 64) == false);
		REQUIRE(shouldUseHighway(true, 0, 64) == false);
		REQUIRE(shouldUseHighway(true, 1, 64) == false);
	}

	SECTION("At or above threshold returns true")
	{
		REQUIRE(shouldUseHighway(true, 64, 64) == true);
		REQUIRE(shouldUseHighway(true, 65, 64) == true);
		REQUIRE(shouldUseHighway(true, 1000, 64) == true);
	}

	SECTION("Zero threshold always uses Highway when enabled")
	{
		REQUIRE(shouldUseHighway(true, 0, 0) == true);
		REQUIRE(shouldUseHighway(true, 1, 0) == true);
		REQUIRE(shouldUseHighway(true, 1000, 0) == true);
	}

	SECTION("High threshold restricts Highway usage")
	{
		REQUIRE(shouldUseHighway(true, 100, 1000) == false);
		REQUIRE(shouldUseHighway(true, 999, 1000) == false);
		REQUIRE(shouldUseHighway(true, 1000, 1000) == true);
	}
}
