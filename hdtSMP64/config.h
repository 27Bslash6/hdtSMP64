#pragma once

#include "LinearMath/btMinMax.h"

#include <string>

namespace hdt
{
	// Highway SIMD batch configuration
	struct HighwayConfig
	{
		bool enabled = true;
		int batchThreshold = 64;

		static constexpr int MIN_THRESHOLD = 0;
		static constexpr int MAX_THRESHOLD = 65536;

		void clampThreshold() { batchThreshold = btClamped(batchThreshold, MIN_THRESHOLD, MAX_THRESHOLD); }
	};

	// Global Highway configuration - loaded once at startup
	extern HighwayConfig g_highwayConfig;

	// Benchmark mode configuration
	struct BenchmarkConfig
	{
		bool enabled = false;
		std::string saveName = ""; // Save file to auto-load (empty = manual load)
		int frames = 2000;
		bool exitWhenDone = true;
		bool suppressUI = true; // TODO: Hide HUD during benchmark
		bool quietMode = false; // TODO: Mute audio during benchmark

		static constexpr int MIN_FRAMES = 10;
		static constexpr int MAX_FRAMES = 10000;

		void clampFrames() { frames = btClamped(frames, MIN_FRAMES, MAX_FRAMES); }
	};

	// Global benchmark configuration - loaded at startup and when config reloads
	extern BenchmarkConfig g_benchmarkConfig;

	void loadConfig();
} // namespace hdt
