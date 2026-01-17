#pragma once

#include "LinearMath/btMinMax.h"

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

	void loadConfig();
} // namespace hdt
