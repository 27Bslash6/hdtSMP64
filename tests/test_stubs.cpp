// Test stubs for symbols needed by Highway solver tests
// Avoids pulling in full hdtSMP64 dependencies

#include "../hdtSMP64/config.h"

namespace hdt
{
	// Global Highway configuration instance
	// Tests can modify this to test different configurations
	HighwayConfig g_highwayConfig = {
		true, // enabled
		64	  // batchThreshold
	};
} // namespace hdt
