#pragma once

// ============================================================================
// SEC-002: XML Resource Limits (CWE-400 Defense)
// ============================================================================
// These constants define maximum limits for resources parsed from physics XML
// files. They prevent denial-of-service attacks via maliciously crafted XML
// that could cause memory exhaustion or CPU spikes.
//
// Values are generous enough for legitimate physics mods while preventing abuse.
// ============================================================================

#include <cstddef>

namespace hdt
{
	namespace xml_limits
	{
		// Maximum hull points for a btConvexHullShape.
		// A complex helmet might have 100-200 points; 512 is very generous.
		constexpr size_t MAX_HULL_POINTS = 512;

		// Maximum bone names in can-collide-with-bone or no-collide-with-bone lists.
		// Most character rigs have < 64 bones total; 64 per list is sufficient.
		constexpr size_t MAX_COLLIDE_LIST = 64;

		// Maximum shape definitions per physics system.
		// Even complex armor sets rarely exceed 50 shapes; 256 is generous.
		constexpr size_t MAX_SHAPES = 256;

		// Maximum bone templates per physics system.
		// Matches Skyrim skeleton constraints; even heavily modified armatures < 256.
		constexpr size_t MAX_BONES = 256;
	} // namespace xml_limits
} // namespace hdt
