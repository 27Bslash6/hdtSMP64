#include "../include/catch.hpp"

// Include the actual implementation
#include "../../hdtSMP64/hdtSaveNameValidator.h"

TEST_CASE("Save name validation - path traversal prevention", "[security]")
{
	using hdt::security::isValidSaveName;

	SECTION("Valid save names are accepted")
	{
		// Normal save names that should pass
		REQUIRE(isValidSaveName("MySave") == true);
		REQUIRE(isValidSaveName("Save_001") == true);
		REQUIRE(isValidSaveName("My-Save-Game") == true);
		REQUIRE(isValidSaveName("Character Name Save") == true);
		REQUIRE(isValidSaveName("Save123") == true);
		REQUIRE(isValidSaveName("a") == true);
		REQUIRE(isValidSaveName("Save.autosave") == true); // Periods in middle OK
	}

	SECTION("Path traversal with ../ is rejected")
	{
		// Classic path traversal attacks
		REQUIRE(isValidSaveName("../evil") == false);
		REQUIRE(isValidSaveName("..\\evil") == false);
		REQUIRE(isValidSaveName("foo/../bar") == false);
		REQUIRE(isValidSaveName("foo\\..\\bar") == false);
		REQUIRE(isValidSaveName("../../etc/passwd") == false);
		REQUIRE(isValidSaveName("..") == false);
	}

	SECTION("Path separators are rejected")
	{
		// Forward and back slashes indicate path manipulation
		REQUIRE(isValidSaveName("foo/bar") == false);
		REQUIRE(isValidSaveName("foo\\bar") == false);
		REQUIRE(isValidSaveName("/absolute/path") == false);
		REQUIRE(isValidSaveName("C:\\Windows\\System32") == false);
	}

	SECTION("Empty names are rejected")
	{
		REQUIRE(isValidSaveName("") == false);
	}

	SECTION("Names exceeding 255 chars are rejected")
	{
		std::string longName(256, 'a');
		REQUIRE(isValidSaveName(longName) == false);

		// 255 chars should be fine
		std::string maxName(255, 'a');
		REQUIRE(isValidSaveName(maxName) == true);
	}

	SECTION("Hidden file names (starting with period) are rejected")
	{
		REQUIRE(isValidSaveName(".hidden") == false);
		REQUIRE(isValidSaveName(".") == false);
	}

	SECTION("Special characters beyond allowed set are rejected")
	{
		// Characters that could be problematic in file systems or shells
		REQUIRE(isValidSaveName("save<script>") == false);
		REQUIRE(isValidSaveName("save>output") == false);
		REQUIRE(isValidSaveName("save:alt") == false);
		REQUIRE(isValidSaveName("save\"quote") == false);
		REQUIRE(isValidSaveName("save|pipe") == false);
		REQUIRE(isValidSaveName("save?query") == false);
		REQUIRE(isValidSaveName("save*wild") == false);
		REQUIRE(isValidSaveName("save\ttab") == false);
		REQUIRE(isValidSaveName("save\nnewline") == false);
		// Null byte: must use string constructor with explicit length to include it
		REQUIRE(isValidSaveName(std::string("save\0null", 9)) == false);
	}

	SECTION("Allowed special characters work correctly")
	{
		// Underscore, hyphen, space, period (not at start) are allowed
		REQUIRE(isValidSaveName("save_with_underscores") == true);
		REQUIRE(isValidSaveName("save-with-hyphens") == true);
		REQUIRE(isValidSaveName("save with spaces") == true);
		REQUIRE(isValidSaveName("save.with.dots") == true);
	}

	SECTION("Unicode edge cases")
	{
		// Non-ASCII characters should be rejected for safety
		// (Windows filesystem can handle them, but we're being conservative)
		REQUIRE(isValidSaveName("save\xC0\x80") == false); // Overlong null
	}
}

TEST_CASE("Save name validation - boundary conditions", "[security]")
{
	using hdt::security::isValidSaveName;

	SECTION("Single character edge cases")
	{
		REQUIRE(isValidSaveName("a") == true);
		REQUIRE(isValidSaveName("1") == true);
		REQUIRE(isValidSaveName("_") == true);
		REQUIRE(isValidSaveName("-") == true);
		REQUIRE(isValidSaveName(" ") == true);	// Single space is technically valid
		REQUIRE(isValidSaveName(".") == false); // Single period is hidden/traversal
	}

	SECTION("Double dot variations")
	{
		// Ensure we catch all forms of parent directory traversal
		REQUIRE(isValidSaveName("..") == false);
		REQUIRE(isValidSaveName("a..") == false);  // Contains ..
		REQUIRE(isValidSaveName("..a") == false);  // Starts with ..
		REQUIRE(isValidSaveName("a..b") == false); // Contains ..
		REQUIRE(isValidSaveName("...") == false);  // Contains ..
	}
}

// ============================================================================
// SEC-002: Unbounded XML Resources (CWE-400)
// ============================================================================
// XML physics definitions are parsed without resource limits. A malicious XML
// file could add unlimited hull points, bone collision lists, shapes, or bones,
// causing memory exhaustion or CPU spike DoS.
//
// These tests verify that configurable limits are enforced.
// ============================================================================

// Include the XML limits header
#include "../../hdtSMP64/hdtXmlLimits.h"

#include <string>
#include <vector>

namespace hdt
{
	namespace xml_limits
	{
		// Simulates the hull point collection with limit enforcement
		class LimitedHullBuilder
		{
		public:
			size_t m_pointCount = 0;
			bool m_limitReached = false;

			bool addPoint(float x, float y, float z)
			{
				(void)x;
				(void)y;
				(void)z;
				if (m_pointCount >= MAX_HULL_POINTS) {
					if (!m_limitReached) {
						m_limitReached = true;
					}
					return false;
				}
				m_pointCount++;
				return true;
			}

			size_t getNumPoints() const { return m_pointCount; }
		};

		// Simulates collision list with limit enforcement
		class LimitedCollideList
		{
		public:
			std::vector<std::string> m_canCollide;
			std::vector<std::string> m_noCollide;
			bool m_limitReached = false;

			bool addCanCollide(const std::string& bone)
			{
				if (m_canCollide.size() >= MAX_COLLIDE_LIST) {
					if (!m_limitReached) {
						m_limitReached = true;
					}
					return false;
				}
				m_canCollide.push_back(bone);
				return true;
			}

			bool addNoCollide(const std::string& bone)
			{
				if (m_noCollide.size() >= MAX_COLLIDE_LIST) {
					if (!m_limitReached) {
						m_limitReached = true;
					}
					return false;
				}
				m_noCollide.push_back(bone);
				return true;
			}
		};

		// Simulates shape registry with limit enforcement
		class LimitedShapeRegistry
		{
		public:
			size_t m_shapeCount = 0;
			bool m_limitReached = false;

			bool registerShape(const std::string& name)
			{
				(void)name;
				if (m_shapeCount >= MAX_SHAPES) {
					if (!m_limitReached) {
						m_limitReached = true;
					}
					return false;
				}
				m_shapeCount++;
				return true;
			}

			size_t getShapeCount() const { return m_shapeCount; }
		};
	} // namespace xml_limits
} // namespace hdt

TEST_CASE("XML resource limits - constants are reasonable", "[security][xml]")
{
	using namespace hdt::xml_limits;

	SECTION("Hull point limit is reasonable for physics meshes")
	{
		// 512 points is generous for any physics collider
		// A complex helmet might have 100-200 points
		REQUIRE(MAX_HULL_POINTS >= 256);  // Must support complex shapes
		REQUIRE(MAX_HULL_POINTS <= 4096); // But not unlimited
	}

	SECTION("Collision list limit is reasonable for bone interactions")
	{
		// 64 bones in collision list covers most character rigs
		REQUIRE(MAX_COLLIDE_LIST >= 32);
		REQUIRE(MAX_COLLIDE_LIST <= 256);
	}

	SECTION("Shape limit is reasonable for physics systems")
	{
		// 256 shapes per system is very generous
		REQUIRE(MAX_SHAPES >= 64);
		REQUIRE(MAX_SHAPES <= 1024);
	}

	SECTION("Bone limit matches practical skeleton constraints")
	{
		// 256 bones covers even the most complex armatures
		REQUIRE(MAX_BONES >= 128);
		REQUIRE(MAX_BONES <= 512);
	}
}

TEST_CASE("XML resource limits - hull points are bounded", "[security][xml]")
{
	using namespace hdt::xml_limits;

	SECTION("Points up to limit are accepted")
	{
		LimitedHullBuilder hull;

		// Add points up to the limit
		for (size_t i = 0; i < MAX_HULL_POINTS; i++) {
			bool added = hull.addPoint(static_cast<float>(i), 0.0f, 0.0f);
			REQUIRE(added == true);
		}

		REQUIRE(hull.getNumPoints() == MAX_HULL_POINTS);
		REQUIRE(hull.m_limitReached == false);
	}

	SECTION("Points beyond limit are rejected")
	{
		LimitedHullBuilder hull;

		// Fill to limit
		for (size_t i = 0; i < MAX_HULL_POINTS; i++) {
			hull.addPoint(static_cast<float>(i), 0.0f, 0.0f);
		}

		// Attempt to add one more
		bool added = hull.addPoint(999.0f, 999.0f, 999.0f);
		REQUIRE(added == false);
		REQUIRE(hull.getNumPoints() == MAX_HULL_POINTS); // Count unchanged
		REQUIRE(hull.m_limitReached == true);

		// Further attempts also rejected
		bool added2 = hull.addPoint(1000.0f, 1000.0f, 1000.0f);
		REQUIRE(added2 == false);
	}

	SECTION("Malicious file with 10000 points is bounded")
	{
		LimitedHullBuilder hull;
		size_t accepted = 0;
		size_t rejected = 0;

		// Simulate malicious XML with 10000 points
		for (size_t i = 0; i < 10000; i++) {
			if (hull.addPoint(static_cast<float>(i), 0.0f, 0.0f))
				accepted++;
			else
				rejected++;
		}

		REQUIRE(accepted == MAX_HULL_POINTS);
		REQUIRE(rejected == 10000 - MAX_HULL_POINTS);
		REQUIRE(hull.m_limitReached == true);
	}
}

TEST_CASE("XML resource limits - collision lists are bounded", "[security][xml]")
{
	using namespace hdt::xml_limits;

	SECTION("Entries up to limit are accepted")
	{
		LimitedCollideList list;

		for (size_t i = 0; i < MAX_COLLIDE_LIST; i++) {
			bool added = list.addCanCollide("bone_" + std::to_string(i));
			REQUIRE(added == true);
		}

		REQUIRE(list.m_canCollide.size() == MAX_COLLIDE_LIST);
	}

	SECTION("Entries beyond limit are rejected")
	{
		LimitedCollideList list;

		// Fill to limit
		for (size_t i = 0; i < MAX_COLLIDE_LIST; i++) {
			list.addCanCollide("bone_" + std::to_string(i));
		}

		// Attempt to add one more
		bool added = list.addCanCollide("evil_bone");
		REQUIRE(added == false);
		REQUIRE(list.m_canCollide.size() == MAX_COLLIDE_LIST);
		REQUIRE(list.m_limitReached == true);
	}

	SECTION("Both can-collide and no-collide have independent limits")
	{
		LimitedCollideList list;

		// Fill can-collide to limit
		for (size_t i = 0; i < MAX_COLLIDE_LIST; i++) {
			list.addCanCollide("can_" + std::to_string(i));
		}

		// no-collide should still accept entries
		bool added = list.addNoCollide("no_0");
		REQUIRE(added == true);

		// Fill no-collide to limit
		for (size_t i = 1; i < MAX_COLLIDE_LIST; i++) {
			list.addNoCollide("no_" + std::to_string(i));
		}

		// Both should be at limit
		REQUIRE(list.m_canCollide.size() == MAX_COLLIDE_LIST);
		REQUIRE(list.m_noCollide.size() == MAX_COLLIDE_LIST);

		// Both should reject further additions
		REQUIRE(list.addCanCollide("extra_can") == false);
		REQUIRE(list.addNoCollide("extra_no") == false);
	}
}

TEST_CASE("XML resource limits - shapes are bounded", "[security][xml]")
{
	using namespace hdt::xml_limits;

	SECTION("Shapes up to limit are accepted")
	{
		LimitedShapeRegistry registry;

		for (size_t i = 0; i < MAX_SHAPES; i++) {
			bool registered = registry.registerShape("shape_" + std::to_string(i));
			REQUIRE(registered == true);
		}

		REQUIRE(registry.getShapeCount() == MAX_SHAPES);
	}

	SECTION("Shapes beyond limit are rejected")
	{
		LimitedShapeRegistry registry;

		// Fill to limit
		for (size_t i = 0; i < MAX_SHAPES; i++) {
			registry.registerShape("shape_" + std::to_string(i));
		}

		// Attempt to add one more
		bool registered = registry.registerShape("evil_shape");
		REQUIRE(registered == false);
		REQUIRE(registry.getShapeCount() == MAX_SHAPES);
		REQUIRE(registry.m_limitReached == true);
	}

	SECTION("Malicious file with 1000 shapes is bounded")
	{
		LimitedShapeRegistry registry;
		size_t accepted = 0;

		for (size_t i = 0; i < 1000; i++) {
			if (registry.registerShape("shape_" + std::to_string(i)))
				accepted++;
		}

		REQUIRE(accepted == MAX_SHAPES);
		REQUIRE(registry.m_limitReached == true);
	}
}
