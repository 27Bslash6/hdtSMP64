#include "../include/catch.hpp"

#include <atomic>
#include <cstdint>

// Standalone frame counter tests
// These validate overflow protection without requiring the full SkinnedMeshWorld

namespace test
{
	// Mirror the production implementation for testing
	// This allows us to test the logic without linking against the full physics engine
	class FrameCounter
	{
	public:
		static constexpr uint32_t RESET_THRESHOLD = 0xC0000000; // ~3 billion
		static constexpr uint32_t RESET_VALUE = 1;				// Reset to 1, not 0 (0 = uninitialized)

		uint32_t getCurrentFrame() const { return m_currentFrame; }

		void setFrame(uint32_t frame) { m_currentFrame = frame; }

		void incrementFrame()
		{
			// FIXED: Reset counter before it gets too high to prevent overflow
			// Reset to 1, not 0 (0 is the uninitialized value for m_lastUpdateFrame)
			if (m_currentFrame >= RESET_THRESHOLD) {
				m_currentFrame = RESET_VALUE;
			}
			else {
				++m_currentFrame;
			}
			m_loggingFrame.store(m_currentFrame, std::memory_order_relaxed);
		}

		uint32_t getLoggingFrame() const { return m_loggingFrame.load(std::memory_order_relaxed); }

	private:
		uint32_t m_currentFrame = 0;
		std::atomic<uint32_t> m_loggingFrame{0};
	};
} // namespace test

TEST_CASE("Frame counter overflow protection", "[frame_counter]")
{
	test::FrameCounter counter;

	SECTION("Normal increment works below threshold")
	{
		counter.setFrame(100);
		counter.incrementFrame();
		REQUIRE(counter.getCurrentFrame() == 101);
		REQUIRE(counter.getLoggingFrame() == 101);
	}

	SECTION("Counter at threshold - next increment should reset to 1")
	{
		// Set to exactly threshold value
		counter.setFrame(test::FrameCounter::RESET_THRESHOLD);
		counter.incrementFrame();

		// After incrementing from threshold, should reset to 1 (not 0, not threshold+1)
		REQUIRE(counter.getCurrentFrame() == test::FrameCounter::RESET_VALUE);
		REQUIRE(counter.getLoggingFrame() == test::FrameCounter::RESET_VALUE);
	}

	SECTION("Counter above threshold should reset to 1")
	{
		// Set above threshold
		counter.setFrame(test::FrameCounter::RESET_THRESHOLD + 100);
		counter.incrementFrame();

		// Should reset to 1
		REQUIRE(counter.getCurrentFrame() == test::FrameCounter::RESET_VALUE);
		REQUIRE(counter.getLoggingFrame() == test::FrameCounter::RESET_VALUE);
	}

	SECTION("Counter just below threshold increments normally")
	{
		counter.setFrame(test::FrameCounter::RESET_THRESHOLD - 1);
		counter.incrementFrame();

		// Should increment to threshold (not reset yet)
		REQUIRE(counter.getCurrentFrame() == test::FrameCounter::RESET_THRESHOLD);
	}

	SECTION("Zero is avoided to distinguish from uninitialized state")
	{
		// This tests that we never reset to 0, because 0 is the uninitialized value
		// for m_lastUpdateFrame in dirty flag comparisons
		counter.setFrame(test::FrameCounter::RESET_THRESHOLD);
		counter.incrementFrame();

		REQUIRE(counter.getCurrentFrame() != 0);
		REQUIRE(counter.getCurrentFrame() == 1);
	}

	SECTION("Near-overflow scenario - UINT32_MAX - 1")
	{
		// Edge case: counter is near UINT32_MAX
		counter.setFrame(0xFFFFFFFE);
		counter.incrementFrame();

		// Should reset to 1 (since 0xFFFFFFFE > RESET_THRESHOLD)
		REQUIRE(counter.getCurrentFrame() == test::FrameCounter::RESET_VALUE);
	}
}

TEST_CASE("Frame counter dirty flag interaction", "[frame_counter]")
{
	// Simulates the dirty flag pattern used in production:
	// - m_lastUpdateFrame initialized to 0
	// - Object is "dirty" if m_lastUpdateFrame != s_currentFrame
	// - Bug: if s_currentFrame wraps to 0, object looks clean when it's not

	uint32_t lastUpdateFrame = 0; // Simulates m_lastUpdateFrame (uninitialized/never updated)

	test::FrameCounter counter;

	SECTION("Fresh object needs update (lastUpdate=0, current=1)")
	{
		counter.setFrame(0);
		counter.incrementFrame(); // Now at 1

		bool needsUpdate = (lastUpdateFrame != counter.getCurrentFrame());
		REQUIRE(needsUpdate == true);
	}

	SECTION("Overflow to 0 would incorrectly skip update - MUST NOT HAPPEN")
	{
		// This test documents the bug we're fixing:
		// If counter overflows to 0, it matches uninitialized lastUpdateFrame

		counter.setFrame(test::FrameCounter::RESET_THRESHOLD);
		counter.incrementFrame();

		// With the fix, counter should be 1, not 0
		// So this check should pass (1 != 0, needs update)
		bool needsUpdate = (lastUpdateFrame != counter.getCurrentFrame());
		REQUIRE(needsUpdate == true);
		REQUIRE(counter.getCurrentFrame() != 0); // Never 0
	}
}
