/**
 * BUG-NEW-003 Regression Test: Non-atomic m_shutdown flag in ActorManager
 *
 * Bug Description:
 * ----------------
 * ActorManager::m_shutdown was a plain bool accessed by multiple threads:
 * - Written in onEvent(ShutdownEvent) BEFORE acquiring the lock
 * - Read in setSkeletonsActive() which can run on different threads
 * - Read in multiple onEvent handlers after acquiring the lock
 *
 * This is undefined behavior (data race) in C++. The fix is to use
 * std::atomic<bool> with proper memory ordering:
 * - store(true, std::memory_order_release) for the write
 * - load(std::memory_order_acquire) for all reads
 *
 * Test Strategy:
 * --------------
 * We test the pattern (not ActorManager directly since it requires game state).
 * The test verifies:
 * 1. Atomic flag provides thread-safe visibility
 * 2. Memory ordering ensures writes are visible to readers
 * 3. No torn reads occur under concurrent access
 */

#include "../include/catch.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

namespace
{
	/**
	 * Mock ActorManager demonstrating the BUGGY pattern.
	 * Plain bool written before lock, read without lock.
	 */
	class BuggyActorManager
	{
	public:
		bool m_shutdown = false; // BUG: Plain bool, data race!
		std::mutex m_lock;
		int processedCount = 0;

		void onShutdown()
		{
			m_shutdown = true; // BUG: Write before lock!
			std::lock_guard<std::mutex> l(m_lock);
			// cleanup...
		}

		void setSkeletonsActive()
		{
			if (m_shutdown) // BUG: Read without lock!
				return;
			// Note: This read races with onShutdown's write
			processedCount++;
		}

		void onFrameEvent()
		{
			std::lock_guard<std::mutex> l(m_lock);
			if (m_shutdown)
				return;
			setSkeletonsActive(); // Calls without holding lock on m_shutdown read
		}
	};

	/**
	 * Mock ActorManager with FIXED pattern.
	 * std::atomic<bool> with proper memory ordering.
	 */
	class FixedActorManager
	{
	public:
		std::atomic<bool> m_shutdown{false}; // FIX: Atomic with memory ordering
		std::mutex m_lock;
		std::atomic<int> processedCount{0};

		void onShutdown()
		{
			m_shutdown.store(true, std::memory_order_release); // FIX: Atomic store with release
			std::lock_guard<std::mutex> l(m_lock);
			// cleanup...
		}

		void setSkeletonsActive()
		{
			if (m_shutdown.load(std::memory_order_acquire)) // FIX: Atomic load with acquire
				return;
			processedCount.fetch_add(1, std::memory_order_relaxed);
		}

		void onFrameEvent()
		{
			std::lock_guard<std::mutex> l(m_lock);
			if (m_shutdown.load(std::memory_order_acquire))
				return;
			setSkeletonsActive();
		}
	};

} // namespace

TEST_CASE("BUG-NEW-003: Atomic shutdown flag prevents data race", "[threading][regression][atomic]")
{
	SECTION("Fixed version: shutdown visibility across threads")
	{
		FixedActorManager mgr;

		// Start frame event threads
		constexpr int NUM_FRAME_THREADS = 4;
		constexpr int ITERATIONS = 10000;

		std::atomic<bool> startFlag{false};
		std::atomic<int> threadsReady{0};
		std::vector<std::thread> frameThreads;

		for (int i = 0; i < NUM_FRAME_THREADS; ++i) {
			frameThreads.emplace_back([&mgr, &startFlag, &threadsReady]() {
				threadsReady.fetch_add(1);
				while (!startFlag.load(std::memory_order_acquire)) {
					std::this_thread::yield();
				}

				for (int j = 0; j < ITERATIONS; ++j) {
					mgr.onFrameEvent();
				}
			});
		}

		// Wait for all threads to be ready
		while (threadsReady.load() < NUM_FRAME_THREADS) {
			std::this_thread::yield();
		}

		// Start all threads simultaneously
		startFlag.store(true, std::memory_order_release);

		// After some processing, trigger shutdown
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		mgr.onShutdown();

		// Wait for all frame threads
		for (auto& t : frameThreads) {
			t.join();
		}

		// Key assertion: once shutdown is set, NO MORE processing should occur
		// Take a snapshot AFTER all threads complete
		int finalCount = mgr.processedCount.load();

		// Do more "frame events" after shutdown - should all be rejected
		for (int i = 0; i < 1000; ++i) {
			mgr.onFrameEvent();
		}

		// Count should not have increased after shutdown
		REQUIRE(mgr.processedCount.load() == finalCount);
	}

	SECTION("Fixed version: immediate shutdown visibility")
	{
		FixedActorManager mgr;

		// Set shutdown first
		mgr.onShutdown();

		// All subsequent frame events should be rejected
		for (int i = 0; i < 100; ++i) {
			mgr.onFrameEvent();
		}

		REQUIRE(mgr.processedCount.load() == 0);
	}

	SECTION("Fixed version: shutdown flag state transitions")
	{
		FixedActorManager mgr;

		// Initial state
		REQUIRE(mgr.m_shutdown.load(std::memory_order_acquire) == false);

		// After shutdown
		mgr.onShutdown();
		REQUIRE(mgr.m_shutdown.load(std::memory_order_acquire) == true);

		// Should remain true (shutdown is irreversible)
		REQUIRE(mgr.m_shutdown.load(std::memory_order_acquire) == true);
	}
}

TEST_CASE("BUG-NEW-003: Memory ordering correctness", "[threading][regression][atomic]")
{
	SECTION("Release-acquire ordering ensures visibility")
	{
		std::atomic<bool> flag{false};
		int data = 0;

		std::thread writer([&]() {
			data = 42;									 // Non-atomic write
			flag.store(true, std::memory_order_release); // Synchronizes-with reader's acquire
		});

		std::thread reader([&]() {
			while (!flag.load(std::memory_order_acquire)) {
				std::this_thread::yield();
			}
			// After acquire, we're guaranteed to see data = 42
			REQUIRE(data == 42);
		});

		writer.join();
		reader.join();
	}

	SECTION("Multiple readers see consistent state")
	{
		FixedActorManager mgr;
		std::atomic<int> readersSeenShutdown{0};
		constexpr int NUM_READERS = 8;

		std::atomic<bool> startFlag{false};
		std::vector<std::thread> readers;

		for (int i = 0; i < NUM_READERS; ++i) {
			readers.emplace_back([&mgr, &readersSeenShutdown, &startFlag]() {
				while (!startFlag.load(std::memory_order_acquire)) {
					std::this_thread::yield();
				}

				// Spin until shutdown is visible
				while (!mgr.m_shutdown.load(std::memory_order_acquire)) {
					std::this_thread::yield();
				}
				readersSeenShutdown.fetch_add(1);
			});
		}

		// Start all readers
		startFlag.store(true, std::memory_order_release);

		// Brief delay to let readers spin
		std::this_thread::sleep_for(std::chrono::microseconds(100));

		// Trigger shutdown
		mgr.onShutdown();

		// Wait for all readers
		for (auto& t : readers) {
			t.join();
		}

		// All readers must have seen the shutdown
		REQUIRE(readersSeenShutdown.load() == NUM_READERS);
	}
}

TEST_CASE("BUG-NEW-003: Stress test concurrent shutdown and events", "[threading][regression][atomic]")
{
	SECTION("Rapid shutdown during heavy event processing")
	{
		for (int trial = 0; trial < 10; ++trial) {
			FixedActorManager mgr;

			std::atomic<bool> done{false};
			std::vector<std::thread> eventThreads;

			// Start event threads
			for (int i = 0; i < 8; ++i) {
				eventThreads.emplace_back([&mgr, &done]() {
					while (!done.load(std::memory_order_acquire)) {
						mgr.onFrameEvent();
						std::this_thread::yield();
					}
				});
			}

			// Let events run briefly
			std::this_thread::sleep_for(std::chrono::milliseconds(1));

			// Shutdown
			mgr.onShutdown();

			// Stop all threads
			done.store(true, std::memory_order_release);

			for (auto& t : eventThreads) {
				t.join();
			}

			// Verify shutdown is set
			REQUIRE(mgr.m_shutdown.load(std::memory_order_acquire) == true);

			// Additional events after shutdown should not process
			int countBefore = mgr.processedCount.load();
			for (int i = 0; i < 100; ++i) {
				mgr.onFrameEvent();
			}
			REQUIRE(mgr.processedCount.load() == countBefore);
		}
	}
}
