/**
 * BUG-003 Regression Test: FrameSyncEvent early return doesn't signal completion
 *
 * Bug Description:
 * ----------------
 * When SkyrimPhysicsWorld::onEvent(FrameSyncEvent) returns early due to m_suspended=true,
 * it fails to signal m_frameSyncComplete, leaving suspend() waiting for 5-second timeout.
 *
 * The issue is in hdtSkyrimPhysicsWorld.cpp:428-463:
 *   void SkyrimPhysicsWorld::onEvent(const FrameSyncEvent& e)
 *   {
 *       if (m_suspended) {
 *           return;  // BUG: doesn't signal m_frameSyncComplete!
 *       }
 *       // ... later code does:
 *       m_frameSyncComplete.store(true, std::memory_order_release);
 *       m_frameSyncCV.notify_all();
 *   }
 *
 * Fix:
 * ----
 * Signal completion in the early return path:
 *   if (m_suspended) {
 *       m_frameSyncComplete.store(true, std::memory_order_release);
 *       m_frameSyncCV.notify_all();
 *       return;
 *   }
 *
 * This test uses a mock that simulates the synchronization behavior without
 * depending on the actual SkyrimPhysicsWorld class (which has heavy dependencies).
 */

#include "../include/catch.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <vector>

// Cross-platform getenv wrapper (MSVC deprecates std::getenv)
inline const char* safe_getenv(const char* name)
{
#ifdef _MSC_VER
	char* value = nullptr;
	size_t len = 0;
	if (_dupenv_s(&value, &len, name) == 0 && value != nullptr) {
		// Note: This leaks memory, but it's only used in tests
		return value;
	}
	return nullptr;
#else
	return std::getenv(name);
#endif
}

namespace
{
	/**
	 * Mock of SkyrimPhysicsWorld synchronization behavior.
	 * Simulates the suspend/resume/FrameSyncEvent interaction.
	 */
	class MockPhysicsWorld
	{
	public:
		std::atomic<bool> m_suspended{false};
		std::atomic<bool> m_frameSyncComplete{true};
		std::mutex m_frameSyncMutex;
		std::condition_variable m_frameSyncCV;

		/**
		 * BUGGY version: Early return doesn't signal completion.
		 * This matches the current broken behavior in hdtSkyrimPhysicsWorld.cpp:428-433
		 */
		void onFrameSyncEventBuggy()
		{
			if (m_suspended) {
				return; // BUG: doesn't signal m_frameSyncComplete!
			}

			// Simulate doing some work (m_tasks.wait())
			std::this_thread::sleep_for(std::chrono::milliseconds(1));

			// Signal completion
			m_frameSyncComplete.store(true, std::memory_order_release);
			m_frameSyncCV.notify_all();
		}

		/**
		 * FIXED version: Early return signals completion.
		 */
		void onFrameSyncEventFixed()
		{
			if (m_suspended) {
				m_frameSyncComplete.store(true, std::memory_order_release);
				m_frameSyncCV.notify_all();
				return;
			}

			// Simulate doing some work (m_tasks.wait())
			std::this_thread::sleep_for(std::chrono::milliseconds(1));

			// Signal completion
			m_frameSyncComplete.store(true, std::memory_order_release);
			m_frameSyncCV.notify_all();
		}

		/**
		 * Simulates FrameEvent starting physics work.
		 * Sets m_frameSyncComplete = false to indicate work is in progress.
		 */
		void onFrameEvent()
		{
			if (m_suspended) {
				return;
			}
			m_frameSyncComplete.store(false, std::memory_order_release);
		}

		/**
		 * Suspend implementation (matches hdtSkyrimPhysicsWorld.h:49-88).
		 * Returns true if completed within timeout, false if timed out.
		 */
		bool suspend(std::chrono::milliseconds timeout = std::chrono::milliseconds(100))
		{
			m_suspended = true;

			std::unique_lock<std::mutex> lk(m_frameSyncMutex);
			bool completed = m_frameSyncCV.wait_for(lk, timeout, [this] {
				return m_frameSyncComplete.load(std::memory_order_acquire);
			});
			return completed;
		}

		void resume() { m_suspended = false; }
	};

} // namespace

TEST_CASE("BUG-003: FrameSyncEvent early return must signal completion", "[sync][regression]")
{
	SECTION("BUGGY: Early return causes suspend() timeout")
	{
		MockPhysicsWorld world;

		// Simulate a physics frame starting
		world.onFrameEvent();
		REQUIRE(world.m_frameSyncComplete.load() == false);

		// Simulate console thread calling suspend while frame is in progress
		std::thread consoleThread([&world]() {
			// This will wait for FrameSyncEvent to signal completion
			// With buggy code, it will timeout because early return doesn't signal
			bool completed = world.suspend(std::chrono::milliseconds(50));

			// BUGGY behavior: should timeout because FrameSyncEvent doesn't signal
			// when m_suspended is already true (the early return path)
			CHECK(completed == false); // Expect timeout with buggy code
		});

		// Give console thread time to enter wait
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		// FrameSyncEvent fires while m_suspended is true (console thread set it)
		// BUGGY: This early returns without signaling
		world.onFrameSyncEventBuggy();

		consoleThread.join();

		// m_frameSyncComplete should still be false because buggy code didn't signal it
		// Wait was satisfied by timeout, not by proper signaling
		// The atomic should remain false since the buggy code never set it
		REQUIRE(world.m_frameSyncComplete.load() == false);
	}

	SECTION("FIXED: Early return signals completion properly")
	{
		MockPhysicsWorld world;

		// Simulate a physics frame starting
		world.onFrameEvent();
		REQUIRE(world.m_frameSyncComplete.load() == false);

		std::atomic<bool> suspendCompleted{false};

		// Simulate console thread calling suspend while frame is in progress
		std::thread consoleThread([&world, &suspendCompleted]() {
			bool completed = world.suspend(std::chrono::milliseconds(500));
			suspendCompleted.store(completed);
		});

		// Give console thread time to enter wait
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		// FrameSyncEvent fires while m_suspended is true (console thread set it)
		// FIXED: This signals completion even on early return
		world.onFrameSyncEventFixed();

		consoleThread.join();

		// With fixed code:
		// 1. suspend() should complete without timeout
		// 2. m_frameSyncComplete should be true
		REQUIRE(suspendCompleted.load() == true);
		REQUIRE(world.m_frameSyncComplete.load() == true);
	}

	SECTION("Normal path: FrameSyncEvent signals when not suspended")
	{
		MockPhysicsWorld world;

		// Not suspended - normal physics frame
		world.onFrameEvent();
		REQUIRE(world.m_frameSyncComplete.load() == false);

		// FrameSyncEvent completes normally
		world.onFrameSyncEventFixed();

		REQUIRE(world.m_frameSyncComplete.load() == true);
	}
}

TEST_CASE("BUG-003: Race between suspend and FrameSyncEvent", "[sync][regression]")
{
	SECTION("Multiple rapid suspend/resume cycles with fixed code")
	{
		MockPhysicsWorld world;

		constexpr int CYCLES = 50;
		std::atomic<int> successCount{0};
		std::atomic<int> timeoutCount{0};

		for (int i = 0; i < CYCLES; ++i) {
			world.resume();

			// Start a physics frame
			world.onFrameEvent();

			// Race: suspend and FrameSyncEvent happen concurrently
			std::thread suspendThread([&world, &successCount, &timeoutCount]() {
				bool completed = world.suspend(std::chrono::milliseconds(100));
				if (completed)
					successCount.fetch_add(1);
				else
					timeoutCount.fetch_add(1);
			});

			// Small random delay to vary the race timing
			std::this_thread::sleep_for(std::chrono::microseconds(i * 10 % 100));

			// FrameSyncEvent fires (fixed version)
			world.onFrameSyncEventFixed();

			suspendThread.join();
		}

		INFO("Successes: " << successCount.load() << ", Timeouts: " << timeoutCount.load());
		// With fixed code, all suspends should complete successfully (no timeouts)
		REQUIRE(timeoutCount.load() == 0);
		REQUIRE(successCount.load() == CYCLES);
	}
}

TEST_CASE("BUG-003: Concurrent FrameEvents during suspend", "[sync][regression]")
{
	SECTION("FrameSyncEvent while already suspended signals correctly")
	{
		MockPhysicsWorld world;

		// Start suspended
		world.m_suspended = true;
		world.m_frameSyncComplete = false;

		// Multiple FrameSyncEvents fire while suspended (edge case)
		for (int i = 0; i < 5; ++i) {
			world.onFrameSyncEventFixed();
			// Each call should signal completion
			REQUIRE(world.m_frameSyncComplete.load() == true);

			// Reset for next iteration
			world.m_frameSyncComplete = false;
		}
	}

	SECTION("BUGGY: Multiple FrameSyncEvents while suspended never signal")
	{
		MockPhysicsWorld world;

		// Start suspended
		world.m_suspended = true;
		world.m_frameSyncComplete = false;

		// Multiple FrameSyncEvents fire while suspended
		for (int i = 0; i < 5; ++i) {
			world.onFrameSyncEventBuggy();
			// BUGGY: completion never signaled
			REQUIRE(world.m_frameSyncComplete.load() == false);
		}
	}
}

TEST_CASE("BUG-003: Stress test suspend synchronization", "[sync][regression][stress]")
{
	SECTION("High-frequency suspend/resume with fixed code")
	{
		MockPhysicsWorld world;

		constexpr int ITERATIONS = 100;
		std::atomic<int> frameCount{0};
		std::atomic<int> suspendSuccessCount{0};
		std::atomic<bool> running{true};

		// Game thread: continuously fires FrameEvent and FrameSyncEvent
		std::thread gameThread([&]() {
			while (running.load()) {
				if (!world.m_suspended.load()) {
					world.onFrameEvent();
					std::this_thread::sleep_for(std::chrono::microseconds(100));
					world.onFrameSyncEventFixed();
					frameCount.fetch_add(1);
				}
				std::this_thread::yield();
			}
		});

		// Console thread: periodically suspends and resumes
		std::thread consoleThread([&]() {
			for (int i = 0; i < ITERATIONS; ++i) {
				std::this_thread::sleep_for(std::chrono::milliseconds(1));

				bool completed = world.suspend(std::chrono::milliseconds(50));
				if (completed)
					suspendSuccessCount.fetch_add(1);

				// Do some "work" while suspended
				std::this_thread::sleep_for(std::chrono::microseconds(500));

				world.resume();
			}
		});

		consoleThread.join();
		running.store(false);
		gameThread.join();

		INFO("Frames: " << frameCount.load() << ", Successful suspends: " << suspendSuccessCount.load());

		// All suspend operations should complete successfully
		REQUIRE(suspendSuccessCount.load() == ITERATIONS);
	}
}

// ============================================================================
// BUG-001: suspend() Doesn't Wait for Collision Workers (CRITICAL)
// ============================================================================
//
// Bug Description:
// ----------------
// suspend() only waits for AsyncTaskGroup::m_tasks, but collision workers
// spawned via hdt_parallel_for_each use the global enkiTS scheduler and are
// NOT tracked. This causes crashes when "smp reset" is triggered during
// active collision processing.
//
// Crash Scenario:
// 1. User triggers "smp reset" during active collision processing
// 2. suspend() sets m_suspended = true, calls m_tasks.wait()
// 3. m_tasks.wait() returns (outer tasks done)
// 4. Physics state cleared, bodies freed/reallocated
// 5. CRASH: Collision workers (still running) access freed memory
//
// Fix:
// ----
// Add WorkerScope RAII class to CollisionDispatcher that tracks active
// workers via std::atomic<int>. suspend() must call waitForCollisionWorkers()
// after m_tasks.wait().

namespace
{
	/**
	 * Mock of CollisionDispatcher worker tracking infrastructure.
	 * This simulates the fix needed in hdtDispatcher.h
	 */
	class MockCollisionDispatcher
	{
	public:
		std::atomic<int> m_activeCollisionWorkers{0};
		std::atomic<bool> m_cancelCollisions{false};

		/**
		 * RAII scope guard for worker tracking.
		 * Increments counter on construction, decrements on destruction.
		 */
		struct WorkerScope
		{
			MockCollisionDispatcher* d;

			explicit WorkerScope(MockCollisionDispatcher* dispatcher) : d(dispatcher)
			{
				d->m_activeCollisionWorkers.fetch_add(1, std::memory_order_acq_rel);
			}

			~WorkerScope() { d->m_activeCollisionWorkers.fetch_sub(1, std::memory_order_acq_rel); }

			// Non-copyable, non-movable
			WorkerScope(const WorkerScope&) = delete;
			WorkerScope& operator=(const WorkerScope&) = delete;
		};

		void waitForCollisionWorkers()
		{
			while (m_activeCollisionWorkers.load(std::memory_order_acquire) > 0) {
				std::this_thread::yield();
			}
		}

		void requestCollisionCancellation() { m_cancelCollisions.store(true, std::memory_order_release); }

		void clearCollisionCancellation() { m_cancelCollisions.store(false, std::memory_order_release); }

		bool isCancelled() const { return m_cancelCollisions.load(std::memory_order_acquire); }

		int getActiveWorkerCount() const { return m_activeCollisionWorkers.load(std::memory_order_acquire); }
	};

	/**
	 * Extended MockPhysicsWorld with collision dispatcher integration.
	 */
	class MockPhysicsWorldWithDispatcher
	{
	public:
		std::atomic<bool> m_suspended{false};
		std::atomic<bool> m_frameSyncComplete{true};
		std::mutex m_frameSyncMutex;
		std::condition_variable m_frameSyncCV;
		MockCollisionDispatcher m_dispatcher;

		/**
		 * BUGGY suspend: Only waits for frame sync, ignores collision workers.
		 */
		bool suspendBuggy(std::chrono::milliseconds timeout = std::chrono::milliseconds(100))
		{
			m_suspended = true;

			std::unique_lock<std::mutex> lk(m_frameSyncMutex);
			bool completed = m_frameSyncCV.wait_for(lk, timeout, [this] {
				return m_frameSyncComplete.load(std::memory_order_acquire);
			});
			// BUG: Does NOT wait for collision workers!
			return completed;
		}

		/**
		 * FIXED suspend: Waits for both frame sync AND collision workers.
		 */
		bool suspendFixed(std::chrono::milliseconds timeout = std::chrono::milliseconds(100))
		{
			m_dispatcher.requestCollisionCancellation();
			m_suspended = true;

			std::unique_lock<std::mutex> lk(m_frameSyncMutex);
			bool completed = m_frameSyncCV.wait_for(lk, timeout, [this] {
				return m_frameSyncComplete.load(std::memory_order_acquire);
			});

			// FIX: Wait for collision workers to complete
			m_dispatcher.waitForCollisionWorkers();
			return completed;
		}

		void resume()
		{
			m_dispatcher.clearCollisionCancellation();
			m_suspended = false;
		}

		void signalFrameComplete()
		{
			m_frameSyncComplete.store(true, std::memory_order_release);
			m_frameSyncCV.notify_all();
		}

		void startFrame() { m_frameSyncComplete.store(false, std::memory_order_release); }
	};

} // namespace

TEST_CASE("BUG-001: WorkerScope RAII counter behavior", "[dispatcher][regression]")
{
	MockCollisionDispatcher dispatcher;

	SECTION("WorkerScope increments counter on construction")
	{
		REQUIRE(dispatcher.getActiveWorkerCount() == 0);

		{
			MockCollisionDispatcher::WorkerScope scope(&dispatcher);
			REQUIRE(dispatcher.getActiveWorkerCount() == 1);
		}

		REQUIRE(dispatcher.getActiveWorkerCount() == 0);
	}

	SECTION("Multiple WorkerScopes stack correctly")
	{
		REQUIRE(dispatcher.getActiveWorkerCount() == 0);

		{
			MockCollisionDispatcher::WorkerScope scope1(&dispatcher);
			REQUIRE(dispatcher.getActiveWorkerCount() == 1);

			{
				MockCollisionDispatcher::WorkerScope scope2(&dispatcher);
				REQUIRE(dispatcher.getActiveWorkerCount() == 2);

				{
					MockCollisionDispatcher::WorkerScope scope3(&dispatcher);
					REQUIRE(dispatcher.getActiveWorkerCount() == 3);
				}

				REQUIRE(dispatcher.getActiveWorkerCount() == 2);
			}

			REQUIRE(dispatcher.getActiveWorkerCount() == 1);
		}

		REQUIRE(dispatcher.getActiveWorkerCount() == 0);
	}

	SECTION("WorkerScope decrements on exception")
	{
		REQUIRE(dispatcher.getActiveWorkerCount() == 0);

		try {
			MockCollisionDispatcher::WorkerScope scope(&dispatcher);
			REQUIRE(dispatcher.getActiveWorkerCount() == 1);
			throw std::runtime_error("test exception");
		}
		catch (...) {
			// Exception caught, scope should have decremented
		}

		REQUIRE(dispatcher.getActiveWorkerCount() == 0);
	}

	SECTION("Concurrent WorkerScopes are thread-safe")
	{
		constexpr int NUM_THREADS = 10;
		constexpr int ITERATIONS = 1000;
		std::atomic<int> maxObserved{0};

		std::vector<std::thread> threads;
		for (int t = 0; t < NUM_THREADS; ++t) {
			threads.emplace_back([&dispatcher, &maxObserved]() {
				for (int i = 0; i < ITERATIONS; ++i) {
					MockCollisionDispatcher::WorkerScope scope(&dispatcher);
					int current = dispatcher.getActiveWorkerCount();
					int prev = maxObserved.load();
					while (current > prev && !maxObserved.compare_exchange_weak(prev, current)) {
						prev = maxObserved.load();
					}
					std::this_thread::yield();
				}
			});
		}

		for (auto& t : threads) {
			t.join();
		}

		// After all threads complete, counter must be zero
		REQUIRE(dispatcher.getActiveWorkerCount() == 0);
		// At peak, we should have seen multiple concurrent workers
		INFO("Max concurrent workers observed: " << maxObserved.load());
		REQUIRE(maxObserved.load() >= 1);
	}
}

TEST_CASE("BUG-001: waitForCollisionWorkers blocks until workers complete", "[dispatcher][regression]")
{
	MockCollisionDispatcher dispatcher;

	SECTION("waitForCollisionWorkers returns immediately when no workers")
	{
		REQUIRE(dispatcher.getActiveWorkerCount() == 0);

		auto start = std::chrono::steady_clock::now();
		dispatcher.waitForCollisionWorkers();
		auto elapsed = std::chrono::steady_clock::now() - start;

		// Should return almost immediately (< 1ms)
		REQUIRE(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() < 10);
	}

	SECTION("waitForCollisionWorkers blocks while workers active")
	{
		std::atomic<bool> workerStarted{false};
		std::atomic<bool> workerShouldExit{false};
		std::atomic<bool> waitCompleted{false};

		// Spawn a simulated worker that holds the counter
		std::thread workerThread([&]() {
			MockCollisionDispatcher::WorkerScope scope(&dispatcher);
			workerStarted.store(true);
			while (!workerShouldExit.load()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		});

		// Wait for worker to start
		while (!workerStarted.load()) {
			std::this_thread::yield();
		}
		REQUIRE(dispatcher.getActiveWorkerCount() == 1);

		// Start a thread that will call waitForCollisionWorkers
		std::thread waitThread([&]() {
			dispatcher.waitForCollisionWorkers();
			waitCompleted.store(true);
		});

		// Give the wait thread time to start waiting
		std::this_thread::sleep_for(std::chrono::milliseconds(20));

		// Wait should NOT have completed yet (worker still active)
		REQUIRE(waitCompleted.load() == false);
		REQUIRE(dispatcher.getActiveWorkerCount() == 1);

		// Signal worker to exit
		workerShouldExit.store(true);
		workerThread.join();

		// Now waitForCollisionWorkers should complete
		waitThread.join();
		REQUIRE(waitCompleted.load() == true);
		REQUIRE(dispatcher.getActiveWorkerCount() == 0);
	}
}

TEST_CASE("BUG-001: Cancellation flag behavior", "[dispatcher][regression]")
{
	MockCollisionDispatcher dispatcher;

	SECTION("Cancellation flag starts false")
	{
		REQUIRE(dispatcher.isCancelled() == false);
	}

	SECTION("requestCollisionCancellation sets flag")
	{
		dispatcher.requestCollisionCancellation();
		REQUIRE(dispatcher.isCancelled() == true);
	}

	SECTION("clearCollisionCancellation clears flag")
	{
		dispatcher.requestCollisionCancellation();
		REQUIRE(dispatcher.isCancelled() == true);

		dispatcher.clearCollisionCancellation();
		REQUIRE(dispatcher.isCancelled() == false);
	}

	SECTION("Workers can check cancellation flag")
	{
		std::atomic<int> iterationsCompleted{0};
		std::atomic<bool> workerExitedEarly{false};

		// Simulate a worker that checks cancellation
		std::thread workerThread([&]() {
			MockCollisionDispatcher::WorkerScope scope(&dispatcher);

			for (int i = 0; i < 100; ++i) {
				if (dispatcher.isCancelled()) {
					workerExitedEarly.store(true);
					return;
				}
				iterationsCompleted.fetch_add(1);
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
		});

		// Let worker run a bit
		std::this_thread::sleep_for(std::chrono::milliseconds(25));

		// Request cancellation
		dispatcher.requestCollisionCancellation();

		workerThread.join();

		// Worker should have exited early
		REQUIRE(workerExitedEarly.load() == true);
		// Should have completed some but not all iterations
		INFO("Iterations completed: " << iterationsCompleted.load());
		REQUIRE(iterationsCompleted.load() < 100);
		REQUIRE(iterationsCompleted.load() > 0);
	}
}

TEST_CASE("BUG-001: BUGGY suspend doesn't wait for collision workers", "[dispatcher][regression]")
{
	MockPhysicsWorldWithDispatcher world;

	SECTION("Buggy suspend returns while workers still active")
	{
		world.startFrame();
		world.signalFrameComplete(); // Frame sync is done

		std::atomic<bool> workerStarted{false};
		std::atomic<bool> workerFinished{false};

		// Simulate a long-running collision worker
		std::thread workerThread([&]() {
			MockCollisionDispatcher::WorkerScope scope(&world.m_dispatcher);
			workerStarted.store(true);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			workerFinished.store(true);
		});

		// Wait for worker to start
		while (!workerStarted.load()) {
			std::this_thread::yield();
		}

		// BUGGY suspend - returns immediately after frame sync
		bool completed = world.suspendBuggy(std::chrono::milliseconds(50));
		REQUIRE(completed == true); // Frame sync completed

		// BUG: Worker is still running!
		REQUIRE(world.m_dispatcher.getActiveWorkerCount() == 1);
		REQUIRE(workerFinished.load() == false);

		// This is the crash scenario: code proceeds to free memory
		// while worker is still accessing it

		workerThread.join();
	}
}

TEST_CASE("BUG-001: FIXED suspend waits for collision workers", "[dispatcher][regression]")
{
	MockPhysicsWorldWithDispatcher world;

	SECTION("Fixed suspend waits for workers before returning")
	{
		world.startFrame();
		world.signalFrameComplete(); // Frame sync is done

		std::atomic<bool> workerStarted{false};
		std::atomic<bool> workerFinished{false};

		// Simulate a long-running collision worker
		std::thread workerThread([&]() {
			MockCollisionDispatcher::WorkerScope scope(&world.m_dispatcher);
			workerStarted.store(true);
			// Check cancellation periodically
			for (int i = 0; i < 20; ++i) {
				if (world.m_dispatcher.isCancelled()) {
					workerFinished.store(true);
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
			workerFinished.store(true);
		});

		// Wait for worker to start
		while (!workerStarted.load()) {
			std::this_thread::yield();
		}

		// FIXED suspend - waits for collision workers
		bool completed = world.suspendFixed(std::chrono::milliseconds(500));
		REQUIRE(completed == true);

		// FIX: Worker must be done by now
		REQUIRE(world.m_dispatcher.getActiveWorkerCount() == 0);
		REQUIRE(workerFinished.load() == true);

		// Safe to proceed with memory operations
		workerThread.join();
	}

	SECTION("Fixed suspend sets cancellation flag before waiting")
	{
		world.startFrame();
		world.signalFrameComplete();

		std::atomic<bool> sawCancellation{false};

		std::thread workerThread([&]() {
			MockCollisionDispatcher::WorkerScope scope(&world.m_dispatcher);
			// Busy loop checking cancellation
			for (int i = 0; i < 1000; ++i) {
				if (world.m_dispatcher.isCancelled()) {
					sawCancellation.store(true);
					return;
				}
				std::this_thread::sleep_for(std::chrono::microseconds(100));
			}
		});

		std::this_thread::sleep_for(std::chrono::milliseconds(5));

		world.suspendFixed(std::chrono::milliseconds(500));

		workerThread.join();

		// Worker should have seen the cancellation flag
		REQUIRE(sawCancellation.load() == true);
	}
}

TEST_CASE("BUG-001: Stress test suspend with collision workers", "[dispatcher][regression][stress]")
{
	SECTION("Rapid suspend/resume with concurrent collision workers")
	{
		MockPhysicsWorldWithDispatcher world;

		constexpr int SUSPEND_CYCLES = 50;
		constexpr int WORKERS_PER_CYCLE = 5;
		std::atomic<int> suspendSuccessCount{0};
		std::atomic<int> totalWorkersCompleted{0};
		std::atomic<bool> running{true};

		// Worker spawner thread - simulates collision dispatch
		std::thread workerSpawner([&]() {
			while (running.load()) {
				if (!world.m_suspended.load()) {
					std::vector<std::thread> workers;
					for (int i = 0; i < WORKERS_PER_CYCLE; ++i) {
						workers.emplace_back([&]() {
							MockCollisionDispatcher::WorkerScope scope(&world.m_dispatcher);
							// Simulate work, checking cancellation
							for (int j = 0; j < 10; ++j) {
								if (world.m_dispatcher.isCancelled())
									break;
								std::this_thread::sleep_for(std::chrono::microseconds(100));
							}
							totalWorkersCompleted.fetch_add(1);
						});
					}
					for (auto& w : workers) {
						w.join();
					}
				}
				std::this_thread::yield();
			}
		});

		// Suspend/resume thread
		std::thread suspender([&]() {
			for (int i = 0; i < SUSPEND_CYCLES; ++i) {
				std::this_thread::sleep_for(std::chrono::milliseconds(2));

				world.startFrame();
				world.signalFrameComplete();

				bool completed = world.suspendFixed(std::chrono::milliseconds(100));

				// After suspendFixed returns, NO workers should be active
				if (world.m_dispatcher.getActiveWorkerCount() == 0) {
					suspendSuccessCount.fetch_add(1);
				}

				std::this_thread::sleep_for(std::chrono::microseconds(500));
				world.resume();
			}
		});

		suspender.join();
		running.store(false);
		workerSpawner.join();

		INFO("Successful suspends (no active workers): " << suspendSuccessCount.load());
		INFO("Total workers completed: " << totalWorkersCompleted.load());

		// ALL suspend operations should find zero active workers
		REQUIRE(suspendSuccessCount.load() == SUSPEND_CYCLES);
	}
}

// ============================================================================
// BUG-002: suspendSimulationUntilFinished() Has No Synchronization
// ============================================================================
//
// Bug Description:
// ----------------
// suspendSimulationUntilFinished() sets m_isStasis = true but doesn't wait for
// async work before running the callback. This can cause data corruption when
// Papyrus modifies physics bodies while async workers are still active.
//
// Location: hdtSkyrimPhysicsWorld.cpp:171-185
//
// Current (Buggy) Code:
//   void SkyrimPhysicsWorld::suspendSimulationUntilFinished(std::function<void(void)> process)
//   {
//       this->m_isStasis = true;
//       try {
//           process();  // BUG: async work may still be running!
//       }
//       ...
//       this->m_isStasis = false;
//   }
//
// Fixed Code:
//   void SkyrimPhysicsWorld::suspendSimulationUntilFinished(std::function<void(void)> process)
//   {
//       this->m_isStasis = true;
//       m_tasks.wait();
//       auto dispatcher = static_cast<CollisionDispatcher*>(m_dispatcher1);
//       dispatcher->waitForCollisionWorkers();
//       try {
//           process();
//       }
//       ...
//       this->m_isStasis = false;
//   }

namespace
{
	/**
	 * Mock AsyncTaskGroup for testing task synchronization.
	 * Simulates pending async tasks that must complete before callback runs.
	 */
	class MockAsyncTaskGroup
	{
	public:
		std::atomic<int> m_pendingTasks{0};
		std::atomic<bool> m_waitCalled{false};

		void addTask() { m_pendingTasks.fetch_add(1, std::memory_order_acq_rel); }

		void completeTask() { m_pendingTasks.fetch_sub(1, std::memory_order_acq_rel); }

		void wait()
		{
			m_waitCalled.store(true, std::memory_order_release);
			// In real code, this blocks until all tasks complete
			// Here we just spin until m_pendingTasks == 0
			while (m_pendingTasks.load(std::memory_order_acquire) > 0) {
				std::this_thread::yield();
			}
		}

		bool hasPending() const { return m_pendingTasks.load(std::memory_order_acquire) > 0; }
	};

	/**
	 * Mock of SkyrimPhysicsWorld for BUG-002 testing.
	 * Includes both AsyncTaskGroup and CollisionDispatcher tracking.
	 */
	class MockSkyrimPhysicsWorldBug002
	{
	public:
		std::atomic<bool> m_isStasis{false};
		MockAsyncTaskGroup m_tasks;
		MockCollisionDispatcher m_dispatcher;

		/**
		 * BUGGY version: Sets stasis but doesn't wait for async work.
		 * Matches current code in hdtSkyrimPhysicsWorld.cpp:171-185
		 */
		void suspendSimulationUntilFinishedBuggy(std::function<void(void)> process)
		{
			m_isStasis = true;
			try {
				process(); // BUG: async work may still be running!
			}
			catch (...) {
				// ignore
			}
			m_isStasis = false;
		}

		/**
		 * FIXED version: Waits for all async work before running callback.
		 */
		void suspendSimulationUntilFinishedFixed(std::function<void(void)> process)
		{
			m_isStasis = true;

			// Wait for async task group
			m_tasks.wait();

			// Wait for collision workers
			m_dispatcher.waitForCollisionWorkers();

			try {
				process();
			}
			catch (...) {
				// ignore
			}
			m_isStasis = false;
		}
	};

} // namespace

TEST_CASE("BUG-002: suspendSimulationUntilFinished must wait for async work", "[suspend][regression]")
{
	SECTION("BUGGY: Callback runs while tasks still pending")
	{
		MockSkyrimPhysicsWorldBug002 world;

		std::atomic<bool> callbackExecuted{false};
		std::atomic<bool> taskWasRunningDuringCallback{false};

		// Simulate a pending async task
		world.m_tasks.addTask();
		REQUIRE(world.m_tasks.hasPending() == true);

		// Start a thread that simulates the async task
		std::thread asyncTask([&world]() {
			// Task runs for 50ms
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			world.m_tasks.completeTask();
		});

		// BUGGY: suspendSimulationUntilFinished doesn't wait
		world.suspendSimulationUntilFinishedBuggy([&]() {
			callbackExecuted.store(true);
			// Record if task was still running when callback executed
			if (world.m_tasks.hasPending()) {
				taskWasRunningDuringCallback.store(true);
			}
		});

		asyncTask.join();

		// BUG: Callback executed while task was still running
		REQUIRE(callbackExecuted.load() == true);
		REQUIRE(taskWasRunningDuringCallback.load() == true); // This is the bug!
	}

	SECTION("BUGGY: Callback runs while collision workers still active")
	{
		MockSkyrimPhysicsWorldBug002 world;

		std::atomic<bool> callbackExecuted{false};
		std::atomic<bool> workerWasRunningDuringCallback{false};

		// Simulate an active collision worker
		std::atomic<bool> workerShouldExit{false};
		std::thread workerThread([&]() {
			MockCollisionDispatcher::WorkerScope scope(&world.m_dispatcher);
			while (!workerShouldExit.load()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
		});

		// Wait for worker to start
		while (world.m_dispatcher.getActiveWorkerCount() == 0) {
			std::this_thread::yield();
		}

		// BUGGY: suspendSimulationUntilFinished doesn't wait
		world.suspendSimulationUntilFinishedBuggy([&]() {
			callbackExecuted.store(true);
			// Record if worker was still running when callback executed
			if (world.m_dispatcher.getActiveWorkerCount() > 0) {
				workerWasRunningDuringCallback.store(true);
			}
		});

		// Signal worker to exit and wait
		workerShouldExit.store(true);
		workerThread.join();

		// BUG: Callback executed while worker was still running
		REQUIRE(callbackExecuted.load() == true);
		REQUIRE(workerWasRunningDuringCallback.load() == true); // This is the bug!
	}

	SECTION("FIXED: Callback waits for pending tasks")
	{
		MockSkyrimPhysicsWorldBug002 world;

		std::atomic<bool> callbackExecuted{false};
		std::atomic<bool> taskWasRunningDuringCallback{false};

		// Simulate a pending async task
		world.m_tasks.addTask();

		// Start a thread that simulates the async task
		std::thread asyncTask([&world]() {
			// Task runs for 50ms
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			world.m_tasks.completeTask();
		});

		// FIXED: suspendSimulationUntilFinished waits for tasks
		world.suspendSimulationUntilFinishedFixed([&]() {
			callbackExecuted.store(true);
			// Record if task was still running when callback executed
			if (world.m_tasks.hasPending()) {
				taskWasRunningDuringCallback.store(true);
			}
		});

		asyncTask.join();

		// FIX: Callback executed AFTER task completed
		REQUIRE(callbackExecuted.load() == true);
		REQUIRE(taskWasRunningDuringCallback.load() == false); // Fixed!
		REQUIRE(world.m_tasks.m_waitCalled.load() == true);	   // wait() was called
	}

	SECTION("FIXED: Callback waits for collision workers")
	{
		MockSkyrimPhysicsWorldBug002 world;

		std::atomic<bool> callbackExecuted{false};
		std::atomic<bool> workerWasRunningDuringCallback{false};
		std::atomic<bool> workerStarted{false};

		// Simulate an active collision worker that respects cancellation
		std::thread workerThread([&]() {
			MockCollisionDispatcher::WorkerScope scope(&world.m_dispatcher);
			workerStarted.store(true);
			// Simulate short work then exit
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
		});

		// Wait for worker to start
		while (!workerStarted.load()) {
			std::this_thread::yield();
		}

		// FIXED: suspendSimulationUntilFinished waits for workers
		world.suspendSimulationUntilFinishedFixed([&]() {
			callbackExecuted.store(true);
			// Record if worker was still running when callback executed
			if (world.m_dispatcher.getActiveWorkerCount() > 0) {
				workerWasRunningDuringCallback.store(true);
			}
		});

		workerThread.join();

		// FIX: Callback executed AFTER worker completed
		REQUIRE(callbackExecuted.load() == true);
		REQUIRE(workerWasRunningDuringCallback.load() == false); // Fixed!
	}
}

TEST_CASE("BUG-002: Exception handling preserved with sync", "[suspend][regression]")
{
	MockSkyrimPhysicsWorldBug002 world;

	SECTION("Exception in callback doesn't leave stasis true")
	{
		REQUIRE(world.m_isStasis.load() == false);

		// Callback throws exception
		world.suspendSimulationUntilFinishedFixed([&]() { throw std::runtime_error("test exception"); });

		// m_isStasis should be reset to false even after exception
		REQUIRE(world.m_isStasis.load() == false);
	}

	SECTION("Multiple exceptions don't corrupt state")
	{
		for (int i = 0; i < 10; ++i) {
			REQUIRE(world.m_isStasis.load() == false);

			world.suspendSimulationUntilFinishedFixed([&]() { throw std::runtime_error("test exception"); });

			REQUIRE(world.m_isStasis.load() == false);
		}
	}
}

TEST_CASE("BUG-002: Stress test suspendSimulationUntilFinished", "[suspend][regression][stress]")
{
	SECTION("Sequential suspend calls with pre-existing work")
	{
		// This test simulates the real scenario:
		// 1. Some async work is already in progress
		// 2. suspendSimulationUntilFinished is called
		// 3. The callback must wait until that work completes
		MockSkyrimPhysicsWorldBug002 world;

		constexpr int ITERATIONS = 20;
		std::atomic<int> successfulSyncs{0};

		for (int i = 0; i < ITERATIONS; ++i) {
			// Simulate in-progress async work
			world.m_tasks.addTask();

			std::thread asyncWork([&world]() {
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				world.m_tasks.completeTask();
			});

			// Also simulate a collision worker
			std::atomic<bool> workerDone{false};
			std::thread workerThread([&]() {
				MockCollisionDispatcher::WorkerScope scope(&world.m_dispatcher);
				std::this_thread::sleep_for(std::chrono::milliseconds(15));
				workerDone.store(true);
			});

			// Give threads a moment to start
			std::this_thread::sleep_for(std::chrono::milliseconds(2));

			// With FIXED code, callback runs only after all work completes
			world.suspendSimulationUntilFinishedFixed([&]() {
				// At this point, all work should be done
				if (!world.m_tasks.hasPending() && world.m_dispatcher.getActiveWorkerCount() == 0) {
					successfulSyncs.fetch_add(1);
				}
			});

			asyncWork.join();
			workerThread.join();
		}

		// All suspends should have found no active work
		REQUIRE(successfulSyncs.load() == ITERATIONS);
	}
}

// ============================================================================
// FrameSyncEvent Collision Worker Race Condition Tests
// ============================================================================

/**
 * BUG-008 Regression Test: FrameSyncEvent doesn't wait for collision workers
 *
 * Bug Description:
 * ----------------
 * FrameSyncEvent::onEvent() calls m_tasks.wait() to wait for the doUpdate2ndStep task,
 * then immediately sets m_frameSyncComplete=true. However, doUpdate2ndStep spawns
 * collision workers via hdt_parallel_for_each that continue running AFTER the task
 * returns. This allows the next frame (FrameEvent) to start while collision workers
 * from the previous frame are still accessing collider tree data.
 *
 * Result: Frame N+1's FrameEvent modifies collider trees while Frame N's workers
 * are still reading them, causing index corruption (e.g., indices 2x-4x out of range).
 *
 * Fix:
 * ----
 * Add waitForCollisionWorkers() call in FrameSyncEvent after m_tasks.wait():
 *   m_tasks.wait();
 *   dispatcher->waitForCollisionWorkers();  // NEW: wait for parallel workers
 *   m_frameSyncComplete.store(true);
 */

namespace
{
	/**
	 * Mock that simulates FrameSyncEvent behavior with collision workers.
	 * Extends MockSkyrimPhysicsWorldBug002 with frame sync semantics.
	 */
	class MockFrameSyncWorld
	{
	public:
		MockAsyncTaskGroup m_tasks;
		MockCollisionDispatcher m_dispatcher;
		std::atomic<bool> m_frameSyncComplete{true};
		std::atomic<bool> m_suspended{false};
		std::mutex m_frameSyncMutex;
		std::condition_variable m_frameSyncCV;

		// Track if data is being accessed by workers
		std::atomic<int> m_colliderTreeAccessCount{0};
		std::atomic<bool> m_corruptionDetected{false};

		/**
		 * BUGGY version: Doesn't wait for collision workers.
		 * This matches the original broken behavior.
		 */
		void onFrameSyncEventBuggy()
		{
			if (m_suspended)
				return;

			// Wait for async task (doUpdate2ndStep)
			m_tasks.wait();

			// BUG: Signal completion without waiting for collision workers!
			m_frameSyncComplete.store(true, std::memory_order_release);
			m_frameSyncCV.notify_all();
		}

		/**
		 * FIXED version: Waits for collision workers before signaling completion.
		 */
		void onFrameSyncEventFixed()
		{
			if (m_suspended)
				return;

			// Wait for async task (doUpdate2ndStep)
			m_tasks.wait();

			// CRITICAL FIX: Wait for collision workers before signaling completion
			m_dispatcher.waitForCollisionWorkers();

			// Now safe to signal completion
			m_frameSyncComplete.store(true, std::memory_order_release);
			m_frameSyncCV.notify_all();
		}

		/**
		 * Simulates FrameEvent starting a new frame.
		 * Should NOT be called while previous frame's workers are active.
		 */
		void onFrameEvent()
		{
			if (m_suspended)
				return;

			// Mark frame as in-progress
			m_frameSyncComplete.store(false, std::memory_order_release);

			// Check if workers from previous frame are still active (CORRUPTION!)
			if (m_colliderTreeAccessCount.load() > 0) {
				m_corruptionDetected.store(true);
			}

			// Simulate starting async physics work
			m_tasks.addTask();
		}

		/**
		 * Simulates a collision worker accessing collider tree data.
		 * Called from parallel worker threads spawned by doUpdate2ndStep.
		 */
		void simulateCollisionWorker(std::chrono::milliseconds workTime)
		{
			MockCollisionDispatcher::WorkerScope scope(&m_dispatcher);
			m_colliderTreeAccessCount.fetch_add(1);

			// Simulate collision processing work
			std::this_thread::sleep_for(workTime);

			m_colliderTreeAccessCount.fetch_sub(1);
		}
	};
} // namespace

TEST_CASE("BUG-008: BUGGY FrameSyncEvent allows frame overlap with workers", "[framesync][regression]")
{
	MockFrameSyncWorld world;

	SECTION("Next frame starts while collision workers from previous frame are running")
	{
		// Frame N: Start physics work
		world.onFrameEvent();

		// Frame N: Spawn collision workers (simulating hdt_parallel_for_each)
		std::atomic<bool> workersStarted{false};
		std::thread workerThread([&]() {
			workersStarted = true;
			world.simulateCollisionWorker(std::chrono::milliseconds(50));
		});

		// Wait for worker to start
		while (!workersStarted)
			std::this_thread::yield();

		// Frame N: doUpdate2ndStep task completes (worker still running!)
		world.m_tasks.completeTask();

		// Frame N: BUGGY FrameSyncEvent signals completion too early
		world.onFrameSyncEventBuggy();

		// Frame N+1: FrameEvent starts immediately (workers still running!)
		REQUIRE(world.m_frameSyncComplete.load() == true);
		world.onFrameEvent();

		// CORRUPTION: We started Frame N+1 while Frame N workers were active
		// In real code, this causes index corruption in collider trees
		REQUIRE(world.m_colliderTreeAccessCount.load() > 0);
		REQUIRE(world.m_corruptionDetected.load() == true);

		workerThread.join();
	}
}

TEST_CASE("BUG-008: FIXED FrameSyncEvent waits for collision workers", "[framesync][regression]")
{
	MockFrameSyncWorld world;

	SECTION("Next frame waits until collision workers complete")
	{
		// Frame N: Start physics work
		world.onFrameEvent();

		// Frame N: Spawn collision workers (simulating hdt_parallel_for_each)
		std::atomic<bool> workersStarted{false};
		std::thread workerThread([&]() {
			workersStarted = true;
			world.simulateCollisionWorker(std::chrono::milliseconds(50));
		});

		// Wait for worker to start
		while (!workersStarted)
			std::this_thread::yield();

		// Frame N: doUpdate2ndStep task completes (worker still running!)
		world.m_tasks.completeTask();

		// Frame N: FIXED FrameSyncEvent waits for collision workers
		std::thread frameSyncThread([&]() { world.onFrameSyncEventFixed(); });

		// Give FrameSyncEvent time to start waiting
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		// FrameSyncEvent should NOT have completed yet (worker still running)
		REQUIRE(world.m_frameSyncComplete.load() == false);

		// Wait for everything to complete
		workerThread.join();
		frameSyncThread.join();

		// NOW frame sync should be complete
		REQUIRE(world.m_frameSyncComplete.load() == true);
		REQUIRE(world.m_colliderTreeAccessCount.load() == 0);

		// Frame N+1: FrameEvent can safely start (no workers active)
		world.onFrameEvent();

		// NO CORRUPTION: Workers completed before new frame started
		REQUIRE(world.m_corruptionDetected.load() == false);
	}

	SECTION("Multiple workers all complete before frame sync signals")
	{
		// Frame N: Start physics work
		world.onFrameEvent();

		// Spawn multiple workers with staggered completion times
		constexpr int NUM_WORKERS = 5;
		std::vector<std::thread> workers;
		std::atomic<int> workersStarted{0};

		for (int i = 0; i < NUM_WORKERS; ++i) {
			workers.emplace_back([&, i]() {
				workersStarted.fetch_add(1);
				world.simulateCollisionWorker(std::chrono::milliseconds(20 + i * 10));
			});
		}

		// Wait for all workers to start
		while (workersStarted.load() < NUM_WORKERS)
			std::this_thread::yield();

		// Complete the async task
		world.m_tasks.completeTask();

		// FIXED FrameSyncEvent should wait for ALL workers
		world.onFrameSyncEventFixed();

		// All workers should be done
		REQUIRE(world.m_dispatcher.getActiveWorkerCount() == 0);
		REQUIRE(world.m_colliderTreeAccessCount.load() == 0);
		REQUIRE(world.m_frameSyncComplete.load() == true);

		for (auto& w : workers)
			w.join();
	}
}

TEST_CASE("BUG-008: Stress test FrameSyncEvent with collision workers", "[framesync][regression][stress]")
{
	MockFrameSyncWorld world;

	SECTION("Rapid frame sequence with workers never causes corruption")
	{
		constexpr int NUM_FRAMES = 50;
		std::atomic<int> framesCompleted{0};
		std::atomic<bool> testRunning{true};

		// Worker thread pool - continuously spawns workers
		std::vector<std::thread> workerSpawners;
		for (int i = 0; i < 3; ++i) {
			workerSpawners.emplace_back([&]() {
				while (testRunning.load()) {
					world.simulateCollisionWorker(std::chrono::milliseconds(5));
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
			});
		}

		// Frame thread - runs frame/sync cycle
		std::thread frameThread([&]() {
			for (int i = 0; i < NUM_FRAMES; ++i) {
				// FrameEvent
				world.onFrameEvent();
				world.m_tasks.addTask();

				// Simulate physics step time
				std::this_thread::sleep_for(std::chrono::milliseconds(2));

				// Complete async task
				world.m_tasks.completeTask();

				// FIXED FrameSyncEvent
				world.onFrameSyncEventFixed();

				framesCompleted.fetch_add(1);
			}
		});

		frameThread.join();
		testRunning.store(false);

		for (auto& w : workerSpawners)
			w.join();

		// Verify no corruption occurred during the entire run
		REQUIRE(framesCompleted.load() == NUM_FRAMES);
		REQUIRE(world.m_corruptionDetected.load() == false);
	}
}

// =============================================================================
// TIME-BASED STRESS TESTS
// These run for a configurable duration rather than fixed iterations.
// Research shows race conditions are better exposed with longer runs.
// Tag: [.stress-extended] - excluded from normal CI, run manually or on merge
// =============================================================================

// DEPRECATED: This mock-based test is superseded by the integration test
// in tests/integration/test_dispatcher_integration.cpp which uses the REAL
// enkiTS scheduler and production synchronization primitives.
// Keeping for reference but disabled.
TEST_CASE("Extended stress test: suspend/resume with collision workers", "[.deprecated][sync][dispatcher]")
{
	// Duration can be overridden via environment variable
	// Default: 30 seconds for local testing, CI can set longer
	const char* durationEnv = safe_getenv("HDT_STRESS_DURATION_SEC");
	const int durationSec = durationEnv ? std::atoi(durationEnv) : 30;

	INFO("Running extended stress test for " << durationSec << " seconds");
	INFO("Set HDT_STRESS_DURATION_SEC environment variable to adjust");

	// Statistics
	std::atomic<uint64_t> suspendCount{0};
	std::atomic<uint64_t> resumeCount{0};
	std::atomic<uint64_t> collisionWorkCompleted{0};
	std::atomic<uint64_t> raceDetected{0};
	std::atomic<bool> testRunning{true};

	// Shared state simulating physics world
	std::atomic<bool> suspended{false};
	std::atomic<int> activeWorkers{0};
	std::atomic<bool> cancelRequested{false};
	std::mutex stateMutex;

	// Collision workers - simulate parallel collision processing
	auto collisionWorker = [&](int workerId) {
		while (testRunning.load(std::memory_order_relaxed)) {
			// Check cancellation before starting work
			if (cancelRequested.load(std::memory_order_acquire)) {
				std::this_thread::yield();
				continue;
			}

			// Track active worker
			activeWorkers.fetch_add(1, std::memory_order_acq_rel);

			// Simulate collision work
			for (int i = 0; i < 10 && !cancelRequested.load(std::memory_order_relaxed); ++i) {
				// The bug: accessing shared state while suspend thinks we're done
				if (suspended.load(std::memory_order_relaxed)) {
					// This would be a bug in production - worker running after suspend
					raceDetected.fetch_add(1, std::memory_order_relaxed);
				}
				std::this_thread::sleep_for(std::chrono::microseconds(10));
			}

			collisionWorkCompleted.fetch_add(1, std::memory_order_relaxed);
			activeWorkers.fetch_sub(1, std::memory_order_acq_rel);

			// Brief pause between work batches
			std::this_thread::sleep_for(std::chrono::microseconds(100));
		}
	};

	// Suspend/resume controller - simulates "smp reset" or loading screen
	auto suspendController = [&]() {
		while (testRunning.load(std::memory_order_relaxed)) {
			// Random-ish delay between suspend attempts
			std::this_thread::sleep_for(std::chrono::milliseconds(5 + (suspendCount.load() % 10)));

			// Request cancellation first (CORRECT pattern)
			cancelRequested.store(true, std::memory_order_release);

			// Wait for workers to complete (CORRECT pattern)
			int spins = 0;
			while (activeWorkers.load(std::memory_order_acquire) > 0) {
				std::this_thread::yield();
				if (++spins > 10000) {
					// Deadlock detection
					FAIL("Workers didn't complete within spin limit");
					break;
				}
			}

			// Now safe to suspend
			suspended.store(true, std::memory_order_release);
			suspendCount.fetch_add(1, std::memory_order_relaxed);

			// Simulate suspended state (config reload, etc)
			std::this_thread::sleep_for(std::chrono::milliseconds(1));

			// Resume
			suspended.store(false, std::memory_order_release);
			cancelRequested.store(false, std::memory_order_release);
			resumeCount.fetch_add(1, std::memory_order_relaxed);
		}
	};

	// Start workers - scale with available cores for maximum stress
	const int NUM_WORKERS = std::max(4, static_cast<int>(std::thread::hardware_concurrency()));

	// Production reality: only ONE thread calls suspend() at a time
	// (either game thread during loading, or console thread for "smp reset")
	// Multiple concurrent suspend callers is undefined behavior.
	// Use a mutex to serialize if testing multiple controllers.
	std::mutex suspendMutex;
	auto serializedSuspendController = [&]() {
		while (testRunning.load(std::memory_order_relaxed)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5 + (suspendCount.load() % 10)));

			// Serialize suspend calls - only one at a time (matches production)
			std::lock_guard<std::mutex> lock(suspendMutex);

			cancelRequested.store(true, std::memory_order_release);

			int spins = 0;
			while (activeWorkers.load(std::memory_order_acquire) > 0) {
				std::this_thread::yield();
				if (++spins > 100000) {
					FAIL("Workers didn't complete within spin limit");
					break;
				}
			}

			suspended.store(true, std::memory_order_release);
			suspendCount.fetch_add(1, std::memory_order_relaxed);

			std::this_thread::sleep_for(std::chrono::milliseconds(1));

			suspended.store(false, std::memory_order_release);
			cancelRequested.store(false, std::memory_order_release);
			resumeCount.fetch_add(1, std::memory_order_relaxed);
		}
	};

	const int NUM_CONTROLLERS = std::max(2, NUM_WORKERS / 4);

	std::vector<std::thread> workers;
	std::vector<std::thread> controllers;

	for (int i = 0; i < NUM_WORKERS; ++i) {
		workers.emplace_back(collisionWorker, i);
	}
	for (int i = 0; i < NUM_CONTROLLERS; ++i) {
		controllers.emplace_back(serializedSuspendController);
	}

	INFO("Stress config: " << NUM_WORKERS << " workers, " << NUM_CONTROLLERS << " controllers (serialized)");

	// Run for specified duration
	auto startTime = std::chrono::steady_clock::now();
	auto endTime = startTime + std::chrono::seconds(durationSec);

	while (std::chrono::steady_clock::now() < endTime) {
		std::this_thread::sleep_for(std::chrono::seconds(1));

		// Progress report
		auto elapsed =
			std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - startTime).count();
		INFO("Progress: " << elapsed << "s / " << durationSec << "s" << " | Suspends: " << suspendCount.load()
						  << " | Work batches: " << collisionWorkCompleted.load()
						  << " | Races: " << raceDetected.load());
	}

	// Shutdown
	testRunning.store(false, std::memory_order_release);
	cancelRequested.store(true, std::memory_order_release);

	for (auto& c : controllers) {
		c.join();
	}
	for (auto& w : workers) {
		w.join();
	}

	// Report results
	INFO("Final statistics:");
	INFO("  Suspend/resume cycles: " << suspendCount.load());
	INFO("  Collision work batches: " << collisionWorkCompleted.load());
	INFO("  Race conditions detected: " << raceDetected.load());

	// The key assertion: no races should be detected with correct synchronization
	REQUIRE(raceDetected.load() == 0);

	// Sanity check: test actually did work
	REQUIRE(suspendCount.load() > 0);
	REQUIRE(collisionWorkCompleted.load() > 0);
}
