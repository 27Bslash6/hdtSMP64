/**
 * Integration test for CollisionDispatcher synchronization primitives.
 *
 * This test uses the REAL production code:
 * - Real enkiTS TaskScheduler via hdt_parallel_for_each
 * - Real WorkerScope RAII pattern
 * - Real atomic counters and cancellation flags
 *
 * The synchronization logic is extracted from hdtDispatcher.h to avoid
 * Bullet Physics dependencies while testing the exact same patterns.
 *
 * Tags:
 *   [integration] - Uses real production code
 *   [sync]        - Tests synchronization primitives
 *   [dispatcher]  - Tests CollisionDispatcher patterns
 */

#include "../include/catch.hpp"

// Real enkiTS scheduler - this is the ACTUAL production scheduler
#include "hdtEnkiTSScheduler.h"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

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
	 * Real CollisionDispatcher synchronization primitives.
	 * This is the EXACT same code from hdtDispatcher.h, extracted to avoid Bullet deps.
	 *
	 * Compare with hdtSMP64/hdtSkinnedMesh/hdtDispatcher.h lines 64-132
	 */
	class DispatcherSync
	{
	public:
		DispatcherSync() : m_activeCollisionWorkers(0), m_cancelCollisions(false) {}

		/**
		 * RAII scope guard for collision worker tracking.
		 * Identical to CollisionDispatcher::WorkerScope
		 */
		struct WorkerScope
		{
			DispatcherSync* d;

			explicit WorkerScope(DispatcherSync* dispatcher) : d(dispatcher)
			{
				d->m_activeCollisionWorkers.fetch_add(1, std::memory_order_acq_rel);
			}

			~WorkerScope() { d->m_activeCollisionWorkers.fetch_sub(1, std::memory_order_acq_rel); }

			WorkerScope(const WorkerScope&) = delete;
			WorkerScope& operator=(const WorkerScope&) = delete;
		};

		/**
		 * Wait for all collision workers to complete.
		 * Identical to CollisionDispatcher::waitForCollisionWorkers()
		 */
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

	private:
		std::atomic<int> m_activeCollisionWorkers;
		std::atomic<bool> m_cancelCollisions;
	};

	/**
	 * Simulates the suspend() logic from hdtSkyrimPhysicsWorld.
	 * Compare with hdtSkyrimPhysicsWorld.h lines 50-103
	 */
	class PhysicsWorldSync
	{
	public:
		PhysicsWorldSync(DispatcherSync& dispatcher) : m_dispatcher(dispatcher), m_suspended(false) {}

		void suspend()
		{
			// BUG-001 FIX: Request collision workers to exit early BEFORE setting suspended
			m_dispatcher.requestCollisionCancellation();

			// Set suspended flag
			m_suspended.store(true, std::memory_order_release);

			// Wait for collision workers (this is the critical part!)
			m_dispatcher.waitForCollisionWorkers();
		}

		void resume()
		{
			m_dispatcher.clearCollisionCancellation();
			m_suspended.store(false, std::memory_order_release);
		}

		bool isSuspended() const { return m_suspended.load(std::memory_order_acquire); }

	private:
		DispatcherSync& m_dispatcher;
		std::atomic<bool> m_suspended;
	};

} // namespace

// =============================================================================
// Basic synchronization tests
// =============================================================================

TEST_CASE("WorkerScope increments and decrements counter", "[integration][sync][dispatcher]")
{
	DispatcherSync sync;

	REQUIRE(sync.getActiveWorkerCount() == 0);

	{
		DispatcherSync::WorkerScope scope(&sync);
		REQUIRE(sync.getActiveWorkerCount() == 1);

		{
			DispatcherSync::WorkerScope scope2(&sync);
			REQUIRE(sync.getActiveWorkerCount() == 2);
		}

		REQUIRE(sync.getActiveWorkerCount() == 1);
	}

	REQUIRE(sync.getActiveWorkerCount() == 0);
}

TEST_CASE("waitForCollisionWorkers blocks until workers complete", "[integration][sync][dispatcher]")
{
	DispatcherSync sync;
	std::atomic<bool> workerStarted{false};
	std::atomic<bool> workerFinished{false};

	std::thread worker([&]() {
		DispatcherSync::WorkerScope scope(&sync);
		workerStarted.store(true, std::memory_order_release);
		std::this_thread::sleep_for(50ms);
		workerFinished.store(true, std::memory_order_release);
	});

	// Wait for worker to start
	while (!workerStarted.load(std::memory_order_acquire)) {
		std::this_thread::yield();
	}

	// Worker is running, count should be 1
	REQUIRE(sync.getActiveWorkerCount() == 1);
	REQUIRE(!workerFinished.load(std::memory_order_acquire));

	// This should block until worker is done
	sync.waitForCollisionWorkers();

	// After wait returns, worker must be finished
	REQUIRE(workerFinished.load(std::memory_order_acquire));
	REQUIRE(sync.getActiveWorkerCount() == 0);

	worker.join();
}

TEST_CASE("Cancellation flag propagates to workers", "[integration][sync][dispatcher]")
{
	DispatcherSync sync;

	REQUIRE(!sync.isCancelled());

	sync.requestCollisionCancellation();
	REQUIRE(sync.isCancelled());

	sync.clearCollisionCancellation();
	REQUIRE(!sync.isCancelled());
}

// =============================================================================
// Integration with real enkiTS scheduler
// =============================================================================

TEST_CASE("Real enkiTS parallel_for with WorkerScope tracking", "[integration][sync][dispatcher]")
{
	DispatcherSync sync;
	std::atomic<int> workDone{0};
	const int WORK_ITEMS = 100;

	// This uses the REAL enkiTS scheduler
	hdt::hdt_parallel_for(0, WORK_ITEMS, [&](int i) {
		DispatcherSync::WorkerScope scope(&sync);

		// Simulate work
		std::this_thread::sleep_for(1ms);
		workDone.fetch_add(1, std::memory_order_relaxed);
	});

	// After parallel_for returns, all workers must be complete
	REQUIRE(sync.getActiveWorkerCount() == 0);
	REQUIRE(workDone.load() == WORK_ITEMS);
}

TEST_CASE("Real enkiTS parallel_for_each with cancellation", "[integration][sync][dispatcher]")
{
	DispatcherSync sync;
	std::atomic<int> workStarted{0};
	std::atomic<int> workCompleted{0};

	std::vector<int> items(50);
	for (int i = 0; i < 50; ++i)
		items[i] = i;

	// Request cancellation before starting
	sync.requestCollisionCancellation();

	hdt::hdt_parallel_for_each(items.begin(), items.end(), [&](int& item) {
		DispatcherSync::WorkerScope scope(&sync);
		workStarted.fetch_add(1, std::memory_order_relaxed);

		// Check cancellation like production code does
		if (sync.isCancelled()) {
			return; // Early exit
		}

		// This shouldn't run because we're cancelled
		workCompleted.fetch_add(1, std::memory_order_relaxed);
	});

	// All work items started (WorkerScope was created)
	REQUIRE(workStarted.load() == 50);
	// But none completed the actual work
	REQUIRE(workCompleted.load() == 0);
	REQUIRE(sync.getActiveWorkerCount() == 0);
}

// =============================================================================
// Stress tests with real parallelism
// =============================================================================

TEST_CASE("Concurrent suspend/resume with real enkiTS workers", "[integration][sync][dispatcher][stress]")
{
	DispatcherSync dispatcher;
	PhysicsWorldSync world(dispatcher);

	std::atomic<bool> testRunning{true};
	std::atomic<int> suspendCycles{0};
	std::atomic<int> workItemsProcessed{0};
	std::atomic<int> racesDetected{0};
	std::atomic<int> activeAfterSuspend{0};

	// Worker thread: processes work using MAIN THREAD's enkiTS calls
	// We DON'T call enkiTS from the worker thread to avoid nested scheduler issues
	std::thread workerThread([&]() {
		while (testRunning.load(std::memory_order_relaxed)) {
			if (world.isSuspended()) {
				std::this_thread::yield();
				continue;
			}

			// Simulate work without enkiTS (just WorkerScope testing)
			{
				DispatcherSync::WorkerScope scope(&dispatcher);

				if (dispatcher.isCancelled()) {
					continue;
				}

				// Check for race: are we running while suspended?
				if (world.isSuspended() && !dispatcher.isCancelled()) {
					racesDetected.fetch_add(1, std::memory_order_relaxed);
				}

				std::this_thread::sleep_for(50us);
				workItemsProcessed.fetch_add(1, std::memory_order_relaxed);
			}
		}
	});

	// Suspend/resume thread
	std::thread suspendThread([&]() {
		while (testRunning.load(std::memory_order_relaxed)) {
			std::this_thread::sleep_for(10ms);

			world.suspend();
			suspendCycles.fetch_add(1, std::memory_order_relaxed);

			int active = dispatcher.getActiveWorkerCount();
			if (active > 0) {
				activeAfterSuspend.fetch_add(1, std::memory_order_relaxed);
			}

			std::this_thread::sleep_for(5ms);
			world.resume();
		}
	});

	// Run for 1 second (shorter for TSan overhead)
	std::this_thread::sleep_for(1s);
	testRunning.store(false, std::memory_order_relaxed);

	world.suspend();
	workerThread.join();
	suspendThread.join();

	INFO("Suspend cycles: " << suspendCycles.load());
	INFO("Work items processed: " << workItemsProcessed.load());
	INFO("Races detected: " << racesDetected.load());
	INFO("Active workers after suspend (BUG!): " << activeAfterSuspend.load());

	REQUIRE(suspendCycles.load() > 0);
	REQUIRE(workItemsProcessed.load() > 0);
	REQUIRE(racesDetected.load() == 0);
	REQUIRE(activeAfterSuspend.load() == 0);
}

TEST_CASE("enkiTS parallel_for with WorkerScope - single batch", "[integration][sync][dispatcher]")
{
	// This test runs enkiTS parallel work from the MAIN thread only
	// to test WorkerScope tracking without scheduler re-entry issues
	DispatcherSync dispatcher;

	std::atomic<int> workDone{0};
	const int WORK_ITEMS = 100;

	// Single batch of parallel work from main thread
	hdt::hdt_parallel_for(0, WORK_ITEMS, [&](int i) {
		DispatcherSync::WorkerScope scope(&dispatcher);
		std::this_thread::sleep_for(100us);
		workDone.fetch_add(1, std::memory_order_relaxed);
	});

	REQUIRE(dispatcher.getActiveWorkerCount() == 0);
	REQUIRE(workDone.load() == WORK_ITEMS);
}

TEST_CASE("Suspend waits for enkiTS workers to complete", "[integration][sync][dispatcher]")
{
	// Test that suspend() properly waits for parallel workers
	// NOTE: enkiTS calls must happen from main thread to avoid TSan false positives
	// with lock-free pipe internals. We use a separate thread for suspend() instead.
	DispatcherSync dispatcher;
	PhysicsWorldSync world(dispatcher);

	std::atomic<int> workersStarted{0};
	std::atomic<int> workersFinished{0};
	std::atomic<bool> suspendComplete{false};
	std::atomic<int> activeAfterSuspend{-1};

	// Suspend thread - simulates console command on different thread
	std::thread suspendThread([&]() {
		// Wait for workers to start
		while (workersStarted.load(std::memory_order_acquire) < 5) {
			std::this_thread::yield();
		}

		// Now suspend - should wait for all workers
		world.suspend();

		// Record state after suspend
		activeAfterSuspend.store(dispatcher.getActiveWorkerCount(), std::memory_order_release);
		suspendComplete.store(true, std::memory_order_release);
	});

	// Main thread runs enkiTS work (avoids TSan issues with lock-free pipe)
	hdt::hdt_parallel_for(0, 20, [&](int i) {
		DispatcherSync::WorkerScope scope(&dispatcher);
		workersStarted.fetch_add(1, std::memory_order_relaxed);

		// Do some work - long enough for suspend to be called mid-work
		std::this_thread::sleep_for(10ms);

		workersFinished.fetch_add(1, std::memory_order_relaxed);
	});

	suspendThread.join();

	INFO("Workers started: " << workersStarted.load());
	INFO("Workers finished: " << workersFinished.load());
	INFO("Active after suspend: " << activeAfterSuspend.load());

	REQUIRE(suspendComplete.load());
	REQUIRE(activeAfterSuspend.load() == 0);
	REQUIRE(workersFinished.load() == 20);
}

TEST_CASE("Extended stress: suspend during heavy parallel workload", "[integration][sync][dispatcher][.stress-"
																	 "extended]")
{
	const char* durationEnv = safe_getenv("HDT_STRESS_DURATION_SEC");
	const int durationSec = durationEnv ? std::atoi(durationEnv) : 30;

	DispatcherSync dispatcher;
	PhysicsWorldSync world(dispatcher);

	std::atomic<bool> testRunning{true};
	std::atomic<int> suspendCycles{0};
	std::atomic<int> workBatches{0};
	std::atomic<int> activeAfterSuspend{0};

	// SINGLE physics thread - matches production pattern
	// In production, dispatchAllCollisionPairs is called from one thread,
	// which then spawns enkiTS workers
	std::thread physicsThread([&]() {
		while (testRunning.load(std::memory_order_relaxed)) {
			if (world.isSuspended()) {
				std::this_thread::yield();
				continue;
			}

			// Simulate collision processing batch (like dispatchAllCollisionPairs)
			std::vector<int> pairs(50);
			for (int i = 0; i < 50; ++i)
				pairs[i] = i;

			// This spawns enkiTS workers - production pattern
			hdt::hdt_parallel_for_each(pairs.begin(), pairs.end(), [&](int& pair) {
				DispatcherSync::WorkerScope scope(&dispatcher);

				if (dispatcher.isCancelled()) {
					return;
				}

				// Simulate collision detection work
				volatile int dummy = 0;
				for (int i = 0; i < 1000; ++i) {
					dummy += i;
				}
				(void)dummy;
			});

			workBatches.fetch_add(1, std::memory_order_relaxed);
		}
	});

	// Suspend/resume controller (simulates console "smp reset" or loading screen)
	std::thread controller([&]() {
		while (testRunning.load(std::memory_order_relaxed)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5 + (suspendCycles.load() % 10)));

			world.suspend();

			// CRITICAL CHECK: After suspend() returns, no workers should be active
			int active = dispatcher.getActiveWorkerCount();
			if (active > 0) {
				activeAfterSuspend.fetch_add(1, std::memory_order_relaxed);
			}

			suspendCycles.fetch_add(1, std::memory_order_relaxed);

			std::this_thread::sleep_for(2ms);
			world.resume();
		}
	});

	// Run for specified duration
	auto endTime = std::chrono::steady_clock::now() + std::chrono::seconds(durationSec);
	while (std::chrono::steady_clock::now() < endTime) {
		std::this_thread::sleep_for(100ms);
	}

	testRunning.store(false, std::memory_order_relaxed);

	// Clean shutdown
	world.suspend();
	physicsThread.join();
	controller.join();

	INFO("Duration: " << durationSec << " seconds");
	INFO("Suspend cycles: " << suspendCycles.load());
	INFO("Work batches: " << workBatches.load());
	INFO("Active workers after suspend (BUG!): " << activeAfterSuspend.load());

	REQUIRE(suspendCycles.load() > 10); // Sanity check (fewer cycles due to longer sleep)
	REQUIRE(workBatches.load() > 10);	// Sanity check
	REQUIRE(activeAfterSuspend.load() == 0);
}
