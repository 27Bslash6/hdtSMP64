#pragma once

#include "hdtSkinnedMesh/hdtDispatcher.h"
#include "hdtSkinnedMesh/hdtEnkiTSScheduler.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshWorld.h"
#include "hdtSkyrimSystem.h"

#include "ActorManager.h"
#include "HookEvents.h"
#include "IEventListener.h"
#include "skse64/PapyrusEvents.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <thread>

namespace hdt
{
	constexpr float RESET_PHYSICS = -10.0f;

	class SkyrimPhysicsWorld : protected SkinnedMeshWorld,
							   public IEventListener<FrameEvent>,
							   public IEventListener<ShutdownEvent>,
							   public BSTEventSink<SKSECameraEvent>,
							   public IEventListener<FrameSyncEvent>
	{
	public:
		static SkyrimPhysicsWorld* get();

		void doUpdate(float delta);
		void doUpdate2ndStep(float delta, const float tick, const float remainingTimeStep);
		void updateActiveState();

		void addSkinnedMeshSystem(SkinnedMeshSystem* system) override;
		void removeSkinnedMeshSystem(SkinnedMeshSystem* system) override;
		void removeSystemByNode(void* root);

		void resetTransformsToOriginal();
		void resetSystems();

		void onEvent(const FrameEvent& e) override;
		void onEvent(const FrameSyncEvent& e) override;
		void onEvent(const ShutdownEvent& e) override;

		EventResult ReceiveEvent(SKSECameraEvent* evn, EventDispatcher<SKSECameraEvent>* dispatcher) override;

		bool isSuspended() { return m_suspended; }

		void suspend(bool loading = false)
		{
			// BUG-001 FIX: Request collision workers to exit early BEFORE setting suspended
			// This allows workers to check isCancelled() and exit gracefully
			auto dispatcher = static_cast<CollisionDispatcher*>(m_dispatcher1);
			dispatcher->requestCollisionCancellation();

			// Set suspended flag FIRST to stop new physics frames from starting
			// This prevents the race where new frames start while we're waiting
			m_suspended = true;
			m_loading = loading;
			_VMESSAGE("Physics suspend: loading=%d, frameSyncComplete=%d", loading,
					  m_frameSyncComplete.load(std::memory_order_acquire));

			// For loading screens (loading=true):
			// - Called from FreezeHandler on the GAME THREAD
			// - Can't wait here because the game thread also runs FrameSyncEvent
			// - Just set m_suspended and return; async tasks will check m_suspended and exit early
			// - Proper synchronization happens in resume() which calls resetSystems()
			//
			// For console commands (loading=false, e.g. "smp reset"):
			// - Called from CONSOLE THREAD
			// - MUST wait for in-progress frame to complete before destroying objects
			// - We use a condition variable instead of WaitforAll() because:
			//   WaitforAll() tries to execute tasks (work-stealing), but those tasks
			//   may have thread-local dependencies and crash on the console thread.
			if (loading) {
				// Loading case: don't wait, the game thread would deadlock
				_VMESSAGE("Physics suspended (loading): no wait needed");
			}
			else {
				// Console command case: wait for any in-progress physics to complete
				// Use condition variable - the game thread notifies when frame sync is done
				_VMESSAGE("Physics suspend: waiting for frame sync via condvar...");
				std::unique_lock<std::mutex> lk(m_frameSyncMutex);
				bool completed = m_frameSyncCV.wait_for(lk, std::chrono::seconds(5), [this] {
					return m_frameSyncComplete.load(std::memory_order_acquire);
				});
				if (!completed) {
					_ERROR("Physics suspend: TIMEOUT waiting for frame sync! Proceeding anyway.");
				}

				// BUG-001 FIX: Wait for collision workers AFTER frame sync
				// The frame may have completed (m_tasks.wait() done) but collision workers
				// spawned via hdt_parallel_for_each may still be running. We must wait
				// for them before proceeding to clear physics state.
				_VMESSAGE("Physics suspend: waiting for collision workers...");
				dispatcher->waitForCollisionWorkers();
				_VMESSAGE("Physics suspend: collision workers done");

				_VMESSAGE("Physics suspended: m_suspended=%d, m_loading=%d, timedOut=%d", m_suspended.load(),
						  m_loading.load(), !completed);
			}
		}

		void resume()
		{
			_VMESSAGE("Physics resume: m_loading=%d", m_loading.load());
			// IMPORTANT: Keep physics suspended until resetSystems() completes
			// Setting m_suspended=false before resetSystems() allows new physics
			// frames to start while systems are being reset, causing crashes
			if (m_loading) {
				_VMESSAGE("Physics resume: calling resetSystems()...");
				resetSystems();
				m_loading = false;
				_VMESSAGE("Physics resume: resetSystems() complete");
			}

			// BUG-001 FIX: Clear cancellation flag before allowing new frames
			static_cast<CollisionDispatcher*>(m_dispatcher1)->clearCollisionCancellation();

			m_suspended = false; // Only resume AFTER reset is complete
			_VMESSAGE("Physics resumed: m_suspended=%d", m_suspended.load());
		}

		void suspendSimulationUntilFinished(std::function<void(void)> process);
		std::atomic_bool m_isStasis = false;

		btVector3 applyTranslationOffset();
		void restoreTranslationOffset(const btVector3&);

		btContactSolverInfo& getSolverInfo() { return btDiscreteDynamicsWorld::getSolverInfo(); }

		// @brief setWind force value for the world
		// @param a_direction wind direction
		// @a_scale Amount to scale the windForce. Defaults to scaleSkyrim
		// @a_smoothingSamples How many samples to smooth. Defaults to 8. Must be greater than 0. Value of 1 means no
		// smoothing
		void setWind(NiPoint3* a_direction, float a_scale = scaleSkyrim, uint32_t a_smoothingSamples = 8);

		AsyncTaskGroup m_tasks;

		bool m_useRealTime = false;
		int min_fps = 60;
		int m_percentageOfFrameTime = 300; // percentage of time per frame doing hdt. Profiler shows 30% is reasonable.
										   // Out of 1000.
		float m_timeTick = 1 / 60.f;
		int m_maxSubSteps = 4;
		bool m_clampRotations = true;
		// @brief rotation speed limit of the PC in radians per second. Must be positive.
		float m_rotationSpeedLimit = 10.f;
		bool m_unclampedResets = true;
		float m_unclampedResetAngle = 120.0f;
		float m_2ndStepAverageProcessingTime = 0;
		float m_averageSMPProcessingTimeInMainLoop = 0;
		bool disabled = false;
		uint8_t m_resetPc;
		bool m_doMetrics = false;
		std::atomic<bool> m_forceMetrics{false}; // User-controlled via 'smp metrics' command
		int m_sampleSize = 5; // how many samples (each sample taken every second) for determining average time per
							  // activeSkeleton.

		// wind settings
		bool m_enableWind = true;
		float m_windStrength = 2.0f;		  // compare to gravity acceleration of 9.8
		float m_distanceForNoWind = 50.0f;	  // how close to wind obstruction to fully block wind
		float m_distanceForMaxWind = 3000.0f; // how far to wind obstruction to not block wind

	private:
		SkyrimPhysicsWorld(void);
		~SkyrimPhysicsWorld(void);

		std::mutex m_lock;

		std::atomic_bool m_suspended;
		std::atomic_bool m_loading;
		std::atomic_bool m_frameSyncComplete{true}; // True when no frame is in progress
		std::mutex m_frameSyncMutex;
		std::condition_variable m_frameSyncCV;
		float m_accumulatedInterval;
		float m_averageInterval;
		float m_SMPProcessingTimeInMainLoop = 0;
	};
} // namespace hdt
