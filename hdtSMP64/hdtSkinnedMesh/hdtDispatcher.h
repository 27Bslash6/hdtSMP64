#pragma once

#include "hdtBulletHelper.h"
#include "hdtEnkiTSScheduler.h"

#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"

#include <atomic>
#include <mutex>
#include <vector>

namespace hdt
{
	class SkinnedMeshBody;

	class CollisionDispatcher : public btCollisionDispatcherMt
	{
	public:
		CollisionDispatcher(btCollisionConfiguration* collisionConfiguration)
			: btCollisionDispatcherMt(collisionConfiguration)
		{}

		btPersistentManifold* getNewManifold(const btCollisionObject* b0, const btCollisionObject* b1) override
		{
			std::lock_guard<decltype(m_lock)> l(m_lock);
			auto ret = btCollisionDispatcherMt::getNewManifold(b0, b1);
			return ret;
		}

		void releaseManifold(btPersistentManifold* manifold) override
		{
			std::lock_guard<decltype(m_lock)> l(m_lock);
			btCollisionDispatcherMt::releaseManifold(manifold);
		}

		bool needsCollision(const btCollisionObject* body0, const btCollisionObject* body1) override;
		void dispatchAllCollisionPairs(btOverlappingPairCache* pairCache, const btDispatcherInfo& dispatchInfo,
									   btDispatcher* dispatcher) override;

		int getNumManifolds() const override;
		btPersistentManifold** getInternalManifoldPointer() override;
		btPersistentManifold* getManifoldByIndexInternal(int index) override;

		void clearAllManifold();
		void clearAllManifoldInternal(); // Internal: caller must hold m_lock

		// Clear all collision state - call during physics reset to prevent stale references
		void clearCollisionState()
		{
			std::lock_guard<decltype(m_lock)> l(m_lock);
			clearAllManifoldInternal(); // Use internal version - we already hold the lock
			m_pairs.clear();
#ifdef CUDA
			{
				std::lock_guard<std::mutex> guard(m_delayedFuncsLock);
				m_delayedFuncs.clear();
			}
#endif
		}

		// Collision cancellation for graceful suspend - enkiTS doesn't support cancellation
		// so these are flags checked by worker code to exit early
		void requestCollisionCancellation() { m_collisionCancelled.store(true, std::memory_order_release); }
		void clearCollisionCancellation() { m_collisionCancelled.store(false, std::memory_order_release); }
		bool isCollisionCancelled() const { return m_collisionCancelled.load(std::memory_order_acquire); }

		// Wait for collision workers to complete - enkiTS handles this via task completion
		// This is a no-op since we wait on the task set, not individual workers
		void waitForCollisionWorkers() {}

		// RAII guard for worker tracking - currently a no-op but allows early exit via isCancelled()
		struct WorkerScope
		{
			CollisionDispatcher* dispatcher;
			WorkerScope(CollisionDispatcher* d) : dispatcher(d) {}
			~WorkerScope() {}
			bool isCancelled() const { return dispatcher->isCollisionCancelled(); }
		};

#ifdef CUDA
		// Sync and apply collision results from previous frame
		// Called at START of physics step, before prediction, to allow GPU overlap with solve
		void syncPreviousCollisionResults();
#endif

		std::mutex m_lock;
		std::vector<std::pair<SkinnedMeshBody*, SkinnedMeshBody*>> m_pairs;
		std::atomic<bool> m_collisionCancelled{false};
#ifdef CUDA
		// Thread-safe delayed function queue (replaces concurrent_vector)
		std::vector<std::function<void()>> m_delayedFuncs;
		std::mutex m_delayedFuncsLock;

		void addDelayedFunc(std::function<void()> func)
		{
			std::lock_guard<std::mutex> guard(m_delayedFuncsLock);
			m_delayedFuncs.push_back(std::move(func));
		}
#endif
	};
} // namespace hdt
