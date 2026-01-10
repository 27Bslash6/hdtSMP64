#pragma once

#include "hdtBulletHelper.h"
#include "hdtEnkiTSScheduler.h"

#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"

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

#ifdef CUDA
		// Sync and apply collision results from previous frame
		// Called at START of physics step, before prediction, to allow GPU overlap with solve
		void syncPreviousCollisionResults();
#endif

		std::mutex m_lock;
		std::vector<std::pair<SkinnedMeshBody*, SkinnedMeshBody*>> m_pairs;
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
