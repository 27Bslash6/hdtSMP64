#include "hdtDispatcher.h"

#include "hdtFrameTimer.h"
#include "hdtSkinnedMeshAlgorithm.h"
#include "hdtSkinnedMeshBody.h"

#include "../hdtLog.h"
#include "../hdtTracy.h"
#ifdef CUDA
#include "hdtCudaInterface.h"
#endif

#include <LinearMath/btPoolAllocator.h>

#ifdef CUDA
// CUDA builds use 1-frame latency for ALL collision types (VV, VT, TT).
// This allows GPU collision detection to overlap with CPU constraint solving,
// recovering ~17% frame time. Results computed in frame N are applied in frame N+1
// via syncPreviousCollisionResults() at frame start.
#define CUDA_DELAYED_COLLISIONS
#endif

namespace hdt
{
	// Internal version - caller must hold m_lock
	void CollisionDispatcher::clearAllManifoldInternal()
	{
		for (int i = 0; i < m_manifoldsPtr.size(); ++i) {
			auto manifold = m_manifoldsPtr[i];
			manifold->~btPersistentManifold();
			if (m_persistentManifoldPoolAllocator->validPtr(manifold))
				m_persistentManifoldPoolAllocator->freeMemory(manifold);
			else
				btAlignedFree(manifold);
		}
		m_manifoldsPtr.clear();
	}

	void CollisionDispatcher::clearAllManifold()
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);
		clearAllManifoldInternal();
	}

	bool needsCollision(const SkinnedMeshBody* shape0, const SkinnedMeshBody* shape1)
	{
		if (!shape0 || !shape1 || shape0 == shape1)
			return false;

		if (shape0->m_isKinematic && shape1->m_isKinematic)
			return false;

		return shape0->canCollideWith(shape1) && shape1->canCollideWith(shape0);
	}

	bool CollisionDispatcher::needsCollision(const btCollisionObject* body0, const btCollisionObject* body1)
	{
		auto shape0 = dynamic_cast<const SkinnedMeshBody*>(body0);
		auto shape1 = dynamic_cast<const SkinnedMeshBody*>(body1);

		if (shape0 || shape1) {
			return hdt::needsCollision(shape0, shape1);
		}
		if (body0->isStaticOrKinematicObject() && body1->isStaticOrKinematicObject())
			return false;
		if (body0->checkCollideWith(body1) || body1->checkCollideWith(body0)) {
			auto rb0 = static_cast<SkinnedMeshBone*>(body0->getUserPointer());
			auto rb1 = static_cast<SkinnedMeshBone*>(body1->getUserPointer());

			return rb0->canCollideWith(rb1) && rb1->canCollideWith(rb0);
		}
		else
			return false;
	}

	void CollisionDispatcher::dispatchAllCollisionPairs(btOverlappingPairCache* pairCache,
														const btDispatcherInfo& dispatchInfo, btDispatcher* dispatcher)
	{
		HDT_ZONE_SCOPED_N("DispatchCollisionPairs");
		auto size = pairCache->getNumOverlappingPairs();
		HDT_ZONE_VALUE(static_cast<int64_t>(size));
		_DMESSAGE("dispatchAllCollisionPairs: entering with %d pairs", size);
		if (!size)
			return;

		m_pairs.reserve(size);
		auto pairs = pairCache->getOverlappingPairArrayPtr();
#ifdef CUDA
		using UpdateMap = std::unordered_map<SkinnedMeshBody*, std::pair<PerVertexShape*, PerTriangleShape*>>;
		UpdateMap to_update;
		// Find bodies and meshes that need collision checking. We want to keep them together in a map so they can
		// be grouped by CUDA stream
		for (int i = 0; i < size; ++i)
#else
		SpinLock lock;
		std::unordered_set<SkinnedMeshBody*> bodies;
		std::unordered_set<PerVertexShape*> vertex_shapes;
		std::unordered_set<PerTriangleShape*> triangle_shapes;

		hdt_parallel_for(
			0, size,
			[&, this](int i)
#endif
		{
#ifndef CUDA
			// BUG-001 FIX: Track this worker so suspend() can wait for it
			WorkerScope workerScope(this);
			if (isCancelled())
				return; // Early exit if suspend requested
#endif

			auto& pair = pairs[i];

			auto shape0 =
				dynamic_cast<SkinnedMeshBody*>(static_cast<btCollisionObject*>(pair.m_pProxy0->m_clientObject));
			auto shape1 =
				dynamic_cast<SkinnedMeshBody*>(static_cast<btCollisionObject*>(pair.m_pProxy1->m_clientObject));

			if (shape0 || shape1) {
				if (hdt::needsCollision(shape0, shape1) && shape0->isBoundingSphereCollided(shape1)) {
#ifdef CUDA
					auto it0 = to_update.insert({shape0, {nullptr, nullptr}}).first;
					auto it1 = to_update.insert({shape1, {nullptr, nullptr}}).first;
					m_pairs.push_back(std::make_pair(shape0, shape1));

					auto a = shape0->m_shape->asPerTriangleShape();
					auto b = shape1->m_shape->asPerTriangleShape();
					if (a)
						it0->second.second = a;
					else
						it0->second.first = shape0->m_shape->asPerVertexShape();
					if (b)
						it1->second.second = b;
					else
						it1->second.first = shape1->m_shape->asPerVertexShape();
					if (a && b) {
						it0->second.first = a->m_verticesCollision;
						it1->second.first = b->m_verticesCollision;
					}
#else
						// FIX: All shared state mutations must be inside the lock scope
						// m_pairs.push_back was previously OUTSIDE the lock - data race!
						HDT_LOCK_GUARD(l, lock);
						bodies.insert(shape0);
						bodies.insert(shape1);
						m_pairs.push_back(std::make_pair(shape0, shape1));

						auto a = shape0->m_shape->asPerTriangleShape();
						auto b = shape1->m_shape->asPerTriangleShape();
						if (a)
							triangle_shapes.insert(a);
						else
							vertex_shapes.insert(shape0->m_shape->asPerVertexShape());
						if (b)
							triangle_shapes.insert(b);
						else
							vertex_shapes.insert(shape1->m_shape->asPerVertexShape());
						if (a && b) {
							vertex_shapes.insert(a->m_verticesCollision);
							vertex_shapes.insert(b->m_verticesCollision);
						}
#endif
				}
			}
			else
				getNearCallback()(pair, *this, dispatchInfo);
#ifdef CUDA
		}
		bool haveCuda = CudaInterface::instance()->hasCuda() &&
						(!FrameTimer::instance()->running() || FrameTimer::instance()->cudaFrame());
		FrameTimer::instance()->logEvent(FrameTimer::e_Start);
		if (haveCuda) {
			bool initialized = true;
			int deviceId = CudaInterface::currentDevice;

			// Build simple vectors of the things to update, and determine whether any new CUDA objects need
			// to be created - either because there isn't one already, or because it's on the wrong device
			for (auto& o : to_update) {
				initialized &= static_cast<bool>(o.first->m_cudaObject) &&
							   o.first->m_cudaObject->deviceId() == deviceId;
				if (o.second.first) {
					initialized &= static_cast<bool>(o.second.first->m_cudaObject) &&
								   o.second.first->m_cudaObject->deviceId() == deviceId;
				}
				if (o.second.second) {
					initialized &= static_cast<bool>(o.second.second->m_cudaObject) &&
								   o.second.second->m_cudaObject->deviceId() == deviceId;
				}
			}

			// Create any new CUDA objects if necessary
			if (!initialized) {
				hdt_parallel_for_each(to_update.begin(), to_update.end(), [this, deviceId](UpdateMap::value_type& o) {
					// BUG-001 FIX: Track worker for suspend() synchronization
					WorkerScope workerScope(this);
					if (isCancelled())
						return;

					CudaInterface::instance()->setCurrentDevice();

					if (!o.first->m_cudaObject || o.first->m_cudaObject->deviceId() != deviceId) {
						o.first->m_cudaObject.reset(new CudaBody(o.first));
					}
					if (o.second.first &&
						(!o.second.first->m_cudaObject || o.second.first->m_cudaObject->deviceId() != deviceId))
					{
						o.second.first->m_cudaObject.reset(new CudaPerVertexShape(o.second.first));
					}
					if (o.second.second &&
						(!o.second.second->m_cudaObject || o.second.second->m_cudaObject->deviceId() != deviceId))
					{
						o.second.second->m_cudaObject.reset(new CudaPerTriangleShape(o.second.second));
					}
				});
			}

			// NOTE: Sync of previous frame's collision results is now done in syncPreviousCollisionResults()
			// which is called at the START of physics step, allowing GPU collision to overlap with CPU solve

			CudaInterface::instance()->setCurrentDevice();
			{
				HDT_ZONE_SCOPED_N("LaunchInternalUpdates");

				// Phase 1: Update bone transforms on CPU (parallel)
				{
					HDT_ZONE_SCOPED_N("UpdateBonesParallel");
					hdt_parallel_for_each(to_update.begin(), to_update.end(), [this](UpdateMap::value_type& o) {
						// BUG-001 FIX: Track worker for suspend() synchronization
						WorkerScope workerScope(this);
						if (isCancelled())
							return;
						o.first->updateBones();
					});
				}

				// Phase 2: Batch and launch GPU work (replaces per-body graph launches)
				CudaInterface::instance()->beginInternalUpdateBatch();
				for (auto& o : to_update) {
					CudaInterface::instance()->addInternalUpdate(
						o.first->m_cudaObject, o.second.first ? o.second.first->m_cudaObject : nullptr,
						o.second.second ? o.second.second->m_cudaObject : nullptr);
				}
				CudaInterface::instance()->launchInternalUpdateBatch();

				// Queue async copy of leaf AABBs to host (runs in parallel with GPU collision)
				CudaInterface::instance()->queueLeafDownloads();
			}

			// ========================================================
			// DOUBLE-BUFFER AABB PIPELINE
			// Uses previous frame's leaf AABBs for CPU tree propagation
			// Sync happens at frame START (syncPreviousCollisionResults) for max GPU overlap
			// ========================================================
			{
				HDT_ZONE_SCOPED_N("TreeUpdatesFromPreviousFrame");

				// First frame bootstrap: must sync to get initial AABBs
				if (CudaInterface::instance()->hasFirstFrame()) {
					HDT_ZONE_SCOPED_N("FirstFrameSync");
					CudaInterface::instance()->synchronize();
					CudaInterface::instance()->clearAllFirstFrames();
				}

				// Tree propagation uses PREVIOUS frame's leaf AABBs
				// Sync already happened at frame START (syncPreviousCollisionResults)
				{
					HDT_ZONE_SCOPED_N("ParallelTreeUpdates");
					hdt_parallel_for_each(to_update.begin(), to_update.end(), [this](UpdateMap::value_type& o) {
						// BUG-001 FIX: Track worker for suspend() synchronization
						WorkerScope workerScope(this);
						if (isCancelled())
							return;

						if (o.second.first && o.second.first->m_cudaObject) {
							o.second.first->m_cudaObject->updateTree();
						}
						if (o.second.second && o.second.second->m_cudaObject) {
							o.second.second->m_cudaObject->updateTree();
						}
						o.first->m_bulletShape.m_aabb = o.first->m_shape->m_tree.aabbAll;
					});
				}
			}
		}
		else {
			hdt_parallel_for_each(to_update.begin(), to_update.end(), [this](UpdateMap::value_type& o) {
				// BUG-001 FIX: Track worker for suspend() synchronization
				WorkerScope workerScope(this);
				if (isCancelled())
					return;

				o.first->internalUpdate();
				if (o.second.first) {
					o.second.first->internalUpdate();
				}
				if (o.second.second) {
					o.second.second->internalUpdate();
				}
				o.first->m_bulletShape.m_aabb = o.first->m_shape->m_tree.aabbAll;
			});
		}

		FrameTimer::instance()->logEvent(FrameTimer::e_Internal);
		if (haveCuda) {
			CudaInterface::instance()->clearBufferPool();

			// Begin batched collision gathering
			CudaInterface::instance()->beginCollisionBatch();

			// Gather collision pairs - parallelized for CPU utilization
			{
				HDT_ZONE_SCOPED_N("CudaGatherPairs");
				const int pairCount = static_cast<int>(m_pairs.size());

				// Gather all collision pairs into batched system
				hdt_parallel_for(0, pairCount, [this](int i) {
					// BUG-001 FIX: Track worker for suspend() synchronization
					WorkerScope workerScope(this);
					if (isCancelled())
						return;

					auto& pair = m_pairs[i];
					if (pair.first->m_shape->m_tree.collapseCollideL(&pair.second->m_shape->m_tree)) {
						// Add to batched collision system
						CudaInterface::instance()->addCollisionPair(pair.first, pair.second);
					}
				});
			}

			// Launch batched collision kernels (processes all pairs collected above)
			CudaInterface::instance()->launchCollisionBatch();

			FrameTimer::instance()->logEvent(FrameTimer::e_Launched);

			// Swap double buffers: current write buffer becomes next frame's read buffer
			// This must happen AFTER all GPU launches but BEFORE frame end
			CudaInterface::instance()->swapAllBuffers();

			// Swap collision result buffers for 1-frame latency pipeline
			// Results written this frame will be applied next frame after sync
			CudaInterface::instance()->swapCollisionResultBuffers();
		}
		else {
			// Now we can process the collisions
			hdt_parallel_for_each(m_pairs.begin(), m_pairs.end(),
								  [this](std::pair<SkinnedMeshBody*, SkinnedMeshBody*>& i) {
									  // BUG-001 FIX: Track worker for suspend() synchronization
									  WorkerScope workerScope(this);
									  if (isCancelled())
										  return;

									  if (i.first->m_shape->m_tree.collapseCollideL(&i.second->m_shape->m_tree)) {
										  SkinnedMeshAlgorithm::processCollision(i.first, i.second, this);
									  }
								  });
			FrameTimer::instance()->logEvent(FrameTimer::e_Launched);
		}

		m_pairs.clear();

		FrameTimer::instance()->addManifoldCount(getNumManifolds());
		FrameTimer::instance()->logEvent(FrameTimer::e_End);
	}

	void CollisionDispatcher::syncPreviousCollisionResults()
	{
		HDT_ZONE_SCOPED_N("SyncPreviousCollisionResults");

		// SYNC AT FRAME START: Ensures previous frame's GPU work is complete.
		// Both internal updates (double buffer) and collision use 1-frame latency.
		// Sync is required to ensure GPU results are ready before CPU reads them.
		// By syncing at frame START, GPU has maximum overlap time (~850μs typical).
		{
			HDT_ZONE_SCOPED_N("GpuSync");
			CudaInterface::instance()->synchronize();
		}

		// Apply collision results (from previous frame, after GPU sync)
		if (CudaInterface::instance()->hasCollisionResults()) {
			HDT_ZONE_SCOPED_N("ApplyCollisionResults");
			CudaInterface::instance()->applyCollisionResults(this);
		}
	}
#else
			});
		FrameTimer::instance()->logEvent(FrameTimer::e_Start);

		// BUG-001 FIX: Wrap all parallel workers with WorkerScope for suspend() synchronization
		hdt_parallel_for_each(bodies.begin(), bodies.end(), [this](SkinnedMeshBody* shape) {
			WorkerScope workerScope(this);
			if (isCancelled())
				return;
			shape->internalUpdate();
		});
		hdt_parallel_for_each(vertex_shapes.begin(), vertex_shapes.end(), [this](PerVertexShape* shape) {
			WorkerScope workerScope(this);
			if (isCancelled())
				return;
			shape->internalUpdate();
		});
		hdt_parallel_for_each(triangle_shapes.begin(), triangle_shapes.end(), [this](PerTriangleShape* shape) {
			WorkerScope workerScope(this);
			if (isCancelled())
				return;
			shape->internalUpdate();
		});

		for (auto body : bodies) {
			body->m_bulletShape.m_aabb = body->m_shape->m_tree.aabbAll;
		}
		FrameTimer::instance()->logEvent(FrameTimer::e_Internal);

		hdt_parallel_for_each(m_pairs.begin(), m_pairs.end(),
							  [this](const std::pair<SkinnedMeshBody*, SkinnedMeshBody*>& i) {
								  WorkerScope workerScope(this);
								  if (isCancelled())
									  return;
								  if (i.first->m_shape->m_tree.collapseCollideL(&i.second->m_shape->m_tree))
									  SkinnedMeshAlgorithm::processCollision(i.first, i.second, this);
							  });
		FrameTimer::instance()->logEvent(FrameTimer::e_Launched);
		m_pairs.clear();
		FrameTimer::instance()->addManifoldCount(getNumManifolds());
		FrameTimer::instance()->logEvent(FrameTimer::e_End);
	}
#endif

	int CollisionDispatcher::getNumManifolds() const
	{
		return m_manifoldsPtr.size();
	}

	btPersistentManifold* CollisionDispatcher::getManifoldByIndexInternal(int index)
	{
		return m_manifoldsPtr[index];
	}

	btPersistentManifold** CollisionDispatcher::getInternalManifoldPointer()
	{
		return btCollisionDispatcherMt::getInternalManifoldPointer();
	}
} // namespace hdt
