#include "hdtDispatcher.h"
#include "hdtSkinnedMeshBody.h"
#include "hdtSkinnedMeshAlgorithm.h"
#include "hdtFrameTimer.h"
#include "../hdtTracy.h"
#ifdef CUDA
#include "hdtCudaInterface.h"
#endif

#include <LinearMath/btPoolAllocator.h>

#ifdef CUDA
// If defined, triangle-vertex and vertex-vertex collision results aren't applied until the next frame. This
// allows GPU collision detection to run concurrently with the rest of the game engine, instead of leaving
// the CPU idle waiting for the results. Triangle-triangle collisions are assumed to require the higher
// accuracy, and are always applied in the current frame.
#define CUDA_DELAYED_COLLISIONS
#endif

namespace hdt
{
	void CollisionDispatcher::clearAllManifold()
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);
		for (int i = 0; i < m_manifoldsPtr.size(); ++i)
		{
			auto manifold = m_manifoldsPtr[i];
			manifold->~btPersistentManifold();
			if (m_persistentManifoldPoolAllocator->validPtr(manifold))
				m_persistentManifoldPoolAllocator->freeMemory(manifold);
			else
				btAlignedFree(manifold);
		}
		m_manifoldsPtr.clear();
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

		if (shape0 || shape1)
		{
			return hdt::needsCollision(shape0, shape1);
		}
		if (body0->isStaticOrKinematicObject() && body1->isStaticOrKinematicObject())
			return false;
		if (body0->checkCollideWith(body1) || body1->checkCollideWith(body0))
		{
			auto rb0 = static_cast<SkinnedMeshBone*>(body0->getUserPointer());
			auto rb1 = static_cast<SkinnedMeshBone*>(body1->getUserPointer());

			return rb0->canCollideWith(rb1) && rb1->canCollideWith(rb0);
		}
		else return false;
	}

	void CollisionDispatcher::dispatchAllCollisionPairs(btOverlappingPairCache* pairCache,
		const btDispatcherInfo& dispatchInfo, btDispatcher* dispatcher)
	{
		HDT_ZONE_SCOPED_N("DispatchCollisionPairs");
		auto size = pairCache->getNumOverlappingPairs();
		HDT_ZONE_VALUE(static_cast<int64_t>(size));
		if (!size) return;

		m_pairs.reserve(size);
		auto pairs = pairCache->getOverlappingPairArrayPtr();
#ifdef CUDA
		using UpdateMap = std::unordered_map<SkinnedMeshBody*, std::pair<PerVertexShape*, PerTriangleShape*> >;
		UpdateMap to_update;
		// Find bodies and meshes that need collision checking. We want to keep them together in a map so they can
		// be grouped by CUDA stream
		for (int i = 0; i < size; ++i)
#else
		SpinLock lock;
		std::unordered_set<SkinnedMeshBody*> bodies;
		std::unordered_set<PerVertexShape*> vertex_shapes;
		std::unordered_set<PerTriangleShape*> triangle_shapes;

		concurrency::parallel_for(0, size, [&](int i)
#endif
			{
				auto& pair = pairs[i];

				auto shape0 = dynamic_cast<SkinnedMeshBody*>(static_cast<btCollisionObject*>(pair.m_pProxy0->m_clientObject));
				auto shape1 = dynamic_cast<SkinnedMeshBody*>(static_cast<btCollisionObject*>(pair.m_pProxy1->m_clientObject));

				if (shape0 || shape1)
				{
					if (hdt::needsCollision(shape0, shape1) && shape0->isBoundingSphereCollided(shape1))
					{
#ifdef CUDA
						auto it0 = to_update.insert({ shape0, {nullptr, nullptr} }).first;
						auto it1 = to_update.insert({ shape1, {nullptr, nullptr} }).first;
#else
						HDT_LOCK_GUARD(l, lock);
						bodies.insert(shape0);
						bodies.insert(shape1);
#endif
						m_pairs.push_back(std::make_pair(shape0, shape1));

						auto a = shape0->m_shape->asPerTriangleShape();
						auto b = shape1->m_shape->asPerTriangleShape();
#ifdef CUDA
						if (a)
							it0->second.second = a;
						else
							it0->second.first = shape0->m_shape->asPerVertexShape();
						if (b)
							it1->second.second = b;
						else
							it1->second.first = shape1->m_shape->asPerVertexShape();
						if (a && b)
						{
							it0->second.first = a->m_verticesCollision;
							it1->second.first = b->m_verticesCollision;
						}
#else
						if (a)
							triangle_shapes.insert(a);
						else
							vertex_shapes.insert(shape0->m_shape->asPerVertexShape());
						if (b)
							triangle_shapes.insert(b);
						else
							vertex_shapes.insert(shape1->m_shape->asPerVertexShape());
						if (a && b)
						{
							vertex_shapes.insert(a->m_verticesCollision);
							vertex_shapes.insert(b->m_verticesCollision);
						}
#endif
					}
				}
				else getNearCallback()(pair, *this, dispatchInfo);
#ifdef CUDA
			}
		bool haveCuda = CudaInterface::instance()->hasCuda() && (!FrameTimer::instance()->running() || FrameTimer::instance()->cudaFrame());
		FrameTimer::instance()->logEvent(FrameTimer::e_Start);
		if (haveCuda)
		{
			bool initialized = true;
			int deviceId = CudaInterface::currentDevice;

			// Build simple vectors of the things to update, and determine whether any new CUDA objects need
			// to be created - either because there isn't one already, or because it's on the wrong device
			for (auto& o : to_update)
			{
				initialized &= static_cast<bool>(o.first->m_cudaObject) && o.first->m_cudaObject->deviceId() == deviceId;
				if (o.second.first)
				{
					initialized &= static_cast<bool>(o.second.first->m_cudaObject) && o.second.first->m_cudaObject->deviceId() == deviceId;
				}
				if (o.second.second)
				{
					initialized &= static_cast<bool>(o.second.second->m_cudaObject) && o.second.second->m_cudaObject->deviceId() == deviceId;
				}
			}

			// Create any new CUDA objects if necessary
			if (!initialized)
			{
				concurrency::parallel_for_each(to_update.begin(), to_update.end(), [deviceId](UpdateMap::value_type& o)
					{
						CudaInterface::instance()->setCurrentDevice();

						if (!o.first->m_cudaObject || o.first->m_cudaObject->deviceId() != deviceId)
						{
							o.first->m_cudaObject.reset(new CudaBody(o.first));
						}
						if (o.second.first && (!o.second.first->m_cudaObject || o.second.first->m_cudaObject->deviceId() != deviceId))
						{
							o.second.first->m_cudaObject.reset(new CudaPerVertexShape(o.second.first));
						}
						if (o.second.second && (!o.second.second->m_cudaObject || o.second.second->m_cudaObject->deviceId() != deviceId))
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
				// Fully parallel: updateBones (CPU) + CUDA launches (mutex-protected graph capture)
				concurrency::parallel_for_each(to_update.begin(), to_update.end(), [](UpdateMap::value_type& o)
					{
						CudaInterface::instance()->setCurrentDevice();
						o.first->updateBones();
						CudaInterface::launchInternalUpdate(
							o.first->m_cudaObject,
							o.second.first ? o.second.first->m_cudaObject : nullptr,
							o.second.second ? o.second.second->m_cudaObject : nullptr);
					});
			}

			// Update the aggregate parts of the AABB trees
			{
				HDT_ZONE_SCOPED_N("SyncAndTreeUpdates");

				{
					HDT_ZONE_SCOPED_N("GlobalGpuSync");
					// Single global sync instead of N per-body syncs
					CudaInterface::instance()->synchronize();
				}

				{
					HDT_ZONE_SCOPED_N("ParallelTreeUpdates");
					// Tree updates are CPU work - can run in parallel
					concurrency::parallel_for_each(to_update.begin(), to_update.end(), [](UpdateMap::value_type& o)
						{
							if (o.second.first)
							{
								o.second.first->m_cudaObject->updateTree();
							}
							if (o.second.second)
							{
								o.second.second->m_cudaObject->updateTree();
							}
							o.first->m_bulletShape.m_aabb = o.first->m_shape->m_tree.aabbAll;
						});
				}
			}
		}
		else
		{
			concurrency::parallel_for_each(to_update.begin(), to_update.end(), [](UpdateMap::value_type& o)
				{
					o.first->internalUpdate();
					if (o.second.first)
					{
						o.second.first->internalUpdate();
					}
					if (o.second.second)
					{
						o.second.second->internalUpdate();
					}
					o.first->m_bulletShape.m_aabb = o.first->m_shape->m_tree.aabbAll;
				});
		}

		FrameTimer::instance()->logEvent(FrameTimer::e_Internal);
		m_delayedFuncs.clear();

		if (haveCuda)
		{
			CudaInterface::instance()->clearBufferPool();

			// Launch collision checking - parallelized for better CPU utilization
			{
				HDT_ZONE_SCOPED_N("CudaQueueCollisions");
				const int pairCount = static_cast<int>(m_pairs.size());

				// All collisions are now delayed - results applied next frame
				// This eliminates the GlobalResultsSync bottleneck (was 17% of frame time)
				concurrency::parallel_for(0, pairCount, [this](int i)
					{
						auto& pair = m_pairs[i];
						if (pair.first->m_shape->m_tree.collapseCollideL(&pair.second->m_shape->m_tree))
						{
							m_delayedFuncs.push_back(SkinnedMeshAlgorithm::queueCollision(pair.first, pair.second, this));
						}
					});
			}

			FrameTimer::instance()->logEvent(FrameTimer::e_Launched);

			// No sync needed - all collision results are applied next frame
			// GPU work continues asynchronously while we proceed with constraint solving
		}
		else
		{
			// Now we can process the collisions
			concurrency::parallel_for_each(m_pairs.begin(), m_pairs.end(),
				[this](std::pair<SkinnedMeshBody*, SkinnedMeshBody*>& i)
				{
					if (i.first->m_shape->m_tree.collapseCollideL(&i.second->m_shape->m_tree))
					{
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
		// Sync and apply collision results from previous frame
		// Called at START of physics step (before prediction) to allow GPU overlap with solve
		if (!m_delayedFuncs.empty())
		{
			HDT_ZONE_SCOPED_N("SyncPreviousCollisions");
			CudaInterface::instance()->synchronize();

			for (auto& f : m_delayedFuncs)
			{
				f();
			}
			m_delayedFuncs.clear();
		}
	}
#else
			});
		FrameTimer::instance()->logEvent(FrameTimer::e_Start);
		concurrency::parallel_for_each(bodies.begin(), bodies.end(), [](SkinnedMeshBody* shape)
		{
			shape->internalUpdate();
		});
		concurrency::parallel_for_each(vertex_shapes.begin(), vertex_shapes.end(), [](PerVertexShape* shape)
		{
			shape->internalUpdate();
		});
		concurrency::parallel_for_each(triangle_shapes.begin(), triangle_shapes.end(), [](PerTriangleShape* shape)
		{
			shape->internalUpdate();
		});
		for (auto body : bodies)
		{
			body->m_bulletShape.m_aabb = body->m_shape->m_tree.aabbAll;
		}
		FrameTimer::instance()->logEvent(FrameTimer::e_Internal);
		concurrency::parallel_for_each(m_pairs.begin(), m_pairs.end(), [&, this](const std::pair<SkinnedMeshBody*, SkinnedMeshBody*>& i)
		{
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
}
