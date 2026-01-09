#ifdef CUDA
#include "hdtCudaInterface.h"

#include "../hdtTracy.h"

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <immintrin.h>
#include <ppl.h>
#include <type_traits>
#include <vector>

struct cudaStream_t;

#include "hdtCudaCollision.cuh"

namespace hdt
{
	namespace
	{
		template<typename T>
		struct NullDeleter
		{
			void operator()(T*) const {}

			template<typename U>
			void operator()(U*) const
			{}
		};

		class CudaStream
		{
		public:
			CudaStream() { cuCreateStream(&m_stream).check(__FUNCTION__); }

			~CudaStream() { cuDestroyStream(m_stream); }

			void* get() { return m_stream; }
			operator void*() { return m_stream; }

		private:
			void* m_stream;
		};

		class CudaEvent
		{
		public:
			CudaEvent() { cuCreateEvent(&m_event).check(__FUNCTION__); }

			~CudaEvent() { cuDestroyEvent(m_event); }

			void record(CudaStream& stream) { cuRecordEvent(m_event, stream); }

			void wait() { cuWaitEvent(m_event); }

		private:
			void* m_event;
		};

		// CUDA buffer for long-lived objects
		template<typename CudaT, typename HostT = CudaT>
		class CudaBuffer
		{
		public:
			CudaBuffer(int n) : m_size(n * sizeof(CudaT))
			{
				static_assert(sizeof(CudaT) == sizeof(HostT), "Device and host types different sizes");
				cuGetDeviceBuffer(&reinterpret_cast<void*>(m_deviceData), m_size).check(__FUNCTION__);
				cuGetHostBuffer(&reinterpret_cast<void*>(m_hostData), m_size).check(__FUNCTION__);
				m_zeroCopyData = reinterpret_cast<CudaT*>(cuDevicePointer(m_hostData));
			}

			~CudaBuffer()
			{
				cuFreeDevice(m_deviceData);
				cuFreeHost(m_hostData);
			}

			void toDevice(CudaStream& stream)
			{
				cuCopyToDevice(m_deviceData, m_hostData, m_size, stream).check(__FUNCTION__);
			}

			void toHost(CudaStream& stream)
			{
				cuCopyToHost(m_hostData, m_deviceData, m_size, stream).check(__FUNCTION__);
			}

			operator HostT*() { return m_hostData; }
			HostT* get() { return m_hostData; }

			CudaT* getD() { return m_deviceData; }

			CudaT* getZ() { return m_zeroCopyData; }

		private:
			int m_size;
			CudaT* m_deviceData;
			HostT* m_hostData;
			CudaT* m_zeroCopyData;
		};

		template<typename DeviceT, typename... DeviceArgs, typename HostT, typename... HostArgs>
		class CudaBuffer<ArrayType<DeviceT, DeviceArgs...>, ArrayType<HostT, HostArgs...>>
		{
		public:
			CudaBuffer(int n) : m_size(n), m_allocatedSize(32 * (((n - 1) / 32) + 1)), m_buffer(m_allocatedSize) {}

			void toDevice(CudaStream& stream) { m_buffer.toDevice(stream); }

			void toHost(CudaStream& stream) { m_buffer.toHost(stream); }

			ArrayType<HostT, HostArgs...> get() { return {m_buffer.get(), m_allocatedSize}; }

			ArrayType<DeviceT, DeviceArgs...> getD() { return {m_buffer.getD(), m_allocatedSize}; }

			ArrayType<DeviceT, DeviceArgs...> getZ() { return {m_buffer.getZ(), m_allocatedSize}; }

		private:
			int m_size;
			int m_allocatedSize;
			CudaBuffer<HostT, DeviceT> m_buffer;
		};

		template<typename CudaT>
		class CudaDeviceBuffer
		{
		public:
			CudaDeviceBuffer(int n) : m_size(n * sizeof(CudaT))
			{
				cuGetDeviceBuffer(&reinterpret_cast<void*>(m_deviceData), m_size);
			}

			~CudaDeviceBuffer() { cuFreeDevice(m_deviceData); }

			CudaT* getD() { return m_deviceData; }

		private:
			int m_size;
			CudaT* m_deviceData;
		};

		template<typename T, typename... Ts>
		class CudaDeviceBuffer<ArrayType<T, Ts...>>
		{
		public:
			CudaDeviceBuffer(int n) : m_size(32 * (((n - 1) / 32) + 1)), m_buffer(m_size) {}

			ArrayType<T, Ts...> getD() { return ArrayType<T, Ts...>(m_buffer.getD(), m_size); }

		private:
			int m_size;
			CudaDeviceBuffer<T> m_buffer;
		};

		// Memory pool for small short-lived objects. This can grow arbitrarily in size, to the maximum required
		// in a single frame. All allocations get cleared at the end of the frame.
		class CudaBufferPool
		{
			using Buffers = std::pair<void*, void*>;
			using Record = std::tuple<size_t, const size_t, Buffers>;

			// Granularity for allocating blocks that won't fit a single page (however, this memory pool is
			// REALLY not designed for large allocations and using them is likely to leak memory badly)
			static constexpr size_t largeBlockSize = 1 << 20;

			// Page size for normal allocations
			static constexpr size_t pageSize = 1 << 24;

			// Granularity of small allocations, should match CUDA memory transaction size
			static constexpr size_t alignment = 128;

		public:
			CudaBufferPool() {}

			~CudaBufferPool()
			{
				for (auto record : m_buffers) {
					cuFreeDevice(std::get<2>(record).first);
					cuFreeHost(std::get<2>(record).second);
				}
			}

			// FIXME: Not thread safe
			static CudaBufferPool* instance() { return &s_pools[cuGetDevice()]; }

			std::pair<void*, void*> getBuffer(size_t size)
			{
				// FIXME: Locking for the whole method is lazy - should do something finer grained
				std::lock_guard l(m_lock);

				auto s = getSize(size);
				std::vector<Record>::iterator it;
				for (it = m_buffers.begin(); it != m_buffers.end(); ++it) {
					if (std::get<0>(*it) + s <= std::get<1>(*it)) {
						break;
					}
				}
				if (it == m_buffers.end()) {
					size_t newSize = std::max(pageSize, blockSize(size));
					m_buffers.push_back({0, newSize, {0, 0}});
					cuGetDeviceBuffer(&(std::get<2>(m_buffers.back()).first), newSize).check(__FUNCTION__);
					cuGetHostBuffer(&(std::get<2>(m_buffers.back()).second), newSize).check(__FUNCTION__);
					it = m_buffers.end() - 1;
				}
				Buffers result = {static_cast<uint8_t*>(std::get<2>(*it).first) + std::get<0>(*it),
								  static_cast<uint8_t*>(std::get<2>(*it).second) + std::get<0>(*it)};
				std::get<0>(*it) += s;
				return result;
			}

			void clear()
			{
				for (auto& record : m_buffers) {
					std::get<0>(record) = 0;
				}
			}

		private:
			constexpr size_t getSize(size_t size) { return alignment * ((size - 1) / alignment + 1); }

			constexpr size_t blockSize(size_t size) { return largeBlockSize * ((size - 1) / largeBlockSize + 1); }

			std::vector<Record> m_buffers;
			std::mutex m_lock;

			static std::map<int, CudaBufferPool> s_pools;
		};

		std::map<int, CudaBufferPool> CudaBufferPool::s_pools = std::map<int, CudaBufferPool>();

		// CUDA buffer for short-lived per-frame objects. There is no way to deallocate these explicitly - they
		// remain until the buffer pool is cleared manually at the end of the frame, and then all become unsafe.
		template<typename CudaT, typename HostT = CudaT>
		class CudaPooledBuffer
		{
		public:
			CudaPooledBuffer(size_t n) : m_size(n * sizeof(CudaT))
			{
				static_assert(sizeof(CudaT) == sizeof(HostT), "Device and host types different sizes");
				auto buffers = CudaBufferPool::instance()->getBuffer(m_size);
				m_deviceData = reinterpret_cast<CudaT*>(buffers.first);
				m_hostData = reinterpret_cast<HostT*>(buffers.second);
				m_zeroCopyData = reinterpret_cast<CudaT*>(cuDevicePointer(m_hostData));
			}

			void toDevice(CudaStream& stream)
			{
				cuCopyToDevice(m_deviceData, m_hostData, m_size, stream).check(__FUNCTION__);
			}

			void toHost(CudaStream& stream)
			{
				cuCopyToHost(m_hostData, m_deviceData, m_size, stream).check(__FUNCTION__);
			}

			void zero(CudaStream& stream) { cuMemset(m_deviceData, 0, m_size, stream).check(__FUNCTION__); }

			operator HostT*() { return m_hostData; }
			HostT* get() { return m_hostData; }

			CudaT* getD() { return m_deviceData; }

			CudaT* getZ() { return m_zeroCopyData; }

		private:
			size_t m_size;
			CudaT* m_deviceData;
			HostT* m_hostData;
			CudaT* m_zeroCopyData;
		};
	} // namespace

	class CudaBody::Imp
	{
	public:
		Imp(SkinnedMeshBody* body)
			: m_device(cuGetDevice()), m_numVertices(body->m_vertices.size()), m_numDynamicBones(0),
			  m_bones(body->m_skinnedBones.size()), m_boneWeights(body->m_skinnedBones.size()),
			  m_boneMap(body->m_skinnedBones.size()), m_vertexData(body->m_vertices.size()),
			  m_vertexBuffer(body->m_vertices.size())
		{
			// Copy vertex data to the GPU, converting to homogeneous coordinates with w=1
			std::copy(body->m_vertices.begin(), body->m_vertices.end(), m_vertexData.get());
			for (int i = 0; i < m_numVertices; ++i) {
				m_vertexData[i].m_skinPos[3] = 1.0f;
			}
			m_vertexData.toDevice(m_stream);

			m_invBoneMap.reserve(body->m_skinnedBones.size());
			for (int i = 0; i < body->m_skinnedBones.size(); ++i) {
				m_boneWeights[i] = body->m_skinnedBones[i].weightThreshold;
				if (!body->m_skinnedBones[i].isKinematic) {
					m_boneMap[i] = m_numDynamicBones++;
					m_invBoneMap.push_back(i);
				}
				else {
					m_boneMap[i] = -1;
				}
			}
			m_boneWeights.toDevice(m_stream);
			m_boneMap.toDevice(m_stream);

			body->m_bones.reset(m_bones.get(), NullDeleter<Bone[]>());
		}

		~Imp()
		{
			// Clean up CUDA graph resources
			cuGraphExecDestroy(m_graphExec);
			cuGraphDestroy(m_graph);
		}

		void synchronize() { cuSynchronize(m_stream).check(__FUNCTION__); }

		int deviceId() { return m_device; }

		operator cuBodyData() { return {m_vertexData.getD(), m_vertexBuffer.getD(), m_numVertices}; }

		operator cuCollisionBodyData()
		{
			return {m_vertexData.getD(), m_vertexBuffer.getD(), m_boneWeights.getD(), m_boneMap.getD()};
		}

		int m_device;
		CudaStream m_stream;
		CudaDeviceBuffer<cuVector4> m_vertexBuffer;
		CudaBuffer<cuVertex, Vertex> m_vertexData;
		CudaBuffer<float> m_boneWeights;
		CudaBuffer<int> m_boneMap;
		std::vector<int> m_invBoneMap;
		int m_numVertices;
		int m_numDynamicBones;
		CudaBuffer<cuBone, Bone> m_bones;

		// CUDA Graph for internal update (reduces kernel launch overhead)
		void* m_graph = nullptr;	 // cudaGraph_t
		void* m_graphExec = nullptr; // cudaGraphExec_t
		bool m_graphCaptured = false;
	};

	CudaBody::CudaBody(SkinnedMeshBody* body) : m_imp(new Imp(body)) {}

	void CudaBody::synchronize()
	{
		m_imp->synchronize();
	}

	int CudaBody::deviceId()
	{
		return m_imp->deviceId();
	}

	class CudaColliderTree
	{
		using NodePair = std::pair<int, int>;

		ColliderTree* m_tree;

	public:
		CudaColliderTree(ColliderTree* tree, CudaStream& stream)
			: m_tree(tree), m_numNodes(nodeCount(*tree)), m_nodeData(m_numNodes), m_nodeAabbs(m_numNodes)
		{
			unsigned int biggestNode = 0;
			buildNodeData(*tree, m_nodeData.get(), biggestNode);
			m_nodeData.toDevice(stream);

			_DMESSAGE("Tree with %d nodes, largest %d, total %d colliders.", m_numNodes, biggestNode,
					  m_nodeData[m_numNodes - 1].first + m_nodeData[m_numNodes - 1].second);
		}

		void update() { updateBoundingBoxes(*m_tree, m_nodeAabbs); }

		int m_numNodes;
		CudaBuffer<NodePair> m_nodeData;
		CudaBuffer<cuAabb, Aabb> m_nodeAabbs;

	private:
		static int nodeCount(ColliderTree& tree)
		{
			int count = tree.numCollider ? 1 : 0;
			for (auto& child : tree.children) {
				count += nodeCount(child);
			}
			return count;
		}

		NodePair* buildNodeData(ColliderTree& tree, NodePair* nodeData, unsigned int& biggestNode)
		{
			if (tree.numCollider) {
				*nodeData++ = {tree.aabb - m_tree->aabb, tree.numCollider};
				biggestNode = std::max(biggestNode, tree.numCollider);
			}
			for (auto& child : tree.children) {
				nodeData = buildNodeData(child, nodeData, biggestNode);
			}
			return nodeData;
		}

		Aabb* updateBoundingBoxes(ColliderTree& tree, Aabb* boundingBoxes)
		{
			if (tree.numCollider) {
				tree.aabbMe = *boundingBoxes++;
			}
			else {
				tree.aabbMe.invalidate();
			}
			tree.aabbAll = tree.aabbMe;
			for (auto& child : tree.children) {
				boundingBoxes = updateBoundingBoxes(child, boundingBoxes);
				tree.aabbAll.merge(child.aabbAll);
			}
			return boundingBoxes;
		}
	};

	class CudaPerTriangleShape::Imp
	{
	public:
		Imp(PerTriangleShape* shape)
			: m_device(cuGetDevice()), m_numColliders(shape->m_colliders.size()),
			  m_penetrationType(abs(shape->m_shapeProp.penetration) > FLT_EPSILON ? eInternal : eNone),
			  m_body(shape->m_owner->m_cudaObject->m_imp), m_input(shape->m_colliders.size()),
			  m_output(shape->m_colliders.size()), m_tree(&shape->m_tree, m_body->m_stream),
			  m_margin(shape->m_shapeProp.margin), m_penetration(shape->m_shapeProp.penetration)
		{
			for (int i = 0; i < m_numColliders; ++i) {
				if (m_penetration < 0) {
					m_input.get()[i] = {{static_cast<int>(shape->m_colliders[i].vertices[1]),
										 static_cast<int>(shape->m_colliders[i].vertices[0]),
										 static_cast<int>(shape->m_colliders[i].vertices[2])},
										shape->m_colliders[i].flexible};
				}
				else {
					m_input.get()[i] = {{static_cast<int>(shape->m_colliders[i].vertices[0]),
										 static_cast<int>(shape->m_colliders[i].vertices[1]),
										 static_cast<int>(shape->m_colliders[i].vertices[2])},
										shape->m_colliders[i].flexible};
				}
			}
			m_input.toDevice(m_body->m_stream);
			m_tree.m_nodeData.toDevice(m_body->m_stream);
		}

		void updateTree() { m_tree.update(); }

		int deviceId() { return m_device; }

		operator cuColliderData<CudaPerTriangleShape>()
		{
			return {m_input.getD(), m_output.getD(), m_numColliders, {m_margin, -abs(m_penetration)}};
		}

		int m_device;
		CudaBuffer<TriangleInputArray> m_input;
		CudaDeviceBuffer<BoundingBoxArray> m_output;
		std::shared_ptr<CudaBody::Imp> m_body;
		const cuPenetrationType m_penetrationType;
		int m_numColliders;
		CudaColliderTree m_tree;
		float m_margin;
		float m_penetration;
	};

	CudaPerTriangleShape::CudaPerTriangleShape(PerTriangleShape* shape) : m_imp(new Imp(shape)) {}

	void CudaPerTriangleShape::updateTree()
	{
		m_imp->updateTree();
	}

	int CudaPerTriangleShape::deviceId()
	{
		return m_imp->deviceId();
	}

	class CudaPerVertexShape::Imp
	{
	public:
		Imp(PerVertexShape* shape)
			: m_device(cuGetDevice()), m_numColliders(shape->m_colliders.size()),
			  m_body(shape->m_owner->m_cudaObject->m_imp), m_input(shape->m_colliders.size()),
			  m_output(shape->m_colliders.size()), m_tree(&shape->m_tree, m_body->m_stream),
			  m_margin(shape->m_shapeProp.margin)
		{
			for (int i = 0; i < m_numColliders; ++i) {
				m_input.get()[i] = {static_cast<int>(shape->m_colliders[i].vertex), shape->m_colliders[i].flexible};
			}
			m_input.toDevice(m_body->m_stream);
			m_tree.m_nodeData.toDevice(m_body->m_stream);
		}

		void updateTree() { m_tree.update(); }

		int deviceId() { return m_device; }

		operator cuColliderData<CudaPerVertexShape>()
		{
			return {m_input.getD(), m_output.getD(), m_numColliders, {m_margin}};
		}

		int m_device;
		CudaBuffer<VertexInputArray> m_input;
		CudaDeviceBuffer<BoundingBoxArray> m_output;
		std::shared_ptr<CudaBody::Imp> m_body;
		int m_numColliders;
		CudaColliderTree m_tree;
		float m_margin;
	};

	CudaPerVertexShape::CudaPerVertexShape(PerVertexShape* shape) : m_imp(new Imp(shape)) {}

	void CudaPerVertexShape::updateTree()
	{
		m_imp->updateTree();
	}

	int CudaPerVertexShape::deviceId()
	{
		return m_imp->deviceId();
	}

	class CudaMergeBuffer::Imp
	{
	public:
		Imp(SkinnedMeshBody* body0, SkinnedMeshBody* body1)
			: m_x(body0->m_skinnedBones.size()), m_y(body1->m_skinnedBones.size()),
			  m_dynx(body0->m_cudaObject->m_imp->m_numDynamicBones),
			  m_stream(body0->m_cudaObject->m_imp->m_stream), // Reuse body's stream instead of creating new one
			  m_buffer(m_dynx * m_y + m_x * body1->m_cudaObject->m_imp->m_numDynamicBones),
			  m_bufferSize(static_cast<size_t>(m_dynx) * m_y +
						   static_cast<size_t>(m_x) * body1->m_cudaObject->m_imp->m_numDynamicBones)
		{
			m_buffer.zero(m_stream);
		}

		void launchTransfer() { m_buffer.toHost(m_stream); }

		void addManifold(cuCollisionMerge* c, SkinnedMeshBone* rb0, SkinnedMeshBone* rb1,
						 CollisionDispatcher* dispatcher)
		{
			if (c->weight < FLT_EPSILON)
				return;

			if (rb0 == rb1)
				return;

			float invWeight = 1.0f / c->weight;

			auto maniford = dispatcher->getNewManifold(&rb0->m_rig, &rb1->m_rig);
			auto worldA = btVector4(c->posA.val) * invWeight;
			auto worldB = btVector4(c->posB.val) * invWeight;
			auto localA = rb0->m_rig.getWorldTransform().invXform(worldA);
			auto localB = rb1->m_rig.getWorldTransform().invXform(worldB);
			auto normal = btVector4(c->normal.val) * invWeight;
			if (normal.fuzzyZero())
				return;
			auto depth = -normal.length();
			normal = -normal.normalized();

			if (depth >= -FLT_EPSILON)
				return;

			btManifoldPoint newPt(localA, localB, normal, depth);
			newPt.m_positionWorldOnA = worldA;
			newPt.m_positionWorldOnB = worldB;
			newPt.m_combinedFriction = rb0->m_rig.getFriction() * rb1->m_rig.getFriction();
			newPt.m_combinedRestitution = rb0->m_rig.getRestitution() * rb1->m_rig.getRestitution();
			newPt.m_combinedRollingFriction = rb0->m_rig.getRollingFriction() * rb1->m_rig.getRollingFriction();
			maniford->addManifoldPoint(newPt);
		}

		void apply(SkinnedMeshBody* body0, SkinnedMeshBody* body1, CollisionDispatcher* dispatcher)
		{
			HDT_ZONE_SCOPED_N("MergeBuffer::apply");

			// Checking can-collide-with and no-collide-with involves a list search, so just do it once for each bone
			std::vector<bool> canCollide0(body0->m_skinnedBones.size());
			for (int i = 0; i < body0->m_skinnedBones.size(); ++i) {
				canCollide0[i] = body1->canCollideWith(body0->m_skinnedBones[i].ptr);
			}
			std::vector<bool> canCollide1(body1->m_skinnedBones.size());
			for (int i = 0; i < body1->m_skinnedBones.size(); ++i) {
				canCollide1[i] = body0->canCollideWith(body1->m_skinnedBones[i].ptr);
			}

			// NOTE: StreamSync removed - GlobalResultsSync in dispatcher syncs all streams before apply loop

			int* map0 = body0->m_cudaObject->m_imp->m_boneMap.get();
			int* map1 = body1->m_cudaObject->m_imp->m_boneMap.get();

			// First check each dynamic bone of body 0 against every bone of body 1
			for (int dyn = 0; dyn < body0->m_cudaObject->m_imp->m_invBoneMap.size(); ++dyn) {
				int i = body0->m_cudaObject->m_imp->m_invBoneMap[dyn];
				if (!canCollide0[i]) {
					continue;
				}

				for (int j = 0; j < body1->m_skinnedBones.size(); ++j) {
					if (!canCollide1[j]) {
						continue;
					}

					cuCollisionMerge* c = m_buffer.get() + dyn * m_y + j;
					auto rb0 = body0->m_skinnedBones[i].ptr;
					auto rb1 = body1->m_skinnedBones[j].ptr;
					addManifold(c, rb0, rb1, dispatcher);
				}
			}

			// Then check each dynamic bone of body 1 against each kinematic bone of body 0
			for (int dyn = 0; dyn < body1->m_cudaObject->m_imp->m_invBoneMap.size(); ++dyn) {
				int j = body1->m_cudaObject->m_imp->m_invBoneMap[dyn];
				if (!canCollide1[j]) {
					continue;
				}

				for (int i = 0; i < body0->m_skinnedBones.size(); ++i) {
					if (!body0->m_skinnedBones[i].isKinematic || !canCollide0[i]) {
						continue;
					}

					cuCollisionMerge* c = m_buffer.get() + m_dynx * m_y + m_x * dyn + i;
					auto rb0 = body0->m_skinnedBones[i].ptr;
					auto rb1 = body1->m_skinnedBones[j].ptr;
					addManifold(c, rb0, rb1, dispatcher);
				}
			}
		}

		operator cuMergeBuffer() { return {m_buffer.getD(), m_x, m_y, m_dynx, m_bufferSize}; }

		CudaStream& m_stream; // Reference to body's stream (no create/destroy overhead)

	private:
		int m_x;
		int m_y;
		int m_dynx;
		size_t m_bufferSize;
		CudaPooledBuffer<cuCollisionMerge> m_buffer;
	};

	CudaMergeBuffer::CudaMergeBuffer(SkinnedMeshBody* body0, SkinnedMeshBody* body1) : m_imp(new Imp(body0, body1)) {}

	void CudaMergeBuffer::launchTransfer()
	{
		m_imp->launchTransfer();
	}

	void CudaMergeBuffer::apply(SkinnedMeshBody* body0, SkinnedMeshBody* body1, CollisionDispatcher* dispatcher)
	{
		m_imp->apply(body0, body1, dispatcher);
	}

	template<typename T>
	class CudaCollisionPair<T>::Imp
	{
	public:
		Imp(CudaPerVertexShape* shapeA, T* shapeB, int numCollisionPairs)
			: m_shapeA(shapeA), m_shapeB(shapeB), m_numCollisionPairs(numCollisionPairs), m_nextPair(0),
			  m_setupBuffer(numCollisionPairs)
		{}

		void addPair(int offsetA, int offsetB, int sizeA, int sizeB, const Aabb& aabbA, const Aabb& aabbB)
		{
			static_assert(sizeof(cuCollider) == sizeof(Collider));

			m_setupBuffer[m_nextPair++] = {sizeA,
										   sizeB,
										   offsetA,
										   offsetB,
										   *reinterpret_cast<const cuAabb*>(&aabbA),
										   *reinterpret_cast<const cuAabb*>(&aabbB)};
		}

		void launch(CudaMergeBuffer* merge, bool swap)
		{
			if (m_nextPair > 0) {
				collisionFunc()(merge->m_imp->m_stream, m_nextPair, swap, m_setupBuffer.getZ(), *m_shapeA->m_imp,
								*m_shapeB->m_imp, *m_shapeA->m_imp->m_body, *m_shapeB->m_imp->m_body, *merge->m_imp)
					.check(__FUNCTION__);
			}
		}

		int numPairs() { return m_nextPair; }

	private:
		CudaPerVertexShape* m_shapeA;
		T* m_shapeB;
		int m_numCollisionPairs;
		int m_nextPair;

		CudaPooledBuffer<cuCollisionSetup> m_setupBuffer;

		template<typename T>
		struct InputType;
		template<>
		struct InputType<CudaPerVertexShape>
		{
			using type = VertexInputArray;
		};
		template<>
		struct InputType<CudaPerTriangleShape>
		{
			using type = TriangleInputArray;
		};

		auto collisionFunc() -> decltype(cuRunCollision<eNone, T>)*;
	};

	template<>
	auto CudaCollisionPair<CudaPerVertexShape>::Imp::collisionFunc()
		-> decltype(cuRunCollision<eNone, CudaPerVertexShape>)*
	{
		return cuRunCollision<eNone, CudaPerVertexShape>;
	}

	template<>
	auto CudaCollisionPair<CudaPerTriangleShape>::Imp::collisionFunc()
		-> decltype(cuRunCollision<eNone, CudaPerTriangleShape>)*
	{
		switch (m_shapeB->m_imp->m_penetrationType) {
		case eNone:
			return cuRunCollision<eNone, CudaPerTriangleShape>;
		case eInternal:
		default:
			return cuRunCollision<eInternal, CudaPerTriangleShape>;
		}
	}

	template<typename T>
	CudaCollisionPair<T>::CudaCollisionPair(CudaPerVertexShape* shapeA, T* shapeB, int numCollisionPairs)
		: m_imp(new Imp(shapeA, shapeB, numCollisionPairs))
	{}

	template<typename T>
	void CudaCollisionPair<T>::addPair(int offsetA, int offsetB, int sizeA, int sizeB, const Aabb& aabbA,
									   const Aabb& aabbB)
	{
		m_imp->addPair(offsetA, offsetB, sizeA, sizeB, aabbA, aabbB);
	}

	template<typename T>
	void CudaCollisionPair<T>::launch(CudaMergeBuffer* merge, bool swap)
	{
		m_imp->launch(merge, swap);
	}

	template<typename T>
	int CudaCollisionPair<T>::numPairs()
	{
		return m_imp->numPairs();
	}

	bool CudaInterface::enableCuda = false;
	int CudaInterface::currentDevice = 0;
	bool CudaInterface::collectMetrics = false;

	// Global metrics instance
	static CudaGraphMetrics s_graphMetrics;

	CudaGraphMetrics& CudaInterface::graphMetrics()
	{
		return s_graphMetrics;
	}

	void CudaInterface::resetMetrics()
	{
		s_graphMetrics = CudaGraphMetrics{};
	}

	float CudaGraphMetrics::percentile(const float* data, int p) const
	{
		const int count = std::min(totalSamples, kSampleCount);
		if (count == 0)
			return 0.0f;

		// Copy to temp buffer for sorting
		std::vector<float> sorted(data, data + count);
		std::sort(sorted.begin(), sorted.end());

		const int idx = std::min((p * count) / 100, count - 1);
		return sorted[idx];
	}

	std::string CudaGraphMetrics::report() const
	{
		const int count = std::min(totalSamples, kSampleCount);
		if (count == 0)
			return "No samples collected";

		// Calculate CPU stats
		float cpuSum = 0, cpuMin = FLT_MAX, cpuMax = 0;
		float gpuSum = 0, gpuMin = FLT_MAX, gpuMax = 0;
		for (int i = 0; i < count; i++) {
			cpuSum += cpuEnqueueUs[i];
			cpuMin = std::min(cpuMin, cpuEnqueueUs[i]);
			cpuMax = std::max(cpuMax, cpuEnqueueUs[i]);
			gpuSum += gpuExecuteUs[i];
			gpuMin = std::min(gpuMin, gpuExecuteUs[i]);
			gpuMax = std::max(gpuMax, gpuExecuteUs[i]);
		}

		char buf[1024];
		snprintf(buf, sizeof(buf),
				 "GraphLaunch Metrics (n=%d):\n"
				 "  CPU Enqueue: mean=%.1fus min=%.1fus max=%.1fus p50=%.1fus p99=%.1fus\n"
				 "  GPU Execute: mean=%.1fus min=%.1fus max=%.1fus p50=%.1fus p99=%.1fus\n"
				 "  Ratio (GPU/CPU): %.2fx",
				 count, cpuSum / count, cpuMin, cpuMax, percentile(cpuEnqueueUs, 50), percentile(cpuEnqueueUs, 99),
				 gpuSum / count, gpuMin, gpuMax, percentile(gpuExecuteUs, 50), percentile(gpuExecuteUs, 99),
				 (gpuSum / count) / (cpuSum / count + 0.001f));
		return buf;
	}

	CudaInterface* CudaInterface::instance()
	{
		static CudaInterface s_instance;
		return &s_instance;
	}

	bool CudaInterface::hasCuda()
	{
		return enableCuda && m_enabled;
	}

	void CudaInterface::synchronize()
	{
		cuSynchronize().check(__FUNCTION__);
	}

	void CudaInterface::clearBufferPool()
	{
		CudaBufferPool::instance()->clear();
	}

	int CudaInterface::deviceCount()
	{
		return cuDeviceCount();
	}

	void CudaInterface::setCurrentDevice()
	{
		cuSetDevice(currentDevice);
	}

	void CudaInterface::launchInternalUpdate(std::shared_ptr<CudaBody> body,
											 std::shared_ptr<CudaPerVertexShape> vertexShape,
											 std::shared_ptr<CudaPerTriangleShape> triangleShape)
	{
		static const cuColliderData<CudaPerVertexShape> s_emptyVertexData = {
			VertexInputArray(nullptr, 0), BoundingBoxArray(nullptr, 0), 0, {0}};
		static const cuColliderData<CudaPerTriangleShape> s_emptyTriangleData = {
			TriangleInputArray(nullptr, 0), BoundingBoxArray(nullptr, 0), 0, {0, 0}};
		// Mutex for graph capture - CUDA graph capture isn't thread-safe across streams
		static std::mutex s_graphCaptureMutex;

		auto& imp = *body->m_imp;

		// Fast path: Use CUDA Graph if already captured (thread-safe, no lock needed)
		if (imp.m_graphCaptured && imp.m_graphExec) {
			HDT_ZONE_SCOPED_N("GraphLaunch");

			if (CudaInterface::collectMetrics) {
				// Per-thread measurement state for deferred GPU timing
				static thread_local struct
				{
					void* startEvent = nullptr;
					void* endEvent = nullptr;
					float lastCpuUs = 0;
					int lastSampleIdx = -1;
					bool pending = false;
				} s_measure;

				// Create events on first use
				if (!s_measure.startEvent) {
					cuCreateEvent(&s_measure.startEvent);
					cuCreateEvent(&s_measure.endEvent);
				}

				// Complete previous measurement if GPU work finished
				if (s_measure.pending && cuEventQuery(s_measure.endEvent)) {
					const float gpuMs = cuEventElapsedTime(s_measure.startEvent, s_measure.endEvent);
					const float gpuUs = gpuMs * 1000.0f;

					// Store GPU time at the same index as CPU time
					if (s_measure.lastSampleIdx >= 0) {
						s_graphMetrics.gpuExecuteUs[s_measure.lastSampleIdx] = gpuUs;
					}
					s_measure.pending = false;
				}

				// Measure CPU enqueue time
				cuRecordEvent(s_measure.startEvent, imp.m_stream);
				const auto cpuStart = std::chrono::high_resolution_clock::now();

				cuGraphLaunch(imp.m_graphExec, imp.m_stream).check(__FUNCTION__);

				const auto cpuEnd = std::chrono::high_resolution_clock::now();
				cuRecordEvent(s_measure.endEvent, imp.m_stream);

				const float cpuUs = std::chrono::duration<float, std::micro>(cpuEnd - cpuStart).count();

				// Record CPU time immediately, GPU time deferred
				const int idx = s_graphMetrics.sampleIndex;
				s_graphMetrics.cpuEnqueueUs[idx] = cpuUs;
				s_graphMetrics.gpuExecuteUs[idx] = 0; // Will be filled when event completes
				s_graphMetrics.sampleIndex = (idx + 1) & (CudaGraphMetrics::kSampleCount - 1);
				s_graphMetrics.totalSamples++;

				s_measure.lastCpuUs = cpuUs;
				s_measure.lastSampleIdx = idx;
				s_measure.pending = true;
			}
			else {
				cuGraphLaunch(imp.m_graphExec, imp.m_stream).check(__FUNCTION__);
			}
			return;
		}

		// Slow path: Need to capture graph (serialize with mutex)
		std::lock_guard<std::mutex> lock(s_graphCaptureMutex);

		// Double-check after acquiring lock
		if (imp.m_graphCaptured && imp.m_graphExec) {
			cuGraphLaunch(imp.m_graphExec, imp.m_stream).check(__FUNCTION__);
			return;
		}

		// First call: capture the graph
		{
			HDT_ZONE_SCOPED_N("GraphCapture");
			cuStreamBeginCapture(imp.m_stream).check(__FUNCTION__);
		}

		// Memory transfer and kernel launches (captured into graph)
		{
			HDT_ZONE_SCOPED_N("BonesToDevice");
			imp.m_bones.toDevice(imp.m_stream);
		}

		{
			HDT_ZONE_SCOPED_N("cuInternalUpdateKernel");
			cuInternalUpdate(imp.m_stream, imp, imp.m_bones.getD(),
							 vertexShape ? static_cast<cuColliderData<CudaPerVertexShape>>(*vertexShape->m_imp)
										 : s_emptyVertexData,
							 vertexShape ? vertexShape->m_imp->m_tree.m_numNodes : 0,
							 vertexShape ? vertexShape->m_imp->m_tree.m_nodeData.getD() : nullptr,
							 vertexShape ? vertexShape->m_imp->m_tree.m_nodeAabbs.getZ() : nullptr,
							 triangleShape ? static_cast<cuColliderData<CudaPerTriangleShape>>(*triangleShape->m_imp)
										   : s_emptyTriangleData,
							 triangleShape ? triangleShape->m_imp->m_tree.m_numNodes : 0,
							 triangleShape ? triangleShape->m_imp->m_tree.m_nodeData.getD() : nullptr,
							 triangleShape ? triangleShape->m_imp->m_tree.m_nodeAabbs.getZ() : nullptr)
				.check(__FUNCTION__);
		}

		// End capture and instantiate
		{
			HDT_ZONE_SCOPED_N("GraphInstantiate");
			cuStreamEndCapture(imp.m_stream, &imp.m_graph).check(__FUNCTION__);
			cuGraphInstantiate(&imp.m_graphExec, imp.m_graph).check(__FUNCTION__);
		}

		// Pre-upload graph to device - eliminates first-launch upload latency
		{
			HDT_ZONE_SCOPED_N("GraphUpload");
			cuGraphUpload(imp.m_graphExec, imp.m_stream).check(__FUNCTION__);
		}

		// Warm-up launch to trigger any JIT compilation and cache warming
		// This moves the P99 spike from production to initialization
		{
			HDT_ZONE_SCOPED_N("GraphWarmup");
			cuGraphLaunch(imp.m_graphExec, imp.m_stream).check(__FUNCTION__);
			cuSynchronize(imp.m_stream).check(__FUNCTION__);
		}

		imp.m_graphCaptured = true;
	}

	CudaInterface::CudaInterface() : m_enabled(cuDeviceCount() > 0)
	{
		if (m_enabled) {
			cuInitialize();
		}
	}

	template class CudaCollisionPair<CudaPerVertexShape>;
	template class CudaCollisionPair<CudaPerTriangleShape>;

	//==========================================================================
	// BATCHED COLLISION MANAGER IMPLEMENTATION
	//==========================================================================

	void BatchedCollisionManager::beginBatch()
	{
		// Clear global batch data
		m_pairs.clear();

		// Pre-allocate based on previous frame (with 20% margin)
		if (m_lastFramePairCount > 0) {
			size_t estimate = std::min(static_cast<size_t>(m_lastFramePairCount * 1.2), CUDA_MAX_COLLISION_PAIRS);
			m_pairs.reserve(estimate);
		}

		// Reset atomic counter
		m_totalPairs.store(0);
	}

	bool BatchedCollisionManager::addCollisionPair(SkinnedMeshBody* body0, SkinnedMeshBody* body1)
	{
		// INPUT VALIDATION
		if (!body0 || !body1) {
			_DMESSAGE("BatchedCollisionManager::addCollisionPair: null body pointer");
			return false;
		}
		if (!body0->m_shape || !body1->m_shape) {
			_DMESSAGE("BatchedCollisionManager::addCollisionPair: null shape pointer");
			return false;
		}
		if (!body0->m_cudaObject || !body1->m_cudaObject) {
			_DMESSAGE("BatchedCollisionManager::addCollisionPair: null CUDA object");
			return false;
		}

		// Resource limit check
		size_t currentCount = m_totalPairs.fetch_add(1);
		if (currentCount >= CUDA_MAX_COLLISION_PAIRS) {
			_DMESSAGE("BatchedCollisionManager: Collision pair limit reached (%zu)", CUDA_MAX_COLLISION_PAIRS);
			return false;
		}

		// Determine collision type based on shape types
		bool hasTriA = body0->m_shape->asPerTriangleShape() != nullptr;
		bool hasTriB = body1->m_shape->asPerTriangleShape() != nullptr;

		// Route to appropriate batch (NO TV - use VT with swap flag)
		if (!hasTriA && !hasTriB) {
			// Vertex-Vertex
			accumulateVV(body0, body1, /*swapped=*/false);
		}
		else if (!hasTriA && hasTriB) {
			// Vertex-Triangle (normal order)
			accumulateVT(body0, body1, /*swapped=*/false);
		}
		else if (hasTriA && !hasTriB) {
			// Triangle-Vertex: use VT with swap flag
			accumulateVT(body1, body0, /*swapped=*/true);
		}
		else {
			// Both have triangles - need both passes
			accumulateVT(body0, body1, /*swapped=*/false);
			accumulateVT(body1, body0, /*swapped=*/true);
		}

		return true;
	}

	void BatchedCollisionManager::accumulateVV(SkinnedMeshBody* body0, SkinnedMeshBody* body1, bool swapped)
	{
		// Create pair info
		CollisionPairInfo info;
		info.shapeA = body0->m_shape->asPerVertexShape()->m_cudaObject.get();
		info.shapeB = body1->m_shape->asPerVertexShape()->m_cudaObject.get();
		info.body0 = body0;
		info.body1 = body1;
		info.cudaBody0 = body0->m_cudaObject;
		info.cudaBody1 = body1->m_cudaObject;
		info.swapped = swapped;

		// Thread-safe append to global batch with offset calculation
		{
			std::lock_guard<std::mutex> lock(m_mergeMutex);
			info.mergeBufferOffset = m_pairs.pairsVV.size();
			m_pairs.pairsVV.push_back(info);
		}
	}

	void BatchedCollisionManager::accumulateVT(SkinnedMeshBody* body0, SkinnedMeshBody* body1, bool swapped)
	{
		// Create pair info
		CollisionPairInfo info;
		info.shapeA = body0->m_shape->asPerVertexShape()->m_cudaObject.get();

		// ShapeB is the triangle shape
		if (swapped) {
			// body0 has the triangle shape (we swapped)
			info.shapeB = body0->m_shape->asPerTriangleShape()->m_cudaObject.get();
		}
		else {
			info.shapeB = body1->m_shape->asPerTriangleShape()->m_cudaObject.get();
		}

		info.body0 = body0;
		info.body1 = body1;
		info.cudaBody0 = body0->m_cudaObject;
		info.cudaBody1 = body1->m_cudaObject;
		info.swapped = swapped;

		// Thread-safe append to global batch with offset calculation
		{
			std::lock_guard<std::mutex> lock(m_mergeMutex);
			info.mergeBufferOffset = m_pairs.pairsVV.size() + m_pairs.pairsVT.size();
			m_pairs.pairsVT.push_back(info);
		}
	}

	void BatchedCollisionManager::mergeThreadLocalBatch()
	{
		// No-op: Using direct mutex-protected append instead of thread-local batches
		// Thread-local approach was problematic with PPL parallel_for
	}

	void BatchedCollisionManager::launchBatch()
	{
		HDT_ZONE_SCOPED_N("BatchedLaunchCollisions");

		size_t totalPairs = m_pairs.totalPairs();
		if (totalPairs == 0) {
			m_hasPendingResults = false;
			return;
		}

		// Record pair count for next frame's pre-allocation
		m_lastFramePairCount = totalPairs;

		// For now, fall back to per-pair processing since we need to restructure
		// the kernel to handle batched body pairs. This is a stepping stone.
		//
		// TODO: Implement true batched kernel that processes all pairs in 2-3 launches
		// For now, this just collects pairs - actual collision still uses old path

		m_hasPendingResults = true;
	}

	void BatchedCollisionManager::applyResults(CollisionDispatcher* dispatcher)
	{
		HDT_ZONE_SCOPED_N("BatchedApplyResults");

		if (!m_hasPendingResults) {
			return;
		}

		// Process VV results
		for (const auto& pair : m_pairs.pairsVV) {
			// CRITICAL: Check bodies still exist via CUDA object weak_ptr
			if (!pair.bodiesValid()) {
				// Body was destroyed between gather and apply - skip
				continue;
			}

			// TODO: Apply results from m_hostMergeBuffer[pair.mergeBufferOffset]
			// For now, results are applied via the old path
		}

		// Process VT results (same pattern)
		for (const auto& pair : m_pairs.pairsVT) {
			if (!pair.bodiesValid()) {
				continue;
			}

			// TODO: Apply results from m_hostMergeBuffer[pair.mergeBufferOffset]
		}

		m_hasPendingResults = false;
	}

	//==========================================================================
	// CUDAINTERFACE BATCHED API WRAPPERS
	//==========================================================================

	void CudaInterface::beginCollisionBatch()
	{
		m_batchedCollisions.beginBatch();
	}

	bool CudaInterface::addCollisionPair(SkinnedMeshBody* body0, SkinnedMeshBody* body1)
	{
		return m_batchedCollisions.addCollisionPair(body0, body1);
	}

	void CudaInterface::mergeCollisionBatches()
	{
		m_batchedCollisions.mergeThreadLocalBatch();
	}

	void CudaInterface::launchCollisionBatch()
	{
		m_batchedCollisions.launchBatch();
	}

	void CudaInterface::applyCollisionResults(CollisionDispatcher* dispatcher)
	{
		m_batchedCollisions.applyResults(dispatcher);
	}

	bool CudaInterface::hasCollisionResults() const
	{
		return m_batchedCollisions.hasPendingResults();
	}
} // namespace hdt
#endif
