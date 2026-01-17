#ifdef CUDA
#include "hdtCudaInterface.h"

#include "../hdtLog.h"
#include "../hdtTracy.h"

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <immintrin.h>
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

			// Overload for raw stream pointer (used by batched operations that need to use
			// a shared stream instead of per-body streams to avoid thread-affinity issues)
			void toDevice(void* rawStream)
			{
				cuCopyToDevice(m_deviceData, m_hostData, m_size, rawStream).check(__FUNCTION__);
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

			// Overload for raw stream pointer (used by batched operations)
			void toDevice(void* rawStream) { m_buffer.toDevice(rawStream); }

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
			: m_tree(tree), m_numNodes(nodeCount(*tree)), m_nodeData(m_numNodes),
			  m_leafAabbBuffers{CudaBuffer<cuAabb, Aabb>(m_numNodes), CudaBuffer<cuAabb, Aabb>(m_numNodes)},
			  m_currentWriteBuffer(0), m_firstFrame(true)
		{
			unsigned int biggestNode = 0;
			buildNodeData(*tree, m_nodeData.get(), biggestNode);
			m_nodeData.toDevice(stream);

			_DMESSAGE("Tree with %d nodes, largest %d, total %d colliders.", m_numNodes, biggestNode,
					  m_nodeData[m_numNodes - 1].first + m_nodeData[m_numNodes - 1].second);
		}

		// Update tree using PREVIOUS frame's leaf AABBs (no sync required!)
		// Zero-copy: GPU writes to buffer N, CPU reads buffer N-1's pinned host memory directly.
		void update()
		{
			HDT_ZONE_SCOPED_N("TreeUpdate");
			// Read from the OTHER buffer (previous frame's data) via zero-copy pinned memory
			int readBuffer = 1 - m_currentWriteBuffer;
			updateBoundingBoxes(*m_tree, m_leafAabbBuffers[readBuffer].get());
		}

		// Get the zero-copy pointer for GPU to write current frame's leaf AABBs
		cuAabb* getCurrentWriteBuffer() { return m_leafAabbBuffers[m_currentWriteBuffer].getZ(); }

		// No-op: Zero-copy means GPU writes directly to pinned host memory.
		// No cudaMemcpyAsync needed - data is already in host-accessible memory.
		void queueLeafDownload(void* /*stream*/)
		{
			// Zero-copy makes this unnecessary. GPU writes via getZ() go directly to
			// the pinned host memory that get() returns. No copy required.
		}

		// Swap double buffers at frame end
		// After this, currentWriteBuffer points to fresh buffer for next frame's GPU writes
		void swapBuffers() { m_currentWriteBuffer = 1 - m_currentWriteBuffer; }

		// Check if this is the first frame (need sync to bootstrap)
		bool isFirstFrame() const { return m_firstFrame; }
		void clearFirstFrame() { m_firstFrame = false; }

		int m_numNodes;
		CudaBuffer<NodePair> m_nodeData;

		// Double-buffered leaf AABBs: GPU writes to current via getZ(), CPU reads previous via get()
		// Zero-copy: getZ() returns device pointer to pinned host memory, get() returns host pointer
		CudaBuffer<cuAabb, Aabb> m_leafAabbBuffers[2];
		int m_currentWriteBuffer; // Index of buffer GPU writes to (0 or 1)
		bool m_firstFrame;		  // True until first frame completes

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
				// Use colliderOffset directly (no pointer subtraction needed)
				*nodeData++ = {static_cast<unsigned int>(tree.colliderOffset), tree.numCollider};
				biggestNode = std::max(biggestNode, tree.numCollider);
			}
			for (auto& child : tree.children) {
				nodeData = buildNodeData(child, nodeData, biggestNode);
			}
			return nodeData;
		}

		Aabb* updateBoundingBoxes(ColliderTree& tree, const Aabb* boundingBoxes)
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
			return const_cast<Aabb*>(boundingBoxes);
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

	void CudaPerTriangleShape::queueLeafDownload(void* stream)
	{
		m_imp->m_tree.queueLeafDownload(stream);
	}

	void CudaPerTriangleShape::swapBuffers()
	{
		m_imp->m_tree.swapBuffers();
	}

	bool CudaPerTriangleShape::isFirstFrame() const
	{
		return m_imp->m_tree.isFirstFrame();
	}

	void CudaPerTriangleShape::clearFirstFrame()
	{
		m_imp->m_tree.clearFirstFrame();
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

	void CudaPerVertexShape::queueLeafDownload(void* stream)
	{
		m_imp->m_tree.queueLeafDownload(stream);
	}

	void CudaPerVertexShape::swapBuffers()
	{
		m_imp->m_tree.swapBuffers();
	}

	bool CudaPerVertexShape::isFirstFrame() const
	{
		return m_imp->m_tree.isFirstFrame();
	}

	void CudaPerVertexShape::clearFirstFrame()
	{
		m_imp->m_tree.clearFirstFrame();
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

			// SNAPSHOT BONE RIG TRANSFORMS at collision gather time
			// These will be used when applying results (next frame after sync)
			// to compute correct local coordinates from world collision points.
			//
			// We store m_rig.getWorldTransform() because that's what the physics solver
			// uses to reconstruct world positions from local coordinates in btManifoldPoint.
			// The GPU computes collision in world space, solver expects local space relative
			// to m_rig.getWorldTransform().
			//
			// IMPORTANT: Initialize ALL entries to identity first (btTransform has no default init!)
			m_transforms0.resize(body0->m_skinnedBones.size());
			m_transforms1.resize(body1->m_skinnedBones.size());
			for (size_t i = 0; i < m_transforms0.size(); ++i) {
				m_transforms0[i].setIdentity();
				if (body0->m_skinnedBones[i].ptr) {
					m_transforms0[i] = body0->m_skinnedBones[i].ptr->m_rig.getWorldTransform();
				}
			}
			for (size_t i = 0; i < m_transforms1.size(); ++i) {
				m_transforms1[i].setIdentity();
				if (body1->m_skinnedBones[i].ptr) {
					m_transforms1[i] = body1->m_skinnedBones[i].ptr->m_rig.getWorldTransform();
				}
			}
		}

		void launchTransfer() { m_buffer.toHost(m_stream); }

		// Rate-limited warning counter for bad collision data
		inline static std::atomic<int> s_badCollisionCount{0};

		// Use STORED transforms (from collision time) for local coordinate conversion
		void addManifold(cuCollisionMerge* c, SkinnedMeshBone* rb0, SkinnedMeshBone* rb1, int boneIdx0, int boneIdx1,
						 CollisionDispatcher* dispatcher)
		{
			// Defense-in-depth: null check for bone pointers
			if (!rb0 || !rb1) {
				_DMESSAGE("[CUDA-COLL] addManifold: null bone pointer rb0=%p rb1=%p", rb0, rb1);
				return;
			}

			if (c->weight < FLT_EPSILON)
				return;

			if (rb0 == rb1)
				return;

			// Validate bone indices
			if (boneIdx0 < 0 || boneIdx0 >= static_cast<int>(m_transforms0.size()) || boneIdx1 < 0 ||
				boneIdx1 >= static_cast<int>(m_transforms1.size()))
			{
				_DMESSAGE("[CUDA-COLL] addManifold: bone index out of range idx0=%d/%zu idx1=%d/%zu", boneIdx0,
						  m_transforms0.size(), boneIdx1, m_transforms1.size());
				return;
			}

			float invWeight = 1.0f / c->weight;

			auto manifold = dispatcher->getNewManifold(&rb0->m_rig, &rb1->m_rig);
			auto worldA = btVector4(c->posA.val) * invWeight;
			auto worldB = btVector4(c->posB.val) * invWeight;

			// USE STORED TRANSFORMS from collision time (not current transforms)
			// This ensures consistent local coordinates despite 1-frame latency
			auto localA = m_transforms0[boneIdx0].invXform(worldA);
			auto localB = m_transforms1[boneIdx1].invXform(worldB);

			// SANITY CHECK: Detect unreasonably large local coordinates (sign of transform mismatch)
			constexpr float LOCAL_SANITY_LIMIT = 1000.0f; // Skyrim units, bones shouldn't be this far
			if (localA.length() > LOCAL_SANITY_LIMIT || localB.length() > LOCAL_SANITY_LIMIT) {
				int count = s_badCollisionCount.fetch_add(1);
				if (count < 10 || (count % 1000 == 0)) {
					_DMESSAGE("[CUDA-COLL] WARNING: Large local coords detected (count=%d)! "
							  "localA=(%.1f,%.1f,%.1f) len=%.1f, localB=(%.1f,%.1f,%.1f) len=%.1f, "
							  "worldA=(%.1f,%.1f,%.1f), worldB=(%.1f,%.1f,%.1f)",
							  count, localA.x(), localA.y(), localA.z(), localA.length(), localB.x(), localB.y(),
							  localB.z(), localB.length(), worldA.x(), worldA.y(), worldA.z(), worldB.x(), worldB.y(),
							  worldB.z());
				}
				return; // Skip this bad manifold
			}

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
			manifold->addManifoldPoint(newPt);
		}

		// Takes locked CudaBody shared_ptrs directly to avoid TOCTOU race.
		// The weak_ptrs stored at collision time are locked once by the caller (applyResults),
		// then passed here - no reads from body->m_cudaObject.
		void apply(SkinnedMeshBody* body0, SkinnedMeshBody* body1, std::shared_ptr<CudaBody> cuda0,
				   std::shared_ptr<CudaBody> cuda1, CollisionDispatcher* dispatcher)
		{
			HDT_ZONE_SCOPED_N("MergeBuffer::apply");

			// CRITICAL: Check validity BEFORE accessing body pointers.
			// If the owning SkinnedMeshBody started destruction, it sets m_valid=false.
			// The body's other members (m_skinnedBones, etc.) may already be destroyed.
			if (!cuda0->isValid() || !cuda1->isValid()) {
				return; // Owner body is being destroyed, don't access its members
			}

			// Caller guarantees cuda0/cuda1 are valid locked shared_ptrs
			if (!cuda0->m_imp || !cuda1->m_imp) {
				return; // Imp not initialized - shouldn't happen but be safe
			}

			auto& imp0 = *cuda0->m_imp;
			auto& imp1 = *cuda1->m_imp;

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

			int* map0 = imp0.m_boneMap.get();
			int* map1 = imp1.m_boneMap.get();

			// First check each dynamic bone of body 0 against every bone of body 1
			for (int dyn = 0; dyn < imp0.m_invBoneMap.size(); ++dyn) {
				int i = imp0.m_invBoneMap[dyn];
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
					addManifold(c, rb0, rb1, i, j, dispatcher); // Pass bone indices
				}
			}

			// Then check each dynamic bone of body 1 against each kinematic bone of body 0
			for (int dyn = 0; dyn < imp1.m_invBoneMap.size(); ++dyn) {
				int j = imp1.m_invBoneMap[dyn];
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
					addManifold(c, rb0, rb1, i, j, dispatcher); // Pass bone indices
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

		// Bone transforms captured at collision gather time
		// Used for correct local coordinate conversion with 1-frame latency
		std::vector<btTransform> m_transforms0;
		std::vector<btTransform> m_transforms1;
	};

	CudaMergeBuffer::CudaMergeBuffer(SkinnedMeshBody* body0, SkinnedMeshBody* body1) : m_imp(new Imp(body0, body1)) {}

	void CudaMergeBuffer::launchTransfer()
	{
		m_imp->launchTransfer();
	}

	void CudaMergeBuffer::apply(SkinnedMeshBody* body0, SkinnedMeshBody* body1, std::shared_ptr<CudaBody> cuda0,
								std::shared_ptr<CudaBody> cuda1, CollisionDispatcher* dispatcher)
	{
		m_imp->apply(body0, body1, cuda0, cuda1, dispatcher);
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
	bool CudaInterface::gpuTimingEnabled = false;

	// Global metrics instances
	static CudaGraphMetrics s_graphMetrics;
	static GpuTimingStats s_gpuTiming;

	// GPU timing events for internal update batch - created lazily, reused across frames
	static struct InternalUpdateTiming
	{
		void* startEvent = nullptr;
		void* afterBonesEvent = nullptr;
		void* afterKernelsEvent = nullptr;
		bool initialized = false;
		bool pending = false; // True if we have events recorded waiting for sync

		void ensure()
		{
			if (!initialized) {
				cuCreateEvent(&startEvent);
				cuCreateEvent(&afterBonesEvent);
				cuCreateEvent(&afterKernelsEvent);
				initialized = true;
			}
		}
	} s_internalTiming;

	CudaGraphMetrics& CudaInterface::graphMetrics()
	{
		return s_graphMetrics;
	}

	GpuTimingStats& CudaInterface::gpuTiming()
	{
		return s_gpuTiming;
	}

	void CudaInterface::resetMetrics()
	{
		s_graphMetrics = CudaGraphMetrics{};
		s_gpuTiming = GpuTimingStats{};
	}

	std::string GpuTimingStats::report() const
	{
		const int count = std::min(totalSamples, kSampleCount);
		if (count == 0)
			return "GPU Timing: No samples collected (enable with 'smp gputiming')";

		char buf[512];
		snprintf(buf, sizeof(buf),
				 "GPU Timing (n=%d, mean of last 64):\n"
				 "  Bones->Device:     %.3f ms\n"
				 "  Internal Kernels:  %.3f ms\n"
				 "  Collision Kernels: %.3f ms\n"
				 "  Sync Wait:         %.3f ms\n"
				 "  TOTAL GPU:         %.3f ms",
				 count, mean(bonesToDeviceMs), mean(internalKernelsMs), mean(collisionKernelsMs), mean(syncWaitMs),
				 mean(bonesToDeviceMs) + mean(internalKernelsMs) + mean(collisionKernelsMs));
		return buf;
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
		// Measure sync wait time if GPU timing enabled
		const auto syncStart = std::chrono::high_resolution_clock::now();

		cuSynchronize().check(__FUNCTION__);

		// Collect GPU timing if enabled and we have pending measurements
		if (gpuTimingEnabled && s_internalTiming.pending) {
			const auto syncEnd = std::chrono::high_resolution_clock::now();
			const float syncWaitMs = std::chrono::duration<float, std::milli>(syncEnd - syncStart).count();

			// Get GPU execution times from events
			const float bonesMs = cuEventElapsedTime(s_internalTiming.startEvent, s_internalTiming.afterBonesEvent);
			const float kernelsMs =
				cuEventElapsedTime(s_internalTiming.afterBonesEvent, s_internalTiming.afterKernelsEvent);

			// For collision timing, we measure total GPU time minus internal update time
			// (collision kernels overlap with internal updates on different streams)
			// This is an approximation - for precise per-kernel timing would need per-stream events
			const float collisionMs = 0.0f; // TODO: Add collision-specific events if needed

			s_gpuTiming.addSample(bonesMs, kernelsMs, collisionMs, syncWaitMs);
			s_internalTiming.pending = false;
		}
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
							 vertexShape ? vertexShape->m_imp->m_tree.getCurrentWriteBuffer() : nullptr,
							 triangleShape ? static_cast<cuColliderData<CudaPerTriangleShape>>(*triangleShape->m_imp)
										   : s_emptyTriangleData,
							 triangleShape ? triangleShape->m_imp->m_tree.m_numNodes : 0,
							 triangleShape ? triangleShape->m_imp->m_tree.m_nodeData.getD() : nullptr,
							 triangleShape ? triangleShape->m_imp->m_tree.getCurrentWriteBuffer() : nullptr)
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

	// Forward declarations for rate-limited warning counters (defined near accumulate functions)
	extern std::atomic<int> s_skippedPairsVV;
	extern std::atomic<int> s_skippedPairsVT;

	BatchedCollisionManager::BatchedCollisionManager()
	{
		// Initialize CUDA events for completion tracking (disabled timing for faster query)
		for (int i = 0; i < 3; ++i) {
			if (!cuCreateEventWithFlags(&m_completionEvents[i], 0x02).check("BatchedCollisionManager::ctor")) {
				_DMESSAGE("[CUDA-PIPE] Failed to create completion event %d", i);
				m_completionEvents[i] = nullptr;
			}
		}
		m_eventsInitialized = true;
		_DMESSAGE("[CUDA-PIPE] BatchedCollisionManager initialized with 3-buffer pipeline");
	}

	BatchedCollisionManager::~BatchedCollisionManager()
	{
		// Destroy CUDA events
		for (int i = 0; i < 3; ++i) {
			if (m_completionEvents[i]) {
				cuDestroyEvent(m_completionEvents[i]);
				m_completionEvents[i] = nullptr;
			}
		}
		m_eventsInitialized = false;
	}

	bool BatchedCollisionManager::syncIfNeeded()
	{
		HDT_ZONE_SCOPED_N("SyncIfNeeded");

		// NOTE: Sync is now done in syncPreviousCollisionResults() at frame start.
		// This function is retained for potential future event-based sync optimization.
		// Currently a no-op since the main sync path handles all GPU synchronization.
		return false; // No sync performed here - sync happens at frame start
	}

	void BatchedCollisionManager::beginBatch()
	{
		// Clear global batch data (pairs to process this frame)
		m_pairs.clear();

		// Note: m_pendingResults buffers are NOT cleared here.
		// With 1-frame latency: current frame writes to writeIndex(), previous frame's
		// results are in readIndex() (being applied after sync). Clearing happens in
		// applyResults() after application, and in launchBatch() before writing.

		// Pre-allocate based on previous frame (with 20% margin)
		if (m_lastFramePairCount > 0) {
			size_t estimate = std::min(static_cast<size_t>(m_lastFramePairCount * 1.2), CUDA_MAX_COLLISION_PAIRS);
			m_pairs.reserve(estimate);
		}

		// Reset atomic counter
		m_totalPairs.store(0);

		// Reset skip warning counters (log once per batch cycle)
		s_skippedPairsVV.store(0);
		s_skippedPairsVT.store(0);
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

	// Rate-limited warning counter for skipped pairs
	static std::atomic<int> s_skippedPairsVV{0};
	static std::atomic<int> s_skippedPairsVT{0};

	void BatchedCollisionManager::accumulateVV(SkinnedMeshBody* body0, SkinnedMeshBody* body1, bool swapped)
	{
		// Create pair info
		CollisionPairInfo info;

		// Get vertex shapes - must be valid for VV collision
		auto perVertex0 = body0->m_shape->asPerVertexShape();
		auto perVertex1 = body1->m_shape->asPerVertexShape();
		if (!perVertex0 || !perVertex0->m_cudaObject || !perVertex1 || !perVertex1->m_cudaObject) {
			// Invalid vertex shapes - skip this pair (rate-limited warning)
			if (s_skippedPairsVV.fetch_add(1) == 0) {
				_DMESSAGE("BatchedCollisionManager: Skipping VV pair with null CUDA object");
			}
			return;
		}

		info.shapeA = perVertex0->m_cudaObject.get();
		info.shapeB = perVertex1->m_cudaObject.get();
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

		// Get vertex shape from body0 - may be direct per-vertex or from triangle shape's vertex collision
		auto perVertex0 = body0->m_shape->asPerVertexShape();
		auto perTri0 = body0->m_shape->asPerTriangleShape();
		if (perVertex0 && perVertex0->m_cudaObject) {
			info.shapeA = perVertex0->m_cudaObject.get();
		}
		else if (perTri0 && perTri0->m_verticesCollision && perTri0->m_verticesCollision->m_cudaObject) {
			info.shapeA = perTri0->m_verticesCollision->m_cudaObject.get();
		}
		else {
			// No valid vertex shape - skip this pair
			if (s_skippedPairsVT.fetch_add(1) == 0) {
				_DMESSAGE("BatchedCollisionManager: Skipping VT pair - no valid vertex shape");
			}
			return;
		}

		// Get triangle shape from body1
		auto perTri1 = body1->m_shape->asPerTriangleShape();
		if (perTri1 && perTri1->m_cudaObject) {
			info.shapeB = perTri1->m_cudaObject.get();
		}
		else {
			// No valid triangle shape - skip this pair
			if (s_skippedPairsVT.fetch_add(1) == 0) {
				_DMESSAGE("BatchedCollisionManager: Skipping VT pair - no valid triangle shape");
			}
			return;
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

	// Launch collision for a single body pair - mirrors launchCollision from hdtSkinnedMeshAlgorithm.cpp
	template<bool Swap, typename T>
	void BatchedCollisionManager::launchSingleCollision(PerVertexShape* shape0, T* shape1,
														std::shared_ptr<CudaMergeBuffer> cudaMerge)
	{
		ColliderTree* c0 = &shape0->m_tree;
		ColliderTree* c1 = &shape1->m_tree;

		std::vector<std::pair<ColliderTree*, ColliderTree*>> pairs;
		pairs.reserve(c0->colliders.size() + c1->colliders.size());
		c0->checkCollisionL(c1, pairs);
		if (pairs.empty())
			return;
		int npairs = pairs.size();

		CudaCollisionPair<typename T::CudaType> collisionPair(shape0->m_cudaObject.get(), shape1->m_cudaObject.get(),
															  npairs);

		// Set up data for each pair of collision trees
		for (int i = 0; i < npairs; ++i) {
			auto a = pairs[i].first;
			auto b = pairs[i].second;
			auto asize = b->isKinematic ? a->dynCollider : a->numCollider;
			auto bsize = a->isKinematic ? b->dynCollider : b->numCollider;

			if (asize > 0 && bsize > 0) {
				// Use colliderOffset directly (no pointer subtraction needed)
				collisionPair.addPair(pairs[i].first->colliderOffset, pairs[i].second->colliderOffset, asize, bsize,
									  a->aabbMe, b->aabbMe);
			}
		}

		// Run the kernel
		collisionPair.launch(cudaMerge.get(), Swap);
	}

	void BatchedCollisionManager::launchBatch()
	{
		HDT_ZONE_SCOPED_N("BatchedLaunchCollisions");

		_DMESSAGE("[CUDA-PIPE] launchBatch: frame=%d, writeIndex=%d", m_frameCount, writeIndex());

		// Get write buffer for this frame's collision results
		auto& writeBuffer = m_pendingResults[writeIndex()];

		// Clear write buffer - it was read and cleared last frame by applyResults,
		// but clear again for safety during bootstrap or edge cases
		writeBuffer.clear();

		size_t totalPairs = m_pairs.totalPairs();
		if (totalPairs == 0) {
			return;
		}

		// Record pair count for next frame's pre-allocation
		m_lastFramePairCount = totalPairs;

		// Pre-allocate for expected results
		writeBuffer.reserve(totalPairs);

		// Process VV pairs
		{
			HDT_ZONE_SCOPED_N("LaunchVVPairs");
			for (const auto& pairInfo : m_pairs.pairsVV) {
				if (!pairInfo.bodiesValid())
					continue;

				auto body0 = pairInfo.body0;
				auto body1 = pairInfo.body1;

				// Get shapes for tree traversal
				auto vertexShape0 = body0->m_shape->asPerVertexShape();
				auto vertexShape1 = body1->m_shape->asPerVertexShape();
				if (!vertexShape0 || !vertexShape1)
					continue;

				// Create merge buffer and launch collision
				auto mergeBuffer = std::make_shared<CudaMergeBuffer>(body0, body1);
				launchSingleCollision<false>(vertexShape0, vertexShape1, mergeBuffer);
				mergeBuffer->launchTransfer();

				// Store for later application (next frame after sync)
				writeBuffer.push_back({mergeBuffer, body0, body1, pairInfo.cudaBody0, pairInfo.cudaBody1});
			}
		}

		// Process VT pairs
		{
			HDT_ZONE_SCOPED_N("LaunchVTPairs");
			for (const auto& pairInfo : m_pairs.pairsVT) {
				if (!pairInfo.bodiesValid())
					continue;

				auto body0 = pairInfo.body0;
				auto body1 = pairInfo.body1;

				// For VT, body0 should have vertex shape and body1 should have triangle shape
				// But if swapped, roles are reversed
				SkinnedMeshBody* vertexBody = pairInfo.swapped ? body1 : body0;
				SkinnedMeshBody* triangleBody = pairInfo.swapped ? body0 : body1;

				// Get vertex shape (may be from triangle body's m_verticesCollision)
				PerVertexShape* vertexShape = nullptr;
				auto directVertex = vertexBody->m_shape->asPerVertexShape();
				auto vertexFromTri = vertexBody->m_shape->asPerTriangleShape();
				if (directVertex) {
					vertexShape = directVertex;
				}
				else if (vertexFromTri && vertexFromTri->m_verticesCollision) {
					vertexShape = vertexFromTri->m_verticesCollision;
				}

				auto triangleShape = triangleBody->m_shape->asPerTriangleShape();
				if (!vertexShape || !triangleShape)
					continue;

				// Create merge buffer and launch collision
				auto mergeBuffer = std::make_shared<CudaMergeBuffer>(body0, body1);
				if (pairInfo.swapped) {
					launchSingleCollision<true>(vertexShape, triangleShape, mergeBuffer);
				}
				else {
					launchSingleCollision<false>(vertexShape, triangleShape, mergeBuffer);
				}
				mergeBuffer->launchTransfer();

				// Store for later application (next frame after sync)
				writeBuffer.push_back({mergeBuffer, body0, body1, pairInfo.cudaBody0, pairInfo.cudaBody1});
			}
		}

		HDT_ZONE_VALUE(static_cast<int64_t>(writeBuffer.size()));

		// NOTE: Event recording disabled - collision work runs on per-merge-buffer streams,
		// not the default stream, so events can't track completion. See syncIfNeeded TODO.
	}

	void BatchedCollisionManager::swapResultBuffers()
	{
		// Advance frame count - this drives the double buffer ping-pong:
		//   writeIndex = m_frameCount & 1
		//   readIndex = (m_frameCount + 1) & 1  (1 frame behind)
		_DMESSAGE("[CUDA-PIPE] swapResultBuffers: frame %d -> %d", m_frameCount, m_frameCount + 1);
		m_frameCount++;
	}

	void BatchedCollisionManager::applyResults(CollisionDispatcher* dispatcher)
	{
		HDT_ZONE_SCOPED_N("BatchedApplyResults");

		// 1-FRAME LATENCY: Apply results from previous frame (GPU complete after sync)
		// Bone transforms stored in CudaMergeBuffer ensure correct local coordinates
		// even though the actual bones may have moved since collision detection.
		int ridx = readIndex();
		int widx = writeIndex();

		// DIAGNOSTIC: Track pending results buffer sizes
		static int s_applyCounter = 0;
		if (++s_applyCounter % 120 == 0) { // Every ~2 seconds
			_MESSAGE("[CUDA-DIAG] Frame %d: pendingResults[0]=%zu, [1]=%zu, ridx=%d, widx=%d", m_frameCount,
					 m_pendingResults[0].size(), m_pendingResults[1].size(), ridx, widx);
		}

		// Need at least 1 frame to have data in read buffer
		if (isBootstrapping()) {
			return;
		}

		// Get buffer from previous frame (GPU work complete after sync at frame start)
		auto& readBuffer = m_pendingResults[ridx];

		// Hold mutex while accessing pending results to prevent race with body destruction
		std::lock_guard<std::mutex> lock(m_resultsMutex);

		if (readBuffer.empty()) {
			return;
		}

		HDT_ZONE_VALUE(static_cast<int64_t>(readBuffer.size()));

		// Apply all pending collision results (from previous frame, now complete after sync)
		for (auto& result : readBuffer) {
			// CRITICAL: Lock weak_ptrs ONCE and use those locks throughout.
			auto cuda0 = result.cudaBody0.lock();
			auto cuda1 = result.cudaBody1.lock();
			if (!cuda0 || !cuda1) {
				// Body was destroyed between gather and apply - skip
				continue;
			}

			// Pass the locked shared_ptrs directly - no further reads from body->m_cudaObject
			result.mergeBuffer->apply(result.body0, result.body1, cuda0, cuda1, dispatcher);
		}

		// Clear buffer after application
		readBuffer.clear();
	}

	void BatchedCollisionManager::removePendingResultsFor(SkinnedMeshBody* body)
	{
		// Hold mutex while modifying pending results
		std::lock_guard<std::mutex> lock(m_resultsMutex);

		// Remove from both buffers (body might have results in either stage of pipeline)
		for (int i = 0; i < 2; ++i) {
			auto& buffer = m_pendingResults[i];
			auto it = std::remove_if(buffer.begin(), buffer.end(), [body](const PendingCollisionResult& r) {
				return r.body0 == body || r.body1 == body;
			});
			buffer.erase(it, buffer.end());
		}
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

	void CudaInterface::removePendingResultsFor(SkinnedMeshBody* body)
	{
		m_batchedCollisions.removePendingResultsFor(body);
	}

	bool CudaInterface::hasCollisionResults() const
	{
		return m_batchedCollisions.hasPendingResults();
	}

	bool CudaInterface::isCollisionBootstrapping() const
	{
		return m_batchedCollisions.isBootstrapping();
	}

	void CudaInterface::swapCollisionResultBuffers()
	{
		m_batchedCollisions.swapResultBuffers();
	}

	//==========================================================================
	// BATCHED INTERNAL UPDATE MANAGER IMPLEMENTATION
	//==========================================================================

	BatchedInternalUpdateManager::~BatchedInternalUpdateManager()
	{
		if (m_batchStream) {
			cuDestroyStream(m_batchStream);
			m_batchStream = nullptr;
		}
	}

	void BatchedInternalUpdateManager::ensureBatchStream()
	{
		int currentDevice = CudaInterface::currentDevice;

		// Create or recreate stream if needed (device changed or not yet created)
		if (!m_batchStream || m_batchStreamDevice != currentDevice) {
			if (m_batchStream) {
				cuDestroyStream(m_batchStream);
				m_batchStream = nullptr;
			}
			cuCreateStream(&m_batchStream).check(__FUNCTION__);
			m_batchStreamDevice = currentDevice;
		}
	}

	void BatchedInternalUpdateManager::beginBatch()
	{
		m_workUnits.clear();

		// Pre-allocate based on previous frame
		if (m_lastFrameBodyCount > 0) {
			m_workUnits.reserve(m_lastFrameBodyCount);
		}
	}

	void BatchedInternalUpdateManager::addBody(std::shared_ptr<CudaBody> body,
											   std::shared_ptr<CudaPerVertexShape> vertexShape,
											   std::shared_ptr<CudaPerTriangleShape> triangleShape)
	{
		if (!body) {
			return;
		}

		m_workUnits.push_back({body, vertexShape, triangleShape});
	}

	void BatchedInternalUpdateManager::launchBatch()
	{
		HDT_ZONE_SCOPED_N("BatchedInternalUpdates");

		if (m_workUnits.empty()) {
			return;
		}

		// CRITICAL FIX: Ensure we have a batch stream created on the MAIN thread.
		// Using per-body streams (created during parallel_for_each on worker threads)
		// causes CUDA driver crashes due to context/thread affinity issues.
		// The original per-body graph launches worked because:
		// 1. Graph capture happened on the worker thread that created the stream
		// 2. cuGraphLaunch is designed to be thread-safe
		// Direct kernel launches don't have the same thread-safety guarantees when
		// using streams created by other threads.
		ensureBatchStream();

		// Record for next frame pre-allocation
		m_lastFrameBodyCount = m_workUnits.size();
		HDT_ZONE_VALUE(static_cast<int64_t>(m_workUnits.size()));

		// Setup GPU timing if enabled
		const bool timing = CudaInterface::gpuTimingEnabled;
		if (timing) {
			s_internalTiming.ensure();
			cuRecordEvent(s_internalTiming.startEvent, m_batchStream);
		}

		// Empty collider data for bodies without shapes
		static const cuColliderData<CudaPerVertexShape> s_emptyVertexData = {
			VertexInputArray(nullptr, 0), BoundingBoxArray(nullptr, 0), 0, {0}};
		static const cuColliderData<CudaPerTriangleShape> s_emptyTriangleData = {
			TriangleInputArray(nullptr, 0), BoundingBoxArray(nullptr, 0), 0, {0, 0}};

		// Phase 1: Upload all bone data to device using the batch stream
		// All uploads are queued on the same stream, ensuring proper ordering
		{
			HDT_ZONE_SCOPED_N("BatchedBonesToDevice");
			for (auto& work : m_workUnits) {
				if (!work.body || !work.body->m_imp) {
					continue; // Skip invalid work units
				}
				auto& imp = *work.body->m_imp;
				// Use batch stream instead of per-body stream to avoid thread-affinity issues
				imp.m_bones.toDevice(m_batchStream);
			}
		}

		// Record after bone transfers
		if (timing) {
			cuRecordEvent(s_internalTiming.afterBonesEvent, m_batchStream);
		}

		// Phase 2: Launch all internal update kernels on the batch stream
		// Using a single stream ensures all operations are properly ordered and
		// avoids the thread-affinity issues with per-body streams.
		{
			HDT_ZONE_SCOPED_N("BatchedKernelLaunches");
			for (auto& work : m_workUnits) {
				if (!work.body || !work.body->m_imp) {
					continue; // Skip invalid work units
				}
				auto& imp = *work.body->m_imp;

				// Validate shape m_imp pointers before use
				bool hasValidVertexShape = work.vertexShape && work.vertexShape->m_imp;
				bool hasValidTriangleShape = work.triangleShape && work.triangleShape->m_imp;

				cuInternalUpdate(
					m_batchStream, imp, imp.m_bones.getD(),
					hasValidVertexShape ? static_cast<cuColliderData<CudaPerVertexShape>>(*work.vertexShape->m_imp)
										: s_emptyVertexData,
					hasValidVertexShape ? work.vertexShape->m_imp->m_tree.m_numNodes : 0,
					hasValidVertexShape ? work.vertexShape->m_imp->m_tree.m_nodeData.getD() : nullptr,
					hasValidVertexShape ? work.vertexShape->m_imp->m_tree.getCurrentWriteBuffer() : nullptr,
					hasValidTriangleShape
						? static_cast<cuColliderData<CudaPerTriangleShape>>(*work.triangleShape->m_imp)
						: s_emptyTriangleData,
					hasValidTriangleShape ? work.triangleShape->m_imp->m_tree.m_numNodes : 0,
					hasValidTriangleShape ? work.triangleShape->m_imp->m_tree.m_nodeData.getD() : nullptr,
					hasValidTriangleShape ? work.triangleShape->m_imp->m_tree.getCurrentWriteBuffer() : nullptr)
					.check(__FUNCTION__);
			}
		}

		// Record after kernel launches
		if (timing) {
			cuRecordEvent(s_internalTiming.afterKernelsEvent, m_batchStream);
			s_internalTiming.pending = true;
		}

		// NOTE: No sync here - caller will sync when needed via CudaInterface::synchronize()
	}

	void BatchedInternalUpdateManager::queueLeafDownloads()
	{
		// No-op: Zero-copy makes explicit D2H copies unnecessary.
		// GPU writes via getZ() go directly to pinned host memory that get() reads.
		// Keeping this function for API compatibility but it does nothing.
	}

	void BatchedInternalUpdateManager::swapAllBuffers()
	{
		HDT_ZONE_SCOPED_N("SwapAllBuffers");
		for (auto& work : m_workUnits) {
			if (work.vertexShape && work.vertexShape->m_imp) {
				work.vertexShape->swapBuffers();
			}
			if (work.triangleShape && work.triangleShape->m_imp) {
				work.triangleShape->swapBuffers();
			}
		}
	}

	bool BatchedInternalUpdateManager::hasFirstFrame() const
	{
		for (const auto& work : m_workUnits) {
			if (work.vertexShape && work.vertexShape->m_imp && work.vertexShape->isFirstFrame()) {
				return true;
			}
			if (work.triangleShape && work.triangleShape->m_imp && work.triangleShape->isFirstFrame()) {
				return true;
			}
		}
		return false;
	}

	void BatchedInternalUpdateManager::clearAllFirstFrames()
	{
		for (auto& work : m_workUnits) {
			if (work.vertexShape && work.vertexShape->m_imp) {
				work.vertexShape->clearFirstFrame();
			}
			if (work.triangleShape && work.triangleShape->m_imp) {
				work.triangleShape->clearFirstFrame();
			}
		}
	}

	//==========================================================================
	// CUDAINTERFACE BATCHED INTERNAL UPDATE API WRAPPERS
	//==========================================================================

	void CudaInterface::beginInternalUpdateBatch()
	{
		m_batchedInternalUpdates.beginBatch();
	}

	void CudaInterface::addInternalUpdate(std::shared_ptr<CudaBody> body,
										  std::shared_ptr<CudaPerVertexShape> vertexShape,
										  std::shared_ptr<CudaPerTriangleShape> triangleShape)
	{
		m_batchedInternalUpdates.addBody(body, vertexShape, triangleShape);
	}

	void CudaInterface::launchInternalUpdateBatch()
	{
		m_batchedInternalUpdates.launchBatch();
	}

	void CudaInterface::queueLeafDownloads()
	{
		m_batchedInternalUpdates.queueLeafDownloads();
	}

	void CudaInterface::swapAllBuffers()
	{
		m_batchedInternalUpdates.swapAllBuffers();
	}

	bool CudaInterface::hasFirstFrame() const
	{
		return m_batchedInternalUpdates.hasFirstFrame();
	}

	void CudaInterface::clearAllFirstFrames()
	{
		m_batchedInternalUpdates.clearAllFirstFrames();
	}

	void* CudaInterface::getBatchStream() const
	{
		return m_batchedInternalUpdates.getBatchStream();
	}
} // namespace hdt
#endif
