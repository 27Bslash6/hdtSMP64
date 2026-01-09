#pragma once
#ifdef CUDA

#include "hdtDispatcher.h"
#include "hdtSkinnedMeshShape.h"

#include <atomic>
#include <mutex>
#include <string>
#include <variant>

namespace hdt
{
	class CudaBody
	{
		friend class CudaPerTriangleShape;
		friend class CudaPerVertexShape;
		friend class CudaInterface;
		friend class CudaMergeBuffer;

	public:
		CudaBody(SkinnedMeshBody* body);
		void synchronize();
		int deviceId();

	private:
		class Imp;
		std::shared_ptr<Imp> m_imp;
	};

	class CudaPerTriangleShape
	{
		template<typename T>
		friend class CudaCollisionPair;
		friend class CudaInterface;

	public:
		class Imp;

		CudaPerTriangleShape(PerTriangleShape* shape);
		void updateTree();
		int deviceId();

	private:
		std::shared_ptr<Imp> m_imp;
	};

	class CudaPerVertexShape
	{
		template<typename T>
		friend class CudaCollisionPair;
		friend class CudaInterface;

	public:
		class Imp;

		CudaPerVertexShape(PerVertexShape* shape);
		void updateTree();
		int deviceId();

	private:
		std::shared_ptr<Imp> m_imp;
	};

	class CudaMergeBuffer
	{
		template<typename T>
		friend class CudaCollisionPair;

	public:
		class Imp;

		CudaMergeBuffer(SkinnedMeshBody* body0, SkinnedMeshBody* body1);

		void launchTransfer();

		void apply(SkinnedMeshBody* body0, SkinnedMeshBody* body1, CollisionDispatcher* dispatcher);

	private:
		std::shared_ptr<Imp> m_imp;
	};

	template<typename T>
	class CudaCollisionPair
	{
	public:
		CudaCollisionPair(CudaPerVertexShape* shapeA, T* shapeB, int numCollisionPairs);

		void addPair(int offsetA, int offsetB, int sizeA, int sizeB, const Aabb& aabbA, const Aabb& aabbB);

		void launch(CudaMergeBuffer* merge, bool swap);

		int numPairs();

	private:
		class Imp;
		std::shared_ptr<Imp> m_imp;
	};

	//==========================================================================
	// BATCHED COLLISION SYSTEM
	// Reduces ~925K kernel launches to 2-3 per frame
	//==========================================================================

	// Hard limit to prevent resource exhaustion (DoS protection)
	static constexpr size_t CUDA_MAX_COLLISION_PAIRS = 2'000'000;

	// Info about a collision pair for result routing
	struct CollisionPairInfo
	{
		// Shape references for routing results back
		CudaPerVertexShape* shapeA = nullptr;
		std::variant<CudaPerVertexShape*, CudaPerTriangleShape*> shapeB;

		// Body raw pointers for result application
		SkinnedMeshBody* body0 = nullptr;
		SkinnedMeshBody* body1 = nullptr;

		// CUDA object weak refs - if these are invalid, body was destroyed
		std::weak_ptr<CudaBody> cudaBody0;
		std::weak_ptr<CudaBody> cudaBody1;

		// Offset into global merge buffer for results
		size_t mergeBufferOffset = 0;

		// Swap flag for VT pairs where triangle is on body0
		bool swapped = false;

		// Check if bodies are still valid
		bool bodiesValid() const { return cudaBody0.lock() && cudaBody1.lock(); }
	};

	// CPU-side batch pair metadata (no CUDA types exposed in header)
	struct CpuBatchPairs
	{
		std::vector<CollisionPairInfo> pairsVV;
		std::vector<CollisionPairInfo> pairsVT;

		void clear()
		{
			pairsVV.clear();
			pairsVT.clear();
		}

		void reserve(size_t expectedPairs)
		{
			pairsVV.reserve(expectedPairs / 2);
			pairsVT.reserve(expectedPairs / 2);
		}

		size_t totalPairs() const { return pairsVV.size() + pairsVT.size(); }
	};

	// Pending collision result for deferred application
	struct PendingCollisionResult
	{
		std::shared_ptr<CudaMergeBuffer> mergeBuffer;
		SkinnedMeshBody* body0;
		SkinnedMeshBody* body1;
		std::weak_ptr<CudaBody> cudaBody0;
		std::weak_ptr<CudaBody> cudaBody1;

		bool bodiesValid() const { return cudaBody0.lock() && cudaBody1.lock(); }
	};

	// Manages batched collision detection
	class BatchedCollisionManager
	{
	public:
		BatchedCollisionManager() = default;

		// Begin a new batch (clears previous data)
		void beginBatch();

		// Add a collision pair to the batch
		// Returns false if limit reached or invalid input
		bool addCollisionPair(SkinnedMeshBody* body0, SkinnedMeshBody* body1);

		// Merge thread-local batches into global batch
		void mergeThreadLocalBatch();

		// Launch all batched kernels
		void launchBatch();

		// Apply results from previous frame
		void applyResults(CollisionDispatcher* dispatcher);

		// Check if there are pending results to apply
		bool hasPendingResults() const { return m_hasPendingResults; }

		// Get total pairs in current batch
		size_t totalPairs() const { return m_pairs.totalPairs(); }

		// Access to pair metadata (for diagnostics)
		const CpuBatchPairs& pairs() const { return m_pairs; }

	private:
		// Accumulate a VV pair into thread-local batch
		void accumulateVV(SkinnedMeshBody* body0, SkinnedMeshBody* body1, bool swapped);

		// Accumulate a VT pair into thread-local batch
		void accumulateVT(SkinnedMeshBody* body0, SkinnedMeshBody* body1, bool swapped);

		// Launch collision for a single pair (called from launchBatch)
		template<bool Swap, typename T>
		void launchSingleCollision(PerVertexShape* vertexShape, T* otherShape,
								   std::shared_ptr<CudaMergeBuffer> mergeBuffer);

		CpuBatchPairs m_pairs;

		// Mutex for thread-safe batch accumulation
		std::mutex m_mergeMutex;

		// Atomic counter for pair limit enforcement
		std::atomic<size_t> m_totalPairs{0};

		// Flag indicating results are ready to apply
		bool m_hasPendingResults = false;

		// Last frame's pair count for pre-allocation
		size_t m_lastFramePairCount = 0;

		// Pending results from launched collisions (for deferred apply)
		std::vector<PendingCollisionResult> m_pendingResults;
	};

	// Metrics for diagnosing CUDA performance variance
	struct CudaGraphMetrics
	{
		// Ring buffer for recent samples (power of 2 for fast modulo)
		static constexpr int kSampleCount = 256;

		// CPU-side timing (wall clock for cudaGraphLaunch call)
		float cpuEnqueueUs[kSampleCount] = {};

		// GPU-side timing (CUDA events around graph execution)
		float gpuExecuteUs[kSampleCount] = {};

		int sampleIndex = 0;
		int totalSamples = 0;

		void addSample(float cpuUs, float gpuUs)
		{
			cpuEnqueueUs[sampleIndex] = cpuUs;
			gpuExecuteUs[sampleIndex] = gpuUs;
			sampleIndex = (sampleIndex + 1) & (kSampleCount - 1);
			totalSamples++;
		}

		// Calculate percentile (0-100) from ring buffer
		float percentile(const float* data, int p) const;

		// Get statistics string for logging
		std::string report() const;
	};

	class CudaInterface
	{
		struct CudaBuffers;

	public:
		static bool enableCuda;
		static int currentDevice;
		static bool collectMetrics; // Toggle via smp metrics command

		static CudaInterface* instance();

		bool hasCuda();

		void synchronize();

		void clearBufferPool();

		int deviceCount();

		void setCurrentDevice();

		// Access metrics for reporting
		static CudaGraphMetrics& graphMetrics();
		static void resetMetrics();

		static void launchInternalUpdate(std::shared_ptr<CudaBody> body,
										 std::shared_ptr<CudaPerVertexShape> vertexShape,
										 std::shared_ptr<CudaPerTriangleShape> triangleShape);

		//======================================================================
		// BATCHED COLLISION API
		// Use these instead of per-pair queueCollision for massive speedup
		//======================================================================

		// Begin collecting collision pairs for batched processing
		void beginCollisionBatch();

		// Add a collision pair to the current batch
		// Returns false if limit reached or invalid input
		bool addCollisionPair(SkinnedMeshBody* body0, SkinnedMeshBody* body1);

		// Merge thread-local batches (call after parallel gather completes)
		void mergeCollisionBatches();

		// Launch all batched collision kernels
		void launchCollisionBatch();

		// Apply collision results from previous frame
		void applyCollisionResults(CollisionDispatcher* dispatcher);

		// Check if there are pending results to apply
		bool hasCollisionResults() const;

		// Access batched collision manager (for diagnostics)
		BatchedCollisionManager& batchedCollisions() { return m_batchedCollisions; }

	private:
		CudaInterface();
		bool m_enabled;
		BatchedCollisionManager m_batchedCollisions;
	};
} // namespace hdt
#endif
