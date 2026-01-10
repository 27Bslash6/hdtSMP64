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
		friend class BatchedInternalUpdateManager;

	public:
		CudaBody(SkinnedMeshBody* body);
		void synchronize();
		int deviceId();

		// Validity tracking - set false when owner SkinnedMeshBody is about to be destroyed.
		// This allows collision results to detect stale body pointers.
		void invalidate() { m_valid.store(false, std::memory_order_release); }
		bool isValid() const { return m_valid.load(std::memory_order_acquire); }

	private:
		class Imp;
		std::shared_ptr<Imp> m_imp;
		std::atomic<bool> m_valid{true};
	};

	class CudaPerTriangleShape
	{
		template<typename T>
		friend class CudaCollisionPair;
		friend class CudaInterface;
		friend class BatchedInternalUpdateManager;

	public:
		class Imp;

		CudaPerTriangleShape(PerTriangleShape* shape);
		void updateTree();
		int deviceId();

		// Double-buffer API for async AABB pipeline
		void queueLeafDownload(void* stream);
		void swapBuffers();
		bool isFirstFrame() const;
		void clearFirstFrame();

	private:
		std::shared_ptr<Imp> m_imp;
	};

	class CudaPerVertexShape
	{
		template<typename T>
		friend class CudaCollisionPair;
		friend class CudaInterface;
		friend class BatchedInternalUpdateManager;

	public:
		class Imp;

		CudaPerVertexShape(PerVertexShape* shape);
		void updateTree();
		int deviceId();

		// Double-buffer API for async AABB pipeline
		void queueLeafDownload(void* stream);
		void swapBuffers();
		bool isFirstFrame() const;
		void clearFirstFrame();

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

		// Apply collision results to physics manifolds
		// Takes locked CudaBody shared_ptrs directly to avoid TOCTOU race
		// (the weak_ptrs are locked once in applyResults, then passed here)
		void apply(SkinnedMeshBody* body0, SkinnedMeshBody* body1, std::shared_ptr<CudaBody> cuda0,
				   std::shared_ptr<CudaBody> cuda1, CollisionDispatcher* dispatcher);

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

	// Manages batched collision detection with 1-frame latency pipeline
	// Frame N: GPU computes collisions -> writes to buffer[writeIdx], stores bone transforms
	// Frame N+1: Sync at frame start, apply results from buffer[readIdx] using stored transforms
	// Stored transforms ensure consistent local coordinate conversion despite latency
	class BatchedCollisionManager
	{
	public:
		BatchedCollisionManager();
		~BatchedCollisionManager();

		// Begin a new batch (clears previous data)
		void beginBatch();

		// Add a collision pair to the batch
		// Returns false if limit reached or invalid input
		bool addCollisionPair(SkinnedMeshBody* body0, SkinnedMeshBody* body1);

		// Merge thread-local batches into global batch
		void mergeThreadLocalBatch();

		// Launch all batched kernels
		void launchBatch();

		// Apply results from previous frame (after sync at frame start)
		void applyResults(CollisionDispatcher* dispatcher);

		// Swap result buffers at end of frame (after launchBatch)
		void swapResultBuffers();

		// Remove all pending results referencing this body (called from destructor)
		void removePendingResultsFor(SkinnedMeshBody* body);

		// Check if there are pending results to apply (from previous frame, GPU complete after sync)
		bool hasPendingResults() const { return !m_pendingResults[readIndex()].empty(); }

		// Check if we're in bootstrap phase (first frame, no results yet)
		// During bootstrap, skip apply. After bootstrap, results are 1 frame old.
		bool isBootstrapping() const { return m_frameCount < 1; }

		// Sync if GPU work on read buffer isn't complete yet
		// Returns true if we had to sync, false if GPU was already done
		bool syncIfNeeded();

		// Get total pairs in current batch
		size_t totalPairs() const { return m_pairs.totalPairs(); }

		// Access to pair metadata (for diagnostics)
		const CpuBatchPairs& pairs() const { return m_pairs; }

		// Get current frame count (for diagnostics)
		int frameCount() const { return m_frameCount; }

	private:
		// Buffer index helpers for 1-frame latency with 3 buffers
		// Frame N: write to buffer N%3, read from buffer (N+2)%3 (which is N-1 in time)
		// 1-frame latency requires sync but avoids physics instability from stale data
		int writeIndex() const { return m_frameCount % 3; }
		int readIndex() const { return (m_frameCount + 2) % 3; } // 1 frame behind write

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

		// Mutex for pending results access - protects m_pendingResults during
		// apply and body removal to prevent dangling pointer access
		std::mutex m_resultsMutex;

		// Atomic counter for pair limit enforcement
		std::atomic<size_t> m_totalPairs{0};

		// Last frame's pair count for pre-allocation
		size_t m_lastFramePairCount = 0;

		//======================================================================
		// 1-FRAME LATENCY TRIPLE BUFFER WITH STORED TRANSFORMS
		// 3 buffers for clean separation:
		//   - Buffer (N % 3): being written by GPU this frame
		//   - Buffer ((N+2) % 3): previous frame's data, read after sync
		//   - Buffer ((N+1) % 3): 2 frames old, already processed, can be reused
		//
		// Bone transforms are stored in CudaMergeBuffer at collision time.
		// When applying results 1 frame later, we use stored transforms
		// for local coordinate conversion (not current transforms).
		// Sync at frame start ensures GPU is done before reading.
		//======================================================================
		std::vector<PendingCollisionResult> m_pendingResults[3];
		void* m_completionEvents[3]; // cudaEvent_t (reserved for future optimization)
		int m_frameCount = 0;		 // Frames since start (for buffer index)
		bool m_eventsInitialized = false;
	};

	//==========================================================================
	// BATCHED INTERNAL UPDATE SYSTEM
	// Reduces ~117 graph launches per frame to ~5 direct kernel launches
	//==========================================================================

	// Work unit for batched internal update - one per body
	struct InternalUpdateWork
	{
		// Body CUDA object (has GPU pointers for vertex data, bones, etc.)
		std::shared_ptr<CudaBody> body;

		// Shape CUDA objects (may be null)
		std::shared_ptr<CudaPerVertexShape> vertexShape;
		std::shared_ptr<CudaPerTriangleShape> triangleShape;
	};

	// Manages batched internal updates (bone transforms + vertex skinning)
	class BatchedInternalUpdateManager
	{
	public:
		BatchedInternalUpdateManager() = default;
		~BatchedInternalUpdateManager();

		// Begin a new batch (clears previous data)
		void beginBatch();

		// Add a body to the batch for internal update
		void addBody(std::shared_ptr<CudaBody> body, std::shared_ptr<CudaPerVertexShape> vertexShape,
					 std::shared_ptr<CudaPerTriangleShape> triangleShape);

		// Upload all bone data and launch batched kernels
		void launchBatch();

		// Queue async leaf AABB downloads for all shapes in the batch
		void queueLeafDownloads();

		// Swap double buffers for all shapes in the batch
		void swapAllBuffers();

		// Check if any shape in the batch is on its first frame
		bool hasFirstFrame() const;

		// Clear first-frame flag for all shapes
		void clearAllFirstFrames();

		// Get the batch stream for external use (sync, etc.)
		void* getBatchStream() const { return m_batchStream; }

		// Get number of bodies in current batch
		size_t bodyCount() const { return m_workUnits.size(); }

	private:
		// Ensure batch stream is created (lazy init on main thread)
		void ensureBatchStream();

		std::vector<InternalUpdateWork> m_workUnits;

		// Pre-allocation hint from previous frame
		size_t m_lastFrameBodyCount = 0;

		// CRITICAL FIX: Dedicated stream for batched operations
		// Created lazily on the main thread to avoid cross-thread stream usage.
		// Using per-body streams (created during parallel_for_each on worker threads)
		// from the main thread causes CUDA driver crashes due to context/thread affinity issues.
		void* m_batchStream = nullptr;
		int m_batchStreamDevice = -1;
	};

	// GPU timing stats for Nsight-style profiling without external tools
	// Measures actual GPU execution time via CUDA events
	struct GpuTimingStats
	{
		static constexpr int kSampleCount = 256;

		// Ring buffers for each GPU phase (milliseconds)
		float bonesToDeviceMs[kSampleCount] = {};
		float internalKernelsMs[kSampleCount] = {};
		float collisionKernelsMs[kSampleCount] = {};
		float syncWaitMs[kSampleCount] = {};

		int sampleIndex = 0;
		int totalSamples = 0;

		void addSample(float bones, float internal, float collision, float sync)
		{
			bonesToDeviceMs[sampleIndex] = bones;
			internalKernelsMs[sampleIndex] = internal;
			collisionKernelsMs[sampleIndex] = collision;
			syncWaitMs[sampleIndex] = sync;
			sampleIndex = (sampleIndex + 1) & (kSampleCount - 1);
			totalSamples++;
		}

		// Get mean of last N samples for a given metric
		float mean(const float* data, int n = 64) const
		{
			if (totalSamples == 0)
				return 0.0f;
			int count = std::min(n, std::min(totalSamples, kSampleCount));
			float sum = 0.0f;
			for (int i = 0; i < count; ++i) {
				int idx = (sampleIndex - 1 - i + kSampleCount) & (kSampleCount - 1);
				sum += data[idx];
			}
			return sum / count;
		}

		// Report string for console output
		std::string report() const;
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
		static GpuTimingStats& gpuTiming();
		static void resetMetrics();
		static bool gpuTimingEnabled; // Toggle via smp gputiming command

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

		// Apply collision results from previous frame (after sync)
		void applyCollisionResults(CollisionDispatcher* dispatcher);

		// Swap collision result buffers at end of frame
		void swapCollisionResultBuffers();

		// Remove pending collision results for a body being destroyed
		void removePendingResultsFor(SkinnedMeshBody* body);

		// Check if there are pending results to apply
		bool hasCollisionResults() const;

		// Check if collision pipeline is still bootstrapping (first frame)
		bool isCollisionBootstrapping() const;

		// Access batched collision manager (for diagnostics)
		BatchedCollisionManager& batchedCollisions() { return m_batchedCollisions; }

		//======================================================================
		// BATCHED INTERNAL UPDATE API
		// Replaces per-body graph launches with batched direct kernel launches
		//======================================================================

		// Begin collecting bodies for batched internal update
		void beginInternalUpdateBatch();

		// Add a body to the internal update batch
		void addInternalUpdate(std::shared_ptr<CudaBody> body, std::shared_ptr<CudaPerVertexShape> vertexShape,
							   std::shared_ptr<CudaPerTriangleShape> triangleShape);

		// Launch all batched internal update kernels
		void launchInternalUpdateBatch();

		//======================================================================
		// DOUBLE-BUFFER AABB PIPELINE API
		// Enables 1-frame latency for collision detection to eliminate sync
		//======================================================================

		// Queue async copy of leaf AABBs from device to pinned host memory
		void queueLeafDownloads();

		// Swap double buffers at frame end (makes current writes become previous reads)
		void swapAllBuffers();

		// Check if any shape is on its first frame (requires sync for bootstrap)
		bool hasFirstFrame() const;

		// Clear first-frame flags after bootstrap sync completes
		void clearAllFirstFrames();

		// Get the batch stream for external sync operations
		void* getBatchStream() const;

		// Access batched internal update manager (for diagnostics)
		BatchedInternalUpdateManager& batchedInternalUpdates() { return m_batchedInternalUpdates; }

	private:
		CudaInterface();
		bool m_enabled;
		BatchedCollisionManager m_batchedCollisions;
		BatchedInternalUpdateManager m_batchedInternalUpdates;
	};
} // namespace hdt
#endif
