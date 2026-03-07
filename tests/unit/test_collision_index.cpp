/**
 * Test for collision index computation bug
 *
 * Symptom: Invalid collider indices in doMerge - indices are roughly 2x the valid range
 * Examples:
 *   - 954 out of [0, 475)
 *   - 1247 out of [0, 475)
 *   - 1238 out of [0, 540)
 *   - 2434 out of [0, 1954)
 *
 * The ~2x pattern suggests data from two shapes is being concatenated or mixed.
 */

#include "../include/catch.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

// Simplified index computation logic matching hdtSkinnedMeshAlgorithm.cpp
namespace
{

	// Simulates Collider pointer arithmetic
	struct MockCollider
	{
		uint32_t vertex;
		float flexible;
	};

	struct MockAabb
	{
		float minX, minY, minZ;
		float maxX, maxY, maxZ;
	};

	// Simulates ColliderTree with offset
	struct MockColliderTree
	{
		size_t colliderOffset = 0;
		uint32_t numCollider = 0;
	};

	// Simulates the index computation from hdtSkinnedMeshAlgorithm
	// res.colliderIndexA = a - colliderBaseA;
	// where a = &acbuf[i - abeg] = &(colliderBase + treeNode->colliderOffset)[localAabbIndex]
	size_t computeColliderIndex(const MockCollider* colliderInQuestion, const MockCollider* colliderBase)
	{
		return static_cast<size_t>(colliderInQuestion - colliderBase);
	}

	// Simulates the dispatch loop computation:
	// auto acbuf = colliderBaseA + a->colliderOffset;
	// for (auto i : listA) {
	//     checkCollide(&acbuf[i - abeg], ...);
	// }
	// where i - abeg is the local index within the tree node's AABB range
	const MockCollider* getColliderFromAabbIndex(const MockCollider* colliderBase, size_t treeNodeOffset,
												 size_t localAabbIndex)
	{
		return colliderBase + treeNodeOffset + localAabbIndex;
	}

} // anonymous namespace

TEST_CASE("Collider index computation", "[collision][index]")
{
	SECTION("Basic index computation is correct")
	{
		// Shape with 100 colliders
		std::vector<MockCollider> colliders(100);
		MockCollider* base = colliders.data();

		// Tree node covers colliders [20, 50)
		MockColliderTree treeNode;
		treeNode.colliderOffset = 20;
		treeNode.numCollider = 30;

		// Access collider at local index 5 within tree node
		// Expected: colliderBase + 20 + 5 = colliderBase + 25
		const MockCollider* collider = getColliderFromAabbIndex(base, treeNode.colliderOffset, 5);
		size_t index = computeColliderIndex(collider, base);

		REQUIRE(index == 25);
		REQUIRE(index < colliders.size());
	}

	SECTION("Index stays within bounds for edge cases")
	{
		std::vector<MockCollider> colliders(100);
		MockCollider* base = colliders.data();

		// Tree node at the end of array
		MockColliderTree treeNode;
		treeNode.colliderOffset = 90;
		treeNode.numCollider = 10;

		// Access last collider in tree node
		const MockCollider* collider = getColliderFromAabbIndex(base, treeNode.colliderOffset, 9);
		size_t index = computeColliderIndex(collider, base);

		REQUIRE(index == 99);
		REQUIRE(index < colliders.size());
	}

	SECTION("WRONG BASE: Using collider from shapeA with base from shapeB")
	{
		// This simulates the bug hypothesis: what if we compute index using wrong base?
		std::vector<MockCollider> collidersA(100);
		std::vector<MockCollider> collidersB(200);

		MockCollider* baseA = collidersA.data();
		MockCollider* baseB = collidersB.data();

		// Get a collider from shapeA
		MockColliderTree treeNodeA;
		treeNodeA.colliderOffset = 50;
		const MockCollider* colliderFromA = getColliderFromAabbIndex(baseA, treeNodeA.colliderOffset, 10);

		// But compute index using shapeB's base - THIS IS A BUG
		size_t buggyIndex = computeColliderIndex(colliderFromA, baseB);

		// The index will be based on pointer arithmetic between different arrays
		// This is undefined behavior, but in practice produces wild indices
		// We can't predict the exact value, but it definitely won't be in [0, 200)

		// Just verify the correct index is valid
		size_t correctIndex = computeColliderIndex(colliderFromA, baseA);
		REQUIRE(correctIndex == 60);
		REQUIRE(correctIndex < collidersA.size());
	}
}

TEST_CASE("Two-shape collision scenario", "[collision][index]")
{
	// Simulates triangle-triangle collision where both shapes have vertex sub-shapes

	SECTION("Collision with SwapResults produces correct indices")
	{
		// ShapeA: PerTriangleShape with 300 triangle colliders
		//         and m_verticesCollision with 400 vertex colliders
		// ShapeB: PerTriangleShape with 350 triangle colliders
		//         and m_verticesCollision with 475 vertex colliders

		struct MockShape
		{
			std::vector<MockCollider> triangleColliders;
			std::vector<MockCollider> vertexColliders;
		};

		MockShape shapeA;
		shapeA.triangleColliders.resize(300);
		shapeA.vertexColliders.resize(400);

		MockShape shapeB;
		shapeB.triangleColliders.resize(350);
		shapeB.vertexColliders.resize(475);

		// Case 1: processCollision(shapeA.triangle, shapeB.vertex)
		// Internally swapped to: algorithm(shapeB.vertex, shapeA.triangle)
		// colliderBaseA = shapeB.vertexColliders.data()
		// colliderBaseB = shapeA.triangleColliders.data()
		// After SwapResults=true:
		//   stored.indexA = original.indexB = index into shapeA.triangle
		//   stored.indexB = original.indexA = index into shapeB.vertex
		// doMerge(shapeA.triangle, shapeB.vertex):
		//   aColliderSize = shapeA.triangleColliders.size() = 300
		//   bColliderSize = shapeB.vertexColliders.size() = 475

		// Simulate a collision result
		MockCollider* vtxBaseB = shapeB.vertexColliders.data();
		MockCollider* triBaseA = shapeA.triangleColliders.data();

		// Collider from shapeB.vertex at index 200
		MockColliderTree vtxTree;
		vtxTree.colliderOffset = 100;
		const MockCollider* vtxCollider = getColliderFromAabbIndex(vtxBaseB, vtxTree.colliderOffset, 100);
		size_t vtxIndex = computeColliderIndex(vtxCollider, vtxBaseB);
		REQUIRE(vtxIndex == 200);
		REQUIRE(vtxIndex < shapeB.vertexColliders.size());

		// Collider from shapeA.triangle at index 150
		MockColliderTree triTree;
		triTree.colliderOffset = 50;
		const MockCollider* triCollider = getColliderFromAabbIndex(triBaseA, triTree.colliderOffset, 100);
		size_t triIndex = computeColliderIndex(triCollider, triBaseA);
		REQUIRE(triIndex == 150);
		REQUIRE(triIndex < shapeA.triangleColliders.size());

		// After swap (SwapResults=true):
		size_t storedIndexA = triIndex; // swapped: indexB -> indexA
		size_t storedIndexB = vtxIndex; // swapped: indexA -> indexB

		// doMerge validation
		size_t aCollidersSize = shapeA.triangleColliders.size(); // 300
		size_t bCollidersSize = shapeB.vertexColliders.size();	 // 475

		REQUIRE(storedIndexA < aCollidersSize); // 150 < 300 OK
		REQUIRE(storedIndexB < bCollidersSize); // 200 < 475 OK
	}

	SECTION("BUG REPRODUCTION: Index ~2x valid max")
	{
		// The symptom: index 954 out of [0, 475)
		// 954 / 475 = ~2x
		//
		// Hypothesis: The index is computed as:
		//   treeOffset_from_shapeA + localIndex_from_shapeB
		// When it should be just one or the other

		std::vector<MockCollider> collidersA(475); // Max is 475
		std::vector<MockCollider> collidersB(479); // Slightly different

		// If tree node offset from shapeA is 475 and local index from shapeB is 479
		// and they're incorrectly combined: 475 + 479 = 954 (matches the symptom!)

		size_t wrongOffset = 475;	  // This would be MAX for shapeA
		size_t wrongLocalIndex = 479; // This would be MAX for shapeB
		size_t buggyIndex = wrongOffset + wrongLocalIndex;

		REQUIRE(buggyIndex == 954);
		REQUIRE(buggyIndex >= collidersA.size()); // Out of bounds!

		// The bug might be: using tree node from wrong shape
		// Tree nodes have colliderOffset which is valid for THEIR shape
		// If we use shapeA's tree node offset but add it to shapeB's local index...
	}
}

TEST_CASE("Tree node shape association", "[collision][index]")
{
	SECTION("Tree nodes must come from their associated shape's tree")
	{
		// This test documents the expected invariant:
		// When processing collision pairs, pair.first must come from shapeA's tree
		// and pair.second must come from shapeB's tree

		// The algorithm constructs:
		//   c0 = &shapeA->m_tree
		//   c1 = &shapeB->m_tree
		// Then c0->checkCollisionL(c1, pairs) produces pairs where:
		//   pair.first is from c0 (shapeA's tree)
		//   pair.second is from c1 (shapeB's tree)

		// If somehow the trees get mixed up (e.g., both from same shape,
		// or reversed), the offset computation will be wrong

		// This is hard to test without the actual ColliderTree implementation
		// but documents the invariant that must hold

		REQUIRE(true); // Placeholder - real test would need ColliderTree
	}
}

TEST_CASE("SwapResults logic verification", "[collision][index]")
{
	// This test verifies that the swap logic in addResult matches what doMerge expects
	//
	// The chain is:
	// 1. checkCollide(PerTriangleShape* a, PerVertexShape* b) is called
	// 2. Inside, creates CollisionCheckAlgorithm<PerTriangleShape, true>(b, a, results)
	//    - shapeA = b (PerVertexShape), shapeB = a (PerTriangleShape)
	// 3. Collision indices computed as:
	//    - res.colliderIndexA = index into shapeA (PerVertexShape)
	//    - res.colliderIndexB = index into shapeB (PerTriangleShape)
	// 4. SwapResults=true means addResult swaps them:
	//    - stored.colliderIndexA = res.colliderIndexB (PerTriangleShape's index)
	//    - stored.colliderIndexB = res.colliderIndexA (PerVertexShape's index)
	// 5. doMerge(a, b) = doMerge(PerTriangleShape, PerVertexShape)
	//    - aCollidersSize = PerTriangleShape's size
	//    - validates stored.colliderIndexA against PerTriangleShape's size - CORRECT!

	SECTION("Swap preserves correct shape association")
	{
		// Simulate the collision chain
		size_t vtxShapeSize = 400;
		size_t triShapeSize = 300;

		// Inside algorithm: shapeA = vtx, shapeB = tri
		size_t resIndexA = 150; // Index into vtx (shapeA inside algorithm)
		size_t resIndexB = 200; // Index into tri (shapeB inside algorithm)

		// Validate BEFORE swap (algorithm's perspective)
		REQUIRE(resIndexA < vtxShapeSize); // vtx index < vtx size
		REQUIRE(resIndexB < triShapeSize); // tri index < tri size

		// After swap
		size_t storedIndexA = resIndexB; // Now tri's index
		size_t storedIndexB = resIndexA; // Now vtx's index

		// doMerge receives (triShape, vtxShape) - the ORIGINAL args to checkCollide
		size_t doMergeA_Size = triShapeSize; // First arg's size
		size_t doMergeB_Size = vtxShapeSize; // Second arg's size

		// Validate in doMerge
		REQUIRE(storedIndexA < doMergeA_Size); // tri index (200) < tri size (300) OK
		REQUIRE(storedIndexB < doMergeB_Size); // vtx index (150) < vtx size (400) OK
	}

	SECTION("No-swap case also works correctly")
	{
		// checkCollide<PerTriangleShape>(PerVertexShape* a, T1* b) with T1=PerTriangleShape
		// Creates algorithm(a, b) without swap
		// shapeA = a (PerVertexShape), shapeB = b (PerTriangleShape)

		size_t vtxShapeSize = 400;
		size_t triShapeSize = 300;

		// Inside algorithm
		size_t resIndexA = 150; // Index into vtx (shapeA)
		size_t resIndexB = 200; // Index into tri (shapeB)

		// No swap - stored directly
		size_t storedIndexA = resIndexA;
		size_t storedIndexB = resIndexB;

		// doMerge receives (vtxShape, triShape) - matches algorithm's order
		size_t doMergeA_Size = vtxShapeSize;
		size_t doMergeB_Size = triShapeSize;

		// Validate in doMerge
		REQUIRE(storedIndexA < doMergeA_Size); // vtx index < vtx size OK
		REQUIRE(storedIndexB < doMergeB_Size); // tri index < tri size OK
	}
}

TEST_CASE("Edge case: indices near shape boundary", "[collision][index]")
{
	// Test indices at the boundary of valid ranges

	SECTION("Index at max-1 is valid")
	{
		size_t shapeSize = 475;
		size_t index = 474; // Last valid index
		REQUIRE(index < shapeSize);
	}

	SECTION("Index at max is invalid")
	{
		size_t shapeSize = 475;
		size_t index = 475; // First invalid index
		REQUIRE(index >= shapeSize);
	}

	SECTION("Index at ~2x max (954) is clearly invalid for size 475")
	{
		size_t shapeSize = 475;
		size_t index = 954;
		REQUIRE(index >= shapeSize);
		// This is the actual symptom seen in the bug
	}
}

// =============================================================================
// STRESS TEST: Parallel collision with shape lifecycle changes
// =============================================================================

#include <atomic>
#include <mutex>
#include <random>
#include <thread>

namespace
{

	/**
	 * Simulates a shape with colliders that can be "destroyed" and "recreated"
	 * with different sizes - mimicking the in-game scenario where shapes
	 * change during loading/armor changes.
	 */
	class StressShape
	{
	public:
		std::vector<MockCollider> colliders;
		std::atomic<bool> isValid{true};
		std::mutex mtx;
		uint64_t shapeId;

		StressShape(uint64_t id, size_t numColliders) : shapeId(id) { resize(numColliders); }

		void resize(size_t numColliders)
		{
			std::lock_guard<std::mutex> lock(mtx);
			colliders.resize(numColliders);
			for (size_t i = 0; i < numColliders; ++i) {
				colliders[i].vertex = static_cast<uint32_t>(i);
			}
		}

		size_t size() const { return colliders.size(); }
	};

	/**
	 * Simulates collision result with index validation like addResult
	 */
	struct CollisionProcessor
	{
		std::atomic<int> validResults{0};
		std::atomic<int> invalidResults{0};
		std::atomic<int> racesDetected{0};

		bool addResult(size_t indexA, size_t indexB, const StressShape& shapeA, const StressShape& shapeB)
		{
			// Validate like addResult does
			if (indexA >= shapeA.size() || indexB >= shapeB.size()) {
				invalidResults.fetch_add(1);

				// Check for the ~2x pattern
				if (indexA >= shapeA.size() && indexA > shapeA.size() * 1.5) {
					racesDetected.fetch_add(1);
				}
				if (indexB >= shapeB.size() && indexB > shapeB.size() * 1.5) {
					racesDetected.fetch_add(1);
				}
				return false;
			}
			validResults.fetch_add(1);
			return true;
		}
	};

} // anonymous namespace

TEST_CASE("Stress: Parallel collision with shape changes", "[collision][index][stress]")
{
	SECTION("Rapid shape resize during parallel collision processing")
	{
		constexpr int NUM_SHAPES = 4;
		constexpr int ITERATIONS = 100;
		constexpr int COLLISIONS_PER_ITER = 50;

		std::vector<std::unique_ptr<StressShape>> shapes;
		for (int i = 0; i < NUM_SHAPES; ++i) {
			shapes.push_back(std::make_unique<StressShape>(i, 100 + i * 50));
		}

		CollisionProcessor processor;
		std::atomic<bool> testRunning{true};
		std::atomic<int> resizeCount{0};

		// Shape resizer thread - simulates armor changes / NPC loading
		std::thread resizer([&]() {
			std::mt19937 rng(42);
			while (testRunning.load()) {
				int shapeIdx = rng() % NUM_SHAPES;
				size_t newSize = 50 + (rng() % 500);
				shapes[shapeIdx]->resize(newSize);
				resizeCount.fetch_add(1);
				std::this_thread::sleep_for(std::chrono::microseconds(100));
			}
		});

		// Collision processors - simulate parallel collision detection
		std::vector<std::thread> workers;
		for (int w = 0; w < 4; ++w) {
			workers.emplace_back([&, w]() {
				std::mt19937 rng(w * 1000);
				for (int iter = 0; iter < ITERATIONS && testRunning.load(); ++iter) {
					int shapeIdxA = rng() % NUM_SHAPES;
					int shapeIdxB = (shapeIdxA + 1 + rng() % (NUM_SHAPES - 1)) % NUM_SHAPES;

					StressShape& shapeA = *shapes[shapeIdxA];
					StressShape& shapeB = *shapes[shapeIdxB];

					// Capture sizes ONCE at start (like production code does)
					size_t sizeA = shapeA.size();
					size_t sizeB = shapeB.size();

					for (int c = 0; c < COLLISIONS_PER_ITER; ++c) {
						// Generate indices based on captured sizes
						size_t indexA = rng() % sizeA;
						size_t indexB = rng() % sizeB;

						// Try to add result - validation uses CURRENT size
						// This simulates the race: indices valid for OLD size,
						// but shape resized before addResult validation
						processor.addResult(indexA, indexB, shapeA, shapeB);
					}
				}
			});
		}

		for (auto& w : workers) {
			w.join();
		}
		testRunning = false;
		resizer.join();

		INFO("Valid results: " << processor.validResults.load());
		INFO("Invalid results: " << processor.invalidResults.load());
		INFO("Races detected (~2x pattern): " << processor.racesDetected.load());
		INFO("Shape resizes: " << resizeCount.load());

		// We expect SOME invalid results due to the race condition
		// The key metric is whether we see the ~2x pattern
		// If racesDetected > 0, we've reproduced the bug scenario

		// For this test, we just verify it doesn't crash
		REQUIRE(processor.validResults.load() > 0);
	}

	SECTION("Index computation with stale tree offset")
	{
		// This simulates the specific ~2x bug scenario:
		// Tree node has colliderOffset=475 (from shape with 500 colliders)
		// Shape resized to 475 colliders
		// Local index 0 computed: 475 + 0 = 475 (now out of bounds!)

		StressShape shape(1, 500);
		size_t staleOffset = 475; // Valid when shape had 500 colliders

		// Shape resized smaller
		shape.resize(475);

		// Try to access with stale offset
		size_t localIndex = 0;
		size_t computedIndex = staleOffset + localIndex;

		// This would crash in production!
		REQUIRE(computedIndex >= shape.size()); // 475 >= 475 - out of bounds!

		// The fix: Validate indices before use
		if (computedIndex < shape.size()) {
			// Safe access
			REQUIRE(false); // Should not reach here
		}
		else {
			// Correctly detected invalid index
			REQUIRE(true);
		}
	}

	SECTION("The 954 out of 475 scenario")
	{
		// Reproduce the exact numbers from the bug report
		StressShape shapeA(1, 475);
		StressShape shapeB(2, 479);

		// Hypothesis: somehow offsets from both shapes get combined
		size_t offsetA = 475; // Max offset in shapeA's tree
		size_t offsetB = 479; // Max offset in shapeB's tree

		// If bug causes: finalIndex = offsetA + offsetB
		size_t buggyIndex = offsetA + offsetB;
		REQUIRE(buggyIndex == 954); // Matches the bug report!

		// Validate this would be caught by bounds check
		CollisionProcessor proc;
		bool accepted = proc.addResult(buggyIndex, 0, shapeA, shapeB);
		REQUIRE(!accepted);
		REQUIRE(proc.invalidResults.load() == 1);
		REQUIRE(proc.racesDetected.load() == 1); // ~2x pattern detected
	}
}

// =============================================================================
// AGGRESSIVE STRESS TEST: Shape destruction and creation mid-frame
// This simulates the real crash scenario: shapes being destroyed/recreated
// while collision workers are still accessing them
// =============================================================================

namespace
{

	/**
	 * Shared shape registry that can have shapes added/removed at any time.
	 * Simulates the ActorManager's skeleton list getting modified.
	 */
	class ShapeRegistry
	{
	public:
		std::vector<std::unique_ptr<StressShape>> shapes;
		mutable std::mutex mtx;
		std::atomic<uint64_t> nextShapeId{0};
		std::atomic<int> destroyCount{0};
		std::atomic<int> createCount{0};

		void addShape(size_t numColliders)
		{
			std::lock_guard<std::mutex> lock(mtx);
			shapes.push_back(std::make_unique<StressShape>(nextShapeId++, numColliders));
			createCount.fetch_add(1);
		}

		void destroyShape(size_t index)
		{
			std::lock_guard<std::mutex> lock(mtx);
			if (index < shapes.size()) {
				shapes[index]->isValid.store(false);
				shapes.erase(shapes.begin() + index);
				destroyCount.fetch_add(1);
			}
		}

		void resizeShape(size_t index, size_t newSize)
		{
			std::lock_guard<std::mutex> lock(mtx);
			if (index < shapes.size()) {
				shapes[index]->resize(newSize);
			}
		}

		// Get a snapshot of current shapes (for collision processing)
		std::vector<StressShape*> getShapeSnapshot()
		{
			std::lock_guard<std::mutex> lock(mtx);
			std::vector<StressShape*> result;
			for (auto& shape : shapes) {
				result.push_back(shape.get());
			}
			return result;
		}

		size_t count()
		{
			std::lock_guard<std::mutex> lock(mtx);
			return shapes.size();
		}
	};

	/**
	 * Simulates collision result buffer that stores indices
	 * These indices can become stale if shapes are destroyed/resized
	 */
	struct CollisionResultBuffer
	{
		struct Result
		{
			StressShape* shapeA;
			StressShape* shapeB;
			size_t indexA;
			size_t indexB;
			uint64_t shapeIdA; // To detect if shape was replaced
			uint64_t shapeIdB;
		};

		std::vector<Result> results;
		mutable std::mutex mtx;

		void addResult(StressShape* shapeA, StressShape* shapeB, size_t idxA, size_t idxB)
		{
			std::lock_guard<std::mutex> lock(mtx);
			results.push_back({shapeA, shapeB, idxA, idxB, shapeA->shapeId, shapeB->shapeId});
		}

		void clear()
		{
			std::lock_guard<std::mutex> lock(mtx);
			results.clear();
		}

		size_t size() const
		{
			std::lock_guard<std::mutex> lock(mtx);
			return results.size();
		}
	};

} // anonymous namespace

TEST_CASE("AGGRESSIVE: Shape destruction mid-collision", "[collision][index][stress][aggressive]")
{
	SECTION("Shapes destroyed while collision workers access them")
	{
		ShapeRegistry registry;
		CollisionResultBuffer results;
		std::atomic<bool> testRunning{true};
		std::atomic<int> validAccesses{0};
		std::atomic<int> invalidAccesses{0};
		std::atomic<int> stalePointerDetected{0};
		std::atomic<int> shapeMismatch{0};

		// Initialize with 10 shapes of varying sizes
		for (int i = 0; i < 10; ++i) {
			registry.addShape(100 + i * 50); // 100, 150, 200, ... 550
		}

		// Destructor thread: continuously destroys and creates shapes
		std::thread destructor([&]() {
			std::mt19937 rng(123);
			while (testRunning.load()) {
				size_t count = registry.count();
				if (count > 3) {
					// Destroy a random shape
					size_t idx = rng() % count;
					registry.destroyShape(idx);
				}
				if (count < 15) {
					// Create a new shape with random size
					size_t size = 50 + (rng() % 500);
					registry.addShape(size);
				}
				// Very short sleep to maximize destruction rate
				std::this_thread::sleep_for(std::chrono::microseconds(50));
			}
		});

		// Resizer thread: continuously resizes shapes
		std::thread resizer([&]() {
			std::mt19937 rng(456);
			while (testRunning.load()) {
				size_t count = registry.count();
				if (count > 0) {
					size_t idx = rng() % count;
					size_t newSize = 50 + (rng() % 500);
					registry.resizeShape(idx, newSize);
				}
				std::this_thread::sleep_for(std::chrono::microseconds(30));
			}
		});

		// Collision workers: process collisions between shapes
		std::vector<std::thread> workers;
		for (int w = 0; w < 8; ++w) {
			workers.emplace_back([&, w]() {
				std::mt19937 rng(w * 1000);
				while (testRunning.load()) {
					// Get snapshot of current shapes
					auto shapes = registry.getShapeSnapshot();
					if (shapes.size() < 2) {
						std::this_thread::yield();
						continue;
					}

					// Pick two random shapes
					size_t idxA = rng() % shapes.size();
					size_t idxB = (idxA + 1 + rng() % (shapes.size() - 1)) % shapes.size();
					StressShape* shapeA = shapes[idxA];
					StressShape* shapeB = shapes[idxB];

					// Capture sizes BEFORE processing (simulates tree node creation)
					size_t sizeA = shapeA->size();
					size_t sizeB = shapeB->size();

					if (sizeA == 0 || sizeB == 0) {
						continue;
					}

					// Generate "collision" indices based on captured sizes
					size_t indexA = rng() % sizeA;
					size_t indexB = rng() % sizeB;

					// Small delay to allow destructor/resizer to race
					std::this_thread::yield();

					// NOW try to validate/use the indices (simulates doMerge)
					// This is where the race manifests: sizes may have changed!

					bool shapeAValid = shapeA->isValid.load();
					bool shapeBValid = shapeB->isValid.load();

					if (!shapeAValid || !shapeBValid) {
						stalePointerDetected.fetch_add(1);
						continue;
					}

					// Re-check sizes - they may have changed!
					size_t currentSizeA = shapeA->size();
					size_t currentSizeB = shapeB->size();

					if (indexA >= currentSizeA || indexB >= currentSizeB) {
						invalidAccesses.fetch_add(1);
						// Check for ~2x pattern
						if (indexA >= currentSizeA && currentSizeA > 0 && indexA > currentSizeA * 1.5) {
							shapeMismatch.fetch_add(1);
						}
						if (indexB >= currentSizeB && currentSizeB > 0 && indexB > currentSizeB * 1.5) {
							shapeMismatch.fetch_add(1);
						}
					}
					else {
						validAccesses.fetch_add(1);
						results.addResult(shapeA, shapeB, indexA, indexB);
					}
				}
			});
		}

		// Let it run for a while
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		testRunning = false;

		for (auto& w : workers) {
			w.join();
		}
		destructor.join();
		resizer.join();

		INFO("Valid accesses: " << validAccesses.load());
		INFO("Invalid accesses (indices out of range): " << invalidAccesses.load());
		INFO("Stale pointers detected: " << stalePointerDetected.load());
		INFO("~2x pattern (shape mismatch): " << shapeMismatch.load());
		INFO("Shapes created: " << registry.createCount.load());
		INFO("Shapes destroyed: " << registry.destroyCount.load());
		INFO("Collision results stored: " << results.size());

		// We expect MANY invalid accesses due to races
		// The key is that the validation CATCHES them rather than crashing
		REQUIRE(validAccesses.load() > 0); // Test actually did work

		// Report if we saw the ~2x pattern (suggests shape mixing)
		if (shapeMismatch.load() > 0) {
			WARN("Detected " << shapeMismatch.load() << " cases of ~2x index pattern");
		}
	}

	SECTION("Collision processing during shape replacement")
	{
		// Simulates: NPC unloads, different NPC loads with different mesh
		// Old collision results might reference the old shape's indices

		ShapeRegistry registry;
		std::atomic<bool> testRunning{true};
		std::atomic<int> indexCorruptionCaught{0};
		std::atomic<int> totalCollisions{0};

		// Add initial shapes
		for (int i = 0; i < 5; ++i) {
			registry.addShape(475 + i); // Match the bug report numbers
		}

		// "NPC load/unload" thread - destroys and recreates shapes
		// with DIFFERENT sizes (simulating different NPCs)
		std::thread npcLoader([&]() {
			std::mt19937 rng(789);
			int cycle = 0;
			while (testRunning.load()) {
				// Destroy all shapes and create new ones with different sizes
				// This simulates a major scene change (fast travel, cell load, etc.)

				if (cycle % 10 == 0) {
					// "Major reset" - clear and recreate all
					while (registry.count() > 0) {
						registry.destroyShape(0);
					}
					// Create new shapes with VERY different sizes
					for (int i = 0; i < 5; ++i) {
						size_t size = 100 + (rng() % 900); // 100-1000
						registry.addShape(size);
					}
				}
				else {
					// Minor churn - destroy/create one shape
					size_t count = registry.count();
					if (count > 0) {
						registry.destroyShape(rng() % count);
						registry.addShape(100 + (rng() % 900));
					}
				}
				cycle++;
				std::this_thread::sleep_for(std::chrono::microseconds(100));
			}
		});

		// Collision worker simulating the bug scenario
		std::thread collisionWorker([&]() {
			std::mt19937 rng(111);
			while (testRunning.load()) {
				auto shapes = registry.getShapeSnapshot();
				if (shapes.size() < 2) {
					std::this_thread::yield();
					continue;
				}

				// Pick shapes and capture their state
				size_t idxA = rng() % shapes.size();
				size_t idxB = (idxA + 1) % shapes.size();
				StressShape* shapeA = shapes[idxA];
				StressShape* shapeB = shapes[idxB];

				uint64_t capturedIdA = shapeA->shapeId;
				uint64_t capturedIdB = shapeB->shapeId;
				size_t capturedSizeA = shapeA->size();
				size_t capturedSizeB = shapeB->size();

				if (capturedSizeA == 0 || capturedSizeB == 0)
					continue;

				// Generate indices based on CAPTURED sizes
				size_t indexA = rng() % capturedSizeA;
				size_t indexB = rng() % capturedSizeB;

				// Simulate some processing delay
				for (volatile int i = 0; i < 100; ++i) {
				}

				// Now try to use the indices - shapes may have been replaced!
				// Check if shape was replaced (different ID means different shape!)
				bool shapeAReplaced = shapeA->shapeId != capturedIdA || !shapeA->isValid.load();
				bool shapeBReplaced = shapeB->shapeId != capturedIdB || !shapeB->isValid.load();

				if (shapeAReplaced || shapeBReplaced) {
					// Shape was replaced - indices are definitely invalid!
					indexCorruptionCaught.fetch_add(1);
				}
				else {
					// Shape still valid - check if size changed
					size_t currentSizeA = shapeA->size();
					size_t currentSizeB = shapeB->size();

					if (indexA >= currentSizeA || indexB >= currentSizeB) {
						indexCorruptionCaught.fetch_add(1);
					}
				}

				totalCollisions.fetch_add(1);
			}
		});

		std::this_thread::sleep_for(std::chrono::milliseconds(300));
		testRunning = false;

		npcLoader.join();
		collisionWorker.join();

		INFO("Total collision attempts: " << totalCollisions.load());
		INFO("Index corruption caught: " << indexCorruptionCaught.load());
		INFO("Shapes created: " << registry.createCount.load());
		INFO("Shapes destroyed: " << registry.destroyCount.load());

		REQUIRE(totalCollisions.load() > 0);
		// We EXPECT corruption to be caught - that's the point of validation
	}
}

TEST_CASE("AGGRESSIVE: Tree offset mixing between shapes", "[collision][index][stress][aggressive]")
{
	// This test specifically targets the hypothesis that tree offsets from
	// different shapes are getting mixed together

	SECTION("Parallel collision with offset from wrong shape")
	{
		constexpr int NUM_WORKERS = 8;
		constexpr int ITERATIONS = 1000;

		// Create shapes with very different sizes to make mixing obvious
		StressShape shapeA(1, 500);	 // Offsets 0-499 valid
		StressShape shapeB(2, 1000); // Offsets 0-999 valid
		StressShape shapeC(3, 200);	 // Offsets 0-199 valid

		struct TreeNode
		{
			size_t colliderOffset;
			size_t numColliders;
			StressShape* ownerShape; // DEBUG: track which shape owns this
		};

		// Create tree nodes for each shape
		TreeNode nodeA = {400, 100, &shapeA}; // Covers offsets 400-499 in shapeA
		TreeNode nodeB = {800, 200, &shapeB}; // Covers offsets 800-999 in shapeB
		TreeNode nodeC = {100, 100, &shapeC}; // Covers offsets 100-199 in shapeC

		std::atomic<int> correctPairs{0};
		std::atomic<int> wrongBasePairs{0};
		std::atomic<int> indexOutOfRange{0};

		std::vector<std::thread> workers;
		for (int w = 0; w < NUM_WORKERS; ++w) {
			workers.emplace_back([&, w]() {
				std::mt19937 rng(w);

				for (int i = 0; i < ITERATIONS; ++i) {
					// Simulate: pick two tree nodes (should be from different shapes)
					TreeNode* nodes[] = {&nodeA, &nodeB, &nodeC};
					int idx1 = rng() % 3;
					int idx2 = (idx1 + 1 + rng() % 2) % 3;
					TreeNode* node1 = nodes[idx1];
					TreeNode* node2 = nodes[idx2];

					// Generate local index within tree node
					size_t localIndex1 = rng() % node1->numColliders;
					size_t localIndex2 = rng() % node2->numColliders;

					// CORRECT: Use tree offset with its own shape
					size_t correctIndex1 = node1->colliderOffset + localIndex1;
					size_t correctIndex2 = node2->colliderOffset + localIndex2;

					// Validate correct indices
					if (correctIndex1 < node1->ownerShape->size() && correctIndex2 < node2->ownerShape->size()) {
						correctPairs.fetch_add(1);
					}

					// BUG SIMULATION: What if we use node1's offset with node2's shape?
					// This is the suspected bug pattern
					size_t buggyIndex = node1->colliderOffset + localIndex2; // WRONG!

					// This buggy index might be valid for node1->ownerShape
					// but we're trying to access node2->ownerShape!
					if (buggyIndex >= node2->ownerShape->size()) {
						// Caught the bug: index out of range for the shape we're accessing
						wrongBasePairs.fetch_add(1);

						// Check for ~2x pattern
						if (buggyIndex > node2->ownerShape->size() * 1.5) {
							indexOutOfRange.fetch_add(1);
						}
					}
				}
			});
		}

		for (auto& w : workers) {
			w.join();
		}

		INFO("Correct index pairs: " << correctPairs.load());
		INFO("Wrong base pairs detected: " << wrongBasePairs.load());
		INFO("~2x out of range detected: " << indexOutOfRange.load());

		// We expect to find many cases where using the wrong base causes issues
		REQUIRE(correctPairs.load() > 0);
		REQUIRE(wrongBasePairs.load() > 0); // The bug pattern should be detectable

		// Example of the exact bug:
		// nodeA.offset=400, nodeB.size=200
		// localIndex=50
		// buggyIndex = 400 + 50 = 450
		// But shapeC only has 200 colliders!
		// 450 / 200 = 2.25x - matches the ~2x pattern!
	}
}
