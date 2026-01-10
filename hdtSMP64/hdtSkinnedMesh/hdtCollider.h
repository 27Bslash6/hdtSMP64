#pragma once

#include "hdtAABB.h"

#include <atomic>
#include <functional>

namespace hdt
{
	struct alignas(16) Collider
	{
		Collider() {}

		Collider(int i0) { vertex = i0; }
		Collider(int i0, int i1, int i2) { vertices[0] = i0, vertices[1] = i1, vertices[2] = i2; }
		Collider(const Collider& rhs) { operator=(rhs); }

		Collider& operator=(const Collider& rhs)
		{
			__m128i* dst = (__m128i*)this;
			__m128i* src = (__m128i*)&rhs;
			auto xmm0 = _mm_load_si128(src + 0);
			_mm_store_si128(dst, xmm0);
			return *this;
		}

		union
		{
			U32 vertex;		 // vertexshape
			U32 vertices[3]; // triangleshape
		};

		float flexible;
		//		inline bool operator <(const Collider& rhs){ return aligned < rhs.aligned; }
	};

	struct alignas(16) ColliderTree
	{
		ColliderTree()
		{
			aabbAll.invalidate();
			aabbMe.invalidate();
		}

		ColliderTree(U32 k) : key(k)
		{
			aabbAll.invalidate();
			aabbMe.invalidate();
		}

		// Default copy/move - colliderOffset is just an index, safe to copy
		// The offset is relative to whichever shape's m_colliders this tree belongs to
		// Pointers are computed on-the-fly using base + colliderOffset
		ColliderTree(const ColliderTree&) = default;
		ColliderTree& operator=(const ColliderTree&) = default;
		ColliderTree(ColliderTree&&) noexcept = default;
		ColliderTree& operator=(ColliderTree&&) noexcept = default;

		Aabb aabbAll;
		Aabb aabbMe;

		U32 isKinematic = 0;

		// NO POINTERS - eliminates cross-shape contamination entirely
		// colliderOffset indexes into the owning shape's m_colliders/m_aabb vectors
		// Actual pointers computed on-the-fly: base + colliderOffset
		size_t colliderOffset = 0;
		U32 numCollider = 0;
		U32 dynCollider = 0;

		U32 dynChild = 0;
		vectorA16<ColliderTree> children;

		vectorA16<Collider> colliders;
		U32 key = 0;

		void insertCollider(const std::vector<U32>& keys, const Collider& c);
		void exportColliders(vectorA16<Collider>& exportTo);
		// Sets colliderOffset for all nodes (no pointer storage)
		void finalizeOffsets();

		void checkCollisionL(ColliderTree* r, std::vector<std::pair<ColliderTree*, ColliderTree*>>& ret);
		void checkCollisionR(ColliderTree* r, std::vector<std::pair<ColliderTree*, ColliderTree*>>& ret);
		void clipCollider(const std::function<bool(const Collider&)>& func);
		void updateKinematic(const std::function<float(const Collider*)>& func);
		void visitColliders(const std::function<void(Collider*)>& func);
		// updateAabb now takes base pointers - computes actual addresses on-the-fly
		void updateAabb(Aabb* aabbBase);
		void optimize();

		bool empty() const { return children.empty() && colliders.empty(); }

		// Helper to get collider pointer from base + offset
		Collider* getColliders(Collider* base) const { return base + colliderOffset; }
		Aabb* getAabbs(Aabb* base) const { return base + colliderOffset; }


		bool collapseCollideL(ColliderTree* r);
		bool collapseCollideR(ColliderTree* r);
	};

	/*
	struct _CRT_ALIGN(16) ColliderTree
	{

		Aabb aabb;
		vectorA16<Node> nodes;
		U32 isKinematic;

		void insertCollider(const std::vector<U32>& keys, const Collider& c);
		void checkCollision(const ColliderTree& r, std::vector<std::pair<Node*, Node*>>& ret);
		void clipCollider(const std::function<bool(const Collider&)>& func);
		void updateKinematic(const std::function<bool(const Collider*)>& func);
		void updateAabb(const std::function<void(Collider*)>& func);
		void visitColliders(const std::function<void(Collider*)>& func);
	};*/
} // namespace hdt
