#pragma once

#include "hdtCollider.h"

namespace hdt
{
	struct CollisionResult
	{
		btVector3 posA;
		btVector3 posB;
		btVector3 normOnB;
		size_t colliderIndexA; // Index into shape's m_colliders (not pointer - avoids stale pointer bugs)
		size_t colliderIndexB; // Index into shape's m_colliders (not pointer - avoids stale pointer bugs)
		float depth;
	};

	struct CheckTriangle
	{
		CheckTriangle(const btVector3& p0, const btVector3& p1, const btVector3& p2, float margin, float prenetration);

		btVector3 p0, p1, p2, normal;
		float margin, prenetration;
		bool valid;
	};

	bool checkSphereSphere(const btVector3& a, const btVector3& b, float ra, float rb, CollisionResult& res);
	bool checkSphereTriangle(const btVector3& s, float r, const CheckTriangle& tri, CollisionResult& res);

#ifndef CUDA
	static inline btVector3 BaryCoord(const btVector3& a, const btVector3& b, const btVector3& c, const btVector3& p)
	{
		const btVector3 ap = a - p;
		const btVector3 bp = b - p;
		const btVector3 cp = c - p;
		const btScalar area_a = btCross(bp, cp).length();
		const btScalar area_b = btCross(cp, ap).length();
		const btScalar area_c = btCross(ap, bp).length();
		const btScalar total = area_a + area_b + area_c;
		return btVector3(area_a / total, area_b / total, area_c / total);
	}
#endif
} // namespace hdt
