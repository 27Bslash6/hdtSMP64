#pragma once

#include "hdtBulletHelper.h"

#include <amp.h>
#include <amp_graphics.h>
#include <amp_math.h>
#include <amp_short_vectors.h>
#include <immintrin.h> // AVX2 intrinsics

namespace hdt
{
	struct Aabb
	{
		Aabb() { invalidate(); }

		Aabb(__m128 mmin, __m128 mmax) : m_min(mmin), m_max(mmax) {}

		__m128 m_min;
		__m128 m_max;

		void extendMargin(float margin)
		{
			auto margin3 = _mm_set_ps1(margin);
			m_min -= margin3;
			m_max += margin3;
		}

		Aabb extended(float margin) const
		{
			auto margin3 = _mm_set_ps1(margin);
			return Aabb(m_min - margin3, m_max + margin3);
		}

		bool collideWith(const btVector3& rhs) const
		{
			auto flag0 = _mm_cmplt_ps(m_min, rhs.get128());
			auto flag1 = _mm_cmplt_ps(rhs.get128(), m_max);
			auto flag = _mm_movemask_ps(_mm_and_ps(flag0, flag1));
			return (flag & 0x7) == 7;
		}

		bool collideWith(const Aabb& rhs) const
		{
			auto flag0 = _mm_cmplt_ps(rhs.m_max, m_min);
			auto flag1 = _mm_cmplt_ps(m_max, rhs.m_min);
			auto flag = _mm_movemask_ps(_mm_or_ps(flag0, flag1));
			return !(flag & 0x7);
			/*
			bool overlap = true;
			overlap = (m_min[0] >= rhs.m_max[0] || m_max[0] <= rhs.m_min[0]) ? false : overlap;
			overlap = (m_min[1] >= rhs.m_max[1] || m_max[1] <= rhs.m_min[1]) ? false : overlap;
			overlap = (m_min[2] >= rhs.m_max[2] || m_max[2] <= rhs.m_min[2]) ? false : overlap;
			return overlap;*/
		}

		bool collideWithSphere(const btVector3& p, float radius) const { return extended(radius).collideWith(p); }

		// AVX2 batch collision check: test 2 AABBs against this AABB simultaneously
		// Returns bitmask: bit 0 = aabb0 collides, bit 1 = aabb1 collides
#ifdef __AVX2__
		__forceinline int collideWith2(const Aabb& aabb0, const Aabb& aabb1) const
		{
			// Broadcast this AABB's min/max to both lanes of 256-bit registers
			__m256 thisMin = _mm256_set_m128(m_min, m_min);
			__m256 thisMax = _mm256_set_m128(m_max, m_max);

			// Pack both test AABBs: [aabb1.min | aabb0.min] and [aabb1.max | aabb0.max]
			__m256 testMin = _mm256_set_m128(aabb1.m_min, aabb0.m_min);
			__m256 testMax = _mm256_set_m128(aabb1.m_max, aabb0.m_max);

			// Collision test: no overlap if any axis is separated
			// flag0: testMax < thisMin (test box entirely below this box on any axis)
			// flag1: thisMax < testMin (this box entirely below test box on any axis)
			__m256 flag0 = _mm256_cmp_ps(testMax, thisMin, _CMP_LT_OQ);
			__m256 flag1 = _mm256_cmp_ps(thisMax, testMin, _CMP_LT_OQ);
			__m256 separated = _mm256_or_ps(flag0, flag1);

			// Extract masks for each AABB (lower 3 bits of each 128-bit lane)
			int mask = _mm256_movemask_ps(separated);
			// mask bits: [7:4] = aabb1, [3:0] = aabb0
			// Collision if NO separation on x,y,z (bits 0-2 for aabb0, bits 4-6 for aabb1)
			int result = 0;
			if (!(mask & 0x07))
				result |= 1; // aabb0 collides
			if (!(mask & 0x70))
				result |= 2; // aabb1 collides
			return result;
		}
#else
		// Scalar fallback for non-AVX2 builds
		__forceinline int collideWith2(const Aabb& aabb0, const Aabb& aabb1) const
		{
			int result = 0;
			if (collideWith(aabb0))
				result |= 1;
			if (collideWith(aabb1))
				result |= 2;
			return result;
		}
#endif

		// AVX-512 batch collision check: test 4 AABBs against this AABB simultaneously
		// Returns bitmask: bit N = aabbN collides (bits 0-3)
#ifdef __AVX512F__
		__forceinline int collideWith4(const Aabb& aabb0, const Aabb& aabb1, const Aabb& aabb2, const Aabb& aabb3) const
		{
			__m512 thisMin = _mm512_broadcast_f32x4(m_min);
			__m512 thisMax = _mm512_broadcast_f32x4(m_max);

			// Build 512-bit registers from four 128-bit AABBs using insertf32x4
			__m512 testMin = _mm512_insertf32x4(
				_mm512_insertf32x4(_mm512_insertf32x4(_mm512_castps128_ps512(aabb0.m_min), aabb1.m_min, 1), aabb2.m_min,
								   2),
				aabb3.m_min, 3);
			__m512 testMax = _mm512_insertf32x4(
				_mm512_insertf32x4(_mm512_insertf32x4(_mm512_castps128_ps512(aabb0.m_max), aabb1.m_max, 1), aabb2.m_max,
								   2),
				aabb3.m_max, 3);

			__mmask16 sep = _mm512_cmp_ps_mask(testMax, thisMin, _CMP_LT_OQ) |
							_mm512_cmp_ps_mask(thisMax, testMin, _CMP_LT_OQ);

			// Each AABB uses 4 bits; check xyz (lower 3 bits of each lane)
			return (!(sep & 0x0007)) | ((!(sep & 0x0070)) << 1) | ((!(sep & 0x0700)) << 2) | ((!(sep & 0x7000)) << 3);
		}
#endif

		// Batch collision check against array of AABBs
		// Uses AVX-512 (4 at a time) or AVX2 (2 at a time) depending on build
		template<typename OutputIt>
		static int collideWithMany(const Aabb& ref, const Aabb* aabbs, int count, OutputIt out)
		{
			int collisions = 0;
			int i = 0;

			// Helper to emit collisions from bitmask
			auto emit = [&](int mask, int base, int n) {
				for (int b = 0; b < n; ++b)
					if (mask & (1 << b)) {
						*out++ = const_cast<Aabb*>(&aabbs[base + b]);
						++collisions;
					}
			};

#ifdef __AVX512F__
			for (; i + 3 < count; i += 4)
				emit(ref.collideWith4(aabbs[i], aabbs[i + 1], aabbs[i + 2], aabbs[i + 3]), i, 4);
#endif
			for (; i + 1 < count; i += 2)
				emit(ref.collideWith2(aabbs[i], aabbs[i + 1]), i, 2);

			if (i < count && ref.collideWith(aabbs[i])) {
				*out++ = const_cast<Aabb*>(&aabbs[i]);
				++collisions;
			}

			return collisions;
		}

		void invalidate()
		{
			m_min = setAll(FLT_MAX);
			m_max = setAll(-FLT_MAX);
		}

		void merge(const btVector3& p)
		{
			m_min = _mm_min_ps(m_min, p.get128());
			m_max = _mm_max_ps(m_max, p.get128());
		}

		void merge(const Aabb& rhs)
		{
			m_min = _mm_min_ps(m_min, rhs.m_min);
			m_max = _mm_max_ps(m_max, rhs.m_max);
		}

		// AVX2 batch merge: merge multiple AABBs into this one efficiently
#ifdef __AVX2__
		void mergeMany(const Aabb* aabbs, int count)
		{
			if (count <= 0)
				return;

			int i = 0;
			// Start with first AABB
			__m256 accMin = _mm256_set_m128(aabbs[0].m_min, m_min);
			__m256 accMax = _mm256_set_m128(aabbs[0].m_max, m_max);
			i = 1;

			// Process pairs with AVX2
			for (; i + 1 < count; i += 2) {
				__m256 pairMin = _mm256_set_m128(aabbs[i + 1].m_min, aabbs[i].m_min);
				__m256 pairMax = _mm256_set_m128(aabbs[i + 1].m_max, aabbs[i].m_max);
				accMin = _mm256_min_ps(accMin, pairMin);
				accMax = _mm256_max_ps(accMax, pairMax);
			}

			// Reduce 256-bit to 128-bit
			__m128 lo_min = _mm256_castps256_ps128(accMin);
			__m128 hi_min = _mm256_extractf128_ps(accMin, 1);
			__m128 lo_max = _mm256_castps256_ps128(accMax);
			__m128 hi_max = _mm256_extractf128_ps(accMax, 1);
			m_min = _mm_min_ps(lo_min, hi_min);
			m_max = _mm_max_ps(lo_max, hi_max);

			// Handle remaining odd element
			if (i < count)
				merge(aabbs[i]);
		}
#else
		// Scalar fallback for non-AVX2 builds
		void mergeMany(const Aabb* aabbs, int count)
		{
			for (int i = 0; i < count; ++i)
				merge(aabbs[i]);
		}
#endif

		void mergeAdd(const btVector3& p)
		{
			m_min = _mm_min_ps(m_min, _mm_add_ps(m_min, p.get128()));
			m_max = _mm_max_ps(m_max, _mm_add_ps(m_max, p.get128()));
		}

		void mergeSub(const btVector3& p)
		{
			m_min = _mm_min_ps(m_min, _mm_sub_ps(m_min, p.get128()));
			m_max = _mm_max_ps(m_max, _mm_sub_ps(m_max, p.get128()));
		}
	};

	struct BoundingSphere
	{
		BoundingSphere() {}

		BoundingSphere(const btVector3& center, float radius) : m_centerRadius(center) { m_centerRadius[3] = radius; }

		bool isCollide(const BoundingSphere& rhs) const
		{
			btVector3 ca = m_centerRadius;
			btVector3 cb = rhs.m_centerRadius;
			float ra = m_centerRadius.w();
			float rb = rhs.m_centerRadius.w();
			return (ca - cb).length2() < (ra + rb) * (ra + rb);
		}

		btVector3 center() const { return m_centerRadius; }
		float radius() const { return m_centerRadius.w(); }
		Aabb getAabb() const { return Aabb(center().get128(), center().get128()).extended(radius()); }

		btVector4 m_centerRadius;
	};
} // namespace hdt
