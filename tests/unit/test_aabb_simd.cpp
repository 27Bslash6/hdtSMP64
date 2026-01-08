#include "../include/catch.hpp"
#include <immintrin.h>
#include <vector>
#include <iterator>

// Standalone AABB struct for testing - mirrors hdtAABB.h logic
namespace test {

struct Aabb {
    __m128 m_min;
    __m128 m_max;

    Aabb() = default;
    Aabb(__m128 mmin, __m128 mmax) : m_min(mmin), m_max(mmax) {}

    // SSE scalar collision check
    bool collideWith(const Aabb& rhs) const {
        auto flag0 = _mm_cmplt_ps(rhs.m_max, m_min);
        auto flag1 = _mm_cmplt_ps(m_max, rhs.m_min);
        auto flag = _mm_movemask_ps(_mm_or_ps(flag0, flag1));
        return !(flag & 0x7);
    }

    // AVX2 batch collision check: test 2 AABBs simultaneously
    __forceinline int collideWith2(const Aabb& aabb0, const Aabb& aabb1) const {
        __m256 thisMin = _mm256_set_m128(m_min, m_min);
        __m256 thisMax = _mm256_set_m128(m_max, m_max);
        __m256 testMin = _mm256_set_m128(aabb1.m_min, aabb0.m_min);
        __m256 testMax = _mm256_set_m128(aabb1.m_max, aabb0.m_max);

        __m256 flag0 = _mm256_cmp_ps(testMax, thisMin, _CMP_LT_OQ);
        __m256 flag1 = _mm256_cmp_ps(thisMax, testMin, _CMP_LT_OQ);
        __m256 separated = _mm256_or_ps(flag0, flag1);

        int mask = _mm256_movemask_ps(separated);
        int result = 0;
        if (!(mask & 0x07)) result |= 1;
        if (!(mask & 0x70)) result |= 2;
        return result;
    }

    // Batch collision check against array
    template<typename OutputIt>
    static int collideWithMany(const Aabb& ref, const Aabb* aabbs, int count, OutputIt out) {
        int collisions = 0;
        int i = 0;

        auto emit = [&](int mask, int base, int n) {
            for (int b = 0; b < n; ++b)
                if (mask & (1 << b)) { *out++ = const_cast<Aabb*>(&aabbs[base + b]); ++collisions; }
        };

        for (; i + 1 < count; i += 2)
            emit(ref.collideWith2(aabbs[i], aabbs[i+1]), i, 2);

        if (i < count && ref.collideWith(aabbs[i]))
            { *out++ = const_cast<Aabb*>(&aabbs[i]); ++collisions; }

        return collisions;
    }
};

// Helper to create AABB from coordinates
static Aabb makeAabb(float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
    return Aabb(
        _mm_set_ps(0, minZ, minY, minX),
        _mm_set_ps(0, maxZ, maxY, maxX)
    );
}

} // namespace test

using namespace test;

TEST_CASE("Aabb::collideWith SSE baseline", "[aabb][sse]") {
    SECTION("Overlapping AABBs") {
        Aabb a = makeAabb(0, 0, 0, 10, 10, 10);
        Aabb b = makeAabb(5, 5, 5, 15, 15, 15);
        REQUIRE(a.collideWith(b) == true);
        REQUIRE(b.collideWith(a) == true);
    }

    SECTION("Non-overlapping AABBs") {
        Aabb a = makeAabb(0, 0, 0, 10, 10, 10);
        Aabb b = makeAabb(20, 20, 20, 30, 30, 30);
        REQUIRE(a.collideWith(b) == false);
        REQUIRE(b.collideWith(a) == false);
    }

    SECTION("Separated on X axis only") {
        Aabb a = makeAabb(0, 0, 0, 10, 10, 10);
        Aabb b = makeAabb(15, 0, 0, 25, 10, 10);
        REQUIRE(a.collideWith(b) == false);
    }

    SECTION("Separated on Y axis only") {
        Aabb a = makeAabb(0, 0, 0, 10, 10, 10);
        Aabb b = makeAabb(0, 15, 0, 10, 25, 10);
        REQUIRE(a.collideWith(b) == false);
    }

    SECTION("Separated on Z axis only") {
        Aabb a = makeAabb(0, 0, 0, 10, 10, 10);
        Aabb b = makeAabb(0, 0, 15, 10, 10, 25);
        REQUIRE(a.collideWith(b) == false);
    }

    SECTION("Contained AABB") {
        Aabb outer = makeAabb(0, 0, 0, 100, 100, 100);
        Aabb inner = makeAabb(25, 25, 25, 75, 75, 75);
        REQUIRE(outer.collideWith(inner) == true);
        REQUIRE(inner.collideWith(outer) == true);
    }
}

TEST_CASE("Aabb::collideWith2 AVX2 batch", "[aabb][avx2]") {
    Aabb ref = makeAabb(0, 0, 0, 10, 10, 10);

    SECTION("Both collide") {
        Aabb a = makeAabb(5, 5, 5, 15, 15, 15);
        Aabb b = makeAabb(-5, -5, -5, 5, 5, 5);
        int mask = ref.collideWith2(a, b);
        REQUIRE((mask & 1) != 0);
        REQUIRE((mask & 2) != 0);
    }

    SECTION("Only first collides") {
        Aabb a = makeAabb(5, 5, 5, 15, 15, 15);
        Aabb b = makeAabb(20, 20, 20, 30, 30, 30);
        int mask = ref.collideWith2(a, b);
        REQUIRE((mask & 1) != 0);
        REQUIRE((mask & 2) == 0);
    }

    SECTION("Only second collides") {
        Aabb a = makeAabb(20, 20, 20, 30, 30, 30);
        Aabb b = makeAabb(5, 5, 5, 15, 15, 15);
        int mask = ref.collideWith2(a, b);
        REQUIRE((mask & 1) == 0);
        REQUIRE((mask & 2) != 0);
    }

    SECTION("Neither collides") {
        Aabb a = makeAabb(20, 20, 20, 30, 30, 30);
        Aabb b = makeAabb(-30, -30, -30, -20, -20, -20);
        int mask = ref.collideWith2(a, b);
        REQUIRE(mask == 0);
    }

    SECTION("Matches scalar version") {
        Aabb testCases[] = {
            makeAabb(5, 5, 5, 15, 15, 15),
            makeAabb(-5, -5, -5, 5, 5, 5),
            makeAabb(20, 20, 20, 30, 30, 30),
            makeAabb(-30, -30, -30, -20, -20, -20),
            makeAabb(0, 0, 0, 10, 10, 10),
            makeAabb(9, 9, 9, 11, 11, 11),
        };

        for (int i = 0; i < 6; i += 2) {
            int mask = ref.collideWith2(testCases[i], testCases[i + 1]);
            bool scalar0 = ref.collideWith(testCases[i]);
            bool scalar1 = ref.collideWith(testCases[i + 1]);
            REQUIRE(((mask & 1) != 0) == scalar0);
            REQUIRE(((mask & 2) != 0) == scalar1);
        }
    }
}

TEST_CASE("Aabb::collideWithMany batch processing", "[aabb][avx2]") {
    Aabb ref = makeAabb(0, 0, 0, 10, 10, 10);

    SECTION("Single element") {
        Aabb aabbs[] = { makeAabb(5, 5, 5, 15, 15, 15) };
        std::vector<Aabb*> results;
        int count = Aabb::collideWithMany(ref, aabbs, 1, std::back_inserter(results));
        REQUIRE(count == 1);
        REQUIRE(results.size() == 1);
        REQUIRE(results[0] == &aabbs[0]);
    }

    SECTION("Two elements, both collide") {
        Aabb aabbs[] = {
            makeAabb(5, 5, 5, 15, 15, 15),
            makeAabb(-5, -5, -5, 5, 5, 5)
        };
        std::vector<Aabb*> results;
        int count = Aabb::collideWithMany(ref, aabbs, 2, std::back_inserter(results));
        REQUIRE(count == 2);
        REQUIRE(results.size() == 2);
    }

    SECTION("Mixed collisions") {
        Aabb aabbs[] = {
            makeAabb(5, 5, 5, 15, 15, 15),     // collides
            makeAabb(20, 20, 20, 30, 30, 30),  // doesn't
            makeAabb(-5, -5, -5, 5, 5, 5),     // collides
            makeAabb(-30, -30, -30, -20, -20, -20),  // doesn't
            makeAabb(9, 9, 9, 11, 11, 11),     // collides
        };
        std::vector<Aabb*> results;
        int count = Aabb::collideWithMany(ref, aabbs, 5, std::back_inserter(results));
        REQUIRE(count == 3);
        REQUIRE(results.size() == 3);
        REQUIRE(results[0] == &aabbs[0]);
        REQUIRE(results[1] == &aabbs[2]);
        REQUIRE(results[2] == &aabbs[4]);
    }

    SECTION("Matches scalar filtering") {
        Aabb aabbs[10];
        for (int i = 0; i < 10; i++) {
            float offset = static_cast<float>(i * 3 - 10);
            aabbs[i] = makeAabb(offset, offset, offset, offset + 8, offset + 8, offset + 8);
        }

        std::vector<Aabb*> batchResults;
        Aabb::collideWithMany(ref, aabbs, 10, std::back_inserter(batchResults));

        std::vector<Aabb*> scalarResults;
        for (int i = 0; i < 10; i++) {
            if (ref.collideWith(aabbs[i])) {
                scalarResults.push_back(&aabbs[i]);
            }
        }

        REQUIRE(batchResults.size() == scalarResults.size());
        for (size_t i = 0; i < batchResults.size(); i++) {
            REQUIRE(batchResults[i] == scalarResults[i]);
        }
    }
}

TEST_CASE("Aabb::mergeMany AVX2 batch merge", "[aabb][avx2]") {
    // Add mergeMany to test::Aabb for testing
    struct AabbWithMerge : public Aabb {
        void mergeMany(const Aabb* aabbs, int count) {
            if (count <= 0) return;

            int i = 0;
            __m256 accMin = _mm256_set_m128(aabbs[0].m_min, m_min);
            __m256 accMax = _mm256_set_m128(aabbs[0].m_max, m_max);
            i = 1;

            for (; i + 1 < count; i += 2) {
                __m256 pairMin = _mm256_set_m128(aabbs[i + 1].m_min, aabbs[i].m_min);
                __m256 pairMax = _mm256_set_m128(aabbs[i + 1].m_max, aabbs[i].m_max);
                accMin = _mm256_min_ps(accMin, pairMin);
                accMax = _mm256_max_ps(accMax, pairMax);
            }

            __m128 lo_min = _mm256_castps256_ps128(accMin);
            __m128 hi_min = _mm256_extractf128_ps(accMin, 1);
            __m128 lo_max = _mm256_castps256_ps128(accMax);
            __m128 hi_max = _mm256_extractf128_ps(accMax, 1);
            m_min = _mm_min_ps(lo_min, hi_min);
            m_max = _mm_max_ps(lo_max, hi_max);

            if (i < count) {
                m_min = _mm_min_ps(m_min, aabbs[i].m_min);
                m_max = _mm_max_ps(m_max, aabbs[i].m_max);
            }
        }

        void mergeScalar(const Aabb* aabbs, int count) {
            for (int i = 0; i < count; ++i) {
                m_min = _mm_min_ps(m_min, aabbs[i].m_min);
                m_max = _mm_max_ps(m_max, aabbs[i].m_max);
            }
        }
    };

    auto getMin = [](const Aabb& a, int idx) { return a.m_min.m128_f32[idx]; };
    auto getMax = [](const Aabb& a, int idx) { return a.m_max.m128_f32[idx]; };

    SECTION("Merge single AABB") {
        AabbWithMerge result;
        result.m_min = _mm_set_ps(0, 0, 0, 0);
        result.m_max = _mm_set_ps(0, 10, 10, 10);

        Aabb toMerge[] = { makeAabb(5, 5, 5, 15, 15, 15) };
        result.mergeMany(toMerge, 1);

        REQUIRE(getMin(result, 0) == 0);
        REQUIRE(getMax(result, 0) == 15);
    }

    SECTION("Merge two AABBs") {
        AabbWithMerge result;
        result.m_min = _mm_set_ps(0, 0, 0, 0);
        result.m_max = _mm_set_ps(0, 10, 10, 10);

        Aabb toMerge[] = {
            makeAabb(-5, -5, -5, 5, 5, 5),
            makeAabb(8, 8, 8, 20, 20, 20)
        };
        result.mergeMany(toMerge, 2);

        REQUIRE(getMin(result, 0) == -5);
        REQUIRE(getMax(result, 0) == 20);
    }

    SECTION("Merge odd number of AABBs") {
        AabbWithMerge result;
        result.m_min = _mm_set_ps(0, 50, 50, 50);
        result.m_max = _mm_set_ps(0, 60, 60, 60);

        Aabb toMerge[] = {
            makeAabb(0, 0, 0, 10, 10, 10),
            makeAabb(20, 20, 20, 30, 30, 30),
            makeAabb(-10, -10, -10, 5, 5, 5)
        };
        result.mergeMany(toMerge, 3);

        REQUIRE(getMin(result, 0) == -10);
        REQUIRE(getMax(result, 0) == 60);
    }

    SECTION("Matches scalar merge") {
        Aabb aabbs[10];
        for (int i = 0; i < 10; i++) {
            float offset = static_cast<float>(i * 5 - 25);
            aabbs[i] = makeAabb(offset, offset, offset, offset + 10, offset + 10, offset + 10);
        }

        AabbWithMerge batchResult;
        batchResult.m_min = _mm_set_ps(0, 100, 100, 100);
        batchResult.m_max = _mm_set_ps(0, 110, 110, 110);

        AabbWithMerge scalarResult;
        scalarResult.m_min = batchResult.m_min;
        scalarResult.m_max = batchResult.m_max;

        batchResult.mergeMany(aabbs, 10);
        scalarResult.mergeScalar(aabbs, 10);

        for (int i = 0; i < 3; i++) {
            REQUIRE(getMin(batchResult, i) == getMin(scalarResult, i));
            REQUIRE(getMax(batchResult, i) == getMax(scalarResult, i));
        }
    }
}
