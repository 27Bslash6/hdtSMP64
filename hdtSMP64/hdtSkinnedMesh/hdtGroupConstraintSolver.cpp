#include "hdtGroupConstraintSolver.h"

#include <LinearMath/btCpuFeatureUtility.h>

#if defined(BT_ALLOW_SSE4)
#include <intrin.h>

#define USE_FMA 1
#define USE_FMA3_INSTEAD_FMA4 1
#define USE_SSE4_DOT 1

#define SSE4_DP(a, b) _mm_dp_ps(a, b, 0x7f)
#define SSE4_DP_FP(a, b) _mm_cvtss_f32(_mm_dp_ps(a, b, 0x7f))

#if USE_SSE4_DOT
#define DOT_PRODUCT(a, b) SSE4_DP(a, b)
#else
#define DOT_PRODUCT(a, b) btSimdDot3(a, b)
#endif

#if USE_FMA
#if USE_FMA3_INSTEAD_FMA4
// a*b + c
#define FMADD(a, b, c) _mm_fmadd_ps(a, b, c)
#define FMADD256(a, b, c) _mm256_fmadd_ps(a, b, c)
// -(a*b) + c
#define FMNADD(a, b, c) _mm_fnmadd_ps(a, b, c)
#define FMNADD256(a, b, c) _mm256_fnmadd_ps(a, b, c)
#else // USE_FMA3
// a*b + c
#define FMADD(a, b, c) _mm_macc_ps(a, b, c)
// -(a*b) + c
#define FMNADD(a, b, c) _mm_nmacc_ps(a, b, c)
#endif
#else // USE_FMA
// c + a*b
#define FMADD(a, b, c) _mm_add_ps(c, _mm_mul_ps(a, b))
// c - a*b
#define FMNADD(a, b, c) _mm_sub_ps(c, _mm_mul_ps(a, b))
#endif
#endif

namespace hdt
{
	inline __m256 pack256(__m128 lo, __m128 hi)
	{
		return _mm256_set_m128(hi, lo);
	}

	inline std::tuple<__m128, __m128> unpack256(__m256 ymm)
	{
		return std::tie(_mm256_castps256_ps128(ymm), _mm256_extractf128_ps(ymm, 1));
	}

	// Enhanced version of gResolveSingleConstraintRowGeneric_sse2 with AVX
	static btScalar gResolveSingleConstraintRowGeneric_avx256(btSolverBody& body1, btSolverBody& body2,
															  const btSolverConstraint& c)
	{
		__m128 tmp = _mm_set_ps1(c.m_jacDiagABInv);
		__m128 deltaImpulse = _mm_set_ps1(c.m_rhs - btScalar(c.m_appliedImpulse) * c.m_cfm);
		const __m128 lowerLimit = _mm_set_ps1(c.m_lowerLimit);
		const __m128 upperLimit = _mm_set_ps1(c.m_upperLimit);

		__m256 invMass = pack256(body1.internalGetInvMass().mVec128, body2.internalGetInvMass().mVec128);
		__m256 deltaLinearVelocity =
			pack256(body1.internalGetDeltaLinearVelocity().mVec128, body2.internalGetDeltaLinearVelocity().mVec128);
		__m256 deltaAngularVelocity =
			pack256(body1.internalGetDeltaAngularVelocity().mVec128, body2.internalGetDeltaAngularVelocity().mVec128);

		__m256 contactNormal = pack256(c.m_contactNormal1.mVec128, c.m_contactNormal2.mVec128);
		__m256 relposCrossNormal = pack256(c.m_relpos1CrossNormal.mVec128, c.m_relpos2CrossNormal.mVec128);
		__m256 deltaVelDotn = _mm256_add_ps(_mm256_dp_ps(contactNormal, deltaLinearVelocity, 0x7f),
											_mm256_dp_ps(relposCrossNormal, deltaAngularVelocity, 0x7f));
		deltaImpulse = FMNADD(_mm256_castps256_ps128(deltaVelDotn), tmp, deltaImpulse);
		deltaImpulse = FMNADD(_mm256_extractf128_ps(deltaVelDotn, 1), tmp, deltaImpulse);

		tmp = _mm_add_ps(c.m_appliedImpulse, deltaImpulse); // sum
		auto appliedImpulse = _mm_max_ps(_mm_min_ps(tmp, upperLimit), lowerLimit);
		deltaImpulse = _mm_sub_ps(appliedImpulse, c.m_appliedImpulse);
		c.m_appliedImpulse = appliedImpulse;

		auto deltaImpulse2 = pack256(deltaImpulse, deltaImpulse);
		auto angularComponent = pack256(c.m_angularComponentA.mVec128, c.m_angularComponentB.mVec128);
		deltaLinearVelocity = FMADD256(_mm256_mul_ps(contactNormal, invMass), deltaImpulse2, deltaLinearVelocity);
		deltaAngularVelocity = FMADD256(angularComponent, deltaImpulse2, deltaAngularVelocity);
		std::tie(body1.internalGetDeltaLinearVelocity().mVec128, body2.internalGetDeltaLinearVelocity().mVec128) =
			unpack256(deltaLinearVelocity);
		std::tie(body1.internalGetDeltaAngularVelocity().mVec128, body2.internalGetDeltaAngularVelocity().mVec128) =
			unpack256(deltaAngularVelocity);

		// Guard against division by zero - jacDiagABInv can be 0 for degenerate constraints
		const btScalar jacInv = c.m_jacDiagABInv;
		return (btFabs(jacInv) > SIMD_EPSILON) ? (deltaImpulse.m128_f32[0] / jacInv) : btScalar(0);
	}

	// Enhanced version of gResolveSingleConstraintRowGeneric_sse2 with AVX
	static btScalar gResolveSingleConstraintRowLowerLimit_avx256(btSolverBody& body1, btSolverBody& body2,
																 const btSolverConstraint& c)
	{
		__m128 tmp = _mm_set_ps1(c.m_jacDiagABInv);
		__m128 deltaImpulse = _mm_set_ps1(c.m_rhs - btScalar(c.m_appliedImpulse) * c.m_cfm);
		const __m128 lowerLimit = _mm_set_ps1(c.m_lowerLimit);

		__m256 invMass = pack256(body1.internalGetInvMass().mVec128, body2.internalGetInvMass().mVec128);
		__m256 deltaLinearVelocity =
			pack256(body1.internalGetDeltaLinearVelocity().mVec128, body2.internalGetDeltaLinearVelocity().mVec128);
		__m256 deltaAngularVelocity =
			pack256(body1.internalGetDeltaAngularVelocity().mVec128, body2.internalGetDeltaAngularVelocity().mVec128);

		__m256 contactNormal = pack256(c.m_contactNormal1.mVec128, c.m_contactNormal2.mVec128);
		__m256 relposCrossNormal = pack256(c.m_relpos1CrossNormal.mVec128, c.m_relpos2CrossNormal.mVec128);
		__m256 deltaVelDotn = _mm256_add_ps(_mm256_dp_ps(contactNormal, deltaLinearVelocity, 0x7f),
											_mm256_dp_ps(relposCrossNormal, deltaAngularVelocity, 0x7f));
		deltaImpulse = FMNADD(_mm256_castps256_ps128(deltaVelDotn), tmp, deltaImpulse);
		deltaImpulse = FMNADD(_mm256_extractf128_ps(deltaVelDotn, 1), tmp, deltaImpulse);

		tmp = _mm_add_ps(c.m_appliedImpulse, deltaImpulse);
		auto appliedImpulse = _mm_max_ps(tmp, lowerLimit);
		deltaImpulse = _mm_sub_ps(appliedImpulse, c.m_appliedImpulse);
		c.m_appliedImpulse = appliedImpulse;

		auto deltaImpulse2 = pack256(deltaImpulse, deltaImpulse);
		auto angularComponent = pack256(c.m_angularComponentA.mVec128, c.m_angularComponentB.mVec128);
		deltaLinearVelocity = FMADD256(_mm256_mul_ps(contactNormal, invMass), deltaImpulse2, deltaLinearVelocity);
		deltaAngularVelocity = FMADD256(angularComponent, deltaImpulse2, deltaAngularVelocity);
		std::tie(body1.internalGetDeltaLinearVelocity().mVec128, body2.internalGetDeltaLinearVelocity().mVec128) =
			unpack256(deltaLinearVelocity);
		std::tie(body1.internalGetDeltaAngularVelocity().mVec128, body2.internalGetDeltaAngularVelocity().mVec128) =
			unpack256(deltaAngularVelocity);

		// Guard against division by zero - jacDiagABInv can be 0 for degenerate constraints
		const btScalar jacInv = c.m_jacDiagABInv;
		return (btFabs(jacInv) > SIMD_EPSILON) ? (deltaImpulse.m128_f32[0] / jacInv) : btScalar(0);
	}

	btSingleConstraintRowSolver GroupConstraintSolver::getResolveSingleConstraintRowGenericAVX()
	{
		return gResolveSingleConstraintRowGeneric_avx256;
	}

	btSingleConstraintRowSolver GroupConstraintSolver::getResolveSingleConstraintRowLowerLimitAVX()
	{
		return gResolveSingleConstraintRowLowerLimit_avx256;
	}

	GroupConstraintSolver::GroupConstraintSolver()
	{
		int cpuFeatures = btCpuFeatureUtility::getCpuFeatures();
		if ((cpuFeatures & btCpuFeatureUtility::CPU_FEATURE_FMA3) &&
			(cpuFeatures & btCpuFeatureUtility::CPU_FEATURE_SSE4_1))
		{
			m_resolveSingleConstraintRowGeneric = gResolveSingleConstraintRowGeneric_avx256;
			m_resolveSingleConstraintRowLowerLimit = gResolveSingleConstraintRowLowerLimit_avx256;
		}
	}
} // namespace hdt
