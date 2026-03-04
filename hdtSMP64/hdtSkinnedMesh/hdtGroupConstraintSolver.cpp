#include "hdtGroupConstraintSolver.h"

#include "LinearMath/btScalar.h"

namespace hdt
{
	// Scalar constraint row resolver (generic — both limits)
	// Previously gResolveSingleConstraintRowGeneric_avx256.
	// The Highway batch solver (hdtHighwaySolverBridge.h) handles the hot path (>= batch threshold).
	// This scalar path handles low constraint counts where SIMD throughput is irrelevant.
	static btScalar gResolveSingleConstraintRowGeneric(btSolverBody& body1, btSolverBody& body2,
													   const btSolverConstraint& c)
	{
		const btScalar deltaVel1Dotn = c.m_contactNormal1.dot(body1.internalGetDeltaLinearVelocity()) +
									   c.m_relpos1CrossNormal.dot(body1.internalGetDeltaAngularVelocity());
		const btScalar deltaVel2Dotn = c.m_contactNormal2.dot(body2.internalGetDeltaLinearVelocity()) +
									   c.m_relpos2CrossNormal.dot(body2.internalGetDeltaAngularVelocity());

		btScalar deltaImpulse = c.m_rhs - btScalar(c.m_appliedImpulse) * c.m_cfm;
		deltaImpulse -= (deltaVel1Dotn + deltaVel2Dotn) * c.m_jacDiagABInv;

		const btScalar sum = btScalar(c.m_appliedImpulse) + deltaImpulse;
		const btScalar appliedImpulse = btClamped(sum, c.m_lowerLimit, c.m_upperLimit);
		deltaImpulse = appliedImpulse - btScalar(c.m_appliedImpulse);
		c.m_appliedImpulse = appliedImpulse;

		body1.internalGetDeltaLinearVelocity() += c.m_contactNormal1 * body1.internalGetInvMass() * deltaImpulse;
		body1.internalGetDeltaAngularVelocity() += c.m_angularComponentA * deltaImpulse;
		body2.internalGetDeltaLinearVelocity() += c.m_contactNormal2 * body2.internalGetInvMass() * deltaImpulse;
		body2.internalGetDeltaAngularVelocity() += c.m_angularComponentB * deltaImpulse;

		const btScalar jacInv = c.m_jacDiagABInv;
		return (btFabs(jacInv) > SIMD_EPSILON) ? (deltaImpulse / jacInv) : btScalar(0);
	}

	// Scalar constraint row resolver (lower limit only — contact constraints)
	// Previously gResolveSingleConstraintRowLowerLimit_avx256.
	static btScalar gResolveSingleConstraintRowLowerLimit(btSolverBody& body1, btSolverBody& body2,
														  const btSolverConstraint& c)
	{
		const btScalar deltaVel1Dotn = c.m_contactNormal1.dot(body1.internalGetDeltaLinearVelocity()) +
									   c.m_relpos1CrossNormal.dot(body1.internalGetDeltaAngularVelocity());
		const btScalar deltaVel2Dotn = c.m_contactNormal2.dot(body2.internalGetDeltaLinearVelocity()) +
									   c.m_relpos2CrossNormal.dot(body2.internalGetDeltaAngularVelocity());

		btScalar deltaImpulse = c.m_rhs - btScalar(c.m_appliedImpulse) * c.m_cfm;
		deltaImpulse -= (deltaVel1Dotn + deltaVel2Dotn) * c.m_jacDiagABInv;

		const btScalar sum = btScalar(c.m_appliedImpulse) + deltaImpulse;
		const btScalar appliedImpulse = btMax(sum, c.m_lowerLimit);
		deltaImpulse = appliedImpulse - btScalar(c.m_appliedImpulse);
		c.m_appliedImpulse = appliedImpulse;

		body1.internalGetDeltaLinearVelocity() += c.m_contactNormal1 * body1.internalGetInvMass() * deltaImpulse;
		body1.internalGetDeltaAngularVelocity() += c.m_angularComponentA * deltaImpulse;
		body2.internalGetDeltaLinearVelocity() += c.m_contactNormal2 * body2.internalGetInvMass() * deltaImpulse;
		body2.internalGetDeltaAngularVelocity() += c.m_angularComponentB * deltaImpulse;

		const btScalar jacInv = c.m_jacDiagABInv;
		return (btFabs(jacInv) > SIMD_EPSILON) ? (deltaImpulse / jacInv) : btScalar(0);
	}

	btSingleConstraintRowSolver GroupConstraintSolver::getResolveSingleConstraintRowGenericAVX()
	{
		return gResolveSingleConstraintRowGeneric;
	}

	btSingleConstraintRowSolver GroupConstraintSolver::getResolveSingleConstraintRowLowerLimitAVX()
	{
		return gResolveSingleConstraintRowLowerLimit;
	}

	GroupConstraintSolver::GroupConstraintSolver()
	{
		m_resolveSingleConstraintRowGeneric = gResolveSingleConstraintRowGeneric;
		m_resolveSingleConstraintRowLowerLimit = gResolveSingleConstraintRowLowerLimit;
	}
} // namespace hdt
