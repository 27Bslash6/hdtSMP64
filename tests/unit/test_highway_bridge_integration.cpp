// Integration test for hdtHighwaySolverBridge full gather/save/solve/delta/scatter pattern
// Tests the ACTUAL code path used in production, not just the raw solver

#include "../../hdtSMP64/BulletDynamics/ConstraintSolver/btSolverBody.h"
#include "../../hdtSMP64/BulletDynamics/ConstraintSolver/btSolverConstraint.h"
#include "../../hdtSMP64/config.h"
#include "../../hdtSMP64/hdtSkinnedMesh/hdtHighwaySolver.h"
#include "../../hdtSMP64/hdtSkinnedMesh/hdtHighwaySolverBridge.h"
#include "../include/catch.hpp"

#include <cfloat>

using namespace hdt;

// Scalar solver implementation for comparison
static btScalar scalarSolveLowerLimit(btSolverBody& bodyA, btSolverBody& bodyB, btSolverConstraint& c)
{
	btScalar deltaImpulse = c.m_rhs - btScalar(c.m_appliedImpulse) * c.m_cfm;

	const btVector3& deltaVelA = bodyA.m_deltaLinearVelocity;
	const btVector3& deltaAngVelA = bodyA.m_deltaAngularVelocity;
	const btVector3& deltaVelB = bodyB.m_deltaLinearVelocity;
	const btVector3& deltaAngVelB = bodyB.m_deltaAngularVelocity;

	btScalar deltaVel1Dotn = c.m_contactNormal1.dot(deltaVelA) + c.m_relpos1CrossNormal.dot(deltaAngVelA);
	btScalar deltaVel2Dotn = c.m_contactNormal2.dot(deltaVelB) + c.m_relpos2CrossNormal.dot(deltaAngVelB);

	deltaImpulse -= (deltaVel1Dotn + deltaVel2Dotn) * c.m_jacDiagABInv;

	const btScalar sum = btScalar(c.m_appliedImpulse) + deltaImpulse;
	if (sum < c.m_lowerLimit) {
		deltaImpulse = c.m_lowerLimit - btScalar(c.m_appliedImpulse);
		c.m_appliedImpulse = c.m_lowerLimit;
	}
	else {
		c.m_appliedImpulse = sum;
	}

	bodyA.m_deltaLinearVelocity += c.m_contactNormal1 * (bodyA.m_invMass.x() * deltaImpulse);
	bodyA.m_deltaAngularVelocity += c.m_angularComponentA * deltaImpulse;
	bodyB.m_deltaLinearVelocity += c.m_contactNormal2 * (bodyB.m_invMass.x() * deltaImpulse);
	bodyB.m_deltaAngularVelocity += c.m_angularComponentB * deltaImpulse;

	return deltaImpulse * (1.0f / c.m_jacDiagABInv);
}

// Helper: Setup a simple contact constraint
static void setupConstraint(btSolverConstraint& c, btSolverBody& bodyA, btSolverBody& bodyB, int idA, int idB,
							btScalar jacDiag, btScalar rhs, btScalar lowerLimit, btScalar upperLimit)
{
	c.m_solverBodyIdA = idA;
	c.m_solverBodyIdB = idB;
	c.m_jacDiagABInv = jacDiag;
	c.m_rhs = rhs;
	c.m_cfm = 0.0f;
	c.m_lowerLimit = lowerLimit;
	c.m_upperLimit = upperLimit;
	c.m_appliedImpulse = 0.0f;

	// Simple constraint along X axis
	c.m_contactNormal1 = btVector3(1, 0, 0);
	c.m_contactNormal2 = btVector3(-1, 0, 0);
	c.m_relpos1CrossNormal = btVector3(0, 0, 0);
	c.m_relpos2CrossNormal = btVector3(0, 0, 0);
	c.m_angularComponentA = btVector3(0, 0, 0);
	c.m_angularComponentB = btVector3(0, 0, 0);
}

TEST_CASE("Highway bridge - matches scalar solver with duplicate body", "[highway][bridge][integration]")
{
	// Enable Highway for this test
	g_highwayConfig.enabled = true;
	g_highwayConfig.batchThreshold = 2; // Use Highway for 2+ constraints

	// Create 3 bodies
	btAlignedObjectArray<btSolverBody> bodiesScalar;
	bodiesScalar.resize(3);
	btAlignedObjectArray<btSolverBody> bodiesHighway;
	bodiesHighway.resize(3);

	for (int i = 0; i < 3; ++i) {
		bodiesScalar[i].m_deltaLinearVelocity.setValue(0, 0, 0);
		bodiesScalar[i].m_deltaAngularVelocity.setValue(0, 0, 0);
		bodiesScalar[i].internalSetInvMass(btVector3(1, 1, 1));

		bodiesHighway[i].m_deltaLinearVelocity.setValue(0, 0, 0);
		bodiesHighway[i].m_deltaAngularVelocity.setValue(0, 0, 0);
		bodiesHighway[i].internalSetInvMass(btVector3(1, 1, 1));
	}

	// Create 2 constraints where body 1 appears twice
	// Constraint 0: body0 <-> body1 (rhs = 10)
	// Constraint 1: body1 <-> body2 (rhs = 20)
	btAlignedObjectArray<btSolverConstraint> constraintsScalar;
	constraintsScalar.resize(2);
	btAlignedObjectArray<btSolverConstraint> constraintsHighway;
	constraintsHighway.resize(2);

	setupConstraint(constraintsScalar[0], bodiesScalar[0], bodiesScalar[1], 0, 1, 1.0f, 10.0f, 0.0f, FLT_MAX);
	setupConstraint(constraintsScalar[1], bodiesScalar[1], bodiesScalar[2], 1, 2, 1.0f, 20.0f, 0.0f, FLT_MAX);

	setupConstraint(constraintsHighway[0], bodiesHighway[0], bodiesHighway[1], 0, 1, 1.0f, 10.0f, 0.0f, FLT_MAX);
	setupConstraint(constraintsHighway[1], bodiesHighway[1], bodiesHighway[2], 1, 2, 1.0f, 20.0f, 0.0f, FLT_MAX);

	// Indices for batch processing
	btAlignedObjectArray<int> indices;
	indices.push_back(0);
	indices.push_back(1);

	// Solve with scalar solver (Gauss-Seidel)
	for (int i = 0; i < 2; ++i) {
		scalarSolveLowerLimit(bodiesScalar[constraintsScalar[i].m_solverBodyIdA],
							  bodiesScalar[constraintsScalar[i].m_solverBodyIdB], constraintsScalar[i]);
	}

	// Solve with Highway bridge (Jacobi-like within batch)
	highway::resolveContactBatchHighway(
		constraintsHighway, bodiesHighway, indices, 0, 2, [&bodiesHighway, &constraintsHighway](int iCons) -> btScalar {
			return scalarSolveLowerLimit(bodiesHighway[constraintsHighway[iCons].m_solverBodyIdA],
										 bodiesHighway[constraintsHighway[iCons].m_solverBodyIdB],
										 constraintsHighway[iCons]);
		});

	// Compare results - should be IDENTICAL for this simple case
	REQUIRE(bodiesHighway[0].m_deltaLinearVelocity.x() == Approx(bodiesScalar[0].m_deltaLinearVelocity.x()));
	REQUIRE(bodiesHighway[1].m_deltaLinearVelocity.x() == Approx(bodiesScalar[1].m_deltaLinearVelocity.x()));
	REQUIRE(bodiesHighway[2].m_deltaLinearVelocity.x() == Approx(bodiesScalar[2].m_deltaLinearVelocity.x()));

	REQUIRE(constraintsHighway[0].m_appliedImpulse == Approx(constraintsScalar[0].m_appliedImpulse));
	REQUIRE(constraintsHighway[1].m_appliedImpulse == Approx(constraintsScalar[1].m_appliedImpulse));
}

TEST_CASE("Highway bridge - multiple iterations with pre-existing velocities", "[highway][bridge][integration]")
{
	g_highwayConfig.enabled = true;
	g_highwayConfig.batchThreshold = 2;

	// Create bodies with NON-ZERO initial velocities (simulates multiple solver iterations)
	btAlignedObjectArray<btSolverBody> bodies;
	bodies.resize(2);
	bodies[0].m_deltaLinearVelocity.setValue(50, 0, 0); // Pre-existing velocity
	bodies[0].m_deltaAngularVelocity.setValue(0, 0, 0);
	bodies[0].internalSetInvMass(btVector3(1, 1, 1));

	bodies[1].m_deltaLinearVelocity.setValue(75, 0, 0); // Pre-existing velocity
	bodies[1].m_deltaAngularVelocity.setValue(0, 0, 0);
	bodies[1].internalSetInvMass(btVector3(1, 1, 1));

	// Single constraint
	btAlignedObjectArray<btSolverConstraint> constraints;
	constraints.resize(1);
	setupConstraint(constraints[0], bodies[0], bodies[1], 0, 1, 1.0f, 10.0f, 0.0f, FLT_MAX);

	btAlignedObjectArray<int> indices;
	indices.push_back(0);

	// Record initial velocities
	btScalar initialVel0 = bodies[0].m_deltaLinearVelocity.x();
	btScalar initialVel1 = bodies[1].m_deltaLinearVelocity.x();

	// Solve
	highway::resolveContactBatchHighway(
		constraints, bodies, indices, 0, 1, [&bodies, &constraints](int iCons) -> btScalar {
			return scalarSolveLowerLimit(bodies[constraints[iCons].m_solverBodyIdA],
										 bodies[constraints[iCons].m_solverBodyIdB], constraints[iCons]);
		});

	// Verify velocities changed (impulse was applied)
	REQUIRE(bodies[0].m_deltaLinearVelocity.x() != initialVel0);
	REQUIRE(bodies[1].m_deltaLinearVelocity.x() != initialVel1);

	// Verify impulse was applied
	REQUIRE(constraints[0].m_appliedImpulse > 0.0f);

	// Verify velocities are reasonable (not NaN or infinity)
	REQUIRE(std::isfinite(bodies[0].m_deltaLinearVelocity.x()));
	REQUIRE(std::isfinite(bodies[1].m_deltaLinearVelocity.x()));
}
