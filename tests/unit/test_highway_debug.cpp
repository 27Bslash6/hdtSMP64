// Debug test to trace exactly what Highway solver bridge does

#include "../../hdtSMP64/BulletDynamics/ConstraintSolver/btSolverBody.h"
#include "../../hdtSMP64/BulletDynamics/ConstraintSolver/btSolverConstraint.h"
#include "../../hdtSMP64/config.h"
#include "../../hdtSMP64/hdtSkinnedMesh/hdtHighwaySolverBridge.h"
#include "../../hdtSMP64/hdtSkinnedMesh/hdtSolverTranspose.h"
#include "../include/catch.hpp"

#include <cfloat>
#include <iostream>

using namespace hdt;

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

TEST_CASE("Debug Highway bridge - trace gather/solve/scatter", "[debug][highway]")
{
	// Create 3 bodies
	btAlignedObjectArray<btSolverBody> bodies;
	bodies.resize(3);
	for (int i = 0; i < 3; ++i) {
		bodies[i].m_deltaLinearVelocity.setValue(0, 0, 0);
		bodies[i].m_deltaAngularVelocity.setValue(0, 0, 0);
		bodies[i].internalSetInvMass(btVector3(1, 1, 1));
	}

	// Create 2 constraints where body 1 appears twice
	btAlignedObjectArray<btSolverConstraint> constraints;
	constraints.resize(2);
	setupConstraint(constraints[0], bodies[0], bodies[1], 0, 1, 1.0f, 10.0f, 0.0f, FLT_MAX);
	setupConstraint(constraints[1], bodies[1], bodies[2], 1, 2, 1.0f, 20.0f, 0.0f, FLT_MAX);

	std::cout << "\n=== BEFORE SOLVE ===\n";
	std::cout << "Body 0 deltaVel: " << bodies[0].m_deltaLinearVelocity.x() << "\n";
	std::cout << "Body 1 deltaVel: " << bodies[1].m_deltaLinearVelocity.x() << "\n";
	std::cout << "Body 2 deltaVel: " << bodies[2].m_deltaLinearVelocity.x() << "\n";

	// Manually execute the bridge pattern with logging
	g_highwayConfig.enabled = true;
	g_highwayConfig.batchThreshold = 2;

	btAlignedObjectArray<int> indices;
	indices.push_back(0);
	indices.push_back(1);

	auto& scratch = highway::getScratchBuffers();
	scratch.ensureCapacity(2);

	// Transpose constraints
	transposeConstraintsToSoA(constraints, indices, 0, 2, scratch.constraintBatch.get());

	std::cout << "\n=== AFTER TRANSPOSE ===\n";
	std::cout << "Constraint 0: bodyIdA=" << scratch.constraintBatch.scalars().bodyIdA[0]
			  << " bodyIdB=" << scratch.constraintBatch.scalars().bodyIdB[0] << "\n";
	std::cout << "Constraint 1: bodyIdA=" << scratch.constraintBatch.scalars().bodyIdA[1]
			  << " bodyIdB=" << scratch.constraintBatch.scalars().bodyIdB[1] << "\n";

	// Gather
	gatherBodyDeltasForBatch(bodies, scratch.constraintBatch.scalars(), scratch.deltaLinAX.data(),
							 scratch.deltaLinAY.data(), scratch.deltaLinAZ.data(), scratch.deltaAngAX.data(),
							 scratch.deltaAngAY.data(), scratch.deltaAngAZ.data(), scratch.deltaLinBX.data(),
							 scratch.deltaLinBY.data(), scratch.deltaLinBZ.data(), scratch.deltaAngBX.data(),
							 scratch.deltaAngBY.data(), scratch.deltaAngBZ.data(), scratch.invMassAX.data(),
							 scratch.invMassAY.data(), scratch.invMassAZ.data(), scratch.invMassBX.data(),
							 scratch.invMassBY.data(), scratch.invMassBZ.data());

	std::cout << "\n=== AFTER GATHER ===\n";
	std::cout << "Constraint 0: deltaLinAX=" << scratch.deltaLinAX[0] << " deltaLinBX=" << scratch.deltaLinBX[0]
			  << "\n";
	std::cout << "Constraint 1: deltaLinAX=" << scratch.deltaLinAX[1] << " deltaLinBX=" << scratch.deltaLinBX[1]
			  << "\n";

	// Save
	std::memcpy(scratch.savedLinAX.data(), scratch.deltaLinAX.data(), 2 * sizeof(float));
	std::memcpy(scratch.savedLinBX.data(), scratch.deltaLinBX.data(), 2 * sizeof(float));

	std::cout << "\n=== SAVED VELOCITIES ===\n";
	std::cout << "savedLinAX[0]=" << scratch.savedLinAX[0] << " savedLinBX[0]=" << scratch.savedLinBX[0] << "\n";
	std::cout << "savedLinAX[1]=" << scratch.savedLinAX[1] << " savedLinBX[1]=" << scratch.savedLinBX[1] << "\n";

	// Solve
	highway::resolveBatchConstraintRowsLowerLimit(
		scratch.constraintBatch.get(), scratch.deltaLinAX.data(), scratch.deltaLinAY.data(), scratch.deltaLinAZ.data(),
		scratch.deltaAngAX.data(), scratch.deltaAngAY.data(), scratch.deltaAngAZ.data(), scratch.deltaLinBX.data(),
		scratch.deltaLinBY.data(), scratch.deltaLinBZ.data(), scratch.deltaAngBX.data(), scratch.deltaAngBY.data(),
		scratch.deltaAngBZ.data(), scratch.invMassAX.data(), scratch.invMassAY.data(), scratch.invMassAZ.data(),
		scratch.invMassBX.data(), scratch.invMassBY.data(), scratch.invMassBZ.data(), 2);

	std::cout << "\n=== AFTER SOLVE ===\n";
	std::cout << "Constraint 0: deltaLinAX=" << scratch.deltaLinAX[0] << " deltaLinBX=" << scratch.deltaLinBX[0]
			  << "\n";
	std::cout << "Constraint 1: deltaLinAX=" << scratch.deltaLinAX[1] << " deltaLinBX=" << scratch.deltaLinBX[1]
			  << "\n";
	std::cout << "appliedImpulse[0]=" << scratch.constraintBatch.scalars().appliedImpulse[0] << "\n";
	std::cout << "appliedImpulse[1]=" << scratch.constraintBatch.scalars().appliedImpulse[1] << "\n";

	// Compute deltas
	for (int i = 0; i < 2; ++i) {
		scratch.deltaLinAX[i] -= scratch.savedLinAX[i];
		scratch.deltaLinBX[i] -= scratch.savedLinBX[i];
	}

	std::cout << "\n=== AFTER DELTA COMPUTATION ===\n";
	std::cout << "Constraint 0: deltaLinAX=" << scratch.deltaLinAX[0] << " deltaLinBX=" << scratch.deltaLinBX[0]
			  << "\n";
	std::cout << "Constraint 1: deltaLinAX=" << scratch.deltaLinAX[1] << " deltaLinBX=" << scratch.deltaLinBX[1]
			  << "\n";

	// Scatter
	scatterBodyDeltasFromBatch(scratch.constraintBatch.scalars(), bodies, scratch.deltaLinAX.data(),
							   scratch.deltaLinAY.data(), scratch.deltaLinAZ.data(), scratch.deltaAngAX.data(),
							   scratch.deltaAngAY.data(), scratch.deltaAngAZ.data(), scratch.deltaLinBX.data(),
							   scratch.deltaLinBY.data(), scratch.deltaLinBZ.data(), scratch.deltaAngBX.data(),
							   scratch.deltaAngBY.data(), scratch.deltaAngBZ.data());

	std::cout << "\n=== AFTER SCATTER ===\n";
	std::cout << "Body 0 deltaVel: " << bodies[0].m_deltaLinearVelocity.x() << "\n";
	std::cout << "Body 1 deltaVel: " << bodies[1].m_deltaLinearVelocity.x() << " (EXPECTED: -10 + 20 = 10)\n";
	std::cout << "Body 2 deltaVel: " << bodies[2].m_deltaLinearVelocity.x() << "\n";

	// Check if body 1 has correct accumulated velocity
	REQUIRE(bodies[1].m_deltaLinearVelocity.x() == Approx(10.0f));
}
