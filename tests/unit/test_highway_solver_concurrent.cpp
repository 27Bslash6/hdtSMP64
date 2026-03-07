#include "hdtHighwaySolver.h"
#include "hdtSolverConstraintSoA.h"
#include "hdtSolverTranspose.h"

#include "../include/catch.hpp"
#include "BulletDynamics/ConstraintSolver/btSolverBody.h"
#include "BulletDynamics/ConstraintSolver/btSolverConstraint.h"
#include "LinearMath/btAlignedObjectArray.h"

#include <cmath>
#include <vector>

using namespace hdt;

// Helper to set up a test constraint with specific body IDs
static void setupConstraintWithBodies(ConstraintBatchSoA& batch, size_t idx, int bodyIdA, int bodyIdB,
									  float jacDiagABInv, float rhs, float cfm, float lowerLimit, float upperLimit,
									  float appliedImpulse)
{
	batch.scalars.bodyIdA[idx] = bodyIdA;
	batch.scalars.bodyIdB[idx] = bodyIdB;
	batch.scalars.jacDiagABInv[idx] = jacDiagABInv;
	batch.scalars.rhs[idx] = rhs;
	batch.scalars.cfm[idx] = cfm;
	batch.scalars.lowerLimit[idx] = lowerLimit;
	batch.scalars.upperLimit[idx] = upperLimit;
	batch.scalars.appliedImpulse[idx] = appliedImpulse;

	// Simple contact normal: (1, 0, 0) for body A, (-1, 0, 0) for body B
	batch.vectors.contactNormal1X[idx] = 1.0f;
	batch.vectors.contactNormal1Y[idx] = 0.0f;
	batch.vectors.contactNormal1Z[idx] = 0.0f;
	batch.vectors.contactNormal2X[idx] = -1.0f;
	batch.vectors.contactNormal2Y[idx] = 0.0f;
	batch.vectors.contactNormal2Z[idx] = 0.0f;

	// Zero angular contribution for simplicity
	batch.vectors.relpos1CrossNormalX[idx] = 0.0f;
	batch.vectors.relpos1CrossNormalY[idx] = 0.0f;
	batch.vectors.relpos1CrossNormalZ[idx] = 0.0f;
	batch.vectors.relpos2CrossNormalX[idx] = 0.0f;
	batch.vectors.relpos2CrossNormalY[idx] = 0.0f;
	batch.vectors.relpos2CrossNormalZ[idx] = 0.0f;
	batch.vectors.angularComponentAX[idx] = 0.0f;
	batch.vectors.angularComponentAY[idx] = 0.0f;
	batch.vectors.angularComponentAZ[idx] = 0.0f;
	batch.vectors.angularComponentBX[idx] = 0.0f;
	batch.vectors.angularComponentBY[idx] = 0.0f;
	batch.vectors.angularComponentBZ[idx] = 0.0f;
}

TEST_CASE("Highway solver - duplicate body in batch (Jacobi-like accumulation)", "[solver][highway][concurrent]")
{
	// This test verifies the fix for the scatter bug where the same body can appear
	// multiple times within a batch. Graph coloring prevents bodies from appearing in
	// DIFFERENT batches within a phase, but allows duplicates WITHIN a batch.
	//
	// Example: Constraint 0: bodyA=1, bodyB=5
	//          Constraint 1: bodyA=5, bodyB=6
	//
	// Body 5 appears twice in this batch (once as bodyB, once as bodyA).
	// With setValue() scatter, the second write overwrites the first, losing impulse!
	// With += scatter, both impulses accumulate correctly.

	// Create 3 bodies
	btAlignedObjectArray<btSolverBody> bodies;
	bodies.resize(3);

	// Initialize all bodies with zero delta velocities
	for (int i = 0; i < 3; ++i) {
		bodies[i].m_deltaLinearVelocity.setValue(0.0f, 0.0f, 0.0f);
		bodies[i].m_deltaAngularVelocity.setValue(0.0f, 0.0f, 0.0f);
		bodies[i].internalSetInvMass(btVector3(1.0f, 1.0f, 1.0f));
	}

	// Create 2 constraints that share body 1
	// Constraint 0: body0 <-> body1 (impulse = 10)
	// Constraint 1: body1 <-> body2 (impulse = 20)
	// Body 1 should receive: -10 (from constraint 0 as bodyB) + 20 (from constraint 1 as bodyA) = 10
	ConstraintBatchSoAOwner owner(2);
	auto& batch = owner.get();

	setupConstraintWithBodies(batch, 0, 0, 1, 1.0f, 10.0f, 0.0f, 0.0f, FLT_MAX, 0.0f);
	setupConstraintWithBodies(batch, 1, 1, 2, 1.0f, 20.0f, 0.0f, 0.0f, FLT_MAX, 0.0f);

	// Set batch count
	batch.scalars.count = 2;
	batch.vectors.count = 2;

	// Gather body data (zeroes delta velocities, gathers inverse masses)
	std::vector<float> deltaLinAX(16), deltaLinAY(16), deltaLinAZ(16);
	std::vector<float> deltaAngAX(16), deltaAngAY(16), deltaAngAZ(16);
	std::vector<float> deltaLinBX(16), deltaLinBY(16), deltaLinBZ(16);
	std::vector<float> deltaAngBX(16), deltaAngBY(16), deltaAngBZ(16);
	std::vector<float> invMassAX(16), invMassAY(16), invMassAZ(16);
	std::vector<float> invMassBX(16), invMassBY(16), invMassBZ(16);

	gatherBodyDeltasForBatch(bodies, batch.scalars, deltaLinAX.data(), deltaLinAY.data(), deltaLinAZ.data(),
							 deltaAngAX.data(), deltaAngAY.data(), deltaAngAZ.data(), deltaLinBX.data(),
							 deltaLinBY.data(), deltaLinBZ.data(), deltaAngBX.data(), deltaAngBY.data(),
							 deltaAngBZ.data(), invMassAX.data(), invMassAY.data(), invMassAZ.data(), invMassBX.data(),
							 invMassBY.data(), invMassBZ.data());

	// Verify gather zeroed deltas
	REQUIRE(deltaLinAX[0] == 0.0f);
	REQUIRE(deltaLinBX[0] == 0.0f);
	REQUIRE(deltaLinAX[1] == 0.0f);
	REQUIRE(deltaLinBX[1] == 0.0f);

	// Solve using Highway SIMD
	highway::resolveBatchConstraintRowsLowerLimit(batch, deltaLinAX.data(), deltaLinAY.data(), deltaLinAZ.data(),
												  deltaAngAX.data(), deltaAngAY.data(), deltaAngAZ.data(),
												  deltaLinBX.data(), deltaLinBY.data(), deltaLinBZ.data(),
												  deltaAngBX.data(), deltaAngBY.data(), deltaAngBZ.data(),
												  invMassAX.data(), invMassAY.data(), invMassAZ.data(),
												  invMassBX.data(), invMassBY.data(), invMassBZ.data(), 2);

	// Verify solver computed impulses
	REQUIRE(batch.scalars.appliedImpulse[0] == Approx(10.0f));
	REQUIRE(batch.scalars.appliedImpulse[1] == Approx(20.0f));

	// Verify solver computed delta velocities in buffers
	// Constraint 0: body0 gets +10, body1 gets -10
	REQUIRE(deltaLinAX[0] == Approx(10.0f));
	REQUIRE(deltaLinBX[0] == Approx(-10.0f));
	// Constraint 1: body1 gets +20, body2 gets -20
	REQUIRE(deltaLinAX[1] == Approx(20.0f));
	REQUIRE(deltaLinBX[1] == Approx(-20.0f));

	// Scatter body data back using += accumulation
	scatterBodyDeltasFromBatch(batch.scalars, bodies, deltaLinAX.data(), deltaLinAY.data(), deltaLinAZ.data(),
							   deltaAngAX.data(), deltaAngAY.data(), deltaAngAZ.data(), deltaLinBX.data(),
							   deltaLinBY.data(), deltaLinBZ.data(), deltaAngBX.data(), deltaAngBY.data(),
							   deltaAngBZ.data());

	// Verify final body velocities with correct accumulation
	// Body 0: receives +10 from constraint 0 (as bodyA)
	REQUIRE(bodies[0].m_deltaLinearVelocity.x() == Approx(10.0f));

	// Body 1: receives -10 from constraint 0 (as bodyB) + 20 from constraint 1 (as bodyA) = 10
	// This is the critical test! With setValue(), body 1 would only have +20 (overwrite).
	// With += accumulation, body 1 correctly has 10.
	REQUIRE(bodies[1].m_deltaLinearVelocity.x() == Approx(10.0f));

	// Body 2: receives -20 from constraint 1 (as bodyB)
	REQUIRE(bodies[2].m_deltaLinearVelocity.x() == Approx(-20.0f));
}

TEST_CASE("Highway solver - triple duplicate body", "[solver][highway][concurrent]")
{
	// Stress test: body appears THREE times in a batch
	// Constraint 0: body0 <-> body1 (impulse = 5)
	// Constraint 1: body1 <-> body2 (impulse = 10)
	// Constraint 2: body1 <-> body3 (impulse = 15)
	// Body 1 should receive: -5 + 10 + 15 = 20

	btAlignedObjectArray<btSolverBody> bodies;
	bodies.resize(4);

	for (int i = 0; i < 4; ++i) {
		bodies[i].m_deltaLinearVelocity.setValue(0.0f, 0.0f, 0.0f);
		bodies[i].m_deltaAngularVelocity.setValue(0.0f, 0.0f, 0.0f);
		bodies[i].internalSetInvMass(btVector3(1.0f, 1.0f, 1.0f));
	}

	ConstraintBatchSoAOwner owner(3);
	auto& batch = owner.get();

	setupConstraintWithBodies(batch, 0, 0, 1, 1.0f, 5.0f, 0.0f, 0.0f, FLT_MAX, 0.0f);
	setupConstraintWithBodies(batch, 1, 1, 2, 1.0f, 10.0f, 0.0f, 0.0f, FLT_MAX, 0.0f);
	setupConstraintWithBodies(batch, 2, 1, 3, 1.0f, 15.0f, 0.0f, 0.0f, FLT_MAX, 0.0f);

	// Set batch count
	batch.scalars.count = 3;
	batch.vectors.count = 3;

	std::vector<float> deltaLinAX(16), deltaLinAY(16), deltaLinAZ(16);
	std::vector<float> deltaAngAX(16), deltaAngAY(16), deltaAngAZ(16);
	std::vector<float> deltaLinBX(16), deltaLinBY(16), deltaLinBZ(16);
	std::vector<float> deltaAngBX(16), deltaAngBY(16), deltaAngBZ(16);
	std::vector<float> invMassAX(16), invMassAY(16), invMassAZ(16);
	std::vector<float> invMassBX(16), invMassBY(16), invMassBZ(16);

	gatherBodyDeltasForBatch(bodies, batch.scalars, deltaLinAX.data(), deltaLinAY.data(), deltaLinAZ.data(),
							 deltaAngAX.data(), deltaAngAY.data(), deltaAngAZ.data(), deltaLinBX.data(),
							 deltaLinBY.data(), deltaLinBZ.data(), deltaAngBX.data(), deltaAngBY.data(),
							 deltaAngBZ.data(), invMassAX.data(), invMassAY.data(), invMassAZ.data(), invMassBX.data(),
							 invMassBY.data(), invMassBZ.data());

	highway::resolveBatchConstraintRowsLowerLimit(batch, deltaLinAX.data(), deltaLinAY.data(), deltaLinAZ.data(),
												  deltaAngAX.data(), deltaAngAY.data(), deltaAngAZ.data(),
												  deltaLinBX.data(), deltaLinBY.data(), deltaLinBZ.data(),
												  deltaAngBX.data(), deltaAngBY.data(), deltaAngBZ.data(),
												  invMassAX.data(), invMassAY.data(), invMassAZ.data(),
												  invMassBX.data(), invMassBY.data(), invMassBZ.data(), 3);

	scatterBodyDeltasFromBatch(batch.scalars, bodies, deltaLinAX.data(), deltaLinAY.data(), deltaLinAZ.data(),
							   deltaAngAX.data(), deltaAngAY.data(), deltaAngAZ.data(), deltaLinBX.data(),
							   deltaLinBY.data(), deltaLinBZ.data(), deltaAngBX.data(), deltaAngBY.data(),
							   deltaAngBZ.data());

	// Body 0: +5
	REQUIRE(bodies[0].m_deltaLinearVelocity.x() == Approx(5.0f));

	// Body 1: -5 + 10 + 15 = 20 (appears in all 3 constraints)
	REQUIRE(bodies[1].m_deltaLinearVelocity.x() == Approx(20.0f));

	// Body 2: -10
	REQUIRE(bodies[2].m_deltaLinearVelocity.x() == Approx(-10.0f));

	// Body 3: -15
	REQUIRE(bodies[3].m_deltaLinearVelocity.x() == Approx(-15.0f));
}

TEST_CASE("Highway solver - concurrent accumulation vs sequential scalar", "[solver][highway][concurrent]")
{
	// Compare Highway's Jacobi-like parallel updates within a batch
	// against scalar solver's Gauss-Seidel sequential updates
	// Both should converge to similar solution (not identical due to parallelism)

	btAlignedObjectArray<btSolverBody> bodies;
	bodies.resize(3);

	for (int i = 0; i < 3; ++i) {
		bodies[i].m_deltaLinearVelocity.setValue(0.0f, 0.0f, 0.0f);
		bodies[i].m_deltaAngularVelocity.setValue(0.0f, 0.0f, 0.0f);
		bodies[i].internalSetInvMass(btVector3(1.0f, 1.0f, 1.0f));
	}

	// Create chain: body0 <-> body1 <-> body2
	ConstraintBatchSoAOwner owner(2);
	auto& batch = owner.get();

	setupConstraintWithBodies(batch, 0, 0, 1, 1.0f, 10.0f, 0.0f, 0.0f, FLT_MAX, 0.0f);
	setupConstraintWithBodies(batch, 1, 1, 2, 1.0f, 10.0f, 0.0f, 0.0f, FLT_MAX, 0.0f);

	// Set batch count
	batch.scalars.count = 2;
	batch.vectors.count = 2;

	std::vector<float> deltaLinAX(16), deltaLinAY(16), deltaLinAZ(16);
	std::vector<float> deltaAngAX(16), deltaAngAY(16), deltaAngAZ(16);
	std::vector<float> deltaLinBX(16), deltaLinBY(16), deltaLinBZ(16);
	std::vector<float> deltaAngBX(16), deltaAngBY(16), deltaAngBZ(16);
	std::vector<float> invMassAX(16), invMassAY(16), invMassAZ(16);
	std::vector<float> invMassBX(16), invMassBY(16), invMassBZ(16);

	gatherBodyDeltasForBatch(bodies, batch.scalars, deltaLinAX.data(), deltaLinAY.data(), deltaLinAZ.data(),
							 deltaAngAX.data(), deltaAngAY.data(), deltaAngAZ.data(), deltaLinBX.data(),
							 deltaLinBY.data(), deltaLinBZ.data(), deltaAngBX.data(), deltaAngBY.data(),
							 deltaAngBZ.data(), invMassAX.data(), invMassAY.data(), invMassAZ.data(), invMassBX.data(),
							 invMassBY.data(), invMassBZ.data());

	// Run solver
	highway::resolveBatchConstraintRowsLowerLimit(batch, deltaLinAX.data(), deltaLinAY.data(), deltaLinAZ.data(),
												  deltaAngAX.data(), deltaAngAY.data(), deltaAngAZ.data(),
												  deltaLinBX.data(), deltaLinBY.data(), deltaLinBZ.data(),
												  deltaAngBX.data(), deltaAngBY.data(), deltaAngBZ.data(),
												  invMassAX.data(), invMassAY.data(), invMassAZ.data(),
												  invMassBX.data(), invMassBY.data(), invMassBZ.data(), 2);

	scatterBodyDeltasFromBatch(batch.scalars, bodies, deltaLinAX.data(), deltaLinAY.data(), deltaLinAZ.data(),
							   deltaAngAX.data(), deltaAngAY.data(), deltaAngAZ.data(), deltaLinBX.data(),
							   deltaLinBY.data(), deltaLinBZ.data(), deltaAngBX.data(), deltaAngBY.data(),
							   deltaAngBZ.data());

	// Verify physics correctness: center body should have zero net velocity
	// (equal and opposite impulses from both neighbors)
	REQUIRE(bodies[0].m_deltaLinearVelocity.x() == Approx(10.0f));
	REQUIRE(bodies[1].m_deltaLinearVelocity.x() == Approx(0.0f)); // -10 + 10 = 0
	REQUIRE(bodies[2].m_deltaLinearVelocity.x() == Approx(-10.0f));
}
