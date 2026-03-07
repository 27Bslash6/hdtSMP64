// Constraint Solver Performance Benchmarks
// Compares Highway batched solver vs scalar solver throughput

#include "hdtHighwaySolver.h"
#include "hdtHighwaySolverBridge.h"
#include "hdtSolverConstraintSoA.h"
#include "hdtSolverTranspose.h"

#include "../include/catch.hpp"
#include "BulletDynamics/ConstraintSolver/btSolverBody.h"
#include "BulletDynamics/ConstraintSolver/btSolverConstraint.h"
#include "LinearMath/btAlignedObjectArray.h"

#include <chrono>
#include <random>
#include <vector>

using namespace hdt;

// Scalar solver baseline (matches Bullet's original implementation)
static btScalar scalarSolveLowerLimit(btSolverBody& bodyA, btSolverBody& bodyB, const btSolverConstraint& c)
{
	btScalar deltaImpulse = c.m_rhs - btScalar(c.m_appliedImpulse) * c.m_cfm;
	const btScalar deltaVel1Dotn = c.m_contactNormal1.dot(bodyA.internalGetDeltaLinearVelocity()) +
								   c.m_relpos1CrossNormal.dot(bodyA.internalGetDeltaAngularVelocity());
	const btScalar deltaVel2Dotn = c.m_contactNormal2.dot(bodyB.internalGetDeltaLinearVelocity()) +
								   c.m_relpos2CrossNormal.dot(bodyB.internalGetDeltaAngularVelocity());
	deltaImpulse -= deltaVel1Dotn * c.m_jacDiagABInv;
	deltaImpulse -= deltaVel2Dotn * c.m_jacDiagABInv;
	const btScalar sum = btScalar(c.m_appliedImpulse) + deltaImpulse;
	if (sum < c.m_lowerLimit) {
		deltaImpulse = c.m_lowerLimit - btScalar(c.m_appliedImpulse);
	}
	bodyA.internalApplyImpulse(c.m_contactNormal1, c.m_angularComponentA, deltaImpulse);
	bodyB.internalApplyImpulse(c.m_contactNormal2, c.m_angularComponentB, deltaImpulse);
	return deltaImpulse * deltaImpulse;
}

static btScalar scalarSolveGeneric(btSolverBody& bodyA, btSolverBody& bodyB, const btSolverConstraint& c)
{
	btScalar deltaImpulse = c.m_rhs - btScalar(c.m_appliedImpulse) * c.m_cfm;
	const btScalar deltaVel1Dotn = c.m_contactNormal1.dot(bodyA.internalGetDeltaLinearVelocity()) +
								   c.m_relpos1CrossNormal.dot(bodyA.internalGetDeltaAngularVelocity());
	const btScalar deltaVel2Dotn = c.m_contactNormal2.dot(bodyB.internalGetDeltaLinearVelocity()) +
								   c.m_relpos2CrossNormal.dot(bodyB.internalGetDeltaAngularVelocity());
	deltaImpulse -= deltaVel1Dotn * c.m_jacDiagABInv;
	deltaImpulse -= deltaVel2Dotn * c.m_jacDiagABInv;
	const btScalar sum = btScalar(c.m_appliedImpulse) + deltaImpulse;
	if (sum < c.m_lowerLimit) {
		deltaImpulse = c.m_lowerLimit - btScalar(c.m_appliedImpulse);
	}
	else if (sum > c.m_upperLimit) {
		deltaImpulse = c.m_upperLimit - btScalar(c.m_appliedImpulse);
	}
	bodyA.internalApplyImpulse(c.m_contactNormal1, c.m_angularComponentA, deltaImpulse);
	bodyB.internalApplyImpulse(c.m_contactNormal2, c.m_angularComponentB, deltaImpulse);
	return deltaImpulse * deltaImpulse;
}

// Helper to create realistic constraint data
static void setupConstraint(btSolverConstraint& c, btSolverBody& bodyA, btSolverBody& bodyB, int bodyAIdx, int bodyBIdx,
							float rhs, float cfm, float lowerLimit, float upperLimit)
{
	c.m_solverBodyIdA = bodyAIdx;
	c.m_solverBodyIdB = bodyBIdx;
	c.m_rhs = rhs;
	c.m_cfm = cfm;
	c.m_lowerLimit = lowerLimit;
	c.m_upperLimit = upperLimit;
	c.m_appliedImpulse = 0.0f;
	c.m_jacDiagABInv = 0.5f;

	// Realistic contact normals and angular components (need to cast away const)
	const_cast<btVector3&>(c.m_contactNormal1).setValue(btScalar(0), btScalar(1), btScalar(0));
	const_cast<btVector3&>(c.m_contactNormal2).setValue(btScalar(0), btScalar(-1), btScalar(0));
	const_cast<btVector3&>(c.m_angularComponentA).setValue(btScalar(0.1), btScalar(0), btScalar(0.1));
	const_cast<btVector3&>(c.m_angularComponentB).setValue(btScalar(-0.1), btScalar(0), btScalar(-0.1));
	const_cast<btVector3&>(c.m_relpos1CrossNormal).setValue(btScalar(0.2), btScalar(0), btScalar(0));
	const_cast<btVector3&>(c.m_relpos2CrossNormal).setValue(btScalar(-0.2), btScalar(0), btScalar(0));
}

TEST_CASE("Constraint solver - Contact batch benchmark (Highway vs Scalar)", "[solver][benchmark][!hide]")
{
	// Simulate realistic batch sizes seen in-game
	const int BATCH_SIZE = 256;	 // Typical contact constraint batch
	const int ITERATIONS = 1000; // Solver iterations across multiple frames

	// Setup bodies
	btAlignedObjectArray<btSolverBody> bodies;
	bodies.resize(BATCH_SIZE * 2);
	for (int i = 0; i < bodies.size(); ++i) {
		bodies[i].internalGetDeltaLinearVelocity().setValue(btScalar(0), btScalar(0), btScalar(0));
		bodies[i].internalGetDeltaAngularVelocity().setValue(btScalar(0), btScalar(0), btScalar(0));
		bodies[i].internalSetInvMass(btVector3(btScalar(1), btScalar(1), btScalar(1)));
	}

	// Setup contact constraints (lower limit only)
	btAlignedObjectArray<btSolverConstraint> constraints;
	constraints.resize(BATCH_SIZE);
	for (int i = 0; i < BATCH_SIZE; ++i) {
		setupConstraint(constraints[i], bodies[i * 2], bodies[i * 2 + 1], i * 2, i * 2 + 1, 0.1f * i, 0.01f, 0.0f,
						FLT_MAX);
	}

	// Setup indices for batch processing
	btAlignedObjectArray<int> indices;
	for (int i = 0; i < BATCH_SIZE; ++i) {
		indices.push_back(i);
	}

	// Warmup
	for (int i = 0; i < 10; ++i) {
		for (int j = 0; j < BATCH_SIZE; ++j) {
			scalarSolveLowerLimit(bodies[constraints[j].m_solverBodyIdA], bodies[constraints[j].m_solverBodyIdB],
								  constraints[j]);
		}
		// Reset
		for (int j = 0; j < bodies.size(); ++j) {
			bodies[j].internalGetDeltaLinearVelocity().setValue(0, 0, 0);
			bodies[j].internalGetDeltaAngularVelocity().setValue(0, 0, 0);
		}
	}

	// Benchmark: Scalar solver
	auto scalarStart = std::chrono::high_resolution_clock::now();
	for (int iter = 0; iter < ITERATIONS; ++iter) {
		for (int i = 0; i < BATCH_SIZE; ++i) {
			scalarSolveLowerLimit(bodies[constraints[i].m_solverBodyIdA], bodies[constraints[i].m_solverBodyIdB],
								  constraints[i]);
		}
	}
	auto scalarEnd = std::chrono::high_resolution_clock::now();
	auto scalarTime = std::chrono::duration_cast<std::chrono::microseconds>(scalarEnd - scalarStart).count();

	// Reset for Highway benchmark
	for (int j = 0; j < bodies.size(); ++j) {
		bodies[j].internalGetDeltaLinearVelocity().setValue(0, 0, 0);
		bodies[j].internalGetDeltaAngularVelocity().setValue(0, 0, 0);
	}
	for (int i = 0; i < BATCH_SIZE; ++i) {
		constraints[i].m_appliedImpulse = 0.0f;
	}

	// Benchmark: Highway batched solver
	auto hwStart = std::chrono::high_resolution_clock::now();
	for (int iter = 0; iter < ITERATIONS; ++iter) {
		highway::resolveContactBatchHighway(
			constraints, bodies, indices, 0, BATCH_SIZE, [&bodies, &constraints](int iCons) -> btScalar {
				return scalarSolveLowerLimit(bodies[constraints[iCons].m_solverBodyIdA],
											 bodies[constraints[iCons].m_solverBodyIdB], constraints[iCons]);
			});
	}
	auto hwEnd = std::chrono::high_resolution_clock::now();
	auto hwTime = std::chrono::duration_cast<std::chrono::microseconds>(hwEnd - hwStart).count();

	float speedup = static_cast<float>(scalarTime) / static_cast<float>(hwTime);

	INFO("Contact Constraints Batch (" << BATCH_SIZE << " constraints, " << ITERATIONS << " iterations)");
	INFO("Scalar:  " << scalarTime << " us (" << (scalarTime / ITERATIONS) << " us/iter)");
	INFO("Highway: " << hwTime << " us (" << (hwTime / ITERATIONS) << " us/iter)");
	INFO("Speedup: " << speedup << "x");

	// Target: 2-3x throughput improvement
	REQUIRE(speedup >= 1.8f); // Allow some margin for CI variance
}

TEST_CASE("Constraint solver - Joint batch benchmark (Highway vs Scalar)", "[solver][benchmark][!hide]")
{
	const int BATCH_SIZE = 128; // Typical joint constraint batch
	const int ITERATIONS = 1000;

	btAlignedObjectArray<btSolverBody> bodies;
	bodies.resize(BATCH_SIZE * 2);
	for (int i = 0; i < bodies.size(); ++i) {
		bodies[i].internalGetDeltaLinearVelocity().setValue(btScalar(0), btScalar(0), btScalar(0));
		bodies[i].internalGetDeltaAngularVelocity().setValue(btScalar(0), btScalar(0), btScalar(0));
		bodies[i].internalSetInvMass(btVector3(btScalar(1), btScalar(1), btScalar(1)));
	}

	// Setup joint constraints (both lower and upper limits)
	btAlignedObjectArray<btSolverConstraint> constraints;
	constraints.resize(BATCH_SIZE);
	for (int i = 0; i < BATCH_SIZE; ++i) {
		setupConstraint(constraints[i], bodies[i * 2], bodies[i * 2 + 1], i * 2, i * 2 + 1, 0.05f * i, 0.01f, -10.0f,
						10.0f);
	}

	btAlignedObjectArray<int> indices;
	for (int i = 0; i < BATCH_SIZE; ++i) {
		indices.push_back(i);
	}

	// Warmup
	for (int i = 0; i < 10; ++i) {
		for (int j = 0; j < BATCH_SIZE; ++j) {
			scalarSolveGeneric(bodies[constraints[j].m_solverBodyIdA], bodies[constraints[j].m_solverBodyIdB],
							   constraints[j]);
		}
		for (int j = 0; j < bodies.size(); ++j) {
			bodies[j].internalGetDeltaLinearVelocity().setValue(0, 0, 0);
			bodies[j].internalGetDeltaAngularVelocity().setValue(0, 0, 0);
		}
	}

	// Benchmark: Scalar solver
	auto scalarStart = std::chrono::high_resolution_clock::now();
	for (int iter = 0; iter < ITERATIONS; ++iter) {
		for (int i = 0; i < BATCH_SIZE; ++i) {
			scalarSolveGeneric(bodies[constraints[i].m_solverBodyIdA], bodies[constraints[i].m_solverBodyIdB],
							   constraints[i]);
		}
	}
	auto scalarEnd = std::chrono::high_resolution_clock::now();
	auto scalarTime = std::chrono::duration_cast<std::chrono::microseconds>(scalarEnd - scalarStart).count();

	// Reset
	for (int j = 0; j < bodies.size(); ++j) {
		bodies[j].internalGetDeltaLinearVelocity().setValue(0, 0, 0);
		bodies[j].internalGetDeltaAngularVelocity().setValue(0, 0, 0);
	}
	for (int i = 0; i < BATCH_SIZE; ++i) {
		constraints[i].m_appliedImpulse = 0.0f;
	}

	// Benchmark: Highway batched solver
	auto hwStart = std::chrono::high_resolution_clock::now();
	for (int iter = 0; iter < ITERATIONS; ++iter) {
		highway::resolveJointBatchHighway(
			constraints, bodies, indices, 0, BATCH_SIZE, [&bodies, &constraints](int iCons) -> btScalar {
				return scalarSolveGeneric(bodies[constraints[iCons].m_solverBodyIdA],
										  bodies[constraints[iCons].m_solverBodyIdB], constraints[iCons]);
			});
	}
	auto hwEnd = std::chrono::high_resolution_clock::now();
	auto hwTime = std::chrono::duration_cast<std::chrono::microseconds>(hwEnd - hwStart).count();

	float speedup = static_cast<float>(scalarTime) / static_cast<float>(hwTime);

	INFO("Joint Constraints Batch (" << BATCH_SIZE << " constraints, " << ITERATIONS << " iterations)");
	INFO("Scalar:  " << scalarTime << " us (" << (scalarTime / ITERATIONS) << " us/iter)");
	INFO("Highway: " << hwTime << " us (" << (hwTime / ITERATIONS) << " us/iter)");
	INFO("Speedup: " << speedup << "x");

	REQUIRE(speedup >= 1.8f);
}

TEST_CASE("Constraint solver - Scaling benchmark (varying batch sizes)", "[solver][benchmark][!hide]")
{
	const std::vector<int> batchSizes = {32, 64, 128, 256, 512, 1024};
	const int ITERATIONS = 500;

	INFO("Scaling analysis: Highway speedup vs batch size");

	for (int BATCH_SIZE : batchSizes) {
		btAlignedObjectArray<btSolverBody> bodies;
		bodies.resize(BATCH_SIZE * 2);
		for (int i = 0; i < bodies.size(); ++i) {
			bodies[i].internalGetDeltaLinearVelocity().setValue(0, 0, 0);
			bodies[i].internalGetDeltaAngularVelocity().setValue(0, 0, 0);
			bodies[i].internalSetInvMass(btVector3(btScalar(1), btScalar(1), btScalar(1)));
		}

		btAlignedObjectArray<btSolverConstraint> constraints;
		constraints.resize(BATCH_SIZE);
		for (int i = 0; i < BATCH_SIZE; ++i) {
			setupConstraint(constraints[i], bodies[i * 2], bodies[i * 2 + 1], i * 2, i * 2 + 1, 0.1f * i, 0.01f, 0.0f,
							FLT_MAX);
		}

		btAlignedObjectArray<int> indices;
		for (int i = 0; i < BATCH_SIZE; ++i) {
			indices.push_back(i);
		}

		// Scalar
		auto scalarStart = std::chrono::high_resolution_clock::now();
		for (int iter = 0; iter < ITERATIONS; ++iter) {
			for (int i = 0; i < BATCH_SIZE; ++i) {
				scalarSolveLowerLimit(bodies[constraints[i].m_solverBodyIdA], bodies[constraints[i].m_solverBodyIdB],
									  constraints[i]);
			}
		}
		auto scalarEnd = std::chrono::high_resolution_clock::now();
		auto scalarTime = std::chrono::duration_cast<std::chrono::microseconds>(scalarEnd - scalarStart).count();

		// Reset
		for (int j = 0; j < bodies.size(); ++j) {
			bodies[j].internalGetDeltaLinearVelocity().setValue(0, 0, 0);
			bodies[j].internalGetDeltaAngularVelocity().setValue(0, 0, 0);
		}
		for (int i = 0; i < BATCH_SIZE; ++i) {
			constraints[i].m_appliedImpulse = 0.0f;
		}

		// Highway
		auto hwStart = std::chrono::high_resolution_clock::now();
		for (int iter = 0; iter < ITERATIONS; ++iter) {
			highway::resolveContactBatchHighway(
				constraints, bodies, indices, 0, BATCH_SIZE, [&bodies, &constraints](int iCons) -> btScalar {
					return scalarSolveLowerLimit(bodies[constraints[iCons].m_solverBodyIdA],
												 bodies[constraints[iCons].m_solverBodyIdB], constraints[iCons]);
				});
		}
		auto hwEnd = std::chrono::high_resolution_clock::now();
		auto hwTime = std::chrono::duration_cast<std::chrono::microseconds>(hwEnd - hwStart).count();

		float speedup = static_cast<float>(scalarTime) / static_cast<float>(hwTime);
		INFO("Batch " << BATCH_SIZE << ": Scalar " << scalarTime << " us, Highway " << hwTime << " us, Speedup "
					  << speedup << "x");
	}

	SUCCEED();
}
