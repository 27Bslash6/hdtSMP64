#include "hdtSkinnedMeshWorld.h"

#include "hdtBoneScaleConstraint.h"
#include "hdtDispatcher.h"
#include "hdtEnkiTSScheduler.h"
#include "hdtSimulationIslandManager.h"
#include "hdtSkinnedMeshAlgorithm.h"
#include "hdtSkyrimPhysicsWorld.h"
#include "hdtSkyrimSystem.h"

#include "../hdtPrefix.h"
#include "../hdtTracy.h"

#include <exception>
#include <random>

namespace hdt
{
	// Static frame counter for dirty flag optimization
	uint32_t SkinnedMeshWorld::s_currentFrame = 0;

	void SkinnedMeshWorld::incrementFrame()
	{
		// Reset counter before overflow to prevent dirty flag comparison bugs.
		// When s_currentFrame wraps to 0, it would match uninitialized m_lastUpdateFrame (= 0),
		// causing objects to incorrectly skip updates. Reset to 1 to avoid this.
		// Threshold of 0xC0000000 (~3 billion) gives ~1.6 years buffer before overflow at 60fps.
		constexpr uint32_t RESET_THRESHOLD = 0xC0000000;
		if (s_currentFrame >= RESET_THRESHOLD) {
			s_currentFrame = 1;
		}
		else {
			++s_currentFrame;
		}
		// Keep logging namespace in sync for log prefix
		hdt::logging::currentFrameNumber.store(s_currentFrame, std::memory_order_relaxed);
	}

	SkinnedMeshWorld::SkinnedMeshWorld()
		: btDiscreteDynamicsWorldMt(nullptr, nullptr, m_solverPool, &m_constraintSolver, nullptr)
	{
		// Use enkiTS for Bullet's task scheduler - replaces PPL to avoid thread pool over-subscription
		btSetTaskScheduler(btGetEnkiTSTaskScheduler());

		// Enable nested parallelism for Mt solver - allows btParallelFor inside convertJoints
		// even when outer threading is running. enkiTS handles nested parallelism well.
		btSequentialImpulseConstraintSolverMt::s_allowNestedParallelForLoops = true;

		m_windSpeed = _mm_setzero_ps();

		auto collisionConfiguration = new btDefaultCollisionConfiguration;
		auto collisionDispatcher = new CollisionDispatcher(collisionConfiguration);
		SkinnedMeshAlgorithm::registerAlgorithm(collisionDispatcher);
		m_dispatcher1 = collisionDispatcher;

		auto broadphase = new btDbvtBroadphase();
		m_broadphasePairCache = broadphase;
		m_solverPool = new btConstraintSolverPoolMt(BT_MAX_THREAD_COUNT);

		// m_islandManager->~btSimulationIslandManager();
		// new (m_islandManager) SimulationIslandManager();
	}

	SkinnedMeshWorld::~SkinnedMeshWorld()
	{
		for (auto system : m_systems) {
			for (int i = 0; i < system->m_meshes.size(); ++i)
				removeCollisionObject(system->m_meshes[i]);
			for (int i = 0; i < system->m_constraints.size(); ++i)
				removeConstraint(system->m_constraints[i]->m_constraint);
			for (int i = 0; i < system->m_bones.size(); ++i)
				removeRigidBody(&system->m_bones[i]->m_rig);

			for (auto i : system->m_constraintGroups)
				for (auto j : i->m_constraints)
					removeConstraint(j->m_constraint);
		}

		m_systems.clear();
	}

	void SkinnedMeshWorld::addSkinnedMeshSystem(SkinnedMeshSystem* system)
	{
		if (std::find(m_systems.begin(), m_systems.end(), system) != m_systems.end())
			return;

		m_systems.push_back(system);

		// Add collision objects for all meshes
		// No pointer refresh needed - offsets are computed on-the-fly
		for (int i = 0; i < system->m_meshes.size(); ++i) {
			SkinnedMeshBody* mesh = system->m_meshes[i];
			addCollisionObject(mesh, 1, 1);
		}

		for (int i = 0; i < system->m_bones.size(); ++i) {
			system->m_bones[i]->m_rig.setActivationState(DISABLE_DEACTIVATION);
			addRigidBody(&system->m_bones[i]->m_rig, 0, 0);
		}

		for (auto i : system->m_constraintGroups)
			for (auto j : i->m_constraints)
				addConstraint(j->m_constraint, true);

		for (int i = 0; i < system->m_constraints.size(); ++i)
			addConstraint(system->m_constraints[i]->m_constraint, true);

		// -10 allows RESET_PHYSICS down the calls. But equality with a float?...
		system->readTransform(RESET_PHYSICS);

		system->m_world = this;
	}

	void SkinnedMeshWorld::removeSkinnedMeshSystem(SkinnedMeshSystem* system)
	{
		auto idx = std::find(m_systems.begin(), m_systems.end(), system);
		if (idx == m_systems.end())
			return;

		for (auto i : system->m_constraintGroups)
			for (auto j : i->m_constraints)
				removeConstraint(j->m_constraint);

		for (int i = 0; i < system->m_meshes.size(); ++i)
			removeCollisionObject(system->m_meshes[i]);
		for (int i = 0; i < system->m_constraints.size(); ++i)
			removeConstraint(system->m_constraints[i]->m_constraint);
		for (int i = 0; i < system->m_bones.size(); ++i)
			removeRigidBody(&system->m_bones[i]->m_rig);

		std::swap(*idx, m_systems.back());
		m_systems.pop_back();

		system->m_world = nullptr;
	}

	void SkinnedMeshWorld::reregisterAllBodies()
	{
		// Log at VERY START to see how many systems we have
		_VMESSAGE("reregisterAllBodies: ENTERING with %zu systems", m_systems.size());

		// Remove all bodies from broadphase, then re-add them
		// This forces a complete rebuild of collision state
		for (auto& system : m_systems) {
			// Remove meshes and bones from collision world
			for (int i = 0; i < system->m_meshes.size(); ++i) {
				SkinnedMeshBody* mesh = system->m_meshes[i];
				if (mesh->getBroadphaseHandle()) {
					getBroadphase()->destroyProxy(mesh->getBroadphaseHandle(), m_dispatcher1);
					mesh->setBroadphaseHandle(nullptr);
				}
			}
			for (int i = 0; i < system->m_bones.size(); ++i) {
				btRigidBody* rig = &system->m_bones[i]->m_rig;
				if (rig->getBroadphaseHandle()) {
					getBroadphase()->destroyProxy(rig->getBroadphaseHandle(), m_dispatcher1);
					rig->setBroadphaseHandle(nullptr);
				}
			}
		}

		// Re-add all bodies to broadphase with fresh proxies
		// No pointer refresh needed - offsets are computed on-the-fly
		for (auto& system : m_systems) {
			for (int i = 0; i < system->m_meshes.size(); ++i) {
				SkinnedMeshBody* mesh = system->m_meshes[i];
				btVector3 minAabb, maxAabb;
				mesh->getCollisionShape()->getAabb(mesh->getWorldTransform(), minAabb, maxAabb);
				btBroadphaseProxy* proxy = getBroadphase()->createProxy(
					minAabb, maxAabb, mesh->getCollisionShape()->getShapeType(), mesh, 1, 1, m_dispatcher1);
				mesh->setBroadphaseHandle(proxy);
			}
			for (int i = 0; i < system->m_bones.size(); ++i) {
				btRigidBody* rig = &system->m_bones[i]->m_rig;
				btVector3 minAabb, maxAabb;
				rig->getCollisionShape()->getAabb(rig->getWorldTransform(), minAabb, maxAabb);
				btBroadphaseProxy* proxy = getBroadphase()->createProxy(
					minAabb, maxAabb, rig->getCollisionShape()->getShapeType(), rig, 0, 0, m_dispatcher1);
				rig->setBroadphaseHandle(proxy);
			}
		}
	}

	int SkinnedMeshWorld::stepSimulation(btScalar remainingTimeStep, int maxSubSteps, btScalar fixedTimeStep)
	{
		HDT_ZONE_SCOPED_N("StepSimulation");
		incrementFrame(); // Advance frame counter for dirty flag optimization
		applyGravity();
		if (hdt::SkyrimPhysicsWorld::get()->m_enableWind)
			applyWind();

		while (remainingTimeStep > fixedTimeStep) {
			internalSingleStepSimulation(fixedTimeStep);
			remainingTimeStep -= fixedTimeStep;
		}
		// For the sake of the bullet library, we don't manage a step that would be lower than a 300Hz frame.
		// Review this when (screens / Skyrim) will allow 300Hz+.
		constexpr auto minPossiblePeriod = 1.0f / 300.0f;
		if (remainingTimeStep > minPossiblePeriod)
			internalSingleStepSimulation(remainingTimeStep);
		clearForces();

		_bodies.clear();
		_shapes.clear();

		return 0;
	}

	void SkinnedMeshWorld::performDiscreteCollisionDetection()
	{
		HDT_ZONE_SCOPED_N("CollisionDetection");
		// NOTE: SystemsInternalUpdate moved to internalSingleStepSimulation for parallelization
		// It now runs in parallel with GPU sync and predict motion
		{
			HDT_ZONE_SCOPED_N("BulletCollisionDetection");
			btDiscreteDynamicsWorldMt::performDiscreteCollisionDetection();
		}
	}

	void SkinnedMeshWorld::applyGravity()
	{
		for (auto& i : m_systems) {
			for (auto& j : i->m_bones) {
				auto body = &j->m_rig;
				if (!body->isStaticOrKinematicObject() && !(body->getFlags() & BT_DISABLE_WORLD_GRAVITY)) {
					body->setGravity(m_gravity * j->m_gravityFactor);
				}
			}
		}

		btDiscreteDynamicsWorldMt::applyGravity();
	}

	void SkinnedMeshWorld::applyWind()
	{
		for (auto& i : m_systems) {
			auto system = static_cast<SkyrimSystem*>(i());
			if (btFuzzyZero(system->m_windFactor)) // skip any systems that aren't affected by wind
				continue;
			for (auto& j : i->m_bones) {
				auto body = &j->m_rig;
				if (!body->isStaticOrKinematicObject() && (rand() % 5)) // apply randomly 80% of the time to desync wind
																		// across npcs
				{
					body->applyCentralForce(m_windSpeed * j->m_windFactor * system->m_windFactor);
				}
			}
		}
	}

	void SkinnedMeshWorld::predictUnconstraintMotion(btScalar timeStep)
	{
		HDT_ZONE_SCOPED_N("PredictMotion");
		for (int i = 0; i < m_nonStaticRigidBodies.size(); i++) {
			btRigidBody* body = m_nonStaticRigidBodies[i];
			if (!body->isStaticOrKinematicObject()) {
				// not realistic, just an approximate
				body->applyDamping(timeStep);
				body->predictIntegratedTransform(timeStep, body->getInterpolationWorldTransform());
			}
			else {
				body->predictIntegratedTransform(timeStep, body->getInterpolationWorldTransform());
			}
		}
	}

	void SkinnedMeshWorld::integrateTransforms(btScalar timeStep)
	{
		HDT_ZONE_SCOPED_N("IntegrateTransforms");
		for (int i = 0; i < m_collisionObjects.size(); ++i) {
			auto body = m_collisionObjects[i];
			if (body->isKinematicObject()) {
				btTransformUtil::integrateTransform(body->getWorldTransform(), body->getInterpolationLinearVelocity(),
													body->getInterpolationAngularVelocity(), timeStep,
													body->getInterpolationWorldTransform());
				body->setWorldTransform(body->getInterpolationWorldTransform());
			}
		}

		btVector3 limitMin(-1e+9f, -1e+9f, -1e+9f);
		btVector3 limitMax(1e+9f, 1e+9f, 1e+9f);
		for (int i = 0; i < m_nonStaticRigidBodies.size(); i++) {
			btRigidBody* body = m_nonStaticRigidBodies[i];
			auto lv = body->getLinearVelocity();
			lv.setMax(limitMin);
			lv.setMin(limitMax);
			body->setLinearVelocity(lv);

			auto av = body->getAngularVelocity();
			av.setMax(limitMin);
			av.setMin(limitMax);
			body->setAngularVelocity(av);
		}

		btDiscreteDynamicsWorldMt::integrateTransforms(timeStep);
	}

	void SkinnedMeshWorld::solveConstraints(btContactSolverInfo& solverInfo)
	{
		HDT_ZONE_SCOPED_N("SolveConstraints");
		BT_PROFILE("solveConstraints");
		if (!m_collisionObjects.size())
			return;

		{
			HDT_ZONE_SCOPED_N("PrepareSolve");
			m_solverPool->prepareSolve(getCollisionWorld()->getNumCollisionObjects(),
									   getCollisionWorld()->getDispatcher()->getNumManifolds());
		}

		m_constraintSolver.m_groups.clear();
		for (auto& i : m_systems)
			for (auto& j : i->m_constraintGroups)
				m_constraintSolver.m_groups.push_back(j);

		btPersistentManifold** manifold = m_dispatcher1->getInternalManifoldPointer();
		int maxNumManifolds = m_dispatcher1->getNumManifolds();
		int numConstraints = static_cast<int>(m_constraints.size());
		int numGroups = static_cast<int>(m_constraintSolver.m_groups.size());
		int numObjects = static_cast<int>(m_collisionObjects.size());

		HDT_PLOT("Manifolds", static_cast<int64_t>(maxNumManifolds));
		HDT_PLOT("Constraints", static_cast<int64_t>(numConstraints));
		HDT_PLOT("ConstraintGroups", static_cast<int64_t>(numGroups));
		HDT_PLOT("CollisionObjects", static_cast<int64_t>(numObjects));

		// DIAGNOSTIC: Track if manifold count is growing (indicates physics instability)
		static int s_frameCounter = 0;
		static int s_maxManifoldsSeen = 0;
		s_maxManifoldsSeen = std::max(s_maxManifoldsSeen, maxNumManifolds);
		if (++s_frameCounter % 120 == 0) { // Every ~2 seconds at 60fps
			_DMESSAGE("[SOLVER-DIAG] Frame %d: manifolds=%d (max=%d), constraints=%d, groups=%d, objects=%d",
					  s_frameCounter, maxNumManifolds, s_maxManifoldsSeen, numConstraints, numGroups, numObjects);
		}

		try {
			{
				HDT_ZONE_SCOPED_N("SolveGroup");
				m_solverPool->solveGroup(&m_collisionObjects[0], m_collisionObjects.size(), manifold, maxNumManifolds,
										 &m_constraints[0], m_constraints.size(), solverInfo, m_debugDrawer,
										 m_dispatcher1);
			}

			{
				HDT_ZONE_SCOPED_N("AllSolved");
				m_solverPool->allSolved(solverInfo, m_debugDrawer);
			}
		}
		catch (const std::exception& e) {
			_ERROR("SolveGroup exception: %s (manifolds=%d, constraints=%d, objects=%d)", e.what(), maxNumManifolds,
				   numConstraints, numObjects);
		}
		catch (...) {
			_ERROR("SolveGroup unknown exception (manifolds=%d, constraints=%d, objects=%d)", maxNumManifolds,
				   numConstraints, numObjects);
		}

		static_cast<CollisionDispatcher*>(m_dispatcher1)->clearAllManifold();
		m_constraintSolver.m_groups.clear();
	}

	void SkinnedMeshWorld::internalSingleStepSimulation(btScalar timeStep)
	{
		// PARALLEL FRAME START: Run GPU sync, predict motion, and internal updates concurrently
		// This hides ~1ms of CPU work within the GPU sync wait time.
		//
		// Dependencies analysis:
		// - GPU sync: Waits for previous frame's GPU work, then applies collision results
		// - Predict motion: Updates body velocities/transforms (no collision dependency)
		// - Systems internal update: Reads game bone transforms (no collision dependency)
		//
		// All three can run in parallel. Collision results are only needed by the solver,
		// which runs AFTER this parallel block.

#ifdef CUDA
		{
			HDT_ZONE_SCOPED_N("ParallelFrameStart");

			// Run all three operations in parallel using enkiTS
			hdt_parallel_invoke(
				// Task 1: GPU sync and apply collision results
				[this]() {
					HDT_ZONE_SCOPED_N("GpuSyncTask");
					static_cast<CollisionDispatcher*>(m_dispatcher1)->syncPreviousCollisionResults();
				},
				// Task 2: Predict motion (applies gravity, velocity, damping)
				[this, timeStep]() {
					HDT_ZONE_SCOPED_N("PredictMotionTask");
					predictUnconstraintMotion(timeStep);
				},
				// Task 3: Systems internal update (reads game bones, updates shapes)
				[this]() {
					HDT_ZONE_SCOPED_N("SystemsInternalUpdateTask");
					HDT_ZONE_VALUE(static_cast<int64_t>(m_systems.size()));
					for (int i = 0; i < m_systems.size(); ++i)
						m_systems[i]->internalUpdate();
				});

			// All tasks complete here due to hdt_parallel_invoke semantics
		}

		// Continue with sequential physics steps (predict motion already done above)
		// Skip the parent's predictUnconstraintMotion call by inlining the rest of the step

		btDispatcherInfo& dispatchInfo = getDispatchInfo();
		dispatchInfo.m_timeStep = timeStep;
		dispatchInfo.m_stepCount = 0;
		dispatchInfo.m_debugDraw = getDebugDrawer();

		{
			HDT_ZONE_SCOPED_N("CreatePredictiveContacts");
			createPredictiveContacts(timeStep);
		}

		// Collision detection (launches new GPU work)
		performDiscreteCollisionDetection();

		{
			HDT_ZONE_SCOPED_N("CalculateSimulationIslands");
			calculateSimulationIslands();
		}

		getSolverInfo().m_timeStep = timeStep;

		// Solve constraints
		solveConstraints(getSolverInfo());

		// Integrate transforms
		integrateTransforms(timeStep);

		{
			HDT_ZONE_SCOPED_N("UpdateActions");
			updateActions(timeStep);
		}
		{
			HDT_ZONE_SCOPED_N("UpdateActivationState");
			updateActivationState(timeStep);
		}
#else
		// Non-CUDA path: Sequential execution
		{
			HDT_ZONE_SCOPED_N("SystemsInternalUpdate");
			HDT_ZONE_VALUE(static_cast<int64_t>(m_systems.size()));
			for (int i = 0; i < m_systems.size(); ++i)
				m_systems[i]->internalUpdate();
		}
		btDiscreteDynamicsWorldMt::internalSingleStepSimulation(timeStep);
#endif
	}
} // namespace hdt
