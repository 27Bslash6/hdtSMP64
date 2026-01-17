#pragma once

#include "hdtSkinnedMeshSystem.h"

#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h>

#include <atomic>

namespace hdt
{
	class SkinnedMeshWorld : protected btDiscreteDynamicsWorldMt
	{
	public:
		SkinnedMeshWorld();
		~SkinnedMeshWorld();

		virtual void addSkinnedMeshSystem(SkinnedMeshSystem* system);
		virtual void removeSkinnedMeshSystem(SkinnedMeshSystem* system);

		// Force re-registration of all collision objects to rebuild broadphase
		// Call during reset to ensure no stale collision state
		void reregisterAllBodies();

		int stepSimulation(btScalar remainingTimeStep, int maxSubSteps = 1,
						   btScalar fixedTimeStep = btScalar(1.) / btScalar(60.)) override;

		// Global frame counter for dirty flag optimization
		static uint32_t getCurrentFrame() { return s_currentFrame.load(std::memory_order_relaxed); }
		static void incrementFrame(); // Defined in .cpp to update both counters

		btVector3& getWind() { return m_windSpeed; }
		const btVector3& getWind() const { return m_windSpeed; }

	protected:
		void resetTransformsToOriginal()
		{
			for (int i = 0; i < m_systems.size(); ++i)
				m_systems[i]->resetTransformsToOriginal();
		}

		void readTransform(float timeStep)
		{
			for (int i = 0; i < m_systems.size(); ++i)
				m_systems[i]->readTransform(timeStep);
		}

		void writeTransform()
		{
			for (int i = 0; i < m_systems.size(); ++i)
				m_systems[i]->writeTransform();
		}

		void applyGravity() override;
		void applyWind();

		void predictUnconstraintMotion(btScalar timeStep) override;
		void integrateTransforms(btScalar timeStep) override;
		void performDiscreteCollisionDetection() override;
		void solveConstraints(btContactSolverInfo& solverInfo) override;
		void internalSingleStepSimulation(btScalar timeStep) override;

		std::vector<Ref<SkinnedMeshSystem>> m_systems;

		btVector3 m_windSpeed; // world windspeed

	private:
		btConstraintSolverPoolMt* m_solverPool;
		static std::atomic<uint32_t> s_currentFrame;
	};
} // namespace hdt
