#include "hdtSkinnedMeshSystem.h"

#include "hdtBoneScaleConstraint.h"
#include "hdtEnkiTSScheduler.h"
#include "hdtSkinnedMeshBody.h"
#include "hdtSkinnedMeshShape.h"

#include "../hdtTracy.h"

namespace hdt
{
	void SkinnedMeshSystem::resetTransformsToOriginal()
	{
		for (int i = 0; i < m_bones.size(); ++i)
			m_bones[i]->resetTransformToOriginal();
	}

	void SkinnedMeshSystem::readTransform(float timeStep)
	{
		if (this->block_resetting)
			return;

		// Use sequential processing during reset (RESET_PHYSICS = -10.0f).
		// Reset may be called from non-game threads (console command),
		// and enkiTS can crash when tasks are added from external threads
		// while the scheduler has residual state from recent game thread work.
		// During normal gameplay (positive timeStep), use parallel processing.
		if (timeStep < 0.0f) {
			for (auto& bone : m_bones)
				bone->readTransform(timeStep);
		}
		else {
			hdt_parallel_for_each(m_bones.begin(), m_bones.end(),
								  [=](Ref<SkinnedMeshBone> bone) { bone->readTransform(timeStep); });
		}

		for (auto i : m_constraints)
			i->scaleConstraint();

		for (auto i : m_constraintGroups)
			i->scaleConstraint();
	}

	void SkinnedMeshSystem::writeTransform()
	{
		for (int i = 0; i < m_bones.size(); ++i) {
			if (m_bones[i]->m_rig.isKinematicObject())
				continue;

			m_bones[i]->writeTransform();
		}
	}

	void SkinnedMeshSystem::internalUpdate()
	{
		HDT_ZONE_SCOPED_N("System::internalUpdate");
		{
			HDT_ZONE_SCOPED_N("BoneUpdates");
			HDT_ZONE_VALUE(static_cast<int64_t>(m_bones.size()));
			for (auto& i : m_bones)
				i->internalUpdate();
		}
		{
			HDT_ZONE_SCOPED_N("MeshUpdates");
			HDT_ZONE_VALUE(static_cast<int64_t>(m_meshes.size()));
			for (auto& i : m_meshes)
				i->updateBoundingSphereAabb();
		}
	}

	// void SkinnedMeshSystem::internalUpdateCL()
	//{
	//	for (auto& i : m_bones)
	//		i->internalUpdate();

	//	//i->internalUpdate();
	//}

	void SkinnedMeshSystem::gather(std::vector<SkinnedMeshBody*>& bodies, std::vector<SkinnedMeshShape*>& shapes)
	{
		for (auto& i : m_meshes) {
			bodies.push_back(i);
			shapes.push_back(i->m_shape);
			auto triShape = dynamic_cast<PerTriangleShape*>(i->m_shape());
			if (triShape)
				shapes.push_back(triShape->m_verticesCollision);
		}
	}
} // namespace hdt
