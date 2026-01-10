#pragma once

#include "hdtConvertNi.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBone.h"

namespace hdt
{
	class SkyrimBone : public SkinnedMeshBone
	{
	public:
		SkyrimBone(IDStr name, NiNode* node, NiNode* skeleton, btRigidBody::btRigidBodyConstructionInfo& ci);

		void resetTransformToOriginal() override;
		void readTransform(float timeStep) override;
		void writeTransform() override;

		int m_depth;
		NiNode* m_node;
		NiNode* m_skeleton;

	private:
		int m_forceUpdateType;
	};
} // namespace hdt
