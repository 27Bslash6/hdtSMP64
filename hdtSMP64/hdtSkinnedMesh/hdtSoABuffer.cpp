#include "hdtSoABuffer.h"

#include "hdtVertex.h"

namespace hdt
{
	void SoAVertexBuffer::transposeFrom(const std::vector<Vertex>& vertices)
	{
		if (vertices.size() != m_count || !m_posX)
			return;

		// Transpose AoS to SoA layout
		for (size_t i = 0; i < m_count; ++i) {
			const Vertex& v = vertices[i];

			// Position components (separate X/Y/Z arrays for optimal SIMD)
			m_posX[i] = v.m_skinPos.x();
			m_posY[i] = v.m_skinPos.y();
			m_posZ[i] = v.m_skinPos.z();

			// Weights interleaved: w0,w1,w2,w3 for vertex i at offset i*4
			const size_t wOffset = i * 4;
			m_weights[wOffset + 0] = v.m_weight[0];
			m_weights[wOffset + 1] = v.m_weight[1];
			m_weights[wOffset + 2] = v.m_weight[2];
			m_weights[wOffset + 3] = v.m_weight[3];

			// Bone indices: 4 indices per vertex
			const size_t bOffset = i * 4;
			m_boneIndices[bOffset + 0] = v.m_boneIdx[0];
			m_boneIndices[bOffset + 1] = v.m_boneIdx[1];
			m_boneIndices[bOffset + 2] = v.m_boneIdx[2];
			m_boneIndices[bOffset + 3] = v.m_boneIdx[3];
		}

		m_dirty = false;
	}

} // namespace hdt
