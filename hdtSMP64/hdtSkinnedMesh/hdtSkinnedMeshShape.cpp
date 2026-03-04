#include "hdtSkinnedMeshShape.h"

#include "../hdtTracy.h"

namespace hdt
{
	SkinnedMeshShape::SkinnedMeshShape(SkinnedMeshBody* body)
	{
		m_owner = body;
		m_owner->m_shape = this;
	}

	SkinnedMeshShape::~SkinnedMeshShape()
	{
		_VMESSAGE("SkinnedMeshShape::~dtor: shape=%p owner=%s m_colliders.data()=%p size=%zu", this,
				  (m_owner && m_owner->m_name()) ? m_owner->m_name()->cstr() : "null", m_colliders.data(),
				  m_colliders.size());
		// m_aabbGridLink.discard_data();
		// m_aabbGridBuffer.discard_data();
	}

	void SkinnedMeshShape::clipColliders()
	{
		auto& v = m_owner->m_vertices;
		m_tree.clipCollider([&, this](const Collider& n) -> bool {
			bool flg = false;
			for (int i = 0; i < getBonePerCollider() && !flg; ++i) {
				float weight = getColliderBoneWeight(&n, i);
				if (weight > FLT_EPSILON &&
					weight > m_owner->m_skinnedBones[getColliderBoneIndex(&n, i)].weightThreshold)
					flg = true;
			}
			return !flg;
		});
	}

#ifdef ENABLE_CL
	static const std::string sourceVtxUpdate = R"__KERNEL(
typedef struct Aabb
{
	float4 aabbMin;
	float4 aabbMax;
} Aabb;

__kernel void updateCollider(__global float4* vertices, __global uint4* colliders, __global Aabb* aabbs, float margin)
{
	int idx = get_global_id(0);
	float4 data = vertices[colliders[idx].x];
	float3 p0 = data.xyz;
	float m = margin * data.w;

	Aabb aabb;
	aabb.aabbMin.xyz = p0 - m;
	aabb.aabbMax.xyz = p0 + m;

	aabbs[idx] = aabb;
}
)__KERNEL";

	hdtCLKernel PerVertexShape::m_kernel;

	void PerVertexShape::internalUpdateCL()
	{
		auto cl = hdtCL::instance();

		if (!m_colliderCL()) {
			m_colliderCL =
				cl->createBuffer(sizeof(Collider) * m_colliders.size(), CL_MEM_READ_ONLY | CL_MEM_HOST_WRITE_ONLY, 0);
			m_aabbCL = cl->createBuffer(sizeof(Aabb) * m_aabb.size(), CL_MEM_WRITE_ONLY | CL_MEM_HOST_READ_ONLY, 0);
			cl->writeBuffer(m_colliderCL, m_colliders.data(), sizeof(Collider) * m_colliders.size(), true);
		}

		m_kernel.lock();
		m_kernel.setArg(0, m_owner->m_vposCL);
		m_kernel.setArg(1, m_colliderCL);
		m_kernel.setArg(2, m_aabbCL);
		m_kernel.setArg(3, m_shapeProp.margin);
		auto e0 = m_kernel.runE({m_colliders.size()});
		m_kernel.unlock();
		m_eDoneCL = cl->readBufferE(m_aabb.data(), m_aabbCL, m_aabb.size() * sizeof(Aabb), {e0});
	}

	static const std::string sourceTriUpdate = R"__KERNEL(
typedef struct Aabb
{
	float3 aabbMin;
	float3 aabbMax;
} Aabb;

void aabbMerge(Aabb* aabb, float3 pt)
{
	aabb->aabbMin = fmin(aabb->aabbMin, pt);
	aabb->aabbMax = fmax(aabb->aabbMax, pt);
}

void aabbExpand(Aabb* aabb, float margin)
{
	aabb->aabbMin -= margin;
	aabb->aabbMax += margin;
}

__kernel void updateCollider(__global float4* vertices, __global uint4* colliders, __global Aabb* aabbs, float penetration, float margin)
{
	int idx = get_global_id(0);
	float3 p0 = vertices[colliders[idx].x].s012;
	float3 p1 = vertices[colliders[idx].y].s012;
	float3 p2 = vertices[colliders[idx].z].s012;
	float m = vertices[colliders[idx].x].w + vertices[colliders[idx].y].w + vertices[colliders[idx].z].w;

	Aabb aabb;
	aabb.aabbMin = aabb.aabbMax = p0;
	aabbMerge(&aabb, p1);
	aabbMerge(&aabb, p2);
	aabbExpand(&aabb, margin * m / 3.0f);

	if(penetration > FLT_EPSILON || penetration < -FLT_EPSILON)
	{
		float3 normal = cross((p1-p0).s012, (p2-p0).s012);
		float len2 = dot(normal, normal);
		if(len2 > FLT_EPSILON * FLT_EPSILON)
		{
			normal = normalize(normal) * penetration;
			aabb.aabbMin = fmin(aabb.aabbMin, aabb.aabbMin - normal);
			aabb.aabbMax = fmax(aabb.aabbMax, aabb.aabbMax - normal);
		}
	}
	aabbs[idx] = aabb;
}
)__KERNEL";

	hdtCLKernel PerTriangleShape::m_kernel;

	void PerTriangleShape::internalUpdateCL()
	{
		auto cl = hdtCL::instance();

		if (!m_colliderCL()) {
			m_colliderCL =
				cl->createBuffer(sizeof(Collider) * m_colliders.size(), CL_MEM_READ_ONLY | CL_MEM_HOST_WRITE_ONLY, 0);
			m_aabbCL = cl->createBuffer(sizeof(Aabb) * m_aabb.size(), CL_MEM_WRITE_ONLY | CL_MEM_HOST_READ_ONLY, 0);
			cl->writeBuffer(m_colliderCL, m_colliders.data(), sizeof(Collider) * m_colliders.size(), true);
		}

		m_kernel.lock();
		m_kernel.setArg(0, m_owner->m_vposCL);
		m_kernel.setArg(1, m_colliderCL);
		m_kernel.setArg(2, m_aabbCL);
		m_kernel.setArg(3, m_shapeProp.penetration);
		m_kernel.setArg(4, m_shapeProp.margin);
		auto e0 = m_kernel.runE({m_colliders.size()});
		m_kernel.unlock();
		m_eDoneCL = cl->readBufferE(m_aabb.data(), m_aabbCL, m_aabb.size() * sizeof(Aabb), {e0});
	}

#endif

	PerVertexShape::PerVertexShape(SkinnedMeshBody* body) : SkinnedMeshShape(body)
	{
#ifdef ENABLE_CL
		auto cl = hdtCL::instance();
		if (cl) {
			if (!m_kernel()) {
				m_kernel.lock();
				if (!m_kernel())
					m_kernel = hdtCLKernel(cl->compile(sourceVtxUpdate), "updateCollider");
				m_kernel.unlock();
			}
		}
#endif
	}

	PerVertexShape::~PerVertexShape() {}

	void PerVertexShape::finishBuild()
	{
		m_tree.optimize();
		m_tree.updateKinematic([this](const Collider* n) { return m_owner->flexible(m_owner->m_vertices[n->vertex]); });

		m_owner->setCollisionFlags(m_tree.isKinematic ? btCollisionObject::CF_KINEMATIC_OBJECT : 0);

		// exportColliders stores offsets in tree nodes (no pointers!)
		m_tree.exportColliders(m_colliders);
#ifdef CUDA
		m_aabb.reset(new Aabb[m_colliders.size()]);
#else
		m_aabb.resize(m_colliders.size());
#endif
		// Clear local colliders from tree nodes (they're now in m_colliders)
		m_tree.finalizeOffsets();

		_VMESSAGE("PerVertexShape::finishBuild: shape=%p owner=%s colliders=%zu", this,
				  (m_owner && m_owner->m_name()) ? m_owner->m_name()->cstr() : "null", m_colliders.size());

		// Validate offsets are within bounds
		if (!validateColliderOffsets()) {
			_ERROR("PerVertexShape::finishBuild: VALIDATION FAILED! shape=%p owner=%s", this,
				   (m_owner && m_owner->m_name()) ? m_owner->m_name()->cstr() : "null");
		}
	}

	void PerVertexShape::internalUpdate()
	{
		HDT_ZONE_SCOPED_N("PerVertexShape::internalUpdate");
#ifdef CUDA
		auto vertices = m_owner->m_vpos.get();
#else
		auto& vertices = m_owner->m_vpos;
#endif // CUDA

		size_t size = m_colliders.size();
		HDT_ZONE_VALUE(static_cast<int64_t>(size));
		{
			HDT_ZONE_SCOPED_N("VertexAABBLoop");
			const float shapePropMargin = m_shapeProp.margin;
			size_t i = 0;

#ifdef __AVX512F__
			// AVX-512 path: process 4 vertices at a time
			for (; i + 3 < size; i += 4) {
				if (i + 12 < size) {
					_mm_prefetch(reinterpret_cast<const char*>(&vertices[m_colliders[i + 12].vertex]), _MM_HINT_T0);
					_mm_prefetch(reinterpret_cast<const char*>(&vertices[m_colliders[i + 13].vertex]), _MM_HINT_T0);
					_mm_prefetch(reinterpret_cast<const char*>(&vertices[m_colliders[i + 14].vertex]), _MM_HINT_T0);
					_mm_prefetch(reinterpret_cast<const char*>(&vertices[m_colliders[i + 15].vertex]), _MM_HINT_T0);
				}

				auto p0 = vertices[m_colliders[i].vertex].m_data;
				auto p1 = vertices[m_colliders[i + 1].vertex].m_data;
				auto p2 = vertices[m_colliders[i + 2].vertex].m_data;
				auto p3 = vertices[m_colliders[i + 3].vertex].m_data;

				__m512 pos = _mm512_insertf32x4(
					_mm512_insertf32x4(_mm512_insertf32x4(_mm512_castps128_ps512(p0), p1, 1), p2, 2), p3, 3);

				__m512 margins = _mm512_insertf32x4(
					_mm512_insertf32x4(
						_mm512_insertf32x4(_mm512_castps128_ps512(_mm_set_ps1(p0.m128_f32[3] * shapePropMargin)),
										   _mm_set_ps1(p1.m128_f32[3] * shapePropMargin), 1),
						_mm_set_ps1(p2.m128_f32[3] * shapePropMargin), 2),
					_mm_set_ps1(p3.m128_f32[3] * shapePropMargin), 3);

				__m512 aabbMin = _mm512_sub_ps(pos, margins);
				__m512 aabbMax = _mm512_add_ps(pos, margins);

				m_aabb[i].m_min = _mm512_castps512_ps128(aabbMin);
				m_aabb[i].m_max = _mm512_castps512_ps128(aabbMax);
				m_aabb[i + 1].m_min = _mm512_extractf32x4_ps(aabbMin, 1);
				m_aabb[i + 1].m_max = _mm512_extractf32x4_ps(aabbMax, 1);
				m_aabb[i + 2].m_min = _mm512_extractf32x4_ps(aabbMin, 2);
				m_aabb[i + 2].m_max = _mm512_extractf32x4_ps(aabbMax, 2);
				m_aabb[i + 3].m_min = _mm512_extractf32x4_ps(aabbMin, 3);
				m_aabb[i + 3].m_max = _mm512_extractf32x4_ps(aabbMax, 3);
			}
#endif
			// Scalar path: process 1 vertex at a time
			for (; i < size; ++i) {
				auto p0 = vertices[m_colliders[i].vertex].m_data;
				auto margin = _mm_set_ps1(p0.m128_f32[3] * shapePropMargin);
				m_aabb[i].m_min = p0 - margin;
				m_aabb[i].m_max = p0 + margin;
			}
		}
		{
			HDT_ZONE_SCOPED_N("TreeUpdateAabb");
			m_tree.updateAabb(getAabbBase());
		}
	}

	void PerVertexShape::autoGen()
	{
		m_tree.children.clear();
		std::vector<U32> keys;
		for (U32 i = 0; i < m_owner->m_vertices.size(); ++i) {
			keys.clear();
			for (int j = 0; j < 4; ++j) {
				if (m_owner->m_vertices[i].m_weight[j] > FLT_EPSILON)
					keys.push_back(m_owner->m_vertices[i].getBoneIdx(j));
			}
			m_tree.insertCollider(keys, Collider(i));
		}
	}

	void PerVertexShape::markUsedVertices(bool* flags)
	{
		for (auto& i : m_colliders)
			flags[i.vertex] = true;
	}

	void PerVertexShape::remapVertices(UINT* map)
	{
		for (auto& i : m_colliders)
			i.vertex = map[i.vertex];
	}

	PerTriangleShape::PerTriangleShape(SkinnedMeshBody* body) : SkinnedMeshShape(body)
	{
#ifdef ENABLE_CL
		auto cl = hdtCL::instance();
		if (cl) {
			if (!m_kernel()) {
				m_kernel.lock();
				if (!m_kernel())
					m_kernel = hdtCLKernel(cl->compile(sourceTriUpdate), "updateCollider");
				m_kernel.unlock();
			}
		}
#endif
	}

	PerTriangleShape::~PerTriangleShape() {}

	void PerTriangleShape::internalUpdate()
	{
		HDT_ZONE_SCOPED_N("PerTriangleShape::internalUpdate");
#ifdef CUDA
		auto vertices = m_owner->m_vpos.get();
#else
		auto& vertices = m_owner->m_vpos;
#endif // CUDA

		size_t size = m_colliders.size();
		HDT_ZONE_VALUE(static_cast<int64_t>(size));
		{
			HDT_ZONE_SCOPED_N("TriangleAABBLoop");
			const float shapePropMargin = m_shapeProp.margin / 3.0f;
			const float absPenetration = btFabs(m_shapeProp.penetration);

			for (size_t i = 0; i < size; ++i) {
				// Prefetch ahead to hide memory latency for triangle vertices
				if (i + 4 < size) {
					const auto* prefetchC = &m_colliders[i + 4];
					_mm_prefetch(reinterpret_cast<const char*>(&vertices[prefetchC->vertices[0]]), _MM_HINT_T0);
					_mm_prefetch(reinterpret_cast<const char*>(&vertices[prefetchC->vertices[1]]), _MM_HINT_T0);
					_mm_prefetch(reinterpret_cast<const char*>(&vertices[prefetchC->vertices[2]]), _MM_HINT_T0);
				}

				auto c = &m_colliders[i];
				auto p0 = vertices[c->vertices[0]].m_data;
				auto p1 = vertices[c->vertices[1]].m_data;
				auto p2 = vertices[c->vertices[2]].m_data;

				// Compute AABB min/max
				auto aabbMin = _mm_min_ps(_mm_min_ps(p0, p1), p2);
				auto aabbMax = _mm_max_ps(_mm_max_ps(p0, p1), p2);

				// Margin from average of w components
				float avgMargin = (p0.m128_f32[3] + p1.m128_f32[3] + p2.m128_f32[3]) * shapePropMargin;
				auto margin4 = _mm_set_ps1(btMax(avgMargin, absPenetration));

				m_aabb[i].m_min = aabbMin - margin4;
				m_aabb[i].m_max = aabbMax + margin4;
			}
		}
		{
			HDT_ZONE_SCOPED_N("TreeUpdateAabb");
			m_tree.updateAabb(getAabbBase());
		}
	}

	void PerTriangleShape::finishBuild()
	{
		m_tree.optimize();
		m_tree.updateKinematic([=](const Collider* c) {
			float k = m_owner->flexible(m_owner->m_vertices[c->vertices[0]]);
			k += m_owner->flexible(m_owner->m_vertices[c->vertices[1]]);
			k += m_owner->flexible(m_owner->m_vertices[c->vertices[2]]);
			return k / 3;
		});

		m_owner->setCollisionFlags(m_tree.isKinematic ? btCollisionObject::CF_KINEMATIC_OBJECT : 0);

		// exportColliders stores offsets in tree nodes (no pointers!)
		m_tree.exportColliders(m_colliders);
#ifdef CUDA
		m_aabb.reset(new Aabb[m_colliders.size()]);
#else
		m_aabb.resize(m_colliders.size());
#endif
		// Clear local colliders from tree nodes (they're now in m_colliders)
		m_tree.finalizeOffsets();

		_VMESSAGE("PerTriangleShape::finishBuild: shape=%p owner=%s colliders=%zu", this,
				  (m_owner && m_owner->m_name()) ? m_owner->m_name()->cstr() : "null", m_colliders.size());

		// Validate offsets are within bounds
		if (!validateColliderOffsets()) {
			_ERROR("PerTriangleShape::finishBuild: VALIDATION FAILED! shape=%p owner=%s", this,
				   (m_owner && m_owner->m_name()) ? m_owner->m_name()->cstr() : "null");
		}

		Ref<PerTriangleShape> holder = this;
		m_verticesCollision = new PerVertexShape(m_owner);
		m_verticesCollision->m_shapeProp.margin = m_shapeProp.margin;
		m_owner->m_shape = this;

		m_verticesCollision->autoGen();
		m_verticesCollision->clipColliders();
		m_verticesCollision->finishBuild();
	}

	void PerTriangleShape::markUsedVertices(bool* flags)
	{
		for (auto& i : m_colliders) {
			flags[i.vertices[0]] = true;
			flags[i.vertices[1]] = true;
			flags[i.vertices[2]] = true;
		}

		m_verticesCollision->markUsedVertices(flags);
	}

	void PerTriangleShape::remapVertices(UINT* map)
	{
		for (auto& i : m_colliders) {
			i.vertices[0] = map[i.vertices[0]];
			i.vertices[1] = map[i.vertices[1]];
			i.vertices[2] = map[i.vertices[2]];
		}

		m_verticesCollision->remapVertices(map);
	}

	void PerTriangleShape::addTriangle(int a, int b, int c)
	{
		assert(a < m_owner->m_vertices.size());
		assert(b < m_owner->m_vertices.size());
		assert(c < m_owner->m_vertices.size());
		Collider collider(a, b, c);
		std::vector<U32> keys;
		std::vector<float> w;
		for (int i = 0; i < 12; ++i) {
			auto weight = getColliderBoneWeight(&collider, i);
			if (weight < FLT_EPSILON)
				continue;
			auto bone = getColliderBoneIndex(&collider, i);
			auto iter = std::find(keys.begin(), keys.end(), bone);
			if (iter != keys.end())
				w[iter - keys.begin()] += weight;
			else {
				keys.push_back(bone);
				w.push_back(weight);
			}
		}

		for (int i = 0; i < keys.size(); ++i)
			for (int j = 1; j < keys.size(); ++j) {
				if (w[j - 1] < w[j] || (w[j - 1] == w[j] && keys[j] < keys[j - 1])) {
					std::swap(keys[j], keys[j - 1]);
					std::swap(w[j], w[j - 1]);
				}
			}

		m_tree.insertCollider(keys, collider);
	}
} // namespace hdt
