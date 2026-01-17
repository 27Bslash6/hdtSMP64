#pragma once

// SoA (Structure of Arrays) buffer for SIMD-friendly vertex data layout
// Used by Highway batch operations for efficient vectorized skinning
//
// Lifecycle:
//   - Allocated once at mesh load time via allocate()
//   - Transposed from AoS vertex data via transposeFrom()
//   - Never resized during internalUpdate (thread safety)
//   - RAII cleanup on destruction

#include "hdtBulletHelper.h"

#include <cstddef>
#include <cstdint>
#include <hwy/aligned_allocator.h>
#include <utility>
#include <vector>

namespace hdt
{
	// Forward declaration
	struct Vertex;

	class SoAVertexBuffer
	{
	public:
		SoAVertexBuffer() = default;
		~SoAVertexBuffer() { deallocate(); }

		// Non-copyable
		SoAVertexBuffer(const SoAVertexBuffer&) = delete;
		SoAVertexBuffer& operator=(const SoAVertexBuffer&) = delete;

		// Movable
		SoAVertexBuffer(SoAVertexBuffer&& other) noexcept
			: m_posX(std::exchange(other.m_posX, nullptr)), m_posY(std::exchange(other.m_posY, nullptr)),
			  m_posZ(std::exchange(other.m_posZ, nullptr)), m_weights(std::exchange(other.m_weights, nullptr)),
			  m_boneIndices(std::exchange(other.m_boneIndices, nullptr)), m_count(std::exchange(other.m_count, 0)),
			  m_capacityBytes(std::exchange(other.m_capacityBytes, 0)), m_dirty(other.m_dirty)
		{}

		SoAVertexBuffer& operator=(SoAVertexBuffer&& other) noexcept
		{
			if (this != &other) {
				deallocate();
				m_posX = std::exchange(other.m_posX, nullptr);
				m_posY = std::exchange(other.m_posY, nullptr);
				m_posZ = std::exchange(other.m_posZ, nullptr);
				m_weights = std::exchange(other.m_weights, nullptr);
				m_boneIndices = std::exchange(other.m_boneIndices, nullptr);
				m_count = std::exchange(other.m_count, 0);
				m_capacityBytes = std::exchange(other.m_capacityBytes, 0);
				m_dirty = other.m_dirty;
			}
			return *this;
		}

		// Allocate aligned buffers for vertexCount vertices
		// Returns false if allocation would exceed MAX_BUFFER_BYTES
		bool allocate(size_t vertexCount)
		{
			deallocate();

			if (vertexCount == 0)
				return true;

			// Calculate required bytes:
			// - 3 float arrays for positions (X, Y, Z)
			// - 1 float array for weights (4 weights per vertex, interleaved)
			// - 1 U32 array for bone indices (4 indices per vertex)
			const size_t posBytes = vertexCount * sizeof(float);
			const size_t weightBytes = vertexCount * 4 * sizeof(float);
			const size_t boneBytes = vertexCount * 4 * sizeof(U32);
			const size_t totalBytes = posBytes * 3 + weightBytes + boneBytes;

			if (totalBytes > MAX_BUFFER_BYTES)
				return false;

			m_posX = static_cast<float*>(hwy::AllocateAlignedBytes(posBytes));
			m_posY = static_cast<float*>(hwy::AllocateAlignedBytes(posBytes));
			m_posZ = static_cast<float*>(hwy::AllocateAlignedBytes(posBytes));
			m_weights = static_cast<float*>(hwy::AllocateAlignedBytes(weightBytes));
			m_boneIndices = static_cast<U32*>(hwy::AllocateAlignedBytes(boneBytes));

			if (!m_posX || !m_posY || !m_posZ || !m_weights || !m_boneIndices) {
				deallocate();
				return false;
			}

			m_count = vertexCount;
			m_capacityBytes = totalBytes;
			m_dirty = true;
			return true;
		}

		void deallocate()
		{
			if (m_posX) {
				hwy::FreeAlignedBytes(m_posX, nullptr, nullptr);
				m_posX = nullptr;
			}
			if (m_posY) {
				hwy::FreeAlignedBytes(m_posY, nullptr, nullptr);
				m_posY = nullptr;
			}
			if (m_posZ) {
				hwy::FreeAlignedBytes(m_posZ, nullptr, nullptr);
				m_posZ = nullptr;
			}
			if (m_weights) {
				hwy::FreeAlignedBytes(m_weights, nullptr, nullptr);
				m_weights = nullptr;
			}
			if (m_boneIndices) {
				hwy::FreeAlignedBytes(m_boneIndices, nullptr, nullptr);
				m_boneIndices = nullptr;
			}
			m_count = 0;
			m_capacityBytes = 0;
		}

		// Transpose AoS vertex data to SoA layout
		// Must be called after allocate() with matching vertex count
		void transposeFrom(const std::vector<Vertex>& vertices);

		// Dirty tracking - marks when bone transforms have changed
		void markDirty() { m_dirty = true; }
		void markClean() { m_dirty = false; }
		bool isDirty() const { return m_dirty; }

		// Accessors for aligned SIMD pointers
		float* posX() { return m_posX; }
		float* posY() { return m_posY; }
		float* posZ() { return m_posZ; }
		float* weights() { return m_weights; }		 // Interleaved w0,w1,w2,w3 per vertex
		U32* boneIndices() { return m_boneIndices; } // 4 indices per vertex

		const float* posX() const { return m_posX; }
		const float* posY() const { return m_posY; }
		const float* posZ() const { return m_posZ; }
		const float* weights() const { return m_weights; }
		const U32* boneIndices() const { return m_boneIndices; }

		size_t count() const { return m_count; }
		size_t capacityBytes() const { return m_capacityBytes; }

		// 64MB buffer cap per NFR-1
		static constexpr size_t MAX_BUFFER_BYTES = 64 * 1024 * 1024;

	private:
		float* m_posX = nullptr;
		float* m_posY = nullptr;
		float* m_posZ = nullptr;
		float* m_weights = nullptr;	  // 4 weights per vertex, interleaved
		U32* m_boneIndices = nullptr; // 4 bone indices per vertex

		size_t m_count = 0;
		size_t m_capacityBytes = 0;
		bool m_dirty = true;
	};

} // namespace hdt
