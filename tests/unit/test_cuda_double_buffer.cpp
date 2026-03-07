#include "../include/catch.hpp"

#include <array>
#include <cstdint>

// Tests for CUDA double-buffer logic used in Phase 3 async AABB pipeline.
// These validate the buffer swap index logic without requiring actual CUDA hardware.

namespace
{
	// Minimal mock of CudaColliderTree's double-buffer state
	class MockDoubleBuffer
	{
	public:
		MockDoubleBuffer() : m_currentWriteBuffer(0), m_firstFrame(true) {}

		// Mimics CudaColliderTree::getCurrentWriteBuffer index selection
		int getWriteBufferIndex() const { return m_currentWriteBuffer; }

		// Mimics CudaColliderTree::update() - reads from OTHER buffer
		int getReadBufferIndex() const { return 1 - m_currentWriteBuffer; }

		// Mimics CudaColliderTree::swapBuffers
		void swapBuffers() { m_currentWriteBuffer = 1 - m_currentWriteBuffer; }

		bool isFirstFrame() const { return m_firstFrame; }
		void clearFirstFrame() { m_firstFrame = false; }

		// Simulate writing data to current write buffer
		void writeToCurrentBuffer(int frameData) { m_buffers[m_currentWriteBuffer] = frameData; }

		// Simulate reading from previous frame's buffer (what CPU tree update does)
		int readFromPreviousBuffer() const { return m_buffers[1 - m_currentWriteBuffer]; }

		// Direct buffer access for verification
		int getBufferData(int index) const { return m_buffers[index]; }

	private:
		int m_currentWriteBuffer;
		bool m_firstFrame;
		std::array<int, 2> m_buffers = {-1, -1}; // -1 = uninitialized
	};
} // namespace

TEST_CASE("Double buffer index alternation", "[cuda][double-buffer]")
{
	MockDoubleBuffer db;

	SECTION("Initial state")
	{
		REQUIRE(db.getWriteBufferIndex() == 0);
		REQUIRE(db.getReadBufferIndex() == 1);
		REQUIRE(db.isFirstFrame() == true);
	}

	SECTION("After first swap")
	{
		db.swapBuffers();
		REQUIRE(db.getWriteBufferIndex() == 1);
		REQUIRE(db.getReadBufferIndex() == 0);
	}

	SECTION("After two swaps returns to initial")
	{
		db.swapBuffers();
		db.swapBuffers();
		REQUIRE(db.getWriteBufferIndex() == 0);
		REQUIRE(db.getReadBufferIndex() == 1);
	}

	SECTION("Write and read indices are always different")
	{
		for (int i = 0; i < 100; ++i) {
			REQUIRE(db.getWriteBufferIndex() != db.getReadBufferIndex());
			db.swapBuffers();
		}
	}
}

TEST_CASE("Double buffer data flow", "[cuda][double-buffer]")
{
	MockDoubleBuffer db;

	SECTION("Frame N-1 data available after swap")
	{
		// Frame 0: Write to buffer 0
		db.writeToCurrentBuffer(100);
		db.clearFirstFrame();
		db.swapBuffers();

		// Frame 1: Read should get frame 0's data from buffer 0
		REQUIRE(db.readFromPreviousBuffer() == 100);

		// Frame 1: Write to buffer 1
		db.writeToCurrentBuffer(200);
		db.swapBuffers();

		// Frame 2: Read should get frame 1's data from buffer 1
		REQUIRE(db.readFromPreviousBuffer() == 200);

		// Frame 2: Write to buffer 0
		db.writeToCurrentBuffer(300);
		db.swapBuffers();

		// Frame 3: Read should get frame 2's data from buffer 0
		REQUIRE(db.readFromPreviousBuffer() == 300);
	}

	SECTION("Buffers contain correct frame data")
	{
		// Simulate 10 frames
		for (int frame = 0; frame < 10; ++frame) {
			int writeIdx = db.getWriteBufferIndex();
			int readIdx = db.getReadBufferIndex();

			// Write current frame number to write buffer
			db.writeToCurrentBuffer(frame);

			// After first frame, read buffer should have previous frame's data
			if (frame > 0) {
				REQUIRE(db.readFromPreviousBuffer() == frame - 1);
			}

			// Verify write went to correct buffer
			REQUIRE(db.getBufferData(writeIdx) == frame);

			db.swapBuffers();
		}
	}
}

TEST_CASE("First frame flag behavior", "[cuda][double-buffer]")
{
	MockDoubleBuffer db;

	SECTION("First frame requires bootstrap sync")
	{
		REQUIRE(db.isFirstFrame() == true);

		// Simulate bootstrap: sync and clear flag
		db.clearFirstFrame();
		REQUIRE(db.isFirstFrame() == false);

		// Flag stays cleared across swaps
		db.swapBuffers();
		REQUIRE(db.isFirstFrame() == false);
		db.swapBuffers();
		REQUIRE(db.isFirstFrame() == false);
	}
}

TEST_CASE("Double buffer latency characteristics", "[cuda][double-buffer]")
{
	MockDoubleBuffer db;

	SECTION("1-frame latency after bootstrap")
	{
		// Bootstrap frame 0
		db.writeToCurrentBuffer(0);
		db.clearFirstFrame();
		db.swapBuffers();

		// Frame 1: Write 1, read 0 (1 frame behind)
		REQUIRE(db.readFromPreviousBuffer() == 0);
		db.writeToCurrentBuffer(1);
		db.swapBuffers();

		// Frame 2: Write 2, read 1 (1 frame behind)
		REQUIRE(db.readFromPreviousBuffer() == 1);
		db.writeToCurrentBuffer(2);
		db.swapBuffers();

		// Frame 3: Write 3, read 2 (1 frame behind)
		REQUIRE(db.readFromPreviousBuffer() == 2);
	}

	SECTION("Read buffer never contains current frame data")
	{
		for (int frame = 0; frame < 20; ++frame) {
			db.writeToCurrentBuffer(frame * 1000); // Distinctive values

			// Read buffer should NOT have current frame's data
			// (except frame 0 which reads uninitialized)
			if (frame > 0) {
				REQUIRE(db.readFromPreviousBuffer() != frame * 1000);
			}

			db.swapBuffers();
		}
	}
}

// Zero-copy semantics test - validates the key insight that getZ() and get()
// point to the same underlying pinned memory, so no explicit copy is needed.
TEST_CASE("Zero-copy buffer semantics", "[cuda][double-buffer][zero-copy]")
{
	// In the real CudaBuffer implementation:
	// - getZ() returns device pointer to pinned host memory
	// - get() returns host pointer to same pinned memory
	// - GPU writes via getZ() are visible to CPU via get() after sync

	// This test validates the CONCEPT, not the actual CUDA implementation
	// (which requires hardware). The key invariant is:
	// After GPU writes to buffer[N].getZ() and we sync,
	// CPU can read the same data from buffer[N].get()

	SECTION("Write and read target same logical buffer")
	{
		MockDoubleBuffer db;

		// Simulate: GPU writes frame data via zero-copy
		int frameData = 42;
		db.writeToCurrentBuffer(frameData);

		// The write buffer index determines which buffer has the data
		int writeIdx = db.getWriteBufferIndex();
		REQUIRE(db.getBufferData(writeIdx) == frameData);

		// After swap, that same buffer becomes the read buffer
		db.swapBuffers();
		int newReadIdx = db.getReadBufferIndex();
		REQUIRE(newReadIdx == writeIdx); // Same buffer, different role
		REQUIRE(db.readFromPreviousBuffer() == frameData);
	}
}
