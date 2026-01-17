# Triple Buffer → Double Buffer Simplification

**Status:** PLANNED
**Risk:** Low
**Effort:** ~30 minutes

## Problem

`BatchedCollisionManager` uses a triple buffer (`m_pendingResults[3]`) for 1-frame latency, but only 2 buffers are ever in use at any time. The third buffer sits empty, wasting memory and adding complexity.

## Current Implementation

```cpp
// hdtCudaInterface.h:247-248
int writeIndex() const { return m_frameCount % 3; }
int readIndex() const { return (m_frameCount + 2) % 3; }

// hdtCudaInterface.h:288
std::vector<PendingCollisionResult> m_pendingResults[3];
```

## Proposed Change

```cpp
// Simplified double buffer
int writeIndex() const { return m_frameCount & 1; }
int readIndex() const { return (m_frameCount + 1) & 1; }

std::vector<PendingCollisionResult> m_pendingResults[2];
```

## Files to Modify

### hdtCudaInterface.h

1. **Line 247-248** - Change index functions:
   ```cpp
   int writeIndex() const { return m_frameCount & 1; }
   int readIndex() const { return (m_frameCount + 1) & 1; }
   ```

2. **Line 276-287** - Update comment:
   ```cpp
   //======================================================================
   // 1-FRAME LATENCY DOUBLE BUFFER WITH STORED TRANSFORMS
   // 2 buffers for clean ping-pong:
   //   - Buffer (N & 1): being written by GPU this frame
   //   - Buffer ((N+1) & 1): previous frame's data, read after sync
   //
   // Bone transforms are stored in CudaMergeBuffer at collision time.
   // When applying results 1 frame later, we use stored transforms
   // for local coordinate conversion (not current transforms).
   // Sync at frame start ensures GPU is done before reading.
   //======================================================================
   ```

3. **Line 288** - Reduce array size:
   ```cpp
   std::vector<PendingCollisionResult> m_pendingResults[2];
   ```

4. **Line 289** - Update events array:
   ```cpp
   void* m_completionEvents[2]; // cudaEvent_t (reserved for future optimization)
   ```

### hdtCudaInterface.cpp

1. **Line 1487** - Update comment:
   ```cpp
   // Clear write buffer - it was read and cleared last frame,
   // but clear again for safety during bootstrap or edge cases
   ```

2. **Line 1581-1583** - Update comment:
   ```cpp
   // Advance frame count - this drives the double buffer ping-pong:
   //   writeIndex = m_frameCount & 1
   //   readIndex = (m_frameCount + 1) & 1  (1 frame behind)
   ```

3. **Line 1601-1602** - Update diagnostic:
   ```cpp
   _MESSAGE("[CUDA-DIAG] Frame %d: pendingResults[0]=%zu, [1]=%zu, ridx=%d, widx=%d", m_frameCount,
            m_pendingResults[0].size(), m_pendingResults[1].size(), ridx, widx);
   ```

4. **Line 1646** - Change loop bound:
   ```cpp
   for (int i = 0; i < 2; ++i) {
   ```

## Verification

1. **Build test:** `just build V1_6_1170_CUDA_AVX2`
2. **Unit test:** Verify `test_cuda_double_buffer.cpp` passes (if exists)
3. **In-game test:**
   - Load save with multiple NPCs
   - Verify no physics glitches
   - Check Tracy for buffer usage
   - Run `smp timing 200` - should show same or better perf

## Why This Is Safe

1. **Sync guarantees ordering:** GPU sync at frame start ensures previous frame's writes are complete before CPU reads
2. **Buffers never overlap:** writeIndex and readIndex are always different (one is even, one is odd)
3. **Clear after read:** Read buffer is cleared in `applyResults()`, ready for reuse 1 frame later
4. **Existing tests:** The deferred collision pipeline is already well-tested

## Memory Savings

- Before: 3 × `sizeof(vector<PendingCollisionResult>)` + 3 events
- After: 2 × `sizeof(vector<PendingCollisionResult>)` + 2 events
- Actual savings depend on pair count, but ~33% reduction in pending results overhead
