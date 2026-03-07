# Automated Benchmark System

Programmatic performance testing infrastructure with Tracy profiling and semantic analysis.

## Overview

The benchmark system enables controlled A/B testing of performance optimizations by:
- Running fixed-length benchmarks (N frames)
- Automatically capturing Tracy profiles
- Exporting structured CSV data
- Optionally ingesting to MCP Qdrant for semantic queries

**Key Design Principle:** Reproducible benchmarks require fixed workload (same save file, same entity count, same location).

## Architecture

### Components

```
┌─────────────────────────────────────────────────────────────┐
│                   Benchmark Pipeline                         │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  1. Python Controller (benchmark_pipeline.py)               │
│     - Updates configs.xml                                   │
│     - Starts Tracy capture server                           │
│     - Launches Skyrim                                       │
│     - Monitors log file                                     │
│     - Exports CSV                                           │
│     - Ingests to MCP                                        │
│                                                              │
│  2. C++ Benchmark Mode                                      │
│     ┌────────────────────────────────────┐                  │
│     │ config.h: BenchmarkConfig struct   │                  │
│     │ config.cpp: XML parser             │                  │
│     │ hdtSkyrimPhysicsWorld.cpp:         │                  │
│     │   - Frame counting                 │                  │
│     │   - Auto-exit on completion        │                  │
│     └────────────────────────────────────┘                  │
│                                                              │
│  3. Tracy Profiler                                          │
│     - Captures zone timing data                             │
│     - Exports to CSV                                        │
│                                                              │
│  4. MCP Semantic Search (Optional)                          │
│     - Vector embeddings of zone names                       │
│     - Semantic queries like "solver bottleneck"             │
│     - Cross-benchmark comparison                            │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

```
User → Python Script → configs.xml
                     ↓
                  Skyrim.exe + Tracy Capture
                     ↓
              FrameTimer counts frames
                     ↓
         [N frames complete] → Auto-exit
                     ↓
              Tracy CSV Export
                     ↓
           MCP Qdrant Ingestion (optional)
                     ↓
          Semantic Performance Queries
```

## Configuration

### Skyrim Launch Configuration

The benchmark system supports **two launch modes**:

#### Mod Organizer 2 Launch (Recommended)

**For modded setups**, use MO2 to launch through its virtual filesystem:

```bash
# Edit .env
MO2_PATH=C:\Modding\MO2\ModOrganizer.exe
MO2_EXECUTABLE=SKSE
MO2_PROFILE=My Benchmark Profile  # Optional
```

**How it works:**
- Launches SKSE via `ModOrganizer.exe -p "Profile" "SKSE"`
- Ensures mods are loaded through MO2's virtual filesystem
- Use a dedicated benchmark profile for reproducibility

**Configuration:**
1. `MO2_PATH` - Path to ModOrganizer.exe (required)
2. `MO2_EXECUTABLE` - Name of executable in MO2 (default: "SKSE")
3. `MO2_PROFILE` - Profile name (optional, uses active if not specified)

#### Direct Skyrim Launch (Fallback)

**For vanilla or manual mod management**, launch SkyrimSE.exe directly:

**Method 1: Command-line argument (highest priority):**
```bash
cd benchmark && uv run benchmark_pipeline.py run --name test --skyrim-exe "D:\Games\Skyrim SE\SkyrimSE.exe"
```

**Method 2: Environment variable:**
```bash
# Edit .env
SKYRIM_PATH=D:\Games\Skyrim Special Edition\SkyrimSE.exe
```

**Method 3: Auto-detection (fallback):**
- Checks Steam registry (AppID 489830)
- Checks common installation paths (C:, D:, E: drives)

**Note:** Direct launch bypasses MO2's virtual filesystem, so mods won't load unless using manual mod installation methods.

### XML Configuration

Add to `configs/configs.xml` inside `<configs>` root:

```xml
<benchmark
  enabled="false"
  save=""
  frames="2000"
  exit-when-done="true"
  suppress-ui="true"
  quiet-mode="false"
/>
```

**Attributes:**
- `enabled` (bool): Enable benchmark mode
- `save` (string): Save file name for auto-load (empty string = manual load)
- `frames` (int, 10-10000): Number of physics frames to capture
- `exit-when-done` (bool): Auto-exit after completion
- `suppress-ui` (bool): Hide HUD during benchmark (TODO)
- `quiet-mode` (bool): Disable audio (TODO)

### C++ Implementation

**Files:**
- `hdtSMP64/config.h` - `BenchmarkConfig` struct definition
- `hdtSMP64/config.cpp` - XML parsing via `parseBenchmarkConfig()`
- `hdtSMP64/hdtSkyrimPhysicsWorld.cpp:446-482` - Frame counting and auto-exit logic
- `hdtSMP64/main.cpp:895-914` - Auto-load save via BGSSaveLoadManager

**Auto-Load Implementation:**
- Uses `BGSSaveLoadManager::Load()` to programmatically load saves
- Triggered 5 seconds after `InputLoaded` event (main menu ready)
- Runs in background thread to avoid blocking game initialization
- Only activates if `save` attribute is non-empty in XML config

**Auto-Exit Implementation:**
- Thread-safe using `std::atomic<bool>` for state tracking
- `compare_exchange_strong()` ensures only ONE exit message posted
- Starts frame counting when physics systems become active
- Exits 2 seconds after frame limit reached to flush logs

## Usage

### Quick Start

```bash
# 0. Install Python dependencies (one-time setup)
cd benchmark && uv sync && cd ..

# 1. Download Tracy profiler tools (one-time setup)
just setup-tracy  # Downloads pre-built binaries

# 2. Configure Skyrim path (if auto-detection fails)
cd benchmark
cp .env.example .env
# Edit .env with your Skyrim/MO2 paths
cd ..

# 3. Create benchmark save in-game
#    - 12 NPCs with physics
#    - Save as: benchmark_12entities

# 4. Configure benchmark in configs.xml
# Add inside <configs> root:
# <benchmark enabled="true" save="benchmark_12entities" frames="2000" exit-when-done="true" />
#
# For manual load instead, use save=""

# 5. Run baseline (auto-loads save, runs 2000 frames, exits automatically)
just bench baseline 2000

# 6. Enable optimization (e.g., Highway SIMD)
# Edit configs.xml: <highway enabled="true" />

# 7. Run test
just bench highway_on 2000

# 8. Compare results
just bench-compare baseline highway_on
```

### Command Reference

**Justfile Commands (Recommended):**

```bash
# Download Tracy tools (one-time setup)
just setup-tracy  # Downloads v0.13.1 pre-built binaries

# Run benchmark
just bench <name> [frames]
just bench baseline          # 2000 frames (default)
just bench quick_test 500    # 500 frames
just bench stress_test 10000 # 10000 frames

# List benchmark results
just bench-list

# Compare two benchmarks
just bench-compare baseline test1

# Clean benchmark results
just bench-clean
```

**Python Commands (Alternative):**

```bash
# Direct Python invocation
cd benchmark && uv run benchmark_pipeline.py run --name <trace_name> --frames <N>
cd benchmark && uv run benchmark_pipeline.py run --name baseline --frames 2000
```

### Console Commands

Manual benchmark control (if not using Python pipeline):

```
smp timing 2000    # Start 2000-frame benchmark
smp stats          # Show current performance
smp metrics        # Toggle continuous metrics logging
```

## Output

### Log File

Location: `Documents/My Games/Skyrim Special Edition/SKSE/hdtSMP64.log`

```
[BENCHMARK] Mode enabled:
[BENCHMARK]   Save:
[BENCHMARK]   Frames: 2000
[BENCHMARK]   Exit when done: yes

... 2000 frames later ...

Timings over 1000 frames:
  CPU:
    Internal update mean 527.930 us, std 139.971 us
    Collision launch mean 835 us, std 335.604 us
    Collision process mean 360 us, std 93.639 us
    Total mean 20801 us, std 1951 us    ← Frame time
    Collision manifolds 170, std 15

[BENCHMARK] Complete: 2000 frames processed
[BENCHMARK] Exiting application in 2 seconds...
```

### Tracy CSV

Location: `results/<trace_name>/trace.csv`

Format:
```csv
name,src_file,src_line,total_ns,total_perc,counts,mean_ns,min_ns,max_ns,std_ns
JointSolverLoop,btSequentialImpulseConstraintSolverMt.cpp,1264,30973675416,22.499005,1489,20801662,14545469,34099594,1951316
StepSimulation,hdtSkinnedMeshWorld.cpp,115,30531977903,22.178159,1489,20505022,14352015,33512110,1924618
...
```

**Key Columns:**
- `mean_ns` - Average execution time
- `total_perc` - Percentage of total frame time
- `std_ns` - Standard deviation (consistency metric)

### MCP Semantic Queries

After ingestion to Qdrant:

```python
# Find solver bottlenecks
mcp__hdt-log__query_tracy(query="constraint solver performance")

# Get top hotspots
mcp__hdt-log__tracy_hotspots(limit=20, min_perc=1.0)

# Compare traces
mcp__hdt-log__tracy_hotspots(trace_name="baseline", limit=20)
mcp__hdt-log__tracy_hotspots(trace_name="highway_on", limit=20)
```

## Best Practices

### Creating Benchmark Saves

1. **Fixed Entity Count**: Always use same number of NPCs
2. **Consistent Location**: Stand in same spot, same camera angle
3. **Active Physics**: Ensure cloth/hair is moving (not frozen)
4. **Named Clearly**: `benchmark_12entities` not `quicksave3`

### Running Benchmarks

1. **Close Background Apps**: Browser, Discord, etc. affect timing
2. **Run Multiple Times**: Average 3 runs for reliability
3. **Check Std Deviation**: High std = inconsistent performance
4. **Warm Up First**: Skip first 100 frames (JIT/cache effects)
5. **Same Hardware State**: Don't benchmark during Windows Update

### Analyzing Results

1. **Focus on Percentages**: `total_perc` shows impact
2. **Zones >5% Matter Most**: Optimize the 80%, not the 20%
3. **Mean vs Std**: Low mean + high std = inconsistent (bad)
4. **Frame Budget**: Target <10ms for 60fps headroom

## Methodology

### Why Fixed Save Files?

**Problem:** Variable entity counts make benchmarks incomparable.

**Before:**
```
Test 1: 7 NPCs  → 15ms avg  ← Not comparable!
Test 2: 12 NPCs → 22ms avg  ← Different workload!
```

**After:**
```
Test 1: 12 NPCs → 20.8ms avg (baseline)
Test 2: 12 NPCs → 12.3ms avg (with optimization)
Delta: -41% ← Real improvement!
```

### Auto-Load vs Manual Load

**Auto-Load (Recommended for automated testing):**
- Set `save="benchmark_12entities"` in XML config
- Game automatically loads save 5 seconds after main menu appears
- Fully hands-off operation - no user interaction required
- Best for CI/CD, regression testing, or batch benchmarking

**Manual Load (Recommended for verification):**
- Leave `save=""` empty in XML config
- User loads save manually from main menu
- Allows verification of correct save, physics state, camera position
- Best for careful performance testing and A/B comparisons

### Frame Count Selection

- **500 frames** - Quick iteration (~8s at 60fps)
- **2000 frames** - Standard benchmark (~33s, good statistics)
- **10000 frames** - Stress test (~2.7min, maximum allowed)

Longer = better statistics, but diminishing returns after 2000.

## Security & Thread Safety

### Critical Bugs Fixed (2026-01-19)

**1. Race Condition in Auto-Exit**
- **Problem**: `static bool wasRunning` not thread-safe
- **Impact**: Multiple PostQuitMessage() calls → crash
- **Fix**: Changed to `std::atomic<bool>` with `compare_exchange_strong()`

**2. Path Traversal Vulnerability**
- **Problem**: No validation on trace name parameter
- **Attack**: `--name "../../../evil"` writes outside project
- **Fix**: Sanitize name with `Path(name).name` + reject path separators

**3. Infinite Loop in Log Tailing**
- **Problem**: No timeout if Skyrim crashes
- **Impact**: Script hangs forever
- **Fix**: Added 10-minute timeout with error message

### Input Validation

**C++ Side:**
```cpp
// Frames clamped to [10, 10000]
g_benchmarkConfig.clampFrames();
```

**Python Side:**
```python
# Sanitize trace name
safe_name = Path(name).name  # Strips "../"
if '/' in name or '\\' in name:
    return None  # Reject
```

## Troubleshooting

### Benchmark Never Completes

**Check 1: Is benchmark mode enabled?**
```xml
<benchmark enabled="true" frames="2000" />
```

**Check 2: Did FrameTimer start?**
```
# In-game console
smp timing 2000
```

**Check 3: Check log for errors**
```
# Look for [BENCHMARK] markers
tail Documents/My\ Games/Skyrim\ Special\ Edition/SKSE/hdtSMP64.log
```

### Script Hangs

**Timeout:** Script has 10-minute timeout. If Skyrim crashes, wait for:
```
[ERROR] Timeout after 600s waiting for benchmark
```

**Manual exit:** Ctrl+C kills script. Tracy capture continues until you kill tracy-capture process.

### Tracy Not Found

**Error:** `[ERROR] Tracy capture tool not found`

**Solution:** Download Tracy profiler tools:
```bash
just setup-tracy
```

This downloads pre-built `tracy-capture.exe` and `tracy-csvexport.exe` (v0.13.1) from the official Tracy releases and installs them to `tools/tracy/`.

**Requirements:** 7-Zip must be installed and in PATH for extraction.

### Skyrim Path Wrong

**Edit benchmark_pipeline.py line 154:**
```python
skyrim_exe = r"D:\Games\Skyrim Special Edition\SkyrimSE.exe"
```

## Performance Targets

### Frame Time Budget

```
Target: <10ms per physics frame

Breakdown:
- Solver: ~5ms (50%)
- Collision: ~2ms (20%)
- AABB Update: ~1ms (10%)
- Other: ~2ms (20%)
```

### Scaling Expectations

```
Entities vs Frame Time (empirical):
- 5 NPCs:   ~8ms
- 10 NPCs:  ~15ms
- 15 NPCs:  ~25ms
- 20 NPCs:  ~40ms (not viable)

Goal: Push 10 NPCs to <10ms
```

## Integration with Development Workflow

### Pre-Commit Benchmark

```bash
# Before committing performance changes
just bench baseline  # Baseline
# ... make changes ...
just bench after_change
# ... compare results ...
git commit -m "perf: optimize solver (20ms → 12ms)"
```

### Regression Detection

```bash
# Automated check
if python scripts/check_regression.py baseline new_build; then
  echo "✓ No regression"
else
  echo "✗ Performance regression detected!"
  exit 1
fi
```

## Limitations

### What This Doesn't Do

1. **GPU profiling** - Tracy captures CPU only (CUDA has separate metrics)
2. **Memory profiling** - No allocation tracking
3. **Multi-process** - Only profiles hdtSMP64.dll, not whole game
4. **Auto-load saves** - User must load manually (intentional design)

### Known Issues

1. **Config file race** - Python writes while Skyrim reads (rare, low impact)
2. **Hardcoded paths** - Skyrim.exe location not auto-detected
3. **CSV size limits** - No cap on CSV size (10K frames = multi-GB)

## Future Enhancements

### Planned Features

- [ ] Auto-detect Skyrim path via registry
- [ ] File locking for config writes
- [ ] CSV size limits and downsampling
- [ ] Suppress UI rendering during benchmark
- [ ] Audio muting (quiet-mode)
- [ ] Multi-benchmark comparison report generator

### Research Topics

- [ ] Statistical significance testing (t-test for A/B)
- [ ] Automated regression detection
- [ ] Continuous benchmarking system
- [ ] Heat map visualization of zone times

## References

- **Tracy Profiler**: https://github.com/wolfpld/tracy
- **Implementation PR**: (link to PR after merge)
- **Security Review**: docs/BENCHMARK_SYSTEM_REVIEW.md (if created)
- **Python Pipeline**: benchmark/benchmark_pipeline.py
- **Usage Guide**: scripts/README_BENCHMARK.md

## Changelog

### 2026-01-19 - Initial Implementation
- Added BenchmarkConfig struct to config.h
- Implemented XML parsing in config.cpp
- Added auto-exit logic in hdtSkyrimPhysicsWorld.cpp
- Created benchmark_pipeline.py automation script
- Fixed critical thread-safety bug (atomic wasRunning)
- Fixed path traversal vulnerability
- Added timeout to prevent infinite loops

---

**Last Updated:** 2026-01-19
**Status:** Production Ready (after critical fixes)
**Maintainer:** @fish
