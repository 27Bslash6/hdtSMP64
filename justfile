# hdtSMP64 build tasks
# Run `just --list` to see all available tasks

# Use PowerShell on Windows
set shell := ["powershell.exe", "-NoLogo", "-Command"]

# Default configuration
default_config := "V1_6_1170_NOCUDA_AVX"
msbuild := "C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/MSBuild/Current/Bin/amd64/MSBuild.exe"
cuda_path := "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.9/"

# Default task: check then build
default: check build

# Run cppcheck static analysis
check:
    @echo "Running cppcheck..."
    powershell.exe -NoProfile -File scripts/cppcheck.ps1

# Build the default configuration
build config=default_config:
    @echo "Building {{config}}..."
    & "{{msbuild}}" hdtSMP64.sln -p:Configuration={{config}} -p:Platform=x64 "-p:CudaToolkitDir={{cuda_path}}" -v:m -m

# Full rebuild (clean + build)
rebuild config=default_config:
    @echo "Rebuilding {{config}}..."
    & "{{msbuild}}" hdtSMP64.sln -p:Configuration={{config}} -p:Platform=x64 "-p:CudaToolkitDir={{cuda_path}}" -v:m -m -t:Rebuild

# Clean build artifacts
clean config=default_config:
    @echo "Cleaning {{config}}..."
    & "{{msbuild}}" hdtSMP64.sln -p:Configuration={{config}} -p:Platform=x64 -v:m -t:Clean

# Build and run tests
test:
    @echo "Building tests..."
    & "{{msbuild}}" tests/hdtSMP64_tests.vcxproj -p:Configuration=Release -p:Platform=x64 -v:m
    @echo "Running tests..."
    & .\tests\x64\tests\Release\hdtSMP64_tests.exe --reporter console

# Full CI pipeline: check, build, test
ci: check build test
    @echo "CI pipeline complete!"

# Download Tracy profiler tools (pre-built binaries v0.13.1)
setup-tracy:
    @echo "Downloading Tracy profiler tools..."
    @if (!(Test-Path tools/tracy)) { New-Item -ItemType Directory -Force tools/tracy | Out-Null }
    @if (Test-Path tools/tracy/tracy-capture.exe) { echo "Tracy tools already installed (run 'just clean-tracy' to reinstall)"; exit 0 }
    @echo "Downloading Tracy v0.13.1 (~76MB)..."
    @$ProgressPreference = 'SilentlyContinue'; Invoke-WebRequest -Uri "https://github.com/wolfpld/tracy/releases/download/v0.13.1/windows-0.13.1.zip" -OutFile "tools/tracy/tracy.zip" -UseBasicParsing
    @echo "Extracting tools..."
    @Expand-Archive -Path "tools/tracy/tracy.zip" -DestinationPath "tools/tracy" -Force
    @Remove-Item "tools/tracy/tracy.zip"
    @echo ""
    @echo "Tracy v0.13.1 installed successfully:"
    @echo "  capture:   tools/tracy/tracy-capture.exe"
    @echo "  csvexport: tools/tracy/tracy-csvexport.exe"

# Clean downloaded Tracy tools
clean-tracy:
    @echo "Removing Tracy tools..."
    @if (Test-Path tools/tracy) { Remove-Item -Recurse -Force tools/tracy; echo "Tracy tools removed" } else { echo "Tracy tools not found" }

# Run automated benchmark (requires Tracy tools - run 'just setup-tracy' first)
bench name frames="2000":
    @echo "Starting benchmark: {{name}}"
    @echo "Frames: {{frames}}"
    @echo ""
    cd benchmark; uv run benchmark_pipeline.py run --name {{name}} --frames {{frames}}

# List available benchmark results
bench-list:
    @echo "Available benchmark results:"
    @echo ""
    @if (Test-Path results) { Get-ChildItem results -Directory | ForEach-Object { echo "  $($_.Name)" } } else { echo "  (no benchmarks found)" }
    @echo ""
    @echo "Run benchmarks with: just bench <name> [frames]"

# Compare two benchmark results
bench-compare baseline test:
    @echo "Comparing benchmarks:"
    @echo "  Baseline: {{baseline}}"
    @echo "  Test:     {{test}}"
    @echo ""
    @$base = "results/{{baseline}}/trace.csv"; $tst = "results/{{test}}/trace.csv"; if (!(Test-Path $base)) { echo "ERROR: Baseline not found: $base"; exit 1 }; if (!(Test-Path $tst)) { echo "ERROR: Test not found: $tst"; exit 1 }; echo "Baseline CSV: $base"; echo "Test CSV:     $tst"; echo ""; echo "Use Tracy GUI or CSV analysis tools to compare"; echo "Key metrics: mean_ns, total_perc, std_ns"

# Clean benchmark results
bench-clean:
    @echo "Cleaning benchmark results..."
    @if (Test-Path results) { Remove-Item -Recurse -Force results/* -ErrorAction SilentlyContinue; echo "Benchmark results cleaned" } else { echo "No results directory found" }

# Build all common CPU configurations (unified _AVX base, Highway handles runtime dispatch)
build-all:
    @echo "Building all CPU configurations..."
    just build V1_6_1170_NOCUDA_AVX
    just build SE_NOCUDA_AVX
    just build VR_NOCUDA_AVX

# Build all CUDA configurations (requires CUDA Toolkit)
build-all-cuda:
    @echo "Building all CUDA configurations..."
    just build V1_6_1170_CUDA_AVX
    just build SE_CUDA_AVX
    just build VR_CUDA_AVX

# Quick build (skip cppcheck)
quick config=default_config:
    just build {{config}}

# Build with Tracy profiler enabled
profile config=default_config:
    @echo "Building {{config}} with Tracy profiler..."
    & "{{msbuild}}" hdtSMP64.sln -p:Configuration={{config}} -p:Platform=x64 "-p:CudaToolkitDir={{cuda_path}}" "-p:ForceImportBeforeCppTargets={{justfile_directory()}}\hdtSMP64\Tracy.props" -v:m -m

# List available build configurations
configs:
    @echo "Available configurations:"
    @echo ""
    @echo "  CPU-only (no CUDA required):"
    @echo "    V1_6_1170_NOCUDA_AVX  (default — AE 1.6.1170)"
    @echo "    SE_NOCUDA_AVX         (Skyrim SE 1.5.97)"
    @echo "    VR_NOCUDA_AVX         (Skyrim VR)"
    @echo ""
    @echo "  CUDA (requires CUDA Toolkit 12.x):"
    @echo "    V1_6_1170_CUDA_AVX"
    @echo "    SE_CUDA_AVX"
    @echo "    VR_CUDA_AVX"
    @echo ""
    @echo "  Highway SIMD provides automatic runtime dispatch (SSE4 → AVX2 → AVX512)."
    @echo "  One binary supports all x86-64 CPUs — no separate AVX2/AVX512 builds needed."
    @echo ""
    @echo "Usage: just build <config>"
    @echo "       just profile <config>  (with Tracy profiler)"
    @echo "       just cuda-info         (check CUDA installation)"

# Check CUDA Toolkit installation
cuda-info:
    @echo "Checking CUDA Toolkit installation..."
    @$nvcc = Get-Command nvcc -ErrorAction SilentlyContinue; if ($nvcc) { nvcc --version } else { echo ""; echo "CUDA Toolkit NOT FOUND"; echo ""; echo "Install from: https://developer.nvidia.com/cuda-downloads"; echo "Select: Windows > x86_64 > 12 > exe (local)"; echo "Required: CUDA Toolkit 12.x with nvcc compiler" }
