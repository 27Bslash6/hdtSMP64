# hdtSMP64 build tasks
# Run `just --list` to see all available tasks

# Use PowerShell on Windows
set shell := ["powershell.exe", "-NoLogo", "-Command"]

# Default configuration
default_config := "V1_6_1170_NOCUDA_AVX2"
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

# Build all common CPU configurations
build-all:
    @echo "Building all CPU configurations..."
    just build V1_6_1170_NOCUDA_NoAVX
    just build V1_6_1170_NOCUDA_AVX
    just build V1_6_1170_NOCUDA_AVX2
    just build V1_6_1170_NOCUDA_AVX512

# Build all CUDA configurations (requires CUDA Toolkit)
build-all-cuda:
    @echo "Building all CUDA configurations..."
    just build V1_6_1170_CUDA_NoAVX
    just build V1_6_1170_CUDA_AVX
    just build V1_6_1170_CUDA_AVX2
    just build V1_6_1170_CUDA_AVX512

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
    @echo "    V1_6_1170_NOCUDA_NoAVX"
    @echo "    V1_6_1170_NOCUDA_AVX"
    @echo "    V1_6_1170_NOCUDA_AVX2"
    @echo "    V1_6_1170_NOCUDA_AVX512"
    @echo "    SE_NOCUDA_AVX2"
    @echo "    VR_NOCUDA_AVX2"
    @echo ""
    @echo "  CUDA (requires CUDA Toolkit 12.x):"
    @echo "    V1_6_1170_CUDA_NoAVX"
    @echo "    V1_6_1170_CUDA_AVX"
    @echo "    V1_6_1170_CUDA_AVX2"
    @echo "    V1_6_1170_CUDA_AVX512"
    @echo "    SE_CUDA_AVX2"
    @echo "    VR_CUDA_AVX2"
    @echo ""
    @echo "Usage: just build <config>"
    @echo "       just profile <config>  (with Tracy profiler)"
    @echo "       just cuda-info         (check CUDA installation)"

# Check CUDA Toolkit installation
cuda-info:
    @echo "Checking CUDA Toolkit installation..."
    @$nvcc = Get-Command nvcc -ErrorAction SilentlyContinue; if ($nvcc) { nvcc --version } else { echo ""; echo "CUDA Toolkit NOT FOUND"; echo ""; echo "Install from: https://developer.nvidia.com/cuda-downloads"; echo "Select: Windows > x86_64 > 12 > exe (local)"; echo "Required: CUDA Toolkit 12.x with nvcc compiler" }
