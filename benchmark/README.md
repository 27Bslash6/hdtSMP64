# Automated Benchmark System

Programmatic performance testing with Tracy profiling and MCP semantic analysis.

## Quick Start

```bash
# Run all commands from project root (hdtSMP64/)

# 1. Download Tracy tools (one-time)
just setup-tracy

# 2. Install Python dependencies
cd benchmark && uv sync && cd ..

# 3. Configure Skyrim paths
cd benchmark && cp .env.example .env
# Edit benchmark/.env with your MO2_PATH or SKYRIM_PATH
cd ..

# 4. Run benchmark
just bench baseline 2000
```

## Configuration

Create `.env` in this directory:

```bash
# For Mod Organizer 2 users (recommended)
MO2_PATH=C:\Modding\MO2\ModOrganizer.exe
MO2_EXECUTABLE=SKSE
MO2_PROFILE=Benchmark Profile  # Optional

# OR for vanilla/direct launch
SKYRIM_PATH=C:\Program Files (x86)\Steam\steamapps\common\Skyrim Special Edition\SkyrimSE.exe
```

## Usage

```bash
# Via justfile (recommended)
just bench baseline 2000
just bench highway_on 2000
just bench-compare baseline highway_on
just bench-list
just bench-clean

# Direct Python invocation (if needed)
uv run benchmark_pipeline.py run --name baseline --frames 2000
uv run benchmark_pipeline.py run --name test --frames 2000 --skyrim-exe "D:\Custom\Path\SkyrimSE.exe"
```

## Directory Structure

```
benchmark/
  benchmark_pipeline.py  # Main automation script
  pyproject.toml         # Python dependencies (uv)
  .env.example          # Configuration template
  .env                  # Your local config (gitignored)
  README.md             # This file
```

## Documentation

See [docs/BENCHMARK_SYSTEM.md](../docs/BENCHMARK_SYSTEM.md) for complete technical documentation including:
- Architecture and data flow
- XML configuration
- Security considerations
- Troubleshooting
- Best practices
