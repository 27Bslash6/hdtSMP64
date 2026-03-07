# hdtSMP64 - Physics for Skyrim SE/AE/VR

> Cloth and hair physics simulation using Bullet Physics with optional CUDA acceleration

[![Game](https://img.shields.io/badge/Game-Skyrim%20SE%2FAE%2FVR-blue)]()
[![CUDA](https://img.shields.io/badge/CUDA-Optional-green)]()
[![License](https://img.shields.io/badge/License-MIT-yellow)]()

---

## Supported Versions

| Version | Platform | Game | Status |
|:--------|:---------|:-----|:------:|
| 1.5.97 | Steam | Skyrim SE | Supported |
| 1.4.15 | Steam | Skyrim VR | Supported |
| 1.6.353 | Steam | Anniversary Edition | Supported |
| 1.6.640 | Steam | Anniversary Edition | Supported |
| 1.6.1170 | Steam | Anniversary Edition (latest) | Supported |
| 1.6.659 | **GOG** | Anniversary Edition | Supported |
| 1.6.1179 | **GOG** | Anniversary Edition (latest) | Supported |

> [!IMPORTANT]
> GOG and Steam versions require **different binaries** due to DRM differences. Select the correct version for your platform in the FOMOD installer.

---

## Features

### Performance Optimizations
- **CUDA GPU Acceleration** - Offloads collision detection to GPU (NVIDIA only)
- **Distance-based Culling** - Disables physics for NPCs beyond configurable distance (default: 500 units)
- **FOV-based Culling** - Optional max angle setting to limit physics to visible NPCs
- **Dynamic Timesteps** - Prevents feedback loops from slow frames

### Collision System
- GPU-accelerated vertex position calculations
- GPU-accelerated bounding box calculations
- Sphere-sphere and sphere-triangle collision checks
- Configurable `can-collide-with-bone` and `no-collide-with-bone`
- External sharing option for inter-character collisions

### Quality of Life
- Improved armor handling to reduce gradual FPS loss
- Better skeleton tracking across cell transitions
- Head part physics support (with limitations)
- Mesh name remapping in defaultBBPs.xml

---

## Quick Start

### Installation
1. Install SKSE64 for your game version
2. Copy `hdtSMP64.dll` to `Data/SKSE/Plugins/`
3. Copy `configs/configs.xml` to `Data/SKSE/Plugins/hdtSkinnedMeshConfigs/`
4. (Optional) Enable CUDA in configs.xml or via console

### Console Commands

| Command | Description |
|---------|-------------|
| `smp` | Show command help |
| `smp fps [N]` | Set/show target FPS (30-240) |
| `smp skeletons [N]` | Set/show max active skeletons |
| `smp autoscale` | Toggle auto-scaling on/off |
| `smp save` | Save current settings to configs.xml |
| `smp stats` | Show performance + solver + actors |
| `smp reset` | Reload configs and reset physics |
| `smp reload` | Hot-reload configs.xml |
| `smp on/off` | Enable/disable physics |
| `smp gpu` | Toggle CUDA acceleration |
| `smp cuda [reset]` | CUDA metrics (toggle/reset) |
| `smp timing [N]` | Profile N frames (default: 200) |
| `smp metrics` | Toggle metrics logging |
| `smp list/detail` | Actor debug info |
| `smp dumptree` | Dump targeted NPC's node tree |

---

## CUDA Support

> **Requirements:** NVIDIA GPU with Compute Capability 5.0+ (Maxwell or newer)

CUDA acceleration is **disabled by default**. Enable via:
- `configs.xml`: Set `cuda-enabled="true"`
- Console: `smp gpu`

### GPU-Accelerated Operations
- Vertex position calculations for skinned meshes
- Collider bounding box calculations
- Aggregate bounding box for collider tree nodes
- Building collider lists for collision checks
- Sphere-sphere and sphere-triangle collision
- Collision result merging

### CPU-Only Operations
- Converting results to Bullet manifolds
- Constraint solver (Bullet library)

### Multi-GPU Support
Configure preferred GPU in `configs.xml`. Default uses device 0 (typically the most powerful card).

### AMD/Radeon
Not supported for GPU acceleration. CPU mode works with any graphics card.

---

## Build Instructions

### Prerequisites
- Visual Studio 2022 (or 2019)
- Git
- CMake
- CUDA Toolkit 12.x (optional, for GPU builds)

### Build Configurations

Format: `{VERSION}_{CUDA}`

| Component | Options |
|:----------|:--------|
| Version | `SE`, `VR`, `V1_6_353`, `V1_6_640`, `V1_6_1170`, `V1_6_1179` |
| CUDA | `CUDA` (GPU), `NOCUDA` (CPU-only) |

SIMD (SSE4/AVX2/AVX-512) is selected automatically at runtime by Google Highway. A single
binary works on all x86-64 CPUs — no separate AVX2 or AVX512 build is needed.

| Platform | Recommended Config |
|:---------|:-------------------|
| Steam AE | `V1_6_1170_NOCUDA` |
| GOG | `V1_6_1179_NOCUDA` |
| Steam SE | `SE_NOCUDA` |
| Steam VR | `VR_NOCUDA` |

### Using Just (Recommended)

```bash
just              # Build default config (V1_6_1170_NOCUDA)
just build        # Build only
just test         # Build + run tests
just configs      # List all configurations
just cuda-info    # Check CUDA installation

# Specific configurations
just build V1_6_1170_CUDA    # Steam AE with CUDA
just build V1_6_1179_NOCUDA  # GOG
just profile V1_6_1170_CUDA  # With Tracy profiler
```

### Manual Build

<details>
<summary>Click to expand manual build instructions</summary>

#### 1. Build Dependencies

Open x64 Native Tools Command Prompt:

```batch
mkdir Dev
cd Dev
git clone https://github.com/microsoft/Detours.git
git clone https://github.com/bulletphysics/bullet3.git

cd Detours
nmake
cd ..\bullet3
cmake .
```

For AVX support in Bullet, use `cmake-gui` and enable `USE_MSVC_AVX`.

Open `bullet3\BULLET_PHYSICS.sln`, select Release, Build Solution.

#### 2. Get SKSE Source

Download appropriate SKSE64 source for your target version. Extract to Dev folder.

#### 3. Get hdtSMP64

```batch
cd Dev\skse64_2_00_19\src\skse64
git init
git remote add origin https://github.com/Karonar1/hdtSMP64.git
git fetch
git checkout master
```

#### 4. Configure Project

In Visual Studio, open hdtSMP64 project properties:

**C/C++ > Additional Include Directories:**
```text
D:\Dev\Detours\include
D:\Dev\bullet3\src
D:\Dev\skse64_2_00_19\src
```

**Linker > Additional Library Directories:**
```text
D:\Dev\bullet3\lib\Release
D:\Dev\Detours\lib.X64
```

Change skse64 project Configuration Type to "Static Library (.lib)".

</details>

---

## NPC Head Parts

Head parts work for NPCs without facegen data, but triggers dark face bug.

### Limitations with Facegen Data
- Only one XML file per face (even with multiple physics-enabled parts)
- NiStringExtraData nodes not auto-copied - add manually or use defaultBBPs.xml mapping
- Unused bones removed during facegen - reference NPC head instead in constraints

---

## Known Issues

- Some bone shape/collision options exist but don't function
- `smp reset` may not reload all meshes correctly
- Bones not referenced by meshes can't be used as kinematic constraint objects
- Hair colors occasionally wrong with multiple NPCs using same hair model
- Physics may be less stable with dynamic timestep calculation

See [Nexus bug reports](https://nexusmods.com/skyrimspecialedition/mods/57339?tab=bugs) for current issues.

---

## Credits

| Contributor | Contribution |
|-------------|--------------|
| **hydrogensaysHDT** | Original plugin creation |
| **aers** | Fixes and improvements |
| **ousnius** | Fixes and consulting |
| **Karonar1** | Bug fixes and maintenance |

---

## Fork History

```text
hydrogensaysHDT/hdt-skyrimse-mods (original)
    └── aers/hdtSMP64
        └── Karonar1/hdtSMP64
            └── This fork
```
