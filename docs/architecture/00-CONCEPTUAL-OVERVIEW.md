# Level 0: Conceptual Overview

**Audience**: Beginners, newcomers to the codebase, modders curious about internals
**Prerequisites**: Basic understanding of 3D games, no C++ required

---

## What is hdtSMP64?

hdtSMP64 is a **physics simulation plugin** for Skyrim Special Edition (and VR/AE). It makes hair, clothing, and accessories move realistically when characters walk, run, or fight.

```
Without Physics:              With Physics:
┌─────────────┐               ┌─────────────┐
│  Character  │               │  Character  │
│   ┌───┐     │               │   ┌~~~┐     │ ← Hair sways
│   │   │     │               │   │   │     │
│   └───┘     │               │   └~~~┘     │ ← Cape flows
│  Static!    │               │  Dynamic!   │
└─────────────┘               └─────────────┘
```

## The Problem We Solve

Skyrim's default animation system is **keyframe-based**: animators pre-define how things move. This works for walking cycles, but:

1. **Hair doesn't react to movement** - Running doesn't make ponytails bounce
2. **Capes are stiff** - They don't flutter in wind or flap when you turn
3. **Accessories are frozen** - Pouches, chains, and jewelry stay perfectly still

hdtSMP64 adds **real-time physics simulation** that responds to:
- Character movement (velocity, acceleration)
- Environmental forces (gravity, wind)
- Collisions (body parts, armor pieces)

## Core Concepts

### 1. Physics Bodies

Every movable part becomes a "physics body" - an object the physics engine can simulate.

```
Hair Strand Example:
                        ← Fixed to head (kinematic)
    ┌─○─┐               ← Bone 1 (can swing)
    │   │
    ○───┤               ← Bone 2 (can swing)
    │   │
    └─○─┘               ← Bone 3 (can swing)
        ↓ gravity
```

**Types of Bodies:**
- **Kinematic**: Follow the game animation exactly (head, hands, torso)
- **Dynamic**: Simulated by physics (hair tips, cape edges)

### 2. Constraints

Bodies are connected by **constraints** - invisible springs and joints that keep things attached.

```
Constraint Types:
┌──────────────────────────────────────────────────┐
│                                                  │
│   Ball Joint        Hinge            Spring      │
│      ○                ○               ○~~~○      │
│     /│\              ═╪═              ←stretch→  │
│    / │ \             / \                         │
│   ○  ○  ○           ○   ○                        │
│                                                  │
│   360° freedom    Single axis     Elastic pull   │
└──────────────────────────────────────────────────┘
```

### 3. Collisions

Without collision detection, hair would pass through the character's body. The **collision system** detects when two objects intersect and pushes them apart.

```
Before Collision:     During Detection:     After Resolution:
     ○                     ○                     ○
     │                     │╲                    │ ╲
     ○        →       ┌────○─┐      →           ○  │
     │                │    │  │                 │  │
  ┌──┴──┐             └────┴──┘              ┌──┴──┘
  │BODY │                                    │BODY │
```

### 4. The Simulation Loop

Every frame, the physics engine runs a simulation "step":

```
┌─────────────────────────────────────────────────────────────┐
│                    PHYSICS FRAME CYCLE                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. READ TRANSFORMS                                         │
│     └─ Get current bone positions from game                 │
│                         ↓                                   │
│  2. APPLY FORCES                                            │
│     └─ Gravity, wind, player movement                       │
│                         ↓                                   │
│  3. PREDICT MOTION                                          │
│     └─ Where would bodies move if unconstrained?            │
│                         ↓                                   │
│  4. DETECT COLLISIONS                                       │
│     └─ Which bodies are intersecting?                       │
│                         ↓                                   │
│  5. SOLVE CONSTRAINTS                                       │
│     └─ Adjust positions to satisfy joints/springs           │
│                         ↓                                   │
│  6. INTEGRATE                                               │
│     └─ Apply final velocities to update positions           │
│                         ↓                                   │
│  7. WRITE TRANSFORMS                                        │
│     └─ Send new bone positions back to game                 │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Where Does This Code Run?

hdtSMP64 hooks into Skyrim via SKSE (Skyrim Script Extender):

```
┌─────────────────────────────────────────────────────────────┐
│                      SKYRIM GAME                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │
│  │   Rendering  │  │   Scripts    │  │   Animation  │       │
│  └──────────────┘  └──────────────┘  └──────────────┘       │
│         │                 │                 │               │
│         └─────────────────┼─────────────────┘               │
│                           │                                 │
│                    ┌──────▼──────┐                          │
│                    │    SKSE     │ ← Plugin interface        │
│                    └──────┬──────┘                          │
└───────────────────────────┼─────────────────────────────────┘
                            │
                    ┌───────▼───────┐
                    │   hdtSMP64    │ ← Our physics plugin
                    │  ┌─────────┐  │
                    │  │ Bullet  │  │ ← Physics engine
                    │  └─────────┘  │
                    └───────────────┘
```

## The Bullet Physics Engine

hdtSMP64 uses **Bullet Physics** - the same engine used in:
- Grand Theft Auto V
- Red Dead Redemption 2
- Many Hollywood films (special effects)

We've customized Bullet for the specific needs of cloth/hair simulation:
- Optimized collision detection for skinned meshes
- Special constraint types for realistic fabric behavior
- Parallel processing for better performance

## Performance Considerations

Physics simulation is computationally expensive. hdtSMP64 uses several strategies:

| Strategy | What It Does |
|----------|--------------|
| **Skeleton Limiting** | Only N closest characters get physics |
| **Substep Clamping** | Max 4 physics steps per frame |
| **Parallel Processing** | Use all CPU cores via enkiTS |
| **Collision Culling** | Skip pairs that can't possibly collide |

## Next Steps

Now that you understand the concepts:

1. **[Level 1: High-Level Pipeline](./01-HIGH-LEVEL-PIPELINE.md)** - How the code is organized
2. **[Level 2: Frame Lifecycle](./02-FRAME-LIFECYCLE.md)** - What happens each frame
3. **[Level 3: Collision Detection](./03-COLLISION-PIPELINE.md)** - How collisions work
