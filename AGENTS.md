# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Project Overview

This is physX.js - a WebAssembly (WASM) port of NVIDIA PhysX 4.1.1 physics engine compiled using Emscripten SDK for web browsers. The project provides JavaScript bindings to access PhysX functionality in web applications.

## Build System

### Prerequisites
- *nix system (Linux/macOS)  
- CMake
- Emscripten SDK (emsdk) version 3.1.0 (tested)

### Core Build Commands

```bash
# Full build
./build.sh

# Clean build artifacts  
./clean.sh
```

The build process:
1. Generates PhysX projects for Emscripten using `./physx/generate_projects.sh emscripten`
2. Compiles PhysX source into static libraries using `emmake make -j4`
3. Creates WASM output in `wasm_build/` directory containing:
   - `physx.release.wasm` - compiled WebAssembly binary
   - `physx.release.js` - JavaScript glue code
   - `PxWebBindings.cpp` - symlinked binding source

### Build Targets
- Release (default, smallest size)
- Profile 
- Checked
- Debug

Modify `build.sh` to change build target and thread count.

## Architecture

### Core Structure
- `/physx/` - PhysX 4.1.1 source code and build system
- `/physx/source/physxwebbindings/` - Emscripten bindings
- `/pxshared/` - PhysX shared foundation libraries
- `/wasm_build/` - Generated WASM output
- `/cmake-build-debug/` - CMake build artifacts

### Binding System
The project uses Emscripten Embind to expose PhysX C++ APIs to JavaScript:

**Main binding file**: `physx/source/physxwebbindings/src/PxWebBindings.cpp`

**Modular binding headers**:
- `ActorBinding.h` - Rigid bodies and actors
- `ControllerBinding.h` - Character controllers  
- `CookingBinding.h` - Mesh cooking/preprocessing
- `JointBinding.h` - Physics joints and constraints
- `MathBinding.h` - Math types (PxVec3, PxQuat, PxTransform)
- `PVDBinding.h` - PhysX Visual Debugger support
- `SceneBinding.h` - Physics scenes and simulation  
- `ShapeBinding.h` - Collision shapes
- `QueryBinding.h` - Scene queries (raycast, overlap, sweep)

### Binding Patterns
```cpp
// Constants
constant("PX_PHYSICS_VERSION", PX_PHYSICS_VERSION);

// Functions with raw pointers
function("PxCreateFoundation", &PxCreateFoundation, allow_raw_pointers());

// Classes with methods  
class_<PxScene>("PxScene")
    .function("setGravity", &PxScene::setGravity)
    .function("getGravity", &PxScene::getGravity);

// Value objects for math types
value_object<PxVec3>("PxVec3")
    .field("x", &PxVec3::x)
    .field("y", &PxVec3::y) 
    .field("z", &PxVec3::z);

// Enums
enum_<PxQueryHitType::Enum>("PxQueryHitType")
    .value("eNONE", PxQueryHitType::Enum::eNONE)
    .value("eBLOCK", PxQueryHitType::Enum::eBLOCK);
```

## Development Workflow

### Making Changes to Bindings
1. Edit binding files in `physx/source/physxwebbindings/src/binding/`
2. Or modify main `PxWebBindings.cpp`
3. Run `./build.sh` to regenerate WASM
4. Test with generated `wasm_build/` output

### PhysX Visual Debugger (PVD) 
- Only available in debug/profile/checked builds (not release)
- Requires Windows with PVD installed
- Uses WebSocket bridge via websockify-js for web connectivity
- See README debug section for full setup

## Current Status
- Modified file: `physx/source/physxwebbindings/src/binding/ControllerBinding.h`
- Recent commits focus on sweep/overlap queries and collision layer support

## TypeScript Integration
Reference implementations available in Oasis Engine:
- [@oasis-engine/physics-physx](https://github.com/oasis-engine/engine/tree/main/packages/physics-physx)
- [Interface definitions](https://github.com/oasis-engine/engine/tree/main/packages/design/src/physics)