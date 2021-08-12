# physX.js

physX (aka PhysX_4.1.1.27006925) compiled by Emscripten SDK.

## Requirement

1. *nix System
2. Cmake
3. [emsdk](https://emscripten.org)

## Usage

Following the instruction to [install emsk](https://emscripten.org/docs/getting_started/downloads.html) (the default
location is in $HOME)

The repo provides simple shell script to compile the wasm and glue Javascript files:

```shell
./build.sh
```

You can modify build.sh to set the number of CPU cores used when compiling and different compile target, including:

1. Release(default, the smallest size)
2. Profile
3. Checked
4. Debug

After finish running build.sh, there will be a folder call wasm_build which including wasm binary files and JavaScript
code used to import the wasm files into the browsers. For convenience, there will also be a link to the binding code in
the folder, you can modify the code as you wish and re-run build.sh to generate the new wasm related files.

## Detailed explanation

You can ignore this part when you first use this repo and look at it when you need to modify the building system for
your own needs. And you can also use this information to build other C++ projects like bullet and box2d.

Emscripten SDK provide a lot of tools to compile C++ code into wasm. In this repo, we will mainly use Embind, emcmake
and emmake. The compiling process is separated into several steps:

1. Based on the Linux cmake, use  'emcmake cmake' to replace 'cmake' and generated Makefile, which you can find in
   physx/buildtools/cmake_generate_projects.py. If the location of emsdk is not in $HOME, you have to modify '
   -DCMAKE_TOOLCHAIN_FILE' to the Emscripten.cmake in you computer.
2. After generated the Makefile by using cmake, you can call 'emmake make' to compile the Physx source code into static
   libraries.
3. The most important steps is to write PxWebBindings.cpp, which will be compiled by using em++ and linked the static
   libraries. After that the wasm and JavaScript glue code will be generated according to the compile flag.(-O3 will
   compress the wasm binary and delete the blank in the glue files.)

### PxWebBindings

Writing PxWebBindings needs to comply with the requirements
of [Embind](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html) . Here are a few more typical
usages：

```c++
constant("PX_PHYSICS_VERSION", PX_PHYSICS_VERSION);

function("PxCreateFoundation", &PxCreateFoundation, allow_raw_pointers());

class_<PxScene>("PxScene")
.function("setGravity", &PxScene::setGravity)
.function("getGravity", &PxScene::getGravity);

value_object<PxVec3>("PxVec3")
.field("x", &PxVec3::x)
.field("y", &PxVec3::y)
.field("z", &PxVec3::z);

enum_<PxQueryHitType::Enum>("PxQueryHitType")
.value("eNONE", PxQueryHitType::Enum::eNONE)
.value("eBLOCK", PxQueryHitType::Enum::eBLOCK)
.value("eTOUCH", PxQueryHitType::Enum::eTOUCH);
```

Special note:

1. When the function has raw pointer, it should be marked by allow_raw_pointers().
2. In some class, the member function will have const and non-const version, the single & notation can't decide which
   version is choose. you must use the static_cast<Return(Class::*)(Parameter...)>(&Class::Function) to determine the
   function signature.

The size of wasm files is depended on the binding APIs, you can only bind the API you need to reduce its size. And the
size of glue file will not change dramatically when you add more binding APIs.

## Debug

[NVIDIA PhysX Visual Debugger(PVD)](https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/VisualDebugger.html)
can be used to debug the physical scene. For minimizing the size of WASM binary, release target don't provide pvd
related APIs. So if you want to link PhysX to PVD, **use debug, profile or checked instead**. The last but not least, PVD can only be
installed on Windows.

There are three steps to use PVD:

1. [Downloads](https://developer.nvidia.com/physx-visual-debugger) and install PVD. You can compile PhysX on Windows and
   run Snippets to test whether it works well.
2. Install Node on Windows(not in Windows Subsystem Linux) and
   clone [websockify-js](https://github.com/novnc/websockify-js/tree/8c0d3e990ca794d078d08d9db29043f56560a18b). Run
   websockify-js by using

```shell
npm install
node .\websockify.js SOURCE_ADDR:PORT 127.0.0.1:5425
```

SOURCE_ADDR:PORT will be used in your WebSocket program. On the other side, 127.0.0.1:5425 is the default TARGET_ADDR:PORT for PVD.

3. Create WebSocket in your code and use it to create the instance of PxPVDTransport.

```JavaScript
const pvdTransport = PhysX.PxPvdTransport.implement({
    connect: function () {
        socket = new WebSocket('ws://SOURCE_ADDR:PORT', ['binary'])
        socket.onopen = () => {
            console.log('Connected to PhysX Debugger');
            queue.forEach(data => socket.send(data));
            queue = []
        }
        socket.onclose = () => {
        }
        return true
    },
    disconnect: function () {
        console.log("Socket disconnect")
    },
    isConnected: function () {
    },
    write: function (inBytes, inLength) {
        const data = PhysX.HEAPU8.slice(inBytes, inBytes + inLength)
        if (socket.readyState === WebSocket.OPEN) {
            if (queue.length) {
                queue.forEach(data => socket.send(data));
                queue.length = 0;
            }
            socket.send(data);
        } else {
            queue.push(data);
        }
        return true;
    }
})

const gPvd = PhysX.PxCreatePvd(foundation);
gPvd.connect(pvdTransport, new PhysX.PxPvdInstrumentationFlags(PhysX.PxPvdInstrumentationFlag.eALL.value));

physics = PhysX.PxCreatePhysics(
    version,
    foundation,
    new PhysX.PxTolerancesScale(),
    true,
    gPvd
)
```

## Acknowledgement

This repo based on the work being done over at [prestomation/PhysX](https://github.com/prestomation/PhysX) to create
emscripten bindings for [NVIDIAGameWorks/PhysX](https://github.com/NVIDIAGameWorks/PhysX).  





