cmake_minimum_required(VERSION 3.23)
project(Physx.js)

set(CMAKE_CXX_STANDARD 14)
set(EMSCRIPTEN_ROOT $ENV{EMSDK}/upstream/emscripten CACHE STRING "Emscripten path")

add_library(physx.js physx/source/physxwebbindings/src/PxWebBindings.cpp)

include_directories(
        physx/include
        physx/include/characterkinematic
        physx/include/collision
        physx/include/common
        physx/include/cooking
        physx/include/extensions
        physx/include/filebuf
        physx/include/foundation
        physx/include/geometry
        physx/include/geomutils
        physx/include/pvd
        physx/include/solver
        physx/include/task
        physx/include/vehicle
        physx/source/foundation/include
        pxshared/include
        "${EMSCRIPTEN_ROOT}/system/include"
)
