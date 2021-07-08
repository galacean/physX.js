#!/bin/bash

cd ./physx
./generate_projects.sh emscripten

cd ./compiler/emscripten-release
emmake make -j4 #thread number

cd ../../../
mkdir wasm_build
cd wasm_build
ln -sf ../physx/source/physxwebbindings/src/PxWebBindings.cpp .
cp ../physx/bin/emscripten/release/physx.release.js . 
cp ../physx/bin/emscripten/release/physx.release.wasm .
