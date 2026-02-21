#!/bin/bash
set -e

# Ensure no stale SIMD env from previous failed runs
unset PX_ENABLE_SIMD

# Build PhysX WebBindings three times:
# 1) SINGLE_FILE=1, WASM=0 -> output: wasm_build/physx.release.downgrade.js
# 2) NO_DYNAMIC_EXECUTION=1, WASM -> outputs: wasm_build/physx.release.js and wasm_build/physx.release.wasm
# 3) NO_DYNAMIC_EXECUTION=1, WASM + SIMD -> outputs: wasm_build/physx.release.simd.js and wasm_build/physx.release.simd.wasm

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
CMAKE_FILE="$ROOT_DIR/physx/source/compiler/cmake/emscripten/PhysXWebBindings.cmake"
BUILD_DIR="$ROOT_DIR/physx/compiler/emscripten-release"
BIN_DIR="$ROOT_DIR/physx/bin/emscripten/release"
OUT_DIR="$ROOT_DIR/wasm_build"

mkdir -p "$OUT_DIR"

backup_cmake() {
  CMAKE_BACKUP_FILE="${CMAKE_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
  cp "$CMAKE_FILE" "$CMAKE_BACKUP_FILE"
  echo "🔒 CMake文件已备份：$CMAKE_BACKUP_FILE"
}

restore_cmake() {
  unset PX_ENABLE_SIMD
  if [ -n "$CMAKE_BACKUP_FILE" ] && [ -f "$CMAKE_BACKUP_FILE" ]; then
    mv "$CMAKE_BACKUP_FILE" "$CMAKE_FILE"
    echo "♻️ 已还原 CMake 文件到初始状态"
  fi
}

enable_single_file() {
  # 注释 NO_DYNAMIC_EXECUTION 版本，启用 SINGLE_FILE 版本
  sed -i.bak \
    -e 's|^SET(EMSCRIPTEN_BASE_OPTIONS "--bind -s MODULARIZE=1 -s EXPORT_NAME=PHYSX -s ALLOW_MEMORY_GROWTH=1 -s NO_DYNAMIC_EXECUTION=1")|## SET(EMSCRIPTEN_BASE_OPTIONS "--bind -s MODULARIZE=1 -s EXPORT_NAME=PHYSX -s ALLOW_MEMORY_GROWTH=1 -s NO_DYNAMIC_EXECUTION=1")|' \
    -e 's|^## SET(EMSCRIPTEN_BASE_OPTIONS "--bind -s MODULARIZE=1 -s EXPORT_NAME=PHYSX -s ALLOW_MEMORY_GROWTH=1 -s SINGLE_FILE=1 -s WASM=0")|SET(EMSCRIPTEN_BASE_OPTIONS "--bind -s MODULARIZE=1 -s EXPORT_NAME=PHYSX -s ALLOW_MEMORY_GROWTH=1 -s SINGLE_FILE=1 -s WASM=0")|' \
    "$CMAKE_FILE"
  rm -f "${CMAKE_FILE}.bak"
}

enable_wasm() {
  # 启用 NO_DYNAMIC_EXECUTION 版本，注释 SINGLE_FILE 版本
  sed -i.bak \
    -e 's|^## SET(EMSCRIPTEN_BASE_OPTIONS "--bind -s MODULARIZE=1 -s EXPORT_NAME=PHYSX -s ALLOW_MEMORY_GROWTH=1 -s NO_DYNAMIC_EXECUTION=1")|SET(EMSCRIPTEN_BASE_OPTIONS "--bind -s MODULARIZE=1 -s EXPORT_NAME=PHYSX -s ALLOW_MEMORY_GROWTH=1 -s NO_DYNAMIC_EXECUTION=1")|' \
    -e 's|^SET(EMSCRIPTEN_BASE_OPTIONS "--bind -s MODULARIZE=1 -s EXPORT_NAME=PHYSX -s ALLOW_MEMORY_GROWTH=1 -s SINGLE_FILE=1 -s WASM=0")|## SET(EMSCRIPTEN_BASE_OPTIONS "--bind -s MODULARIZE=1 -s EXPORT_NAME=PHYSX -s ALLOW_MEMORY_GROWTH=1 -s SINGLE_FILE=1 -s WASM=0")|' \
    "$CMAKE_FILE"
  rm -f "${CMAKE_FILE}.bak"
}

run_generate_and_make() {
  echo "🛠️  生成项目 (emscripten) ..."
  (cd "$ROOT_DIR/physx" && ./generate_projects.sh emscripten)

  echo "🏗️  编译 (emmake make) ..."
  (cd "$BUILD_DIR" && emmake make -j4)
}

# 备份CMake并确保还原
trap restore_cmake EXIT
backup_cmake

# 1) 降级构建：SINGLE_FILE=1, WASM=0
console_msg() { echo "$1"; }
console_msg "\n=== 1/3: 降级构建 (SINGLE_FILE=1, WASM=0) ==="
enable_single_file
run_generate_and_make

# 复制降级输出
if [ -f "$BIN_DIR/physx.release.js" ]; then
  cp "$BIN_DIR/physx.release.js" "$OUT_DIR/physx.release.downgrade.js"
  echo "📦 已生成降级版：$OUT_DIR/physx.release.downgrade.js"
else
  echo "❌ 未找到降级输出 $BIN_DIR/physx.release.js"
  exit 1
fi

# 2) 正常构建：WASM + JS
console_msg "\n=== 2/3: 正常构建 (NO_DYNAMIC_EXECUTION=1, WASM) ==="
enable_wasm
run_generate_and_make

# 复制正常输出
if [ -f "$BIN_DIR/physx.release.js" ]; then
  cp "$BIN_DIR/physx.release.js" "$OUT_DIR/physx.release.js"
  echo "📦 已生成主版JS：$OUT_DIR/physx.release.js"
else
  echo "❌ 未找到主版输出 $BIN_DIR/physx.release.js"
  exit 1
fi

if [ -f "$BIN_DIR/physx.release.wasm" ]; then
  cp "$BIN_DIR/physx.release.wasm" "$OUT_DIR/physx.release.wasm"
  echo "📦 已生成WASM：$OUT_DIR/physx.release.wasm"
else
  echo "❌ 未找到主版WASM输出 $BIN_DIR/physx.release.wasm"
  exit 1
fi

# 3) SIMD 构建：WASM + SIMD
console_msg "\n=== 3/3: SIMD 构建 (NO_DYNAMIC_EXECUTION=1, WASM + SIMD) ==="
export PX_ENABLE_SIMD=1
run_generate_and_make
unset PX_ENABLE_SIMD

# 复制 SIMD 输出
if [ -f "$BIN_DIR/physx.release.js" ]; then
  cp "$BIN_DIR/physx.release.js" "$OUT_DIR/physx.release.simd.js"
  echo "📦 已生成SIMD JS：$OUT_DIR/physx.release.simd.js"
else
  echo "❌ 未找到SIMD输出 $BIN_DIR/physx.release.js"
  exit 1
fi

if [ -f "$BIN_DIR/physx.release.wasm" ]; then
  cp "$BIN_DIR/physx.release.wasm" "$OUT_DIR/physx.release.simd.wasm"
  echo "📦 已生成SIMD WASM：$OUT_DIR/physx.release.simd.wasm"
else
  echo "❌ 未找到SIMD WASM输出 $BIN_DIR/physx.release.wasm"
  exit 1
fi

# 可选：在输出目录放一个源码链接（与之前脚本保持一致）
ln -snf "$ROOT_DIR/physx/source/physxwebbindings/src/PxWebBindings.cpp" "$OUT_DIR/PxWebBindings.cpp"

echo "\n✅ 构建完成。产物位于：$OUT_DIR"
