#!/bin/bash
set -e

# Ensure no stale SIMD env from previous failed runs
unset PX_ENABLE_SIMD

# Build PhysX WebBindings twice:
# 1) NO_DYNAMIC_EXECUTION=1, WASM -> outputs: wasm_build/physx.release.js and wasm_build/physx.release.wasm
# 2) NO_DYNAMIC_EXECUTION=1, WASM + SIMD -> outputs: wasm_build/physx.release.simd.js and wasm_build/physx.release.simd.wasm

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$ROOT_DIR/physx/compiler/emscripten-release"
BIN_DIR="$ROOT_DIR/physx/bin/emscripten/release"
OUT_DIR="$ROOT_DIR/wasm_build"

mkdir -p "$OUT_DIR"

run_generate_and_make() {
  echo "🛠️  生成项目 (emscripten) ..."
  (cd "$ROOT_DIR/physx" && ./generate_projects.sh emscripten)

  echo "🏗️  编译 (emmake make) ..."
  (cd "$BUILD_DIR" && emmake make -j4)
}

# 1) 正常构建：WASM + JS
echo "=== 1/2: 正常构建 (NO_DYNAMIC_EXECUTION=1, WASM) ==="
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

# 2) SIMD 构建：WASM + SIMD
echo "=== 2/2: SIMD 构建 (NO_DYNAMIC_EXECUTION=1, WASM + SIMD) ==="
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

echo "✅ 构建完成。产物位于：$OUT_DIR"
