#!/bin/bash
set -e

# Ensure no stale SIMD env from previous failed runs
unset PX_ENABLE_SIMD

# Build PhysX WebBindings:
# Release: physx.release.js/wasm + physx.release.simd.js/wasm
# Debug:   physx.debug.js/wasm   + physx.debug.simd.js/wasm (with PVD support)

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT_DIR="$ROOT_DIR/wasm_build"

mkdir -p "$OUT_DIR"

generate_projects() {
  echo "🛠️  生成项目 (emscripten) ..."
  (cd "$ROOT_DIR/physx" && ./generate_projects.sh emscripten)
}

build() {
  local build_type=$1  # release or debug
  local build_dir="$ROOT_DIR/physx/compiler/emscripten-$build_type"

  echo "🏗️  编译 $build_type (emmake make) ..."
  (cd "$build_dir" && emmake make -j4)
}

copy_output() {
  local build_type=$1  # release or debug
  local simd=$2        # "" or ".simd"
  local bin_dir="$ROOT_DIR/physx/bin/emscripten/$build_type"
  local src_name="physx.${build_type}"
  local dst_name="physx.${build_type}${simd}"

  if [ -f "$bin_dir/${src_name}.js" ]; then
    cp "$bin_dir/${src_name}.js" "$OUT_DIR/${dst_name}.js"
    if [ -n "$simd" ]; then
      sed -i '' "s/${src_name}.wasm/${dst_name}.wasm/g" "$OUT_DIR/${dst_name}.js"
    fi
    echo "📦 已生成 JS：$OUT_DIR/${dst_name}.js"
  else
    echo "❌ 未找到输出 $bin_dir/${src_name}.js"
    exit 1
  fi

  if [ -f "$bin_dir/${src_name}.wasm" ]; then
    cp "$bin_dir/${src_name}.wasm" "$OUT_DIR/${dst_name}.wasm"
    echo "📦 已生成 WASM：$OUT_DIR/${dst_name}.wasm"
  else
    echo "❌ 未找到 WASM 输出 $bin_dir/${src_name}.wasm"
    exit 1
  fi
}

# ==================== Non-SIMD ====================
echo "=== 1/2: 标准构建 ==="
generate_projects

build release
copy_output release ""

build debug
copy_output debug ""

# ==================== SIMD ====================
echo "=== 2/2: SIMD 构建 ==="
export PX_ENABLE_SIMD=1
generate_projects

build release
copy_output release ".simd"

build debug
copy_output debug ".simd"

unset PX_ENABLE_SIMD

echo "✅ 构建完成。产物位于：$OUT_DIR"
