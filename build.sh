#!/bin/bash
set -e

# Ensure no stale SIMD env from previous failed runs
unset PX_ENABLE_SIMD

# Build PhysX WebBindings:
# Each build produces classic JS and ESM loaders for the same WASM.
# BUILD_DEBUG=1 also builds debug variants with PVD support.

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

  if ! cmp -s "$bin_dir/${src_name}.wasm" "$bin_dir/module/${src_name}.wasm"; then
    echo "❌ classic 与 ESM 的 WASM 产物不一致：${src_name}${simd}"
    exit 1
  fi

  cp "$bin_dir/${src_name}.js" "$OUT_DIR/${dst_name}.js"
  cp "$bin_dir/module/${src_name}.mjs" "$OUT_DIR/${dst_name}.mjs"
  cp "$bin_dir/${src_name}.wasm" "$OUT_DIR/${dst_name}.wasm"
  if [ -n "$simd" ]; then
    sed -i '' "s/${src_name}.wasm/${dst_name}.wasm/g" "$OUT_DIR/${dst_name}.js" "$OUT_DIR/${dst_name}.mjs"
  fi
  echo "📦 已生成 ${dst_name}.{js,mjs,wasm}：$OUT_DIR"
}

# ==================== Non-SIMD ====================
echo "=== 1/2: 标准构建 ==="
generate_projects

build release
copy_output release ""

if [ "${BUILD_DEBUG:-0}" = "1" ]; then
  build debug
  copy_output debug ""
fi

# ==================== SIMD ====================
echo "=== 2/2: SIMD 构建 ==="
export PX_ENABLE_SIMD=1
generate_projects

build release
copy_output release ".simd"

if [ "${BUILD_DEBUG:-0}" = "1" ]; then
  build debug
  copy_output debug ".simd"
fi

unset PX_ENABLE_SIMD

echo "✅ 构建完成。产物位于：$OUT_DIR"
