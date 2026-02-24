#!/bin/bash
# 脚本：构建并上传WASM文件，替换JS中的引用
# 用途：
#   1. 构建 PhysX WASM（标准 + SIMD）
#   2. 上传 wasm_build/ 中的文件到 CDN
#   3. 替换 JS 中的 wasmBinaryFile 为远程 URL

set -e  # 出错时退出

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
WASM_DIR="$ROOT_DIR/wasm_build"

# 需要上传的文件列表（名称 => 描述）
FILES=(
    "physx.release.wasm:Release WASM"
    "physx.release.js:Release JS"
    "physx.release.simd.wasm:SIMD WASM"
    "physx.release.simd.js:SIMD JS"
)

SKIP_BUILD=false

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [--skip-build]"
            exit 1
            ;;
    esac
done

# ============================================
# 阶段 1: 构建
# ============================================
if [ "$SKIP_BUILD" = false ]; then
    echo "🏗️  阶段 1/3: 构建 PhysX WASM..."
    echo ""

    if [ ! -f "$ROOT_DIR/build.sh" ]; then
        echo "❌ 错误：未找到 build.sh"
        exit 1
    fi

    (cd "$ROOT_DIR" && ./build.sh)

    echo ""
    echo "✅ 构建完成"
    echo ""
else
    echo "⏭️  跳过构建阶段 (--skip-build)"
    echo ""
fi

# ============================================
# 阶段 2: 上传
# ============================================
echo "🚀 阶段 2/3: 上传文件到 CDN..."
echo ""

# 检查构建产物是否存在
for entry in "${FILES[@]}"; do
    FILE_NAME="${entry%%:*}"
    FILE_DESC="${entry##*:}"
    if [ ! -f "$WASM_DIR/$FILE_NAME" ]; then
        echo "❌ 错误：未找到 $FILE_DESC 文件 $WASM_DIR/$FILE_NAME"
        exit 1
    fi
done

# 上传函数：上传文件并返回 CDN URL
upload_file() {
    local file_path=$1
    local file_desc=$2

    echo "📤 正在上传 $file_desc: $file_path..."
    local output
    output=$(cli upload "$file_path" 2>&1) || {
        echo "❌ $file_desc 上传失败："
        echo "$output"
        exit 1
    }

    local url
    url=$(echo "$output" | grep -oE 'https?://[^[:space:]]+' | head -n1)
    if [ -z "$url" ]; then
        echo "❌ 无法从上传输出中提取URL"
        echo "上传输出：$output"
        exit 1
    fi

    echo "✅ $file_desc 上传完成: $url"
    echo "$url"
}

# 上传 release WASM 并替换 JS 中的引用
WASM_URL=$(upload_file "$WASM_DIR/physx.release.wasm" "Release WASM")
WASM_URL=$(echo "$WASM_URL" | tail -1)

# 备份并替换 release JS 中的 wasmBinaryFile
BACKUP_FILE="$WASM_DIR/physx.release.js.backup.$(date +%Y%m%d_%H%M%S)"
cp "$WASM_DIR/physx.release.js" "$BACKUP_FILE"

sed -i.tmp "s|locateFile(\"physx\.release\.wasm\")|\"$WASM_URL\"|g" "$WASM_DIR/physx.release.js"
rm -f "$WASM_DIR/physx.release.js.tmp"

if grep -qF "$WASM_URL" "$WASM_DIR/physx.release.js"; then
    echo "✅ Release JS wasm 路径替换成功"
else
    echo "❌ 替换失败，恢复备份"
    cp "$BACKUP_FILE" "$WASM_DIR/physx.release.js"
    exit 1
fi

# 上传替换后的 release JS
JS_URL=$(upload_file "$WASM_DIR/physx.release.js" "Release JS")
JS_URL=$(echo "$JS_URL" | tail -1)

# 上传 SIMD WASM 并替换 SIMD JS 中的引用
SIMD_WASM_URL=$(upload_file "$WASM_DIR/physx.release.simd.wasm" "SIMD WASM")
SIMD_WASM_URL=$(echo "$SIMD_WASM_URL" | tail -1)

SIMD_BACKUP_FILE="$WASM_DIR/physx.release.simd.js.backup.$(date +%Y%m%d_%H%M%S)"
cp "$WASM_DIR/physx.release.simd.js" "$SIMD_BACKUP_FILE"

sed -i.tmp "s|locateFile(\"physx\.release\.simd\.wasm\")|\"$SIMD_WASM_URL\"|g" "$WASM_DIR/physx.release.simd.js"
rm -f "$WASM_DIR/physx.release.simd.js.tmp"

if grep -qF "$SIMD_WASM_URL" "$WASM_DIR/physx.release.simd.js"; then
    echo "✅ SIMD JS wasm 路径替换成功"
else
    echo "❌ 替换失败，恢复备份"
    cp "$SIMD_BACKUP_FILE" "$WASM_DIR/physx.release.simd.js"
    exit 1
fi

SIMD_JS_URL=$(upload_file "$WASM_DIR/physx.release.simd.js" "SIMD JS")
SIMD_JS_URL=$(echo "$SIMD_JS_URL" | tail -1)

# ============================================
# 阶段 3: 复制到 e2e 目录
# ============================================
E2E_DEV_DIR="/Users/chenmo/Code/Galacean/engine/e2e/.dev"

if [ -d "$E2E_DEV_DIR" ]; then
    echo ""
    echo "📂 阶段 3/3: 复制到 e2e 目录..."

    cp "$BACKUP_FILE" "$E2E_DEV_DIR/physx.release.js"
    cp "$WASM_DIR/physx.release.wasm" "$E2E_DEV_DIR/physx.release.wasm"
    cp "$SIMD_BACKUP_FILE" "$E2E_DEV_DIR/physx.release.simd.js"
    cp "$WASM_DIR/physx.release.simd.wasm" "$E2E_DEV_DIR/physx.release.simd.wasm"

    echo "✅ 已复制到 $E2E_DEV_DIR"
else
    echo "⚠️  未找到 e2e 目录: $E2E_DEV_DIR，跳过复制"
fi

echo ""
echo "🎉 完成！"
echo ""
echo "🔗 CDN URLs："
echo "   Release WASM:  $WASM_URL"
echo "   Release JS:    $JS_URL"
echo "   SIMD WASM:     $SIMD_WASM_URL"
echo "   SIMD JS:       $SIMD_JS_URL"
