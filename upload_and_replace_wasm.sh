#!/bin/bash

# 脚本：构建并上传WASM文件，替换JS中的引用
# 用途：
#   1. 构建 PhysX WASM（包括 downgrade.js 和 wasm 版本）
#   2. 上传 wasm_build/ 中的文件到 CDN
#   3. 替换 JS 中的 wasmBinaryFile 为远程 URL

set -e  # 出错时退出

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
WASM_FILE="$ROOT_DIR/wasm_build/physx.release.wasm"
JS_FILE="$ROOT_DIR/wasm_build/physx.release.js"
DOWNGRADE_JS_FILE="$ROOT_DIR/wasm_build/physx.release.downgrade.js"

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
    echo "🏗️  阶段 1/2: 构建 PhysX WASM..."
    echo ""

    if [ ! -f "$ROOT_DIR/build.sh" ]; then
        echo "❌ 错误：未找到 build.sh"
        exit 1
    fi

    # 执行构建
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
echo "🚀 阶段 2/2: 上传文件到 CDN..."
echo ""

# 检查构建产物是否存在
if [ ! -f "$WASM_FILE" ]; then
    echo "❌ 错误：未找到文件 $WASM_FILE"
    exit 1
fi

if [ ! -f "$JS_FILE" ]; then
    echo "❌ 错误：未找到文件 $JS_FILE"
    exit 1
fi

if [ ! -f "$DOWNGRADE_JS_FILE" ]; then
    echo "❌ 错误：未找到文件 $DOWNGRADE_JS_FILE"
    exit 1
fi

echo "📤 开始上传 WASM 文件..."

# 上传WASM文件并获取URL
echo "📤 正在上传 $WASM_FILE..."
UPLOAD_OUTPUT=$(cli upload "$WASM_FILE" 2>&1) || {
    echo "❌ 上传失败："
    echo "$UPLOAD_OUTPUT"
    exit 1
}

echo "✅ 上传完成"
echo "上传输出：$UPLOAD_OUTPUT"

# 从上传输出中提取URL
# 假设输出包含URL，我们需要解析它
# 这里可能需要根据实际的cli upload输出格式调整
UPLOAD_URL=$(echo "$UPLOAD_OUTPUT" | grep -oE 'https?://[^[:space:]]+' | head -n1) || {
    echo "❌ 无法从上传输出中提取URL"
    echo "上传输出：$UPLOAD_OUTPUT"
    exit 1
}

if [ -z "$UPLOAD_URL" ]; then
    echo "❌ 提取的URL为空"
    echo "上传输出：$UPLOAD_OUTPUT"
    exit 1
fi

echo "📋 提取到的URL：$UPLOAD_URL"

# 创建JS文件的备份
BACKUP_FILE="${JS_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
cp "$JS_FILE" "$BACKUP_FILE"
echo "💾 已创建备份文件：$BACKUP_FILE"

# 替换JS文件中的wasmBinaryFile
echo "🔄 正在替换 $JS_FILE 中的 wasmBinaryFile..."

# 使用sed替换wasmBinaryFile="physx.release.wasm"为实际URL
sed -i.tmp "s|wasmBinaryFile=\"physx\.release\.wasm\"|wasmBinaryFile=\"$UPLOAD_URL\"|g" "$JS_FILE"

# 删除sed创建的临时文件
rm -f "${JS_FILE}.tmp"

# 验证替换是否成功
if grep -F "$UPLOAD_URL" "$JS_FILE" > /dev/null; then
    echo "✅ 替换成功！"
    echo "📄 已将 wasmBinaryFile 更新为：$UPLOAD_URL"
else
    echo "❌ 替换失败，恢复备份文件"
    cp "$BACKUP_FILE" "$JS_FILE"
    exit 1
fi

# 显示更改后的内容（只显示相关行）
echo ""
echo "📋 验证更改："
grep -n "wasmBinaryFile=" "$JS_FILE" || echo "未找到wasmBinaryFile行"

# 上传JavaScript文件
echo ""
echo "🚀 开始上传 JavaScript 文件..."
echo "📤 正在上传 $JS_FILE..."

JS_UPLOAD_OUTPUT=$(cli upload "$JS_FILE" 2>&1) || {
    echo "❌ JS文件上传失败："
    echo "$JS_UPLOAD_OUTPUT"
    echo "⚠️  WASM文件已成功上传并替换，但JS文件上传失败"
    exit 1
}

echo "✅ JS文件上传完成"
echo "JS上传输出：$JS_UPLOAD_OUTPUT"

# 从JS上传输出中提取URL
JS_UPLOAD_URL=$(echo "$JS_UPLOAD_OUTPUT" | grep -oE 'https?://[^[:space:]]+' | head -n1) || {
    echo "❌ 无法从JS上传输出中提取URL"
    echo "JS上传输出：$JS_UPLOAD_OUTPUT"
    exit 1
}

if [ -z "$JS_UPLOAD_URL" ]; then
    echo "❌ 提取的JS URL为空"
    echo "JS上传输出：$JS_UPLOAD_OUTPUT"
    exit 1
fi

echo "📋 提取到的JS URL：$JS_UPLOAD_URL"

# 上传 Downgrade JavaScript 文件
echo ""
echo "📤 开始上传 Downgrade JavaScript 文件..."
echo "📤 正在上传 $DOWNGRADE_JS_FILE..."

DOWN_JS_UPLOAD_OUTPUT=$(cli upload "$DOWNGRADE_JS_FILE" 2>&1) || {
    echo "❌ 降级版JS文件上传失败："
    echo "$DOWN_JS_UPLOAD_OUTPUT"
    echo "⚠️  主版JS和WASM已成功处理，但降级版JS上传失败"
    exit 1
}

echo "✅ 降级版JS文件上传完成"
echo "Downgrade JS上传输出：$DOWN_JS_UPLOAD_OUTPUT"

DOWN_JS_UPLOAD_URL=$(echo "$DOWN_JS_UPLOAD_OUTPUT" | grep -oE 'https?://[^[:space:]]+' | head -n1) || {
    echo "❌ 无法从降级版JS上传输出中提取URL"
    echo "降级版JS上传输出：$DOWN_JS_UPLOAD_OUTPUT"
    exit 1
}

if [ -z "$DOWN_JS_UPLOAD_URL" ]; then
    echo "❌ 提取的降级版JS URL为空"
    echo "降级版JS上传输出：$DOWN_JS_UPLOAD_OUTPUT"
    exit 1
fi

echo "📋 提取到的降级版JS URL：$DOWN_JS_UPLOAD_URL"

echo ""
echo "🎉 完成！构建和上传全部成功"
echo ""
echo "📁 备份文件：$BACKUP_FILE"
echo ""
echo "🔗 CDN URLs："
echo "   WASM:              $UPLOAD_URL"
echo "   JavaScript:        $JS_UPLOAD_URL"
echo "   JavaScript (降级): $DOWN_JS_UPLOAD_URL"
