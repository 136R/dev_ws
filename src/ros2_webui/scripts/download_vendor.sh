#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# 下载 ros2_webui 前端离线依赖到 static/vendor/
#
# 在【有网的开发机】上运行一次即可，产物随代码提交；
# Orange Pi 实机运行时无外网，直接使用已提交的 vendor 文件。
#
#   bash src/ros2_webui/scripts/download_vendor.sh
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENDOR_DIR="$SCRIPT_DIR/../static/vendor"
mkdir -p "$VENDOR_DIR"

# 版本固定，避免上游变动导致行为漂移。
# 每个依赖给出 unpkg / jsdelivr 两个镜像，任一可用即可（网络偶发抖动时自动回退）。
# 注：nipplejs 0.10.2 仅发布 dist/nipplejs.js（已是构建产物），另存为 nipplejs.min.js。
ROSLIB_URLS=(
  "https://unpkg.com/roslib@1.4.1/build/roslib.min.js"
  "https://cdn.jsdelivr.net/npm/roslib@1.4.1/build/roslib.min.js"
)
NIPPLE_URLS=(
  "https://cdn.jsdelivr.net/npm/nipplejs@0.10.2/dist/nipplejs.js"
  "https://unpkg.com/nipplejs@0.10.2/dist/nipplejs.js"
)
OPENPROPS_URLS=(
  "https://unpkg.com/open-props@1.7.7/open-props.min.css"
  "https://cdn.jsdelivr.net/npm/open-props@1.7.7/open-props.min.css"
)

# fetch <out> <url...>：依次尝试镜像，任一成功即停止；带重试。
fetch() {
  local out="$1"; shift
  local url
  for url in "$@"; do
    echo "↓ $url"
    if curl -fsSL --retry 3 --retry-delay 1 "$url" -o "$out"; then
      return 0
    fi
    echo "  …失败，尝试下一个镜像" >&2
  done
  echo "✗ 全部镜像均失败：$out" >&2
  return 1
}

fetch "$VENDOR_DIR/roslib.min.js"     "${ROSLIB_URLS[@]}"
fetch "$VENDOR_DIR/nipplejs.min.js"   "${NIPPLE_URLS[@]}"
fetch "$VENDOR_DIR/open-props.min.css" "${OPENPROPS_URLS[@]}"

# 校验 roslib 是否含 CBOR 支持（地图大消息压缩的前提）
if [ "$(grep -c -i cbor "$VENDOR_DIR/roslib.min.js" || true)" -gt 0 ]; then
  echo "✓ roslib.min.js 含 CBOR 支持"
else
  echo "✗ 警告：roslib.min.js 未检测到 cbor，地图传输将退化为 JSON" >&2
fi

echo ""
echo "=== static/vendor 内容 ==="
ls -lh "$VENDOR_DIR"
