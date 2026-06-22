#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# pi-wifi.sh —— 一键切换 Orange Pi 的 WiFi 模式（自发热点 AP ⇄ 连接热点 client）
#
# 依赖：NetworkManager（nmcli）。Orange Pi/Ubuntu 默认即用。
# 共享(shared)模式下 Pi 固定网关 IP = 10.42.0.1，自带 DHCP+NAT。
#
# 用法：
#   ./pi-wifi.sh hotspot              # 开启自发热点（AP 模式）
#   ./pi-wifi.sh client               # 连接默认 WiFi（CLIENT_SSID，自动关热点）
#   ./pi-wifi.sh client <SSID> [PASS] # 连接到指定 WiFi（客户端模式）
#   ./pi-wifi.sh off                  # 关闭热点
#   ./pi-wifi.sh status               # 查看当前网卡/连接/IP
#   ./pi-wifi.sh list                 # 列出已保存连接 + 周边 WiFi
#   ./pi-wifi.sh autostart on|off     # 开机是否自动开热点
#
# 自定义热点名/密码（二选一）：
#   1) 改下面 AP_SSID / AP_PASS 默认值；或
#   2) 运行时用环境变量覆盖： AP_SSID=Robot AP_PASS=pass1234 ./pi-wifi.sh hotspot
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

# ── 可配置项 ──────────────────────────────────────────────────────────────────
AP_SSID="${AP_SSID:-MyRobotAP}"     # 热点名称
AP_PASS="${AP_PASS:-robot12345}"    # 热点密码（≥8 位）
AP_CON="pi-hotspot"                 # NetworkManager 连接 profile 名
AP_BAND="${AP_BAND:-bg}"            # bg=2.4GHz（兼容好）；a=5GHz（受监管域限制，可能起不来）

CLIENT_SSID="${CLIENT_SSID:-Magic5Pro}"      # 默认连接的 WiFi 名（client 不带参数时用）
CLIENT_PASS="${CLIENT_PASS:-123456789k}"     # 默认连接的 WiFi 密码

# ── root 权限：nmcli 写操作需要，未 root 则用 sudo 重入（-E 保留 AP_* 环境变量）──
if [ "$(id -u)" -ne 0 ]; then exec sudo -E bash "$0" "$@"; fi

# ── 自动探测 WiFi 网卡名（通常 wlan0）────────────────────────────────────────
WIFI_DEV="$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi"{print $1; exit}')"
if [ -z "${WIFI_DEV:-}" ]; then
  echo "✗ 未找到 WiFi 网卡。请确认 NetworkManager 在管理无线网卡。" >&2
  exit 1
fi

CMD="${1:-status}"

ensure_ap_profile() {
  if ! nmcli -t -f NAME connection show | grep -qx "$AP_CON"; then
    nmcli connection add type wifi ifname "$WIFI_DEV" con-name "$AP_CON" \
      autoconnect no ssid "$AP_SSID" >/dev/null
  fi
  nmcli connection modify "$AP_CON" \
    802-11-wireless.mode ap \
    802-11-wireless.band "$AP_BAND" \
    802-11-wireless.ssid "$AP_SSID" \
    ipv4.method shared \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$AP_PASS" >/dev/null
}

show_ip() {
  local ip
  ip="$(ip -4 -o addr show "$WIFI_DEV" 2>/dev/null | awk '{print $4}' | cut -d/ -f1)"
  echo "  网卡 $WIFI_DEV IP：${ip:-（无）}"
}

case "$CMD" in
  hotspot|ap)
    echo "▶ 开启自发热点：SSID=$AP_SSID  band=$AP_BAND  dev=$WIFI_DEV"
    ensure_ap_profile
    nmcli connection up "$AP_CON" >/dev/null
    echo "✓ 热点已开启"
    show_ip
    echo "  连此热点后浏览器打开： http://10.42.0.1:8080"
    ;;

  client|join)
    SSID="${2:-$CLIENT_SSID}"           # 不给则用默认 WiFi
    if   [ $# -ge 3 ];                then PASS="$3"            # 显式给了密码
    elif [ "$SSID" = "$CLIENT_SSID" ]; then PASS="$CLIENT_PASS" # 默认 WiFi → 默认密码
    else                                  PASS=""              # 其它网络没给密码 → 当作已保存
    fi
    if [ -z "$SSID" ]; then echo "用法：$0 client [SSID] [PASS]" >&2; exit 2; fi
    echo "▶ 关闭热点并连接到：$SSID"
    nmcli connection down "$AP_CON" 2>/dev/null || true
    if [ -n "$PASS" ]; then
      nmcli device wifi connect "$SSID" password "$PASS" ifname "$WIFI_DEV"
    else
      nmcli device wifi connect "$SSID" ifname "$WIFI_DEV"   # 连已保存的网络可省密码
    fi
    echo "✓ 已连接"
    show_ip
    ;;

  off|stop)
    echo "▶ 关闭热点"
    nmcli connection down "$AP_CON" 2>/dev/null || true
    echo "✓ 已关闭（如需联网请用：$0 client <SSID> <PASS>）"
    ;;

  autostart)
    MODE="${2:-}"
    case "$MODE" in
      on)  ensure_ap_profile
           nmcli connection modify "$AP_CON" autoconnect yes connection.autoconnect-priority 10 >/dev/null
           echo "✓ 已设为开机自动开热点（建议把家里 WiFi 的 autoconnect 关掉以免冲突）" ;;
      off) ensure_ap_profile
           nmcli connection modify "$AP_CON" autoconnect no >/dev/null
           echo "✓ 已关闭热点开机自启" ;;
      *)   echo "用法：$0 autostart on|off" >&2; exit 2 ;;
    esac
    ;;

  status)
    echo "== WiFi 网卡 =="; nmcli device status | grep -E "DEVICE|wifi" || true
    echo "== 活动连接 =="; nmcli -t -f NAME,DEVICE,TYPE connection show --active | grep -E "wifi|$WIFI_DEV" || echo "  （无）"
    show_ip
    ;;

  list)
    echo "== 已保存连接 =="; nmcli -t -f NAME,TYPE connection show | grep wifi || true
    echo "== 周边 WiFi =="; nmcli device wifi list ifname "$WIFI_DEV" 2>/dev/null | head -15 || true
    ;;

  *)
    echo "用法：$0 {hotspot|client [SSID] [PASS]|off|status|list|autostart on|off}" >&2
    exit 2
    ;;
esac
