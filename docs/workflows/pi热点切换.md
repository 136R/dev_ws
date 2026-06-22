# Pi WiFi 模式切换（自发热点 ⇄ 连接热点）

用 [`pi-wifi.sh`](./pi-wifi.sh) 在 Orange Pi 上一键切换网络模式。底层是 NetworkManager（`nmcli`）。

> ⚠️ 单张无线网卡**不能同时**既当热点又连别的 WiFi，二者互斥（运行 `hotspot`/`client` 会自动切换）。
> 自发热点（shared 模式）下 Pi 没有 WiFi 外网，需要联网装东西请先 `client` 连外网，或用网线。

## 安装

```bash
chmod +x ~/dev_ws/docs/workflows/pi-wifi.sh
# 可选：做个软链方便全局调用
sudo ln -sf ~/dev_ws/docs/workflows/pi-wifi.sh /usr/local/bin/pi-wifi
```

## 常用命令

| 操作 | 命令 |
| --- | --- |
| 开自发热点（AP） | `pi-wifi hotspot` |
| 连接**默认 WiFi**（省参数） | `pi-wifi client` |
| 连接到某 WiFi（客户端） | `pi-wifi client <SSID> <密码>` |
| 连已保存的 WiFi | `pi-wifi client <SSID>` |
| 关热点 | `pi-wifi off` |
| 查看状态/IP | `pi-wifi status` |
| 列出已存连接+周边WiFi | `pi-wifi list` |
| 开机自动开热点 | `pi-wifi autostart on` / `off` |

脚本会用 `sudo` 自动提权（`nmcli` 写操作需要）。

## 自定义热点名/密码

改脚本顶部对应变量，或运行时用环境变量覆盖：

```bash
AP_SSID=Robot AP_PASS=pass12345 pi-wifi hotspot          # 自发热点
CLIENT_SSID=别的WiFi CLIENT_PASS=xxx pi-wifi client       # 默认客户端 WiFi
```

默认值（脚本顶部）：
- 热点：SSID `MyRobotAP`、密码 `robot12345`、2.4GHz（`bg`）。
- 客户端默认 WiFi：SSID `Magic5Pro`、密码 `123456789k` → 直接 `pi-wifi client` 即连。

## 配合 Web UI 的典型流程

```bash
# 1) 装依赖（需外网，先连家里 WiFi）
pi-wifi client 家里WiFi 密码
sudo apt install -y ros-humble-rosbridge-suite

# 2) 切自发热点，给手机/笔记本直连
pi-wifi hotspot                 # Pi 固定 IP 10.42.0.1

# 3) 起机器人栈 + Web UI
ros2 launch ros2_webui webui.launch.py

# 4) 手机连 MyRobotAP → 浏览器 http://10.42.0.1:8080
```

想 Pi 一开机就自带热点：`pi-wifi autostart on`（同时建议把家里 WiFi 的 autoconnect 关掉避免开机抢占：
`nmcli connection modify "家里WiFi" autoconnect no`）。

## 排查

- `pi-wifi status` 看当前网卡/活动连接/IP；热点正常时 IP 应为 `10.42.0.1`。
- 网卡是否支持 AP：`iw list | grep -A10 "Supported interface modes"` 里要有 `* AP`；没有就得换支持 AP 的 USB 无线网卡。
- `ipv4.method shared` 报缺 dnsmasq：`sudo apt install dnsmasq-base`。
- 开了防火墙(ufw) 记得放行：`sudo ufw allow 8080/tcp && sudo ufw allow 9090/tcp`。
