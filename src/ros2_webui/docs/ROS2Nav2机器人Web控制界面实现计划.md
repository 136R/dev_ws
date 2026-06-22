# ROS2 Nav2 机器人 Web 控制界面 实现计划

> 开始前必读：仓库根目录 CLAUDE.md 和 src/my\_bot\_nav/，理解已有包结构与话题约定。

---

## 一、项目背景与硬性约束


| 项目           | 说明                                                                        |
| -------------- | --------------------------------------------------------------------------- |
| 平台           | ROS2 Humble，Orange Pi 5 Pro，差速小车，Nav2 已跑通                         |
| 网络           | Pi 作 WiFi 热点，手机/笔记本直连；Pi 配置环境时可临时联网；实际运行时无外网 |
| 访问           | http://<Pi热点IP>:8080，浏览器即用，无需安装 App                            |
| Phase 1 主设备 | 电脑浏览器（鼠标交互，无需触控适配）                                        |
| Phase 2 适配   | 手机触控（手势缩放/虚拟摇杆触控优化，Phase 1 不做）                         |
| 开发流程       | ① 开发机浏览器跑通 → ② 部署 Pi → ③ Phase 2 手机适配                    |

---

## 二、技术选型（已定，勿更改）

### 依赖库（共 3 个）


| 组件            | 选择                              | 理由                                                          |
| --------------- | --------------------------------- | ------------------------------------------------------------- |
| ROS↔浏览器桥接 | ros-humble-rosbridge-suite        | 官方标准 WebSocket 桥                                         |
| JS ROS 客户端   | roslibjs（≥ 0.20，含 CBOR 支持） | 官方库，主动维护                                              |
| 虚拟摇杆        | nipplejs                          | 移动触控最广泛方案                                            |
| CSS 设计 token  | open-props.min.css                | 单文件免构建，提供颜色/间距/字体 token；手写暗色 CSS 直接引用 |
| 地图/TF 渲染    | 原生 Canvas API，自实现           | 见下方说明                                                    |
| 前端框架        | 原生 HTML + CSS + JS，单文件      | 无构建工具，Pi 直接 serve                                     |
| 静态服务        | Pythonhttp.server                 |                                                               |

### ⚠️ 明确不用的库（重要）

* ❌ 禁止使用 ros2djs / EaselJS：已停止维护；其 TFClient 默认依赖 tf2\_web\_republisher（本项目未安装）；且与 CBOR 解码消息不兼容。
* ❌ 禁止使用 Pico CSS 或其他 CSS 组件框架：组件预置样式与手写 UI 对抗；data-theme="dark" 是 Pico 的机制，Open Props 不内置，勿混用。
* ❌ 禁止使用任何 CDN URL：Pi 无外网，所有依赖必须本地化。

### 离线依赖管理

所有依赖通过 scripts/download\_vendor.sh 在开发机（有网时）一次性下载到 static/vendor/，随代码提交，Pi 上直接使用。

---

## 三、包结构（ament\_cmake 正规包）

```
src/ros2_webui/            ← 新建，位于 src/ 下，与 my_bot_nav 并列
├── package.xml
├── CMakeLists.txt
├── launch/
│   └── webui.launch.py    ← 一键启动 rosbridge + 文件服务
├── static/
│   ├── index.html         ← 主页面（含全部逻辑，单文件）
│   └── vendor/
│       ├── roslib.min.js
│       ├── nipplejs.min.js
│       └── open-props.min.css
└── scripts/
    └── download_vendor.sh
```

CMakeLists.txt 核心内容：

```cmake
cmake_minimum_required(VERSION 3.8)
project(ros2_webui)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch static
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
```

package.xml 核心内容：

```xml
<package format="3">
  <name>ros2_webui</name>
  <version>0.1.0</version>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>rosbridge_server</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

> 如此方可用 ros2 launch ros2\_webui webui.launch.py 找到包。

---

## 四、ROS2 话题与帧约定

> 实现前通过 ros2 topic list 核对实际名称，所有话题名集中在 index.html 顶部 CONFIG 对象，方便修改。


| 功能         | 话题 / Action                                  | 消息类型                   | 备注                                                   |
| ------------ | ---------------------------------------------- | -------------------------- | ------------------------------------------------------ |
| 占用栅格地图 | /map                                           | nav\_msgs/OccupancyGrid    | CBOR + 节流                                            |
| 坐标变换     | /tf+/tf\_static                                | tf2\_msgs/TFMessage        | 直接订阅，JS 内组合                                    |
| 激光雷达     | /scan\_filtered                                | sensor\_msgs/LaserScan     | ⚠️ 不用/scan（已滤自反射）                           |
| 手动遥控     | /cmd\_vel\_keyboard                            | geometry\_msgs/Twist       | 摇杆输出                                               |
| 单点导航     | /goal\_pose                                    | geometry\_msgs/PoseStamped | 直接 pub，绕开 Action 坑                               |
| IMU 数据     | /imu\_broad/imu（实机）<br>/imu（Gazebo 仿真） | sensor\_msgs/Imu           | ⚠️ 两者不同，仿真阶段必须切换；用ros2 topic list核对 |
| Nav2 反馈    | /navigate\_to\_pose/\_action/feedback          | feedback topic             | 订阅导航状态用                                         |

机器人基座帧：base\_footprint（全栈 Nav2 约定，不用 base\_link）
TF 链：map → odom → base\_footprint（直接订阅 /tf 在 JS 中组合，无需 tf2\_web\_republisher）

⚠️ 无电量数据：本机器人无 /battery\_state 发布者，不实现电量面板。

---

## 五、性能架构（最高优先级，本次重做核心动机）

> 上一版"地图更新延迟大"根因：rosbridge 默认用 JSON 编码 OccupancyGrid（384×384 地图 ≈ 500KB），WebSocket 传输成为瓶颈。
> 官方解法：[RobotWebTools/rosbridge\_suite#367](https://github.com/RobotWebTools/rosbridge_suite/issues/367)

### 5.1 CBOR 压缩（必须）

所有大型消息订阅必须启用 CBOR 二进制压缩：

```javascript
// /map：CBOR + 节流（地图基本静态，低频即可）
const mapTopic = new ROSLIB.Topic({
    ros: ros,
    name: CONFIG.TOPIC_MAP,
    messageType: 'nav_msgs/OccupancyGrid',
    compression: 'cbor',
    throttle_rate: 1000   // ms，即 1Hz；建图/导航阶段地图可能变化，1Hz 更稳妥
});

// /scan_filtered：CBOR + 10Hz
const scanTopic = new ROSLIB.Topic({
    ros: ros,
    name: CONFIG.TOPIC_SCAN,
    messageType: 'sensor_msgs/LaserScan',
    compression: 'cbor',
    throttle_rate: 100
});
```

### 5.2 Canvas 分层（必须）

```html
<div id="map-container" style="position:relative; width:100%; height:100%;">
  <!-- 层 1：静态地图，仅 /map 到达或 viewport 变化时重绘 -->
  <canvas id="canvas-map"     style="position:absolute;top:0;left:0;width:100%;height:100%;"></canvas>
  <!-- 层 2：动态叠加，机器人/激光/路径点，requestAnimationFrame 驱动 -->
  <canvas id="canvas-overlay" style="position:absolute;top:0;left:0;width:100%;height:100%;"></canvas>
</div>
```

两层 canvas 的尺寸均等于容器的显示尺寸（像素），不是地图格子数。
地图像素数据存储在离屏 mapOffscreen（原生栅格分辨率），渲染时通过 viewport 变换投影到显示 canvas。
canvas-overlay 用 requestAnimationFrame 每帧重绘，使用完全相同的 viewport 变换 → 两层永远对齐。

```javascript
// 初始化和窗口 resize 时设置 canvas 为容器实际像素尺寸
function resizeCanvases() {
    const container = document.getElementById('map-container');
    ['canvas-map', 'canvas-overlay'].forEach(id => {
        const c = document.getElementById(id);
        c.width  = container.clientWidth;
        c.height = container.clientHeight;
    });
    renderMapLayer();  // ← resize 后必须重绘地图层
}
window.addEventListener('resize', resizeCanvases);
// 页面加载后立即调用一次
```

### 5.3 OccupancyGrid 自渲染（分离存储与渲染）

架构关键：decodeMap() 只更新离屏缓冲；renderMapLayer() 负责把缓冲以当前 viewport 投影到显示 canvas。
viewport 变化（手势缩放/平移）时只需调用 renderMapLayer()，不必重新解码地图数据。

```javascript
let mapInfo    = null;
let mapOffscreen = null;  // OffscreenCanvas，原生栅格分辨率，Y 轴未翻转

// ── Step A：解码 OccupancyGrid 消息到离屏缓冲 ──────────────────────────
function decodeMap(mapMsg) {
    mapInfo = mapMsg.info;  // { resolution, origin:{position,orientation}, width, height }
    const data = mapMsg.data;

    // ⚠️ CBOR 解码类型诊断（首次接收时打印，用于排查符号问题）
    if (!mapOffscreen) {
        console.log('[MAP] data type:', data.constructor.name,
                    '| first 5 values:', Array.from(data.slice(0, 5)),
                    '| contains 255:', Array.from(data).includes(255));
    }

    mapOffscreen = new OffscreenCanvas(mapInfo.width, mapInfo.height);
    const octx = mapOffscreen.getContext('2d');
    const img  = octx.createImageData(mapInfo.width, mapInfo.height);

    for (let i = 0; i < data.length; i++) {
        const v = data[i];
        // ⚠️ int8 → CBOR → Uint8Array 时 -1 变 255；两种情况都判断
        const isUnknown = (v === -1 || v === 255);
        let r, g, b;
        if      (isUnknown) { r=128; g=128; b=128; }  // 未知：灰
        else if (v === 0)   { r=240; g=240; b=240; }  // 空闲：浅灰白
        else                { r= 30; g= 30; b= 30; }  // 占用：深
        img.data[i*4]=r; img.data[i*4+1]=g;
        img.data[i*4+2]=b; img.data[i*4+3]=255;
    }
    octx.putImageData(img, 0, 0);
    // 注意：mapOffscreen 的行 0 = 地图 Y=0（最低行），Canvas Y 轴在渲染时翻转
    renderMapLayer();  // ← 解码完立即渲染
}

// ── Step B：将离屏缓冲以当前 viewport 投影到显示 canvas ────────────────
// 此函数在两种情况下调用：① 新地图数据到达，② 用户缩放/平移（viewport 变化）
function renderMapLayer() {
    if (!mapOffscreen || !mapInfo) return;
    const canvas = document.getElementById('canvas-map');
    const ctx    = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.save();
    // 与 worldToCanvas 完全等价的变换：
    //   canvas_x = grid_x * scale + panX
    //   canvas_y = (height - grid_y) * scale + panY
    // 等价于：translate(panX, panY + height*scale) → scale(scale, -scale)
    ctx.translate(viewport.panX, viewport.panY + mapInfo.height * viewport.scale);
    ctx.scale(viewport.scale, -viewport.scale);
    ctx.drawImage(mapOffscreen, 0, 0);
    ctx.restore();
}
```

> 一致性约束：renderMapLayer() 的变换与 worldToCanvas() 必须数学等价。
> 修改任意一处时，另一处同步更新，否则地图与机器人/激光错位。

### 5.4 坐标转换（世界坐标 ↔ Canvas 像素）

> ⚠️ worldToCanvas 的数学变换必须与 renderMapLayer() 中的 ctx.translate/scale 完全等价，两者联动修改。

```javascript
let viewport    = { scale: 1, panX: 0, panY: 0 };
let fitViewport = null;  // 首张地图 fit-to-view 的快照，用于双击复位

// ── fit-to-view：收到首张 /map 时自动调用 ──────────────────────────────
function fitToView() {
    if (!mapInfo) return;
    const canvas = document.getElementById('canvas-map');
    const scaleX = canvas.width  / mapInfo.width;
    const scaleY = canvas.height / mapInfo.height;
    const scale  = Math.min(scaleX, scaleY) * 0.90;   // 留 10% 边距
    const panX   = (canvas.width  - mapInfo.width  * scale) / 2;
    const panY   = (canvas.height - mapInfo.height * scale) / 2;
    viewport    = { scale, panX, panY };
    fitViewport = { scale, panX, panY };               // 保存 fit 快照
    onViewportChanged();
}

// 双击复位到 fit-to-view 状态（不是复位到默认 scale=1）
overlayCanvas.addEventListener('dblclick', () => {
    if (fitViewport) { viewport = { ...fitViewport }; onViewportChanged(); }
});

// decodeMap() 中，仅首次收到地图时触发 fit-to-view：
// if (!mapOffscreen) { /* 首次 */ setTimeout(fitToView, 0); }
// 之后地图更新只调用 renderMapLayer()，不重置视口

// ── 世界坐标(m) → 显示 canvas 像素 ────────────────────────────────────
function worldToCanvas(wx, wy) {
    if (!mapInfo) return null;
    const mx = (wx - mapInfo.origin.position.x) / mapInfo.resolution;
    const my = (wy - mapInfo.origin.position.y) / mapInfo.resolution;
    return {
        x: mx * viewport.scale + viewport.panX,
        y: (mapInfo.height - my) * viewport.scale + viewport.panY
    };
}

// ── 显示 canvas 像素 → 世界坐标(m)（点击事件用）──────────────────────
function canvasToWorld(cx, cy) {
    if (!mapInfo) return null;
    const mx = (cx - viewport.panX) / viewport.scale;
    const my = mapInfo.height - (cy - viewport.panY) / viewport.scale;
    return {
        x: mx * mapInfo.resolution + mapInfo.origin.position.x,
        y: my * mapInfo.resolution + mapInfo.origin.position.y
    };
}

// viewport 变化后（缩放/平移）调用
function onViewportChanged() {
    renderMapLayer();
    // overlay 层由 RAF 自动重绘，无需手动触发
}
```

### 5.5 TF 直接订阅（JS 内组合，无 tf2\_web\_republisher）

```javascript
const tfBuffer = {};  // key: "parent->child"

function onTFMessage(msg) {
    msg.transforms.forEach(t => {
        const key = `${t.header.frame_id}->${t.child_frame_id}`;
        tfBuffer[key] = t.transform;  // { translation:{x,y,z}, rotation:{x,y,z,w} }
    });
}

// map->base_footprint = map->odom ∘ odom->base_footprint（2D 近似）
function getRobotPose() {
    const t1 = tfBuffer['map->odom'];
    const t2 = tfBuffer['odom->base_footprint'];
    if (!t1 || !t2) return null;
    const yaw1 = quatToYaw(t1.rotation);
    const yaw2 = quatToYaw(t2.rotation);
    return {
        x:   t1.translation.x + Math.cos(yaw1)*t2.translation.x - Math.sin(yaw1)*t2.translation.y,
        y:   t1.translation.y + Math.sin(yaw1)*t2.translation.x + Math.cos(yaw1)*t2.translation.y,
        yaw: yaw1 + yaw2
    };
}

function quatToYaw(q) {
    return Math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
}
```

### 5.6 激光帧 TF 组合（⚠️ 不能只用 map→base\_footprint）

/scan\_filtered 的 header.frame\_id 是激光传感器安装帧（如 imu\_link、laser、base\_link），
不是base\_footprint。

⚠️ 遍历方向必须向上（targetFrame → base\_footprint），不能向下。
原因：base\_link 有多个子帧（左轮/右轮/雷达/IMU），向下 startsWith 遇到的第一个子帧不确定，
会拐入错误分支返回零偏移。向上沿"唯一父帧"查找则无歧义。

```javascript
/**
 * 从 targetFrame 沿 /tf_static 唯一父帧向上走到 base_footprint，
 * 收集路径后反向复合，返回 targetFrame 相对 base_footprint 的 2D 偏移。
 */
function getStaticOffset(targetFrame, maxHops = 6) {
    // ── Phase 1：向上收集路径（child→parent 方向）────────────────────
    const path = [];   // 元素 { tf } 按 child→parent 顺序
    let cur = targetFrame;

    for (let i = 0; i < maxHops; i++) {
        if (cur === 'base_footprint') break;

        // 找唯一父帧：key 末尾为 '->cur'（每帧只有一个父帧，所以唯一）
        const key = Object.keys(tfBuffer).find(k => k.endsWith('->' + cur));
        if (!key) {
            console.warn(`[TF] getStaticOffset: 找不到 "${cur}" 的父帧，静态链不完整。` +
                         `请检查 /tf_static 是否已进 tfBuffer（见 QoS 排查指引）`);
            return { dx: 0, dy: 0, dyaw: 0 };
        }
        path.push(tfBuffer[key]);           // 存 TF（parent→cur 方向的变换）
        cur = key.split('->')[0];           // 上移到父帧
    }

    if (cur !== 'base_footprint') {
        console.warn(`[TF] getStaticOffset: 超过 ${maxHops} 跳仍未到达 base_footprint`);
        return { dx: 0, dy: 0, dyaw: 0 };
    }

    // ── Phase 2：反向复合（parent→child 方向，即从 base_footprint 出发）──
    // path 是 [T(parent_N-1→target), ..., T(base_footprint→first_child)]
    // 反转后按 parent→child 顺序累加
    let dx = 0, dy = 0, dyaw = 0;
    for (let i = path.length - 1; i >= 0; i--) {
        const tf   = path[i];
        const yOff = quatToYaw(tf.rotation);
        dx   += Math.cos(dyaw) * tf.translation.x - Math.sin(dyaw) * tf.translation.y;
        dy   += Math.sin(dyaw) * tf.translation.x + Math.cos(dyaw) * tf.translation.y;
        dyaw += yOff;
    }
    return { dx, dy, dyaw };
}

// 渲染激光点：先拿机器人位姿，再加激光帧静态偏移
function renderLaserScan(scan) {
    const frameId = scan.header.frame_id;       // 例如 "imu_link"
    const robot   = getRobotPose();
    if (!robot) return;

    const off = getStaticOffset(frameId);       // base_footprint→frameId 偏移
    // 激光帧在地图中的位姿
    const lx   = robot.x   + Math.cos(robot.yaw) * off.dx - Math.sin(robot.yaw) * off.dy;
    const ly   = robot.y   + Math.sin(robot.yaw) * off.dx + Math.cos(robot.yaw) * off.dy;
    const lyaw = robot.yaw + off.dyaw;

    const ctx = document.getElementById('canvas-overlay').getContext('2d');
    ctx.fillStyle = '#ff4444';
    for (let i = 0; i < scan.ranges.length; i++) {
        const r = scan.ranges[i];
        if (!isFinite(r) || r < scan.range_min || r > scan.range_max) continue;
        const angle = scan.angle_min + i * scan.angle_increment;
        const wx = lx + r * Math.cos(lyaw + angle);
        const wy = ly + r * Math.sin(lyaw + angle);
        const cp = worldToCanvas(wx, wy);
        if (!cp) continue;
        ctx.beginPath();
        ctx.arc(cp.x, cp.y, 2, 0, Math.PI * 2);
        ctx.fill();
    }
}
```

> 调试 tip 1：首次收到 /scan\_filtered 时打印 console.log('laser frame\_id:', scan.header.frame\_id)，
> 用 ros2 run tf2\_tools view\_frames 生成 TF 树图，核对 base\_footprint→…→frameId 链路存在。

> 调试 tip 2（QoS 排查）：getStaticOffset 返回零偏移时，先确认/tf\_static 帧已进tfBuffer，
> 而不是直接怀疑算法。/tf\_static 使用 transient\_local QoS（latched），rosbridge 订阅时
> 若 QoS 不匹配会静默丢弃。验证方法：console.log('[TF] buffer keys:', Object.keys(tfBuffer))，
> 确认能看到 base\_footprint->base\_link 等静态帧；若缺失，检查 rosbridge 版本或改用
> /tf\_static 的 best\_effort 重映射。

---

## 六、Demo 模式（?demo=1）

URL 带 ?demo=1 时，完全脱离 ROS，在浏览器内生成假数据，用于：

* 纯 UI 迭代调试（无需启动 ROS 或连接机器人）
* Gazebo 联调前的 UI 自测

```javascript
const DEMO_MODE = new URLSearchParams(window.location.search).get('demo') === '1';

if (DEMO_MODE) {
    // 1. 生成 100×100 假地图（边框为障碍，中间空闲，若干方块障碍物）
    // 2. setInterval 模拟机器人绕圆运动（持续更新 tfBuffer）
    // 3. 生成假激光扫描（均匀 360 点，半径 1.5m，跟随机器人）
    // 4. 跳过 WebSocket 连接，顶部状态栏显示橙色"DEMO 模式"
    // 5. 导航/摇杆按钮正常响应 UI，点击后 toast 提示"Demo：不发送到 ROS"
}
```

---

## 七、MVP 功能需求

> 相机、禁行区、电量面板不在本版本范围，勿实现。

### F1：地图 + 实时位姿 + 激光点云

* 自渲染 OccupancyGrid 到 canvas-map（见第五节）
* 收到首张/map 时自动执行fitToView()，使地图居中适配画布；后续地图更新不重置视口
* canvas-overlay RAF 循环中绘制：
  * 机器人：三角箭头，顶点指向 yaw 方向，叠加在地图坐标对应像素
  * 激光点：红色 2px 圆，使用 renderLaserScan(scan) 渲染（⚠️ 见 5.6 节：必须用 scan.header.frame\_id 查 TF，不能直接用 base\_footprint 位姿）
* 地图交互（Phase 1 桌面版）：
  * 滚轮缩放（以鼠标位置为中心）
  * 鼠标拖动平移（与长按发目标互斥，见 F2）
  * 双击复位到首次 fit-to-view 状态（不是 scale=1 的初始状态）

### F2：地图点击发送导航目标

交互流程（Phase 1 桌面版）：

1. 鼠标在地图上按住不动 500ms（长按）→ overlay 出现目标标记预览
2. 底部滑出确认条，显示世界坐标 (x, y)
3. 用户点击"确认"→ publish 到 /goal\_pose；点击"取消"→ 清除预览

⚠️ 长按与拖动平移的冲突处理（鼠标 mousemove > 5px 即判定为平移，取消长按计时器）：

```javascript
let longPressTimer = null;
let longPressOrigin = null;   // mousedown 时的坐标（用于延迟计算 canvasToWorld）

overlayCanvas.addEventListener('mousedown', e => {
    if (e.button !== 0) return;
    longPressOrigin = { x: e.offsetX, y: e.offsetY, clientX: e.clientX, clientY: e.clientY };
    longPressTimer  = setTimeout(() => {
        // 500ms 无移动 → 长按触发，显示预览
        const world = canvasToWorld(longPressOrigin.x, longPressOrigin.y);
        if (world) showGoalPreview(longPressOrigin.x, longPressOrigin.y, world);
        longPressOrigin = null;
    }, 500);
});

overlayCanvas.addEventListener('mousemove', e => {
    if (longPressOrigin) {
        const dx = e.clientX - longPressOrigin.clientX;
        const dy = e.clientY - longPressOrigin.clientY;
        if (dx*dx + dy*dy > 25) {   // >5px（5²=25，避免 sqrt）
            clearTimeout(longPressTimer);
            longPressTimer  = null;
            longPressOrigin = null;
            // 继续执行正常平移逻辑
        }
    }
    // ... 拖动平移代码
});

overlayCanvas.addEventListener('mouseup',   () => { clearTimeout(longPressTimer); longPressTimer = null; longPressOrigin = null; });
overlayCanvas.addEventListener('mouseleave',() => { clearTimeout(longPressTimer); longPressTimer = null; longPressOrigin = null; });
```

Phase 1 不考虑目标姿态，朝向固定为单位四元数（w:1，Nav2 BT 自行规划路径）：

```javascript
const goalPosePub = new ROSLIB.Topic({
    ros: ros,
    name: CONFIG.TOPIC_GOAL_POSE,
    messageType: 'geometry_msgs/PoseStamped'
});

function sendSingleGoal(wx, wy) {
    goalPosePub.publish(new ROSLIB.Message({
        header: { frame_id: 'map' },
        pose: {
            position:    { x: wx, y: wy, z: 0 },
            orientation: { x: 0,  y: 0,  z: 0, w: 1 }  // 固定单位四元数
        }
    }));
}
```

限制（Phase 1 已知）：无法从 JS 侧 cancel 正在执行的目标，用急停代替。

### F3：虚拟摇杆遥控

* nipplejs 固定摇杆（Phase 1 鼠标拖动即可，Phase 2 再优化触控体验）
* 摇杆输出 → /cmd\_vel\_keyboard（geometry\_msgs/Twist）
  * 前后 → linear.x，范围 ±CONFIG.MAX\_LINEAR（默认 0.3 m/s）
  * 左右 → angular.z，范围 ±CONFIG.MAX\_ANGULAR（默认 1.0 rad/s）
* 松开摇杆立即发零速

### 急停：持续零速锁定

急停不是一次性发零速，而是进入"锁定"状态，持续以固定频率发送零速，直到用户手动解除：

```javascript
let estopActive = false;
let estopTimer  = null;

function activateEstop() {
    estopActive = true;
    document.getElementById('btn-estop').classList.add('active');  // 按钮变色提示激活
    // 以 10Hz 持续发送零速，覆盖任何残留指令
    estopTimer = setInterval(() => {
        cmdVelPub.publish(new ROSLIB.Message({
            linear:  { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 }
        }));
    }, 100);
}

function releaseEstop() {
    estopActive = false;
    clearInterval(estopTimer);
    document.getElementById('btn-estop').classList.remove('active');
}

// 急停期间拦截摇杆输入
function onJoystickMove(data) {
    if (estopActive) return;   // 锁定中，忽略摇杆
    // ... 正常发布速度指令
}
```

* 急停激活时，摇杆和导航指令均被拦截（if (estopActive) return）
* 再次点击急停按钮，或点击专属"解除急停"按钮，调用 releaseEstop()
* 急停状态在 UI 上有明显持续提示（按钮常亮红色）

### F4：传感器面板

面板可折叠，无数据时显示 --：

* IMU 姿态角（roll / pitch / yaw，单位°，从 CONFIG.TOPIC\_IMU 四元数转换）
* 导航状态（空闲 / 导航中 / 目标达到 / 失败，来自 feedback topic）

⚠️ IMU 话题因环境而异：

* 实机：/imu\_broad/imu
* Gazebo 仿真：/imu

CONFIG.TOPIC\_IMU 默认设为实机值。仿真调试时若面板持续显示 --，先执行ros2 topic list | grep imu 确认实际话题名，再修改 CONFIG。

---

## 八、UI/UX 规范

### 布局（Phase 1 桌面优先）

```
八、UI/UX 规范

布局（Phase 1 桌面优先）

┌──────────────────────────────────────────────┐
│ Nav2 控制台        [● ROS连接] [导航状态] [急停] │  顶部状态栏，高度约 46px
├──────────────────────────────────────────────┤
│                        [单点导航] [复位视图] [图层] │  右上角浮动操作按钮
│              地图画布区                        │  占剩余高度约 70%
│        canvas-map + canvas-overlay           │ 
│                                              │
│                                              │  
│                                              │
├────────────────┬──────────────┬──────────────┤
│ SLAM 里程计     │   遥控        │   导航        │  底部信息/控制区，约 30%
│ X / Y / Yaw     │ nipplejs摇杆  │ 状态 / 目标点 │
│ 当前线速度/角速度│ 零速按钮      │ Nav2输出v/ω   │
└────────────────┴──────────────┴──────────────┘
```

* 配色：深色主题（#1a1a1a 背景，#e0e0e0 文字）
* 字号：主操作元素 ≥ 14px（桌面标准）
* 急停按钮：红色 #ff3b30，激活时常亮；解除急停按钮紧邻，仅激活时可见
* 长按交互：鼠标 mousedown 计时 500ms 触发，mouseup 在 500ms 内则取消（不触发导航）

地图风格

* 地图使用浅灰栅格风格。
* 黑色块表示障碍物。
* 中灰色块表示未知 / 未探索区域。
* 红色点表示激光点云。
* 蓝色线表示路径。
* 绿色三角表示机器人当前位置和朝向。
* 黄色准星表示目标点。

右上角地图按钮

```
[单点导航]  [复位视图]  [图层]
```

* 单点导航：进入/提示单点导航交互。
* 复位视图：恢复首次 fit-to-view 视角。
* 图层：点击后展开图层控制面板。
  图层面板内容：
  ```
  图层控制
  [开关] 显示地图
  [开关] 显示激光
  [开关] 显示机器人
  [开关] 显示路径
  [开关] 显示目标点
  ```

底部信息区

SLAM 里程计：

```
X        1.82 m	Y       -0.34 m	Yaw    126.3 °
当前线速度 linear.x   +0.18 m/s	当前角速度 angular.z  -0.42 rad/s
```

遥控：

```
nipplejs 摇杆
松手即零速
[零速]
```

导航：

```
状态       导航中	目标       2.41, -0.82
v          +0.20 m/s	ω          -0.36 rad/s
```

其中 SLAM 里程计 下方的速度表示机器人当前实际速度；导航 下方的 v / ω 表示 Nav2 输出速度，用于和实际速度对照排查问题。

长按导航交互

* 鼠标 mousedown 后计时 500ms。
* 500ms 内 mouseup：取消，不触发导航。
* 鼠标移动超过 5px：取消长按，进入正常地图平移。
* 长按成功后：
  * 地图上显示目标点预览。
  * 底部弹出目标确认条：

```
目标预览
X 1.24 m   Y -0.58 m
[确认] [取消]
```

### Phase 2 待补充（不在本版本实现）

* 双指捏合缩放、单指拖动平移（替换滚轮/鼠标方案）
* nipplejs 触控优化（固定位置、防误触）
* 按钮尺寸放大到 ≥ 44×44px（移动端触摸目标标准）

---

## 九、CSS 视觉规范（Open Props 暗色单主题）

### 使用原则

* index.html 中 <link rel="stylesheet" href="vendor/open-props.min.css"> 引入设计 token
* 只用 Open Props 的 CSS 变量，手写所有组件样式；不引入任何 CSS 组件框架
* Phase 1 暗色单主题：在 :root {} 中直接将 Open Props token 映射到语义变量，无需主题切换机制

```css
/* index.html 内 <style> 块，紧跟 Open Props 引入后 */

/* ── 语义 token（Open Props → 业务变量）──────────────────────── */
:root {
    --bg-base    : var(--gray-9);   /* 中性深灰，Open Props gray 不带蓝色调 */
    --bg-surface : var(--gray-8);
    --bg-panel   : var(--gray-7);
    --text-main  : var(--gray-1);
    --text-dim   : var(--gray-4);
    --accent     : var(--blue-5);
    --danger     : var(--red-5);    /* 急停按钮 */
    --success    : var(--green-5);  /* 连接状态 */
    --warn       : var(--orange-5); /* Demo 模式标识 */
    --radius     : var(--radius-2);
    --shadow     : var(--shadow-2);
    --font-ui    : var(--font-sans);
    --size-base  : var(--font-size-1);
}

/* ── 全局重置 ─────────────────────────────────────────────────── */
*, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
body { background: var(--bg-base); color: var(--text-main);
       font-family: var(--font-ui); font-size: var(--size-base); height: 100dvh; overflow: hidden; }

/* ── 其余组件样式在 Demo 模式 UI 迭代阶段完成，按实际参考图手写 ─ */
```

> 注意：Open Props 不使用 data-theme="dark"（那是 Pico CSS 的机制）。暗色主题直接在 :root 中定义，无需切换逻辑。

---

## 十、ROS Launch 文件

launch/webui.launch.py 完成：

1. 启动 rosbridge\_websocket 节点，端口 9090
2. ExecuteProcess 启动 python3 -m http.server 8080，cwd 指向已安装的 static/ 路径（get\_package\_share\_directory('ros2\_webui') + '/static'）
3. LogInfo 打印：请用浏览器打开 http://<IP>:8080，IP 查询：hostname -I

---

## 十一、实现步骤（严格按顺序，每步验收后再继续）

Step 1 — 包骨架 + vendor 下载

* 建 src/ros2\_webui/，写 package.xml、CMakeLists.txt
* scripts/download\_vendor.sh 下载以下三个文件到 static/vendor/：
  * roslib.min.js（roslibjs ≥ 0.20，确认含 CBOR：grep -c cbor static/vendor/roslib.min.js > 0）
  * nipplejs.min.js
  * open-props.min.css（从 https://unpkg.com/open-props 或 GitHub release 下载）
* 脚本末尾打印文件大小：ls -lh static/vendor/
* 验收：colcon build --packages-select ros2\_webui 通过

Step 2 — 连接框架 + Demo 模式

* 搭 index.html 骨架，引入 vendor/open-props.min.css，顶部 CONFIG 常量
* 实现 connectRos() + setupSubscriptions() 含自动重连（见第十二节代码规范）
* 实现 ?demo=1 检测与假数据生成器
* 连接状态指示（绿/黄/红），Demo 模式顶栏显示橙色"DEMO 模式"
* 验收：连接后断开 rosbridge，3s 内自动重连，状态指示正确

Step 3 — Canvas 分层 + 地图渲染 + fit-to-view

* resizeCanvases() 初始化双层 canvas（显示尺寸）
* 实现 decodeMap() → mapOffscreen，renderMapLayer() → viewport 投影
* CBOR 订阅 /map → decodeMap()；首次收到时调用fitToView()
* 鼠标交互：滚轮缩放（以鼠标位置为中心）、拖动平移、双击复位到 fit 状态
* 验收：Demo 假地图自动居中；缩放/平移后双击回到 fit 状态；地图层与 overlay 不错位

Step 4 — TF + 机器人位姿

* 订阅 /tf /tf\_static，填充 tfBuffer，实现 getRobotPose()
* overlay RAF 绘制机器人箭头（用 worldToCanvas 转换）
* 验收：Demo 模式下箭头运动且与地图对齐；缩放时不错位

Step 5 — 激光点云（⚠️ 用 frame\_id 查 TF）

* CBOR 订阅 /scan\_filtered（10Hz）
* 首次收到时 console.log('laser frame\_id:', scan.header.frame\_id) 确认帧名
* 实现 getStaticOffset(frameId) + renderLaserScan(scan)（见 5.6 节）
* 验收：激光点在地图正确位置；机器人移动时激光跟随；缩放时不错位

Step 6 — 单点导航 + 急停锁定

* 长按鼠标 500ms（移动 >5px 取消）→ 预览标记 → 确认条 → sendSingleGoal()
* 实现 activateEstop() / releaseEstop()，持续 10Hz 发零速
* 验收：真实/Gazebo 场景机器人导航；急停后持续静止直到手动解除

Step 7 — 摇杆遥控

* nipplejs 集成，速度映射，急停拦截检查（if (estopActive) return）

Step 8 — 传感器面板

* 订阅 CONFIG.TOPIC\_IMU → 欧拉角（⚠️ 仿真/实机话题不同，见 F4 说明）
* feedback → 导航状态

Step 9 — CSS 视觉 + Demo UI 迭代

* 按照第九节 CSS 规范手写组件样式（以参考图为准）
* 在 ?demo=1 模式下迭代 UI，无需连接 ROS

Step 10 — Launch + 部署

* 完善 webui.launch.py，Pi 上一键启动验收

---

## 十二、代码规范

```javascript
// index.html 顶部 CONFIG（所有可调参数集中在此，勿散落全文）
const CONFIG = {
    ROSBRIDGE_PORT   : 9090,
    TOPIC_MAP        : '/map',
    TOPIC_TF         : '/tf',
    TOPIC_TF_STATIC  : '/tf_static',
    TOPIC_SCAN       : '/scan_filtered',    // 已滤自反射，不用 /scan
    TOPIC_CMD_VEL    : '/cmd_vel_keyboard',
    TOPIC_GOAL_POSE  : '/goal_pose',        // 单点导航：直接 pub PoseStamped
    // ⚠️ IMU 话题因环境不同：实机=/imu_broad/imu，仿真=/imu
    // 仿真阶段面板恒 "--" 时，先 ros2 topic list | grep imu 确认再修改
    TOPIC_IMU        : '/imu_broad/imu',
    ROBOT_BASE_FRAME : 'base_footprint',    // 不用 base_link
    MAP_FRAME        : 'map',
    MAX_LINEAR       : 0.3,    // m/s
    MAX_ANGULAR      : 1.0,    // rad/s
    SCAN_THROTTLE_MS : 100,
    MAP_THROTTLE_MS  : 1000,   // 1Hz；纯导航场景可调高到 2000
    ESTOP_INTERVAL_MS: 100,    // 急停锁定发送间隔（10Hz）
    RECONNECT_DELAY_MS: 3000,  // 断连后重试间隔
};

// ── WebSocket 连接 + 自动重连 ───────────────────────────────────────────
// roslibjs 默认不自动重连；监听 'close' 事件手动重建
let ros = null;

function connectRos() {
    ros = new ROSLIB.Ros({
        url: `ws://${window.location.hostname}:${CONFIG.ROSBRIDGE_PORT}`
    });
    ros.on('connection', () => {
        updateStatusIndicator('connected');
        setupSubscriptions();   // 重连后重建所有订阅（旧 Topic 对象已失效）
    });
    ros.on('error', () => updateStatusIndicator('error'));
    ros.on('close', () => {
        updateStatusIndicator('disconnected');
        setTimeout(connectRos, CONFIG.RECONNECT_DELAY_MS);  // 3s 后重试
    });
}

function setupSubscriptions() {
    // 在此集中创建/重建所有 ROSLIB.Topic 订阅
    // subscribeMap(); subscribeTF(); subscribeScan(); subscribeIMU();
}
```

* 每处 ROSLIB 订阅注释说明消息类型和关键字段
* 话题无数据时显示 --，不崩溃
* 单文件，无 npm，无构建步骤

---

## 十三、验收清单

构建与启动

* [ ]  colcon build --packages-select ros2\_webui 通过
* [ ]  ros2 launch ros2\_webui webui.launch.py 一条命令启动
* [ ]  static/vendor/ 含 roslib.min.js、nipplejs.min.js、open-props.min.css

Demo 模式

* [ ]  ?demo=1 下地图自动居中（fit-to-view），机器人/激光正确显示
* [ ]  Demo 模式下急停锁定/解除正常；摇杆急停期间无效

视口与渲染

* [ ]  首次收到 /map 自动 fit-to-view，地图居中适配画布
* [ ]  双击复位到 fit 状态（不是 scale=1）
* [ ]  缩放/平移时机器人箭头和激光点与地图无错位
* [ ]  窗口 resize 后 canvas 自适应，不错位

交互

* [ ]  鼠标移动 >5px 后，长按计时器取消，正常平移地图
* [ ]  长按 500ms 不移动，出现目标预览 + 确认条
* [ ]  确认后机器人导航到目标点
* [ ]  激光点位置正确（用 scan.header.frame\_id 走 TF 链，不是直接用 base\_footprint）

急停与遥控

* [ ]  急停激活后机器人持续静止，直到手动点"解除急停"
* [ ]  摇杆控制运动，松手停止，急停期间摇杆无效

传感器

* [ ]  IMU 姿态角实时更新（实机 /imu\_broad/imu，仿真 /imu）
* [ ]  导航状态正确显示

鲁棒性

* [ ]  ROS 断连后 3s 自动重连，连接状态指示正确，重连后订阅自动恢复
* [ ]  开发机 file:// 打开时 Demo 模式正常，不报 WebSocket 错误

---

## 十四、Phase 2（本版本不实现）

* 多点巡航（/follow\_waypoints Action，需先验证 roslibjs ROS2 action 兼容性，见 [roslibjs#782](https://github.com/RobotWebTools/roslibjs/issues/782)）
* 手机触控适配（双指缩放、单指平移替换鼠标方案；触控目标 ≥ 44×44px；nipplejs 触控优化）
* 相机画面（web\_video\_server 或 mjpeg\_streamer）
* 禁行区 / Costmap 可视化编辑
* 电量显示（需 STM32 端发布 /battery\_state）
* 目标朝向设置（Phase 1 固定单位四元数）
* 用户认证
