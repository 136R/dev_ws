#!/usr/bin/env python3
"""
motor_calibrate.py — 电机 PID 参数自动标定工具
=================================================

原理（三阶段自动运行）：
  Phase 1  静态增益 Km：纯P稳态关系
             RPM_ss = target × Km·kp / (1 + Km·kp)
             → Km = (1/kp) × RPM_ss / (target − RPM_ss)

  Phase 2  机械时间常数 τ：闭环阶跃响应
             闭环一阶时间常数 τ_cl = 时间到达 63.2% × RPM_ss
             → τ = τ_cl × (1 + Km·kp)  = τ_cl × target / (target − RPM_ss)

  Phase 3  极点配置：给定期望 Ts（调节时间）与 ζ（阻尼比），求 kp, ki
             ωn = 4 / (ζ·Ts)
             ki = τ·ωn² / Km
             kp_new = (2·ζ·ωn·τ − 1) / Km
             k_ff = 1 / Km（可选，默认不启用）

前提条件（必须满足）：
  - 固件中 ki=0, k_ff=0（Config.h 默认值已满足）
  - kp=10（Config.h PID_DEFAULT_KP，标定时需与此处 --kp 一致）
  - 电机可以自由转动，不要手动阻力

用法:
  python3 motor_calibrate.py                         # 标定左轮，默认参数
  python3 motor_calibrate.py --motor right            # 标定右轮
  python3 motor_calibrate.py --target 80 --ts 0.8    # 指定目标转速和期望调节时间
  python3 motor_calibrate.py --port /dev/ttyS3       # 指定串口

依赖:  pip3 install pyserial
"""

import serial, struct, time, sys, math, argparse, statistics

# ── 协议常量（与 Config.h / test_usart1.py 一致）─────────────
FRAME_HEADER = 0x5A
DEVICE_ID    = 0x01
CRC8_POLY    = 0x8C

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ CRC8_POLY if crc & 1 else crc >> 1
    return crc & 0xFF

def build_cmd(left_rpm: float, right_rpm: float) -> bytes:
    body = struct.pack('>BBBBhhB',
                       FRAME_HEADER, 10, DEVICE_ID, 0x01,
                       int(left_rpm * 10), int(right_rpm * 10), 0x00)
    return body + bytes([crc8(body)])

def recv_rpm(ser: serial.Serial, timeout=1.0):
    """读取一帧 0x04，返回 (left_rpm, right_rpm) 或 None"""
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        buf += b
        if len(buf) < 2:
            if buf[0] != FRAME_HEADER: buf.clear()
            continue
        if buf[0] != FRAME_HEADER:
            buf.pop(0); continue
        flen = buf[1]
        if flen < 6 or flen > 64:
            buf.clear(); continue
        while len(buf) < flen and time.monotonic() < deadline:
            c = ser.read(flen - len(buf))
            if c: buf += c
        if len(buf) < flen:
            return None
        frame = bytes(buf[:flen])
        if frame[2] != DEVICE_ID or crc8(frame[:-1]) != frame[-1]:
            buf = buf[1:]; continue
        if frame[3] != 0x04:
            buf = buf[1:]; continue
        payload = frame[4:-2]
        _, l10, r10 = struct.unpack('>Ihh', payload[:8])
        return l10 / 10.0, r10 / 10.0
    return None

def collect(ser, motor_side, duration_s, cmd_left, cmd_right, label=""):
    """发送指令并收集指定时长的 RPM 数据，返回 [(t, rpm), ...]"""
    ser.write(build_cmd(cmd_left, cmd_right))
    t0 = time.monotonic()
    data = []
    print(f"  {label}", end="", flush=True)
    while time.monotonic() - t0 < duration_s:
        res = recv_rpm(ser, timeout=0.5)
        if res is None:
            continue
        t = time.monotonic() - t0
        rpm = res[0] if motor_side == 'left' else res[1]
        data.append((t, rpm))
        if len(data) % 10 == 0:
            print(".", end="", flush=True)
    print(f" ({len(data)} 点)")
    return data

# ── 分析函数 ──────────────────────────────────────────────────

def steady_state(data, last_frac=0.3):
    """取最后 last_frac 的均值作为稳态值"""
    n = max(1, int(len(data) * last_frac))
    vals = [r for _, r in data[-n:]]
    return statistics.mean(vals), statistics.stdev(vals) if len(vals) > 1 else 0.0

def find_tau_cl(data, rpm_ss):
    """找到 RPM 首次达到 63.2% 稳态值的时间（闭环时间常数）"""
    target_63 = 0.632 * rpm_ss
    for i in range(1, len(data)):
        if data[i][1] >= target_63:
            # 线性插值
            t0, r0 = data[i-1]
            t1, r1 = data[i]
            if r1 > r0:
                tau_cl = t0 + (target_63 - r0) / (r1 - r0) * (t1 - t0)
                return tau_cl
    return None

def simulate_validate(Km, tau, kp, ki, k_ff, target, dt=0.005, seconds=4.0):
    """内嵌仿真（复现 motor.c 逻辑）验证计算出的增益"""
    PWM_ARR = 3599
    pid = {'kp': kp, 'ki': ki, 'k_ff': k_ff,
           'e_prev': 0.0, 'output': 0.0, 'output_max': float(PWM_ARR)}
    motor = {'filtered_rpm': 0.0, 'lpf_alpha': 0.8,
             'rpm_target': target, 'pid': pid}

    rpm_real = 0.0
    data = []
    steps = int(seconds / dt)
    for _ in range(steps):
        # LPF
        a = motor['lpf_alpha']
        motor['filtered_rpm'] = a * rpm_real + (1-a) * motor['filtered_rpm']
        # 动态 output_max
        ff = abs(pid['k_ff'] * motor['rpm_target'])
        pid['output_max'] = max(0.0, float(PWM_ARR) - ff)
        # PI
        e = motor['rpm_target'] - motor['filtered_rpm']
        p_term = pid['kp'] * (e - pid['e_prev'])
        i_term = pid['ki'] * dt * e
        skip_i = ((pid['output'] >= pid['output_max']) and (i_term > 0.0)) or \
                 ((pid['output'] <= -pid['output_max']) and (i_term < 0.0))
        delta = p_term + (0.0 if skip_i else i_term)
        skip_d = ((pid['output'] >= pid['output_max']) and (delta > 0.0)) or \
                 ((pid['output'] <= -pid['output_max']) and (delta < 0.0))
        if not skip_d:
            pid['output'] += delta
        pid['output'] = max(-pid['output_max'], min(pid['output_max'], pid['output']))
        pid['e_prev'] = e
        pwm = pid['output'] + pid['k_ff'] * motor['rpm_target']
        pwm = max(-float(PWM_ARR), min(float(PWM_ARR), pwm))
        # 电机模型
        rpm_real += (Km * pwm - rpm_real) / tau * dt
        data.append(rpm_real)

    win = int(0.5 / dt)
    ss   = sum(data[-win:]) / win
    sse  = abs(ss - target)
    overshoot = max(0.0, max(data) - target)

    # 调节时间
    tol = 0.05 * target  # 5% 容差
    settle_t = None
    for i in range(len(data) - win):
        if all(abs(v - target) < tol for v in data[i:i+win]):
            settle_t = i * dt
            break
    return sse, overshoot, settle_t

# ── 主程序 ────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description='电机 PID 自动标定')
    p.add_argument('--port',   default='/dev/ttyS7')
    p.add_argument('--baud',   type=int, default=460800)
    p.add_argument('--motor',  choices=['left','right'], default='left')
    p.add_argument('--kp',     type=float, default=10.0,  help='固件当前 kp（需与 Config.h 一致）')
    p.add_argument('--target', type=float, default=100.0, help='标定目标转速 RPM（建议 60-120）')
    p.add_argument('--ts',     type=float, default=1.0,   help='期望调节时间 Ts [s]')
    p.add_argument('--zeta',   type=float, default=0.8,   help='阻尼比 ζ（推荐 0.7-1.0）')
    p.add_argument('--ff',     action='store_true',       help='同时计算并启用 k_ff')
    args = p.parse_args()

    print("=" * 60)
    print("  电机 PID 自动标定工具")
    print(f"  串口: {args.port} @ {args.baud}  电机: {args.motor}")
    print(f"  期望: Ts={args.ts}s, ζ={args.zeta}")
    print("=" * 60)
    print()
    print("⚠  前提检查：")
    print(f"   Config.h: PID_DEFAULT_KP={args.kp}, PID_DEFAULT_KI=0, k_ff=0")
    print("   电机可以自由转动，无外部阻力")
    input("   确认无误后按 Enter 开始标定...")
    print()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print(f"❌ 串口打开失败: {e}")
        sys.exit(1)

    time.sleep(0.2)
    ser.reset_input_buffer()

    L = args.motor == 'left'
    tgt = args.target

    # ── Phase 0: 先停止电机 ──────────────────────────────────
    print("── Phase 0  初始化：停止电机 ──")
    collect(ser, args.motor, 2.0, 0, 0, "等待电机停止")
    print()

    # ── Phase 1: 静态增益 Km ─────────────────────────────────
    print("── Phase 1  静态增益 Km 测量 ──")
    print(f"  发送 target={tgt} RPM，等待稳态（ki=0，纯P）...")
    cmd_l = tgt if L else 0
    cmd_r = 0 if L else tgt
    data1 = collect(ser, args.motor, 4.0, cmd_l, cmd_r, "采集稳态数据")

    rpm_ss, rpm_std = steady_state(data1)
    if rpm_ss < 1.0:
        print(f"❌ 稳态转速太低 ({rpm_ss:.1f} RPM)，检查接线或增大 --target")
        sys.exit(1)
    if abs(rpm_ss - tgt) < 1.0:
        print(f"❌ 稳态接近目标值，ki 可能不为0（检查 Config.h）")
        sys.exit(1)

    Km = (1.0 / args.kp) * rpm_ss / (tgt - rpm_ss)
    print(f"  稳态 RPM_ss = {rpm_ss:.2f} ± {rpm_std:.2f} RPM")
    print(f"  Km = {Km:.5f} RPM/count  (理论: {200.0/3599:.5f})")
    print()

    # ── Phase 2: 机械时间常数 τ ──────────────────────────────
    print("── Phase 2  机械时间常数 τ 测量 ──")
    print("  先回零...")
    collect(ser, args.motor, 2.0, 0, 0, "等待归零")

    print(f"  阶跃至 {tgt} RPM，测量上升过程...")
    t_step = time.monotonic()
    ser.write(build_cmd(cmd_l, cmd_r))
    data2 = []
    t0 = time.monotonic()
    print("  采集阶跃响应", end="", flush=True)
    while time.monotonic() - t0 < 3.0:
        res = recv_rpm(ser, timeout=0.5)
        if res is None: continue
        t = time.monotonic() - t0
        rpm = res[0] if L else res[1]
        data2.append((t, rpm))
        if len(data2) % 5 == 0:
            print(".", end="", flush=True)
    print(f" ({len(data2)} 点)")

    tau_cl = find_tau_cl(data2, rpm_ss)
    if tau_cl is None:
        print(f"❌ 未找到 63.2% 穿越点，响应异常")
        sys.exit(1)

    tau = tau_cl * (tgt / (tgt - rpm_ss))
    print(f"  闭环时间常数 τ_cl = {tau_cl:.4f} s")
    print(f"  机械时间常数  τ   = {tau:.4f} s  (理论: 0.2s)")
    print()

    # ── Phase 3: 极点配置计算 ────────────────────────────────
    print("── Phase 3  极点配置计算 ──")
    Ts   = args.ts
    zeta = args.zeta
    wn   = 4.0 / (zeta * Ts)
    ki   = tau * wn**2 / Km
    kp   = (2 * zeta * wn * tau - 1) / Km
    kff  = 1.0 / Km if args.ff else 0.0

    # kp 合理性检查
    if kp < 0:
        print(f"  ⚠  kp={kp:.1f} < 0，Ts 期望太大；自动调整到最小稳定 kp")
        kp = 0.5 / Km   # 最小实用值
    if ki < 0:
        ki = 0.1

    print(f"  目标性能: Ts={Ts}s, ζ={zeta}")
    print(f"  ωn = {wn:.3f} rad/s  ({wn/(2*math.pi):.3f} Hz)")
    print()

    # ── Phase 4: 仿真验证 ────────────────────────────────────
    print("── Phase 4  仿真验证（使用实测 Km, τ）──")
    sse, overshoot, settle_t = simulate_validate(Km, tau, kp, ki, kff, tgt)

    st_str = f"{settle_t:.3f}s" if settle_t else "未收敛(>4s)"
    ok_conv = settle_t is not None
    ok_sse  = sse < tgt * 0.03
    ok_over = overshoot < tgt * 0.20

    def pf(cond): return "✅" if cond else "⚠ "
    print(f"  {pf(ok_conv)} 调节时间:  {st_str}  (期望 ≤{Ts}s)")
    print(f"  {pf(ok_sse)}  稳态误差:  {sse:.2f} RPM  (期望 <{tgt*0.03:.1f})")
    print(f"  {pf(ok_over)} 超调量:    {overshoot:.1f} RPM  (期望 <{tgt*0.20:.1f})")
    print()

    # ── 结果输出 ─────────────────────────────────────────────
    print("=" * 60)
    print("  标定结果（写入 Config.h）")
    print("=" * 60)
    print(f"""
  /* 电机 PID 参数（{args.motor} 轮，由 motor_calibrate.py 自动标定）
   * 实测: Km={Km:.5f} RPM/count, τ={tau:.4f}s
   * 期望: Ts={Ts}s, ζ={zeta}
   */
  #define PID_DEFAULT_KP   {kp:.1f}f
  #define PID_DEFAULT_KI   {ki:.1f}f
""")
    if args.ff:
        print(f"  /* 前馈增益（可选，需先确认 PI 稳定后再启用）*/")
        print(f"  /* k_ff = {kff:.3f}f  写入 Motor_Init 中 motor->pid.k_ff */")
        print()

    print(f"  /* lpf_alpha 推荐值（基于 τ 和噪声水平） */")
    # 推荐截止频率约为 ωn/2，对应 alpha
    fc_rec = wn / (2 * math.pi) * 2   # 截止频率约 2×闭环带宽
    # alpha 从 fc 反推：fc = fs/2π * arccos(1 - (1-alpha)²/(2*(1-alpha))) ... 近似
    # 简化近似: alpha ≈ 1 - 2π*fc/fs
    alpha_approx = 1.0 - 2 * math.pi * fc_rec / CONTROL_HZ
    alpha_rec = max(0.3, min(0.95, alpha_approx))
    print(f"  /* lpf_alpha = {alpha_rec:.2f}f  (截止频率≈{fc_rec:.1f}Hz, 200Hz采样) */")
    print()
    print(f"  /* 参数设置后重新编译烧录，再次阶跃测试验证 */")

    # 停止电机
    ser.write(build_cmd(0, 0))
    ser.close()
    print()
    print("  电机已停止，标定完成。")
    print("=" * 60)


CONTROL_HZ = 200   # 与 Config.h 一致

if __name__ == '__main__':
    main()
