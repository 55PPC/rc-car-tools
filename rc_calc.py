#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RC 遥控车研发计算工具  rc_calc.py
=================================
纯 Python 3 标准库，无需安装任何依赖。macOS / Windows 通用。

运行:   python3 rc_calc.py          (交互菜单)
        python3 rc_calc.py --demo   (用示例参数跑一遍所有计算)
也可以在别的脚本里 import rc_calc 直接调用函数。

说明:
- 所有结果都是【理论值 / 初步估算】，不考虑打滑、轮胎变形、电压跌落等，
  务必与【实测数据】分开记录、对照修正。
- 材料强度为常见参考值，具体以供应商材料数据表为准。
"""

import math
import sys

# ---------------------------------------------------------------------------
# 1. 齿比 / 减速比
# ---------------------------------------------------------------------------

def gear_ratio(driven_teeth, drive_teeth):
    """单级齿比 = 从动齿数 / 主动齿数"""
    return driven_teeth / drive_teeth


def final_drive_ratio(pinion, spur, internal_ratio=1.0, extra_ratios=()):
    """
    总减速比 FDR = (主齿 spur / 马达齿 pinion) × 内部减速比 × 其他级
    internal_ratio: 变速箱/差速器内部减速比（如差速器伞齿 37:13 → 2.846）
    """
    fdr = gear_ratio(spur, pinion) * internal_ratio
    for r in extra_ratios:
        fdr *= r
    return fdr


# ---------------------------------------------------------------------------
# 2. 电机 / 车速
# ---------------------------------------------------------------------------

LIPO_NOMINAL_V = 3.7   # 每节锂电标称电压
LIPO_FULL_V = 4.2      # 每节满电电压


def battery_voltage(cells, per_cell=LIPO_NOMINAL_V):
    return cells * per_cell


def motor_rpm(kv, voltage, load_factor=1.0):
    """电机转速 rpm = KV × 电压 × 负载系数（空载=1，带载一般取 0.8~0.9）"""
    return kv * voltage * load_factor


def wheel_rpm(motor_rpm_value, fdr):
    return motor_rpm_value / fdr


def speed_kmh(wheel_rpm_value, tire_diameter_mm):
    """车速 km/h = 轮转速 × π × 轮径"""
    m_per_min = wheel_rpm_value * math.pi * tire_diameter_mm / 1000.0
    return m_per_min * 60.0 / 1000.0


def top_speed(kv, voltage, fdr, tire_diameter_mm, load_factor=1.0):
    """一步算出理论极速 (km/h)"""
    return speed_kmh(wheel_rpm(motor_rpm(kv, voltage, load_factor), fdr), tire_diameter_mm)


def fdr_for_target_speed(target_kmh, kv, voltage, tire_diameter_mm, load_factor=1.0):
    """反推：要达到目标车速需要的总减速比"""
    rpm = motor_rpm(kv, voltage, load_factor)
    wheel_rpm_needed = target_kmh * 1000.0 / 60.0 / (math.pi * tire_diameter_mm / 1000.0)
    return rpm / wheel_rpm_needed


def rollout_mm(fdr, tire_diameter_mm):
    """Rollout：电机转一圈车前进的距离 (mm)，房车圈常用"""
    return math.pi * tire_diameter_mm / fdr


# ---------------------------------------------------------------------------
# 3. 扭矩 / 推力 / 加速度
# ---------------------------------------------------------------------------

def motor_kt(kv):
    """扭矩常数 Kt (N·m/A) = 60 / (2π·KV) ≈ 9.549 / KV"""
    return 60.0 / (2.0 * math.pi * kv)


def motor_torque(kv, current_a, no_load_current_a=0.0):
    """电机输出扭矩 (N·m) = Kt × (I - I0)"""
    return motor_kt(kv) * max(current_a - no_load_current_a, 0.0)


def wheel_torque_total(motor_torque_nm, fdr, efficiency=0.85):
    """所有驱动轮合计扭矩 (N·m) = 电机扭矩 × 减速比 × 传动效率"""
    return motor_torque_nm * fdr * efficiency


def tractive_force(wheel_torque_nm, tire_diameter_mm):
    """地面推力 (N) = 轮扭矩 / 轮半径"""
    return wheel_torque_nm / (tire_diameter_mm / 2000.0)


def acceleration(force_n, mass_kg, mu=None):
    """加速度 (m/s²)；给出摩擦系数 mu 时，会被抓地力上限 mu·g 截断（四驱近似）"""
    a = force_n / mass_kg
    if mu is not None:
        a = min(a, mu * 9.81)
    return a


def time_to_speed(target_kmh, accel):
    """匀加速近似：达到目标速度所需时间 (s)"""
    return (target_kmh / 3.6) / accel


# ---------------------------------------------------------------------------
# 4. 齿轮几何（标准直齿，压力角 20°）
# ---------------------------------------------------------------------------

def dp_to_module(dp):
    """英制径节 DP（如 48P、64P）转模数 m = 25.4 / DP"""
    return 25.4 / dp


def module_to_dp(m):
    return 25.4 / m


def gear_geometry(module, teeth, pressure_angle_deg=20.0):
    """返回标准直齿轮的主要尺寸 (mm)"""
    d = module * teeth
    ha = module            # 齿顶高
    hf = 1.25 * module     # 齿根高
    return {
        "分度圆直径 d": d,
        "齿顶圆直径 da": d + 2 * ha,
        "齿根圆直径 df": d - 2 * hf,
        "基圆直径 db": d * math.cos(math.radians(pressure_angle_deg)),
        "齿距 p": math.pi * module,
        "齿厚 s": math.pi * module / 2,
        "全齿高 h": ha + hf,
    }


def center_distance(module, z1, z2):
    """标准中心距 a = m(z1+z2)/2"""
    return module * (z1 + z2) / 2.0


def min_teeth_no_undercut(pressure_angle_deg=20.0):
    """不根切最少齿数 z_min = 2 / sin²α（20° 时约 17）"""
    return 2.0 / math.sin(math.radians(pressure_angle_deg)) ** 2


def pinion_spur_combos(module, target_center_mm, pinion_range=(10, 40),
                       spur_range=(40, 120), tol_mm=0.05):
    """在固定中心距下列出可用的 马达齿/主齿 组合"""
    total = 2.0 * target_center_mm / module
    res = []
    for p in range(pinion_range[0], pinion_range[1] + 1):
        s = round(total - p)
        if spur_range[0] <= s <= spur_range[1]:
            err = center_distance(module, p, s) - target_center_mm
            if abs(err) <= tol_mm:
                res.append((p, s, s / p, err))
    return res


def mesh_center_for_pinion_change(module, pinion, spur):
    """换马达齿后电机座要调到的中心距"""
    return center_distance(module, pinion, spur)


# ---------------------------------------------------------------------------
# 5. 齿轮强度（Lewis 弯曲应力，初步校核）
# ---------------------------------------------------------------------------

# 20° 全齿高标准齿形的 Lewis 齿形系数 Y
_LEWIS_Y = [(12, .245), (13, .261), (14, .277), (15, .290), (16, .296), (17, .303),
            (18, .309), (19, .314), (20, .322), (21, .328), (22, .331), (24, .337),
            (26, .346), (28, .353), (30, .359), (34, .371), (38, .384), (43, .397),
            (50, .409), (60, .422), (75, .435), (100, .447), (150, .460), (300, .472)]


def lewis_y(teeth):
    if teeth <= _LEWIS_Y[0][0]:
        return _LEWIS_Y[0][1]
    if teeth >= _LEWIS_Y[-1][0]:
        return _LEWIS_Y[-1][1]
    for (z0, y0), (z1, y1) in zip(_LEWIS_Y, _LEWIS_Y[1:]):
        if z0 <= teeth <= z1:
            return y0 + (y1 - y0) * (teeth - z0) / (z1 - z0)


def gear_tangential_force(torque_nm, module, teeth):
    """切向力 Ft (N) = 2T / d"""
    d_m = module * teeth / 1000.0
    return 2.0 * torque_nm / d_m


def lewis_bending_stress(torque_nm, module, teeth, face_width_mm, dynamic_factor=1.5):
    """
    Lewis 弯曲应力 σ (MPa) = Kv·Ft / (b·m·Y)
    dynamic_factor: 动载/冲击系数，RC 落地、急加速冲击大，建议取 1.5~3
    """
    ft = gear_tangential_force(torque_nm, module, teeth)
    return dynamic_factor * ft / (face_width_mm * module * lewis_y(teeth))


# ---------------------------------------------------------------------------
# 6. 轴（传动轴、半轴、差速器轴）
# ---------------------------------------------------------------------------

# 参考屈服强度 (MPa)。塑料无明显屈服，取拉伸强度参考值。以实际材料数据表为准！
MATERIALS = {
    "PA66尼龙":          60,
    "PA66+30%玻纤":      150,
    "POM赛钢":           65,
    "6061-T6铝":         275,
    "7075-T6铝":         500,
    "45#钢(调质)":       355,
    "40Cr(调质)":        785,
    "钛合金TC4":         830,
}


def shaft_torsion_stress(torque_nm, diameter_mm, inner_diameter_mm=0.0):
    """实心/空心轴扭转剪应力 τ (MPa) = 16·T·D / (π(D⁴-d⁴))"""
    D, d = diameter_mm, inner_diameter_mm
    t_nmm = torque_nm * 1000.0
    return 16.0 * t_nmm * D / (math.pi * (D ** 4 - d ** 4))


def shaft_min_diameter(torque_nm, yield_mpa, safety_factor=2.0):
    """
    实心轴所需最小直径 (mm)
    许用剪应力取 0.5·σs / 安全系数（最大剪应力理论，偏保守）
    """
    tau_allow = 0.5 * yield_mpa / safety_factor
    return (16.0 * torque_nm * 1000.0 / (math.pi * tau_allow)) ** (1.0 / 3.0)


def shaft_twist_deg(torque_nm, length_mm, diameter_mm, shear_modulus_mpa):
    """扭转角 (°) = T·L / (G·J)；钢 G≈79000、铝 G≈26000 MPa"""
    J = math.pi * diameter_mm ** 4 / 32.0
    return math.degrees(torque_nm * 1000.0 * length_mm / (shear_modulus_mpa * J))


# ---------------------------------------------------------------------------
# 7. 电池 / 功率 / 车身
# ---------------------------------------------------------------------------

def run_time_min(capacity_mah, avg_current_a, usable=0.8):
    """续航 (min)，usable：可用容量比例（锂电建议不放到底，取 0.8）"""
    return capacity_mah / 1000.0 * usable / avg_current_a * 60.0


def max_battery_current(capacity_mah, c_rating):
    """电池最大持续放电电流 (A) = 容量(Ah) × C 数"""
    return capacity_mah / 1000.0 * c_rating


def electrical_power(voltage, current_a):
    return voltage * current_a


def weight_distribution(front_kg, rear_kg):
    total = front_kg + rear_kg
    return {"总重 kg": total, "前轴荷 %": front_kg / total * 100, "后轴荷 %": rear_kg / total * 100}


def cg_from_axle_loads(front_kg, rear_kg, wheelbase_mm):
    """重心到前轴的水平距离 (mm) = 后轴荷 × 轴距 / 总重"""
    return rear_kg * wheelbase_mm / (front_kg + rear_kg)


def rollover_lateral_g(track_width_mm, cg_height_mm):
    """静态侧翻临界横向加速度 (g) ≈ 轮距 / (2·重心高)；大于轮胎 μ 时先侧滑"""
    return track_width_mm / (2.0 * cg_height_mm)


# ===========================================================================
# 交互菜单
# ===========================================================================

def ask(prompt, default=None, cast=float):
    tip = f" [{default}]" if default is not None else ""
    while True:
        s = input(f"  {prompt}{tip}: ").strip()
        if s == "" and default is not None:
            return cast(default)
        try:
            return cast(s)
        except ValueError:
            print("    ↳ 输入无效，请重新输入数字")


def ask_voltage():
    mode = input("  电压输入方式：1=锂电节数  2=直接输入电压 [1]: ").strip() or "1"
    if mode == "2":
        return ask("电压 V", 7.4)
    cells = ask("锂电节数 S", 2, int)
    full = (input("  按 满电4.2V(f) 还是 标称3.7V(n) 计算 [n]: ").strip() or "n").lower()
    v = battery_voltage(cells, LIPO_FULL_V if full == "f" else LIPO_NOMINAL_V)
    print(f"    ↳ 电压 = {v:.2f} V")
    return v


def menu_ratio():
    print("\n【齿比 / 总减速比】")
    p = ask("马达齿 pinion 齿数", 18, int)
    s = ask("主齿 spur 齿数", 72, int)
    i = ask("内部减速比（差速器/变速箱，没有填 1）", 2.6)
    fdr = final_drive_ratio(p, s, i)
    print(f"\n  一级齿比 = {s}/{p} = {s/p:.3f}")
    print(f"  总减速比 FDR = {fdr:.3f} : 1")
    d = ask("轮胎直径 mm（算 rollout，可回车跳过）", 0)
    if d > 0:
        print(f"  Rollout = {rollout_mm(fdr, d):.2f} mm / 电机转")


def menu_speed():
    print("\n【理论车速】")
    kv = ask("电机 KV", 3300)
    v = ask_voltage()
    fdr = ask("总减速比 FDR", 8.0)
    d = ask("轮胎直径 mm", 108)
    lf = ask("带载转速系数（空载=1，带载估算 0.85）", 0.85)
    rpm0 = motor_rpm(kv, v)
    print(f"\n  电机空载转速 = {rpm0:,.0f} rpm")
    print(f"  空载理论极速 = {top_speed(kv, v, fdr, d):.1f} km/h")
    print(f"  带载估算极速 = {top_speed(kv, v, fdr, d, lf):.1f} km/h  (×{lf})")
    print("  ※ 理论值，实际受空气阻力、轮胎膨胀、电压跌落影响")


def menu_target():
    print("\n【反推：目标车速 → 减速比 / 马达齿】")
    tgt = ask("目标车速 km/h", 60)
    kv = ask("电机 KV", 3300)
    v = ask_voltage()
    d = ask("轮胎直径 mm", 108)
    lf = ask("带载转速系数", 0.85)
    fdr = fdr_for_target_speed(tgt, kv, v, d, lf)
    print(f"\n  需要总减速比 ≈ {fdr:.3f}")
    s = ask("主齿 spur 齿数（反算马达齿，回车跳过）", 0, int)
    if s > 0:
        i = ask("内部减速比", 2.6)
        p = s * i / fdr
        print(f"  需要马达齿 ≈ {p:.2f}，可选 {math.floor(p)} 或 {math.ceil(p)} 齿：")
        for pp in sorted({max(math.floor(p), 1), math.ceil(p)}):
            f = final_drive_ratio(pp, s, i)
            print(f"    {pp}T → FDR {f:.3f}，速度 {top_speed(kv, v, f, d, lf):.1f} km/h")


def menu_torque():
    print("\n【扭矩 / 推力 / 加速度】")
    kv = ask("电机 KV", 3300)
    cur = ask("电流 A（峰值或工作电流）", 60)
    i0 = ask("空载电流 A", 2)
    fdr = ask("总减速比 FDR", 8.0)
    eff = ask("传动效率", 0.85)
    d = ask("轮胎直径 mm", 108)
    m = ask("整车质量 kg", 2.0)
    mu = ask("轮胎摩擦系数 μ（地毯~1.0，柏油~0.8，土~0.6）", 0.8)
    tm = motor_torque(kv, cur, i0)
    tw = wheel_torque_total(tm, fdr, eff)
    f = tractive_force(tw, d)
    a_raw = acceleration(f, m)
    a = acceleration(f, m, mu)
    print(f"\n  Kt = {motor_kt(kv)*1000:.3f} mN·m/A")
    print(f"  电机扭矩 = {tm:.3f} N·m")
    print(f"  驱动轮合计扭矩 = {tw:.3f} N·m")
    print(f"  地面推力 = {f:.1f} N")
    print(f"  动力加速度 = {a_raw:.2f} m/s² ({a_raw/9.81:.2f} g)")
    if a < a_raw:
        print(f"  ⚠ 超过抓地极限，实际受限于 μg = {a:.2f} m/s²（会打滑）")
    tgt = ask("估算加速到多少 km/h 的时间（回车跳过）", 0)
    if tgt > 0:
        print(f"  匀加速近似 0→{tgt:.0f} km/h ≈ {time_to_speed(tgt, a):.2f} s（偏乐观）")


def menu_gear_geom():
    print("\n【齿轮几何 / 中心距】")
    u = input("  输入 1=模数 m   2=径节 DP（如 48P） [1]: ").strip() or "1"
    m = ask("模数 m", 1.0) if u == "1" else dp_to_module(ask("径节 DP", 48))
    print(f"    ↳ 模数 m = {m:.4f}  (≈ {module_to_dp(m):.1f} DP)")
    z1 = ask("齿轮1 齿数", 13, int)
    z2 = ask("齿轮2 齿数（单个齿轮填 0）", 37, int)
    for z in [z1] + ([z2] if z2 > 0 else []):
        print(f"\n  —— {z} 齿 ——")
        for k, val in gear_geometry(m, z).items():
            print(f"    {k:<10} = {val:.3f} mm")
        if z < min_teeth_no_undercut():
            print(f"    ⚠ 少于 {min_teeth_no_undercut():.0f} 齿，标准齿形会根切，需变位或接受强度下降")
    if z2 > 0:
        print(f"\n  标准中心距 a = {center_distance(m, z1, z2):.3f} mm")
        print(f"  传动比 = {z2/z1:.3f}")


def menu_combos():
    print("\n【固定中心距下的 马达齿/主齿 组合】")
    m = ask("模数 m（48P≈0.529，64P≈0.397）", 0.6)
    a = ask("电机轴到主齿轴中心距 mm", 26.0)
    pr = (ask("马达齿 最小", 12, int), ask("马达齿 最大", 30, int))
    sr = (ask("主齿 最小", 50, int), ask("主齿 最大", 100, int))
    tol = ask("允许中心距误差 mm", 0.1)
    res = pinion_spur_combos(m, a, pr, sr, tol)
    if not res:
        print("  没有找到组合，放宽误差或范围试试")
    for p, s, r, e in res:
        print(f"  {p:>3}T / {s:>3}T  齿比 {r:.3f}  中心距误差 {e:+.3f} mm")


def menu_gear_strength():
    print("\n【齿轮弯曲强度（Lewis 初步校核）】")
    t = ask("该齿轮承受扭矩 N·m", 0.5)
    m = ask("模数 m", 1.0)
    z = ask("齿数", 13, int)
    b = ask("齿宽 mm", 6.0)
    kv = ask("动载/冲击系数（RC 建议 1.5~3）", 2.0)
    ft = gear_tangential_force(t, m, z)
    sigma = lewis_bending_stress(t, m, z, b, kv)
    print(f"\n  Lewis Y = {lewis_y(z):.3f}")
    print(f"  切向力 Ft = {ft:.1f} N")
    print(f"  齿根弯曲应力 σ ≈ {sigma:.1f} MPa")
    print("  各材料安全系数（屈服或拉伸强度 / σ）：")
    for name, ys in MATERIALS.items():
        sf = ys / sigma
        flag = "✓" if sf >= 2 else ("△" if sf >= 1.2 else "✗")
        print(f"    {flag} {name:<12} {ys:>4} MPa  →  SF {sf:.2f}")
    print("  ※ 塑料齿轮还需考虑温升、蠕变、疲劳，SF 宜 ≥ 2~3")


def menu_shaft():
    print("\n【轴强度（传动轴 / 半轴 / 差速器轴）】")
    t = ask("轴承受扭矩 N·m（单根半轴约=轮总扭矩/驱动轮数，再乘冲击系数）", 1.0)
    d = ask("轴外径 mm", 5.0)
    di = ask("内径 mm（实心填 0）", 0.0)
    tau = shaft_torsion_stress(t, d, di)
    print(f"\n  扭转剪应力 τ = {tau:.1f} MPa")
    sf_target = ask("目标安全系数", 2.0)
    print("  各材料：允许剪应力 = 0.5σs，安全系数 / 最小实心直径")
    for name, ys in MATERIALS.items():
        sf = 0.5 * ys / tau
        dmin = shaft_min_diameter(t, ys, sf_target)
        flag = "✓" if sf >= sf_target else "✗"
        print(f"    {flag} {name:<12} SF {sf:5.2f}   最小直径 {dmin:.2f} mm")
    L = ask("轴长 mm（算扭转角，回车跳过）", 0)
    if L > 0:
        print(f"  扭转角：钢 {shaft_twist_deg(t, L, d, 79000):.2f}°，铝 {shaft_twist_deg(t, L, d, 26000):.2f}°（实心近似）")


def menu_battery():
    print("\n【电池 / 功率 / 续航】")
    cap = ask("电池容量 mAh", 5000)
    c = ask("放电倍率 C", 50)
    v = ask_voltage()
    cur = ask("平均电流 A", 20)
    print(f"\n  电池最大持续电流 = {max_battery_current(cap, c):.0f} A")
    print(f"  平均功率 = {electrical_power(v, cur):.0f} W")
    print(f"  续航估算（用 80% 容量）= {run_time_min(cap, cur):.1f} min")


def menu_chassis():
    print("\n【车身：轴荷 / 重心 / 侧翻】")
    f = ask("前轴称重 kg", 1.0)
    r = ask("后轴称重 kg", 1.1)
    wb = ask("轴距 mm", 260)
    tw = ask("轮距 mm", 190)
    h = ask("重心高度 mm", 40)
    for k, val in weight_distribution(f, r).items():
        print(f"  {k} = {val:.2f}")
    print(f"  重心距前轴 = {cg_from_axle_loads(f, r, wb):.1f} mm")
    print(f"  静态侧翻临界横向加速度 ≈ {rollover_lateral_g(tw, h):.2f} g"
          "（大于轮胎 μ 则先侧滑而不翻）")


MENU = [
    ("齿比 / 总减速比 / Rollout", menu_ratio),
    ("理论车速（KV、电压、齿比、轮径）", menu_speed),
    ("反推：目标车速 → 减速比 / 马达齿", menu_target),
    ("扭矩 / 推力 / 加速度", menu_torque),
    ("齿轮几何 / 中心距（模数、DP 换算）", menu_gear_geom),
    ("固定中心距下的齿数组合", menu_combos),
    ("齿轮弯曲强度校核（Lewis）", menu_gear_strength),
    ("轴强度 / 最小轴径 / 扭转角", menu_shaft),
    ("电池 / 功率 / 续航", menu_battery),
    ("车身：轴荷 / 重心 / 侧翻", menu_chassis),
]


def demo():
    """示例：1/10 短卡，2S，3300KV，18T/72T，差速器 2.6，轮径 108mm"""
    print("=== 示例计算（理论值）===")
    fdr = final_drive_ratio(18, 72, 2.6)
    v = battery_voltage(2)
    print(f"FDR = {fdr:.3f}")
    print(f"极速(空载) = {top_speed(3300, v, fdr, 108):.1f} km/h, "
          f"带载0.85 = {top_speed(3300, v, fdr, 108, 0.85):.1f} km/h")
    print(f"达到 60km/h 需 FDR = {fdr_for_target_speed(60, 3300, v, 108, 0.85):.3f}")
    tm = motor_torque(3300, 60, 2)
    tw = wheel_torque_total(tm, fdr)
    print(f"电机扭矩 {tm:.3f} N·m，轮合计扭矩 {tw:.3f} N·m，推力 {tractive_force(tw, 108):.1f} N")
    print(f"m1 13T 几何: {gear_geometry(1.0, 13)}")
    print(f"m1 13/37 中心距 = {center_distance(1.0, 13, 37)} mm")
    print(f"m1 13T 6mm宽 承受0.5N·m 弯曲应力 = {lewis_bending_stress(0.5, 1, 13, 6, 2):.1f} MPa")
    print(f"5mm实心轴 1N·m 剪应力 = {shaft_torsion_stress(1.0, 5):.1f} MPa")
    print(f"45#钢 1N·m SF2 最小轴径 = {shaft_min_diameter(1.0, 355, 2):.2f} mm")
    print(f"0.6模数 中心距26mm 组合: {pinion_spur_combos(0.6, 26, (12, 30), (50, 100), 0.1)[:3]}")
    print(f"5000mAh 20A 续航 = {run_time_min(5000, 20):.1f} min")


def main():
    if "--demo" in sys.argv:
        demo()
        return
    print("=" * 46)
    print("   RC 遥控车研发计算工具   (结果为理论/估算值)")
    print("=" * 46)
    while True:
        print()
        for i, (name, _) in enumerate(MENU, 1):
            print(f"  {i:>2}. {name}")
        print("   0. 退出")
        c = input("\n选择功能编号: ").strip()
        if c in ("0", "q", "exit"):
            break
        if c.isdigit() and 1 <= int(c) <= len(MENU):
            try:
                MENU[int(c) - 1][1]()
            except (ZeroDivisionError, ValueError) as e:
                print(f"  计算出错（检查是否有 0 或负数输入）: {e}")
            input("\n回车返回菜单...")
        else:
            print("  无效编号")


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print("\n已退出")
