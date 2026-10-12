# -*- coding: utf-8 -*-
"""
gen_data.py — 黑盒一致性测试的合成 IMU 数据生成器
=================================================
生成与真值姿态自洽的 acc/gyro/mag 序列（体坐标系），供
  · 对端实现（搭建说明见 reference_harness/）
  · run_py.py（本仓库 Python 实现）
两边喂同一份数据做逐拍输出比对。

场景（200 Hz）：
  [0, 3.0s)   静置（初始横滚/俯仰 5°/-3°，陀螺带恒定零偏）
  [3.0, 3.3)  偏航角速度 0 → 0.8 rad/s 渐升
  [3.3, 4.0)  恒速 0.8 rad/s
  [4.0, 4.3)  渐降到 0
  [4.3, 5.0s] 再次静置

输出 CSV（%.17g 往返精度）：t,ax,ay,az,gx,gy,gz,mx,my,mz
"""
import math

import numpy as np

FS = 200.0
DT = 1.0 / FS
G0 = 9.795502520545487
GYRO_BIAS = np.array([0.002, -0.001, 0.0015])
MAG_NAV = np.array([0.0, 1.0, -0.4])
rng = np.random.default_rng(42)


def quat_mul(a, b):
    w, x, y, z = a
    w2, x2, y2, z2 = b
    return np.array([
        w * w2 - x * x2 - y * y2 - z * z2,
        w * x2 + x * w2 + y * z2 - z * y2,
        w * y2 - x * z2 + y * w2 + z * x2,
        w * z2 + x * y2 - y * x2 + z * w2,
    ])


def quat_norm(q):
    return q / math.sqrt(float(q @ q))


def rv_to_quat(rv):
    n2 = float(rv @ rv)
    if n2 < 1e-8:
        w = 1.0 - n2 * (0.125 - n2 / 384.0)
        s = 0.5 - n2 * (1.0 / 48.0 - n2 / 3840.0)
    else:
        n = math.sqrt(n2)
        w = math.cos(n * 0.5)
        s = math.sin(n * 0.5) / n
    return np.array([w, s * rv[0], s * rv[1], s * rv[2]])


def quat_dcm(q):
    w, x, y, z = q
    return np.array([
        [w*w+x*x-y*y-z*z, 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), w*w-x*x+y*y-z*z, 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), w*w-x*x-y*y+z*z],
    ])


def euler_to_quat(phi, theta, psi):
    cr, sr = math.cos(phi*.5), math.sin(phi*.5)
    cp, sp = math.cos(theta*.5), math.sin(theta*.5)
    cy, sy = math.cos(psi*.5), math.sin(psi*.5)
    return np.array([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                     cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])


def yaw_rate(t):
    if t < 3.0:
        return 0.0
    if t < 3.3:
        return 0.8 * (t - 3.0) / 0.3
    if t < 4.0:
        return 0.8
    if t < 4.3:
        return 0.8 * (1.0 - (t - 4.0) / 0.3)
    return 0.0


def gen(n_samples: int, path: str):
    q = euler_to_quat(math.radians(5.0), math.radians(-3.0), 0.0)
    rows = []
    for k in range(n_samples):
        t = k * DT
        w_nav = np.array([0.0, 0.0, yaw_rate(t)])
        w_body = quat_dcm(q).T @ w_nav
        q = quat_norm(quat_mul(q, rv_to_quat(w_body * DT)))
        r = quat_dcm(q)
        acc = r.T @ np.array([0.0, 0.0, G0]) + rng.normal(0, 0.01, 3)
        gyro = w_body + GYRO_BIAS + rng.normal(0, 0.0005, 3)
        mag = r.T @ MAG_NAV + rng.normal(0, 0.005, 3)
        rows.append([t, *acc, *gyro, *mag])
    arr = np.array(rows)
    head = "t,ax,ay,az,gx,gy,gz,mx,my,mz"
    np.savetxt(path, arr, delimiter=",", header=head, comments="",
               fmt="%.17g")
    print(f"{path}: {n_samples} 行 ({n_samples/FS:.2f}s @ {FS:.0f}Hz)")


if __name__ == "__main__":
    gen(1000, "data_1000.csv")
    gen(3000, "data_3000.csv")
