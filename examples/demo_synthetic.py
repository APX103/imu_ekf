# -*- coding: utf-8 -*-
"""
demo_synthetic.py — 用合成 IMU 数据验证 ekf7 包的正确性
================================================================
场景：200Hz，先静置 8s（带陀螺零偏），然后绕垂直轴匀速旋转 3 段
（0°→150°→150°→0°），整机带 5°/-3° 初始横滚俯仰。
对比滤波器输出姿态与真值四元数的误差。

运行：python3 demo_synthetic.py
"""
import math

import numpy as np

from ekf7 import (
    Parameters,
    ImuPipeline,
    quat_from_euler,
    quat_from_rotation_vector,
    quat_multiply,
    quat_normalized,
    quat_to_dcm,
    quat_to_euler,
    wrap_to_pi,
)

FS = 200.0
DT = 1.0 / FS
T_TOTAL = 24.0
N = int(T_TOTAL * FS)
G0 = 9.795502520545487

rng = np.random.default_rng(7)

GYRO_BIAS = np.array([0.002, -0.001, 0.0015])  # rad/s 恒定零偏
ACC_NOISE = 0.02   # m/s^2
GYRO_NOISE = 0.0008  # rad/s
MAG_NOISE = 0.01

MAG_NAV = np.array([0.0, 1.0, -0.4])  # ENU：指北 + 向下分量（磁倾角）


def yaw_rate(t: float) -> float:
    """往复偏航运动（符合该算法的设计场景；见 README 关于匀速旋转的坑）。"""
    if t < 8.0:
        return 0.0
    amp_deg = 60.0        # 偏航幅度 ±60°
    period = 10.0         # 周期 10s
    return math.radians(amp_deg * 2.0 * math.pi / period * math.cos(2.0 * math.pi * (t - 8.0) / period))


def simulate():
    q = quat_from_euler(math.radians(5.0), math.radians(-3.0), 0.0)
    accs, gyros, mags, qs_true = [], [], [], []
    for k in range(N):
        t = k * DT
        w_nav = np.array([0.0, 0.0, yaw_rate(t)])
        # 关键：与滤波器传播一致，真值用体坐标系角速度做右乘递推
        w_body = quat_to_dcm(q).T @ w_nav
        q = quat_normalized(quat_multiply(q, quat_from_rotation_vector(w_body * DT)))
        r = quat_to_dcm(q)
        acc_body = r.T @ np.array([0.0, 0.0, G0])
        mag_body = r.T @ MAG_NAV
        accs.append(acc_body + rng.normal(0.0, ACC_NOISE, 3))
        gyros.append(w_body + GYRO_BIAS + rng.normal(0.0, GYRO_NOISE, 3))
        mags.append(mag_body + rng.normal(0.0, MAG_NOISE, 3))
        qs_true.append(q.copy())
    return np.array(accs), np.array(gyros), np.array(mags), np.array(qs_true)


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def dcm_to_quat(r: np.ndarray) -> np.ndarray:
    """旋转矩阵 -> 四元数（wxyz），用于把安装矩阵换回四元数做合成。"""
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return quat_normalized(np.array([w, x, y, z]))


def angle_error_deg(q_est, q_true) -> float:
    qe = quat_multiply(quat_conj(q_true), q_est)
    return math.degrees(2.0 * math.atan2(float(np.linalg.norm(qe[1:])), abs(qe[0])))


def run(name: str, amode: int, use_mag: bool):
    accs, gyros, mags, qs_true = simulate()
    pipe = ImuPipeline(fs=FS, work_mode=1, amode=amode)
    errs, errs_rpy = [], []
    for k in range(N):
        out = pipe.calculate(accs[k], gyros[k], mags[k] if use_mag else None)
        # 安装矩阵在运行中可能被自标定更新，逐拍合成回绝对姿态
        q_install = dcm_to_quat(pipe.rotm_install)
        q_abs = quat_normalized(quat_multiply(out["quat"], q_install))
        errs.append(angle_error_deg(q_abs, qs_true[k]))
        e_est = quat_to_euler(q_abs)
        e_true = quat_to_euler(qs_true[k])
        errs_rpy.append(np.degrees(e_est - e_true))
    errs = np.array(errs)
    errs_rpy = np.array(errs_rpy)
    yaw_err = np.array([wrap_to_pi(math.radians(e)) * 57.29577951308232 for e in errs_rpy[:, 2]])
    after_init = slice(int(8.0 * FS), N)  # 初始化完成之后
    print(f"--- {name} (amode={amode}, mag={'on' if use_mag else 'off'}) ---")
    print(f"  姿态总角误差  稳态最大 {errs[after_init].max():6.3f}°   末端 {errs[-1]:6.3f}°")
    print(
        f"  roll 误差稳态最大 {np.abs(errs_rpy[after_init, 0]).max():6.3f}°   "
        f"pitch {np.abs(errs_rpy[after_init, 1]).max():6.3f}°   "
        f"yaw {np.abs(yaw_err[after_init]).max():6.3f}°"
    )
    print(f"  滤波器内部陀螺零偏估计: {np.degrees(pipe.filter.gyro_in_bias)} deg/s")
    print(f"  管道级陀螺零偏(自标定): {np.degrees(pipe.gyro_bias)} deg/s")
    print(f"  末端速度 |v| = {np.linalg.norm(out['vel']):.4f} m/s")
    return errs


if __name__ == "__main__":
    run("九轴模式", amode=9, use_mag=True)
    run("六轴模式", amode=6, use_mag=False)
