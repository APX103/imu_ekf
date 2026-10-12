# -*- coding: utf-8 -*-
from __future__ import annotations
"""初始对准与状态初始化：加计调平、磁航向粗对准、初始状态/协方差。
"""

import math

import numpy as np

from .math_utils import cross3
from .parameters import Parameters
from .quaternion import quat_from_euler, quat_to_dcm, quat_to_euler


def align_tilt(q: np.ndarray, init_accel: np.ndarray) -> np.ndarray:
    """用平均比力方向把 z 轴对到重力反方向（调平）。"""
    ax, ay, az = init_accel
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if 5.0 < norm < 14.0:
        horiz = math.sqrt(ax * ax + ay * ay)
        tilt = math.atan2(horiz, az)
        if tilt > 0.001:
            axis = cross3(init_accel, np.array([0.0, 0.0, 1.0]))
            axis_norm = float(np.linalg.norm(axis))
            if axis_norm > 1e-12:
                axis = axis / axis_norm
                s = math.sin(0.5 * tilt)
                return np.array([math.cos(0.5 * tilt), axis[0] * s, axis[1] * s, axis[2] * s])
    return q


def align_heading(q: np.ndarray, mag_meas: np.ndarray, declination: float) -> np.ndarray:
    """保留当前 roll/pitch，用磁向量解算并写入 yaw = dec + atan2(mN_x, mN_y)。"""
    e = quat_to_euler(q)
    e[2] = 0.0
    q_level = quat_from_euler(e[0], e[1], e[2])
    mag_nav = quat_to_dcm(q_level) @ mag_meas
    e[2] = declination + math.atan2(mag_nav[0], mag_nav[1])
    return quat_from_euler(e[0], e[1], e[2])


# ---------------------------------------------------------------------------
# 状态初始化（StateInitializer）
# ---------------------------------------------------------------------------


def init_states(acc_in, gyro_in, mag_in, gravity, dt, param: Parameters):
    q = np.array([1.0, 0.0, 0.0, 0.0])
    if all(abs(g) < 0.05 for g in gyro_in):
        q = align_tilt(q, acc_in)
    if float(np.linalg.norm(mag_in)) > 0.0:
        declination = param.mag_decl_deg * (math.pi / 180.0)
        q = align_heading(q, mag_in, declination)
    return q


def init_covariance(param: Parameters, dt, amode: int) -> np.ndarray:
    if amode != 6:
        quat_diag = param.quat_err * np.array([1.0, 1.0, 1.0, 1.0])
    else:
        quat_diag = param.quat_err * np.array([1.0, 1.0, 1.0, 0.0])  # 6 轴：q3(yaw) 误差置 0
    vel_diag = np.array([param.vel_err_ne, param.vel_err_ne, param.vel_err_d])
    d = np.concatenate([quat_diag * quat_diag, vel_diag * vel_diag])
    return np.diag(d)


