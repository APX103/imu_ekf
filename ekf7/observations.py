# -*- coding: utf-8 -*-
from __future__ import annotations
"""观测更新：ZUPT 零速观测（四档自适应 R/门限）与磁航向一维观测。

两者均带卡方式门控：任一轴/本拍超门限则整体拒绝本次观测。
"""

import math

import numpy as np

from .jacobian import calc_h_hdgc
from .quaternion import quat_normalized, quat_to_dcm


def velocity_observation_fuse(state_q, state_v, p, obs_var, gate_size):
    """ZUPT：观测值恒为 0 速度。H = [0_3x4, I_3]。任一轴超门限则整体拒绝。"""
    h = np.zeros((3, 7))
    h[0, 4] = h[1, 5] = h[2, 6] = 1.0
    ph = h @ p
    s = ph @ h.T + np.diag([obs_var] * 3)
    innov = state_v.copy()
    for i in range(3):
        if state_v[i] * state_v[i] / (gate_size * gate_size * s[i, i]) > 1.0:
            return state_q, state_v, p, innov, s  # 拒绝本次观测
    k = p @ h.T @ np.linalg.inv(s)
    x = np.concatenate([state_q, state_v])
    x = x - k @ innov
    q = quat_normalized(x[:4])
    v = x[4:]
    p_new = p - k @ ph
    p_new = 0.5 * (p_new + p_new.T)
    for i in range(7):
        if p_new[i, i] < 0.0:
            p_new[i, i] = 0.0
    return q, v, p_new, innov, s


def mag_heading_observation_fuse(state_q, state_v, p, mag_data, meas_dec, innov_gate, r_mag):
    """磁航向一维观测：innov = atan2(mN_x, mN_y) - dec，H 用解析雅可比。"""
    mag_nav = quat_to_dcm(state_q) @ mag_data
    nu = math.atan2(mag_nav[0], mag_nav[1]) - meas_dec
    if nu > math.pi:
        nu -= 2.0 * math.pi
    elif nu < -math.pi:
        nu += 2.0 * math.pi
    h_vec = calc_h_hdgc(mag_data[0], mag_data[1], mag_data[2], *state_q)
    ph = h_vec @ p
    s = float(ph @ h_vec) + r_mag
    if nu * nu / (innov_gate * innov_gate * s) > 1.0:
        return state_q, state_v, p, nu  # 拒绝
    k = (p @ h_vec) / s
    x = np.concatenate([state_q, state_v]) - k * nu
    q = quat_normalized(x[:4])
    v = x[4:]
    p_new = p - np.outer(k, ph)
    p_new = 0.5 * (p_new + p_new.T)
    for i in range(7):
        if p_new[i, i] < 0.0:
            p_new[i, i] = 0.0
    return q, v, p_new, nu


