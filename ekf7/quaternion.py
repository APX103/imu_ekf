# -*- coding: utf-8 -*-
from __future__ import annotations
"""四元数工具（w-x-y-z 顺序）：乘法、归一化、DCM、旋转向量/欧拉角互转。

小角度下旋转向量转四元数使用泰勒级数展开（避免除零）。
"""

import math

import numpy as np


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w, x, y, z = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w * w2 - x * x2 - y * y2 - z * z2,
            w * x2 + x * w2 + y * z2 - z * y2,
            w * y2 - x * z2 + y * w2 + z * x2,
            w * z2 + x * y2 - y * x2 + z * w2,
        ]
    )


def quat_normalized(q: np.ndarray) -> np.ndarray:
    n = math.sqrt(float(np.dot(q, q)))
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    """C# ToDirectionCosine：返回体->导航旋转矩阵（BodyToNav）。"""
    w, x, y, z = q
    w2, x2, y2, z2 = w * w, x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [w2 + x2 - y2 - z2, 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), w2 - x2 + y2 - z2, 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), w2 - x2 - y2 + z2],
        ]
    )


def quat_from_rotation_vector(rv: np.ndarray) -> np.ndarray:
    """旋转向量 -> 四元数，小角度用泰勒级数（与 C# 一致）。"""
    n2 = float(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2])
    if n2 < 1e-8:
        w = 1.0 - n2 * (0.125 - n2 / 384.0)
        s = 0.5 - n2 * (1.0 / 48.0 - n2 / 3840.0)
    else:
        n = math.sqrt(n2)
        half = n * 0.5
        w = math.cos(half)
        s = math.sin(half) / n
    return np.array([w, s * rv[0], s * rv[1], s * rv[2]])


def quat_from_euler(phi: float, theta: float, psi: float) -> np.ndarray:
    cr, sr = math.cos(phi * 0.5), math.sin(phi * 0.5)
    cp, sp = math.cos(theta * 0.5), math.sin(theta * 0.5)
    cy, sy = math.cos(psi * 0.5), math.sin(psi * 0.5)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """返回 [roll(phi), pitch(theta), yaw(psi)]（rad）。注意 pitch 取负号，与 C# 一致。"""
    w, x, y, z = q
    num = 2.0 * (x * z - w * y)
    num = max(-1.0, min(1.0, num))
    return np.array(
        [
            math.atan2(2.0 * (y * z + w * x), w * w - x * x - y * y + z * z),
            -math.asin(num),
            math.atan2(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z),
        ]
    )


