# -*- coding: utf-8 -*-
from __future__ import annotations
"""角度包裹、叉积、标量一阶低通等基础工具。"""

import math

import numpy as np


def wrap_to_pi(angle: float) -> float:
    angle = math.fmod(angle, 2.0 * math.pi)
    if angle > math.pi:
        angle -= 2.0 * math.pi
    elif angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def cross3(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.array(
        [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
    )


def kfilter_inplace(prev: np.ndarray, x: np.ndarray, k: float) -> None:
    """C# MathUtil.KFilterInPlace：标量一阶低通（他们称之为 K 滤波）。"""
    prev *= (1.0 - k)
    prev += x * k


