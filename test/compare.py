# -*- coding: utf-8 -*-
"""compare.py — 逐拍比对两份实现的输出（CSV 格式相同）。"""
import sys

import numpy as np

cs = np.loadtxt(sys.argv[1], delimiter=",", skiprows=1)
py = np.loadtxt(sys.argv[2], delimiter=",", skiprows=1)
assert cs.shape == py.shape, f"形状不一致: {cs.shape} vs {py.shape}"

names = ["k", "q0", "q1", "q2", "q3", "roll", "pitch", "yaw",
         "vx", "vy", "vz", "px", "py", "pz", "flag"]
flag_match = np.array_equal(cs[:, 14], py[:, 14])

print(f"{'列':>6} {'最大绝对差':>14} {'最大相对差':>14}  首个>|1e-9|的拍")
worst = 0.0
for i in range(1, 14):
    d = np.abs(cs[:, i] - py[:, i])
    denom = np.maximum(np.abs(cs[:, i]), 1e-300)
    rel = d / denom
    idx = np.argmax(d)
    first = np.argmax(d > 1e-9) if (d > 1e-9).any() else -1
    worst = max(worst, d.max())
    print(f"{names[i]:>6} {d.max():14.3e} {rel.max():14.3e}  "
          f"{'k='+str(first) if first >= 0 else '无'}  (最大 @k={idx})")
print(f"motionlessFlag 全序列一致: {flag_match}")
tol = 1e-6
print(f"\n判定: 最大绝对差 {worst:.3e} → {'✅ 一致（< 1e-6，浮点库差异量级）' if worst < tol and flag_match else '❌ 超差，需排查'}")
