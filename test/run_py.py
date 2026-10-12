# -*- coding: utf-8 -*-
"""run_py.py — 黑盒一致性测试的 Python 侧驱动（喂同一份 CSV，输出同格式结果）。"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from ekf7 import ImuPipeline

in_path, out_path = sys.argv[1], sys.argv[2]
amode = int(sys.argv[3])
fs = float(sys.argv[4])

data = np.loadtxt(in_path, delimiter=",", skiprows=1)
pipe = ImuPipeline(fs=fs, work_mode=1, amode=amode)

rows = ["k,q0,q1,q2,q3,roll,pitch,yaw,vx,vy,vz,px,py,pz,flag"]
for k in range(data.shape[0]):
    t, ax, ay, az, gx, gy, gz, mx, my, mz = data[k]
    out = pipe.calculate(np.array([ax, ay, az]), np.array([gx, gy, gz]),
                         np.array([mx, my, mz]))
    q, e, v, p = out["quat"], out["euler_rad"], out["vel"], out["pos"]
    fmt = "%d," + ",".join(["%.17g"] * 13) + ",%d"
    rows.append(fmt % (k, q[0], q[1], q[2], q[3], e[0], e[1], e[2],
                       v[0], v[1], v[2], p[0], p[1], p[2], out["motionless"]))

with open(out_path, "w") as fh:
    fh.write("\n".join(rows) + "\n")
print(f"Python(本仓库): {data.shape[0]} 拍 -> {out_path} (amode={amode}, fs={fs})")
