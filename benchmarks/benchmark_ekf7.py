# -*- coding: utf-8 -*-
"""ekf7 Python 单帧处理延时基准测试。

运行：
    python3 benchmarks/benchmark_ekf7.py

默认测 amode=6 与 amode=9 两种模式，输出单帧耗时、等效帧率、
200Hz 下的 CPU 占用估算。
"""
from __future__ import annotations

import platform
import sys
import time
from pathlib import Path

import numpy as np

# 允许从仓库根目录导入 ekf7
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from ekf7 import ImuPipeline


FS = 200.0
WARMUP = 100
N = 20000

ACC = np.array([0.1, -0.2, 9.79])
GYRO = np.array([0.01, -0.015, 0.005])
MAG = np.array([0.15, 0.45, -0.35])


def run_once(amode: int) -> dict:
    pipe = ImuPipeline(fs=FS, work_mode=1, amode=amode)
    mag = MAG if amode == 9 else None

    for _ in range(WARMUP):
        pipe.calculate(ACC, GYRO, mag)

    t0 = time.perf_counter()
    for _ in range(N):
        pipe.calculate(ACC, GYRO, mag)
    elapsed = time.perf_counter() - t0

    return {
        "amode": amode,
        "frames": N,
        "elapsed_s": elapsed,
        "per_frame_us": elapsed / N * 1e6,
        "fps": N / elapsed,
        "cpu_200hz_pct": (elapsed / (N / FS)) * 100,
    }


def main() -> None:
    print(f"Platform: {platform.platform()}")
    print(f"Python: {sys.version.split()[0]}")
    print(f"NumPy: {np.__version__}")
    print(f"Benchmark: {N} frames after {WARMUP} warmup, fs={FS}Hz\n")

    for amode in (6, 9):
        r = run_once(amode)
        print(f"amode={r['amode']}:")
        print(f"  elapsed      : {r['elapsed_s']:.3f} s")
        print(f"  per frame    : {r['per_frame_us']:.1f} us")
        print(f"  fps          : {r['fps']:.0f}")
        print(f"  CPU @ 200Hz  : {r['cpu_200hz_pct']:.2f}%")
        print()


if __name__ == "__main__":
    main()
