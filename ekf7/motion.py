# -*- coding: utf-8 -*-
from __future__ import annotations
"""三级静止检测器：0=运动 1=低角速率 2=仅偏航静止 4=完全静止。

检测方式为窗口峰峰值 + 阈值突破时锚点重置（注意其对匀速旋转的误判局限，见 README）。
"""

import numpy as np

from .parameters import Parameters


class MotionState:
    def __init__(self) -> None:
        # C# 字段默认值全为 0.0（double 默认），不是 ±inf —— 黑盒一致性要求逐位一致
        self.imu_sum_time_abs = 0.0
        self.acc_raw_max_abs = np.zeros(3)
        self.acc_raw_min_abs = np.zeros(3)
        self.gyro_raw_max_abs = np.zeros(3)
        self.gyro_raw_min_abs = np.zeros(3)
        self.yaw_sum_time = 0.0
        self.yaw_max = 0.0
        self.yaw_min = 0.0
        self.gyro_ml_sum_time = 0.0
        self.motionless_flag_out = 0


def motion_detector_detect(acc_in, gyro_in, ml: MotionState, param: Parameters, dt) -> int:
    ml.imu_sum_time_abs += dt
    still = 0
    for i in range(3):
        ml.acc_raw_max_abs[i] = max(ml.acc_raw_max_abs[i], acc_in[i])
        ml.acc_raw_min_abs[i] = min(ml.acc_raw_min_abs[i], acc_in[i])
        ml.gyro_raw_max_abs[i] = max(ml.gyro_raw_max_abs[i], gyro_in[i])
        ml.gyro_raw_min_abs[i] = min(ml.gyro_raw_min_abs[i], gyro_in[i])
    if not (
        abs(ml.gyro_raw_max_abs[0] - ml.gyro_raw_min_abs[0]) < 0.010471204188481676
        and abs(ml.gyro_raw_max_abs[1] - ml.gyro_raw_min_abs[1]) < 0.010471204188481676
        and abs(ml.gyro_raw_max_abs[2] - ml.gyro_raw_min_abs[2]) < 0.010471204188481676
        and abs(ml.acc_raw_max_abs[0] - ml.acc_raw_min_abs[0]) < 0.2
        and abs(ml.acc_raw_max_abs[1] - ml.acc_raw_min_abs[1]) < 0.2
        and abs(ml.acc_raw_max_abs[2] - ml.acc_raw_min_abs[2]) < 0.2
    ):
        ml.imu_sum_time_abs = 0.0
        ml.acc_raw_max_abs = acc_in.copy()
        ml.acc_raw_min_abs = acc_in.copy()
        ml.gyro_raw_max_abs = gyro_in.copy()
        ml.gyro_raw_min_abs = gyro_in.copy()
    if ml.imu_sum_time_abs >= 2.0:
        ml.imu_sum_time_abs = 2.0
        still = 1
    ml.yaw_sum_time += dt
    yaw_still = 0
    ml.yaw_max = max(ml.yaw_max, gyro_in[2])
    ml.yaw_min = min(ml.yaw_min, gyro_in[2])
    if not (abs(ml.yaw_max - ml.yaw_min) < 5.0 / 573.0):
        ml.yaw_sum_time = 0.0
        ml.yaw_max = gyro_in[2]
        ml.yaw_min = gyro_in[2]
    if ml.yaw_sum_time >= 1.0:
        ml.yaw_sum_time = 1.0
        yaw_still = 1
    gyro_norm = float(np.linalg.norm(gyro_in))
    ml.gyro_ml_sum_time += dt
    gyro_still = 0
    if param.work_mode == 1:
        gyro_thresh, time_thresh = 50.0 / 573.0, 0.5
    else:
        gyro_thresh, time_thresh = 0.02617801047120419, 0.2
    if gyro_norm > gyro_thresh:
        ml.gyro_ml_sum_time = 0.0
    if ml.gyro_ml_sum_time >= time_thresh:
        ml.gyro_ml_sum_time = time_thresh
        gyro_still = 1
    if still == 1:
        flag = 4
    elif yaw_still == 1:
        flag = 2
    elif gyro_still == 1:
        flag = 1
    else:
        flag = 0
    ml.motionless_flag_out = flag
    return flag


# ---------------------------------------------------------------------------
