# -*- coding: utf-8 -*-
from __future__ import annotations
"""静止自标定：安装矩阵（加计调平）与陀螺零偏的自动估计。

上电前 10 秒内，检测到持续静止后用窗口均值估计二者，完成后由调用方重启滤波器。
"""

import numpy as np

from .alignment import align_tilt
from .parameters import Parameters
from .quaternion import quat_to_dcm


# 静止自标定（ImuAutoCalibrator：安装矩阵 + 陀螺零偏）
# ---------------------------------------------------------------------------


class AutoCalState:
    def __init__(self) -> None:
        # 同上：C# 默认全 0
        self.imu_sum_time_abs = 0.0
        self.acc_raw_max_abs = np.zeros(3)
        self.acc_raw_min_abs = np.zeros(3)
        self.gyro_raw_max_abs = np.zeros(3)
        self.gyro_raw_min_abs = np.zeros(3)
        self.motionless_cnt = 0.0
        self.acc_in_sum1 = np.zeros(3)
        self.gyro_in_sum1 = np.zeros(3)
        self.imu_in_cnt1 = 0.0
        self.run_flag = 0.0


class AutoCalResult:
    def __init__(self) -> None:
        self.gyro_bias = np.zeros(3)
        self.imu_r = np.eye(3)
        self.completed = False


def imu_auto_calibrator_run(acc_in, gyro_in, state: AutoCalState, dt, param: Parameters) -> AutoCalResult:
    result = AutoCalResult()
    n = int(param.fs * param.ipack_mean)
    state.imu_sum_time_abs += dt
    still = 0
    for i in range(3):
        state.acc_raw_max_abs[i] = max(state.acc_raw_max_abs[i], acc_in[i])
        state.acc_raw_min_abs[i] = min(state.acc_raw_min_abs[i], acc_in[i])
        state.gyro_raw_max_abs[i] = max(state.gyro_raw_max_abs[i], gyro_in[i])
        state.gyro_raw_min_abs[i] = min(state.gyro_raw_min_abs[i], gyro_in[i])
    if not (
        abs(state.gyro_raw_max_abs[0] - state.gyro_raw_min_abs[0]) < param.auto_cal_gyro_max_min
        and abs(state.gyro_raw_max_abs[1] - state.gyro_raw_min_abs[1]) < param.auto_cal_gyro_max_min
        and abs(state.gyro_raw_max_abs[2] - state.gyro_raw_min_abs[2]) < param.auto_cal_gyro_max_min
        and abs(state.acc_raw_max_abs[0] - state.acc_raw_min_abs[0]) < param.auto_cal_acc_max_min * 2.0
        and abs(state.acc_raw_max_abs[1] - state.acc_raw_min_abs[1]) < param.auto_cal_acc_max_min * 2.0
        and abs(state.acc_raw_max_abs[2] - state.acc_raw_min_abs[2]) < param.auto_cal_acc_max_min * 2.0
    ):
        state.imu_sum_time_abs = 0.0
        state.acc_raw_max_abs = acc_in.copy()
        state.acc_raw_min_abs = acc_in.copy()
        state.gyro_raw_max_abs = gyro_in.copy()
        state.gyro_raw_min_abs = gyro_in.copy()
    if state.imu_sum_time_abs >= 2.0:
        state.imu_sum_time_abs = 2.0
        still = 1
    if still == 1:
        state.motionless_cnt += 1.0
        n3 = float(n) * 0.5
        if state.motionless_cnt < n3 * 1.0:
            state.acc_in_sum1 = np.zeros(3)
            state.gyro_in_sum1 = np.zeros(3)
            state.imu_in_cnt1 = 0.0
        elif state.motionless_cnt < n3 * 2.0:
            state.acc_in_sum1 = state.acc_in_sum1 + acc_in
            state.gyro_in_sum1 = state.gyro_in_sum1 + gyro_in
            state.imu_in_cnt1 += 1.0
        elif not (state.motionless_cnt < n3 * 3.0):
            result.gyro_bias = state.gyro_in_sum1 / state.imu_in_cnt1
            init_accel = state.acc_in_sum1 / state.imu_in_cnt1
            q = align_tilt(np.array([1.0, 0.0, 0.0, 0.0]), init_accel)
            result.imu_r = quat_to_dcm(q)
            state.motionless_cnt = 0.0
            state.run_flag = 1.0
            result.completed = True
    else:
        state.motionless_cnt = 0.0
    return result


# ---------------------------------------------------------------------------
