# -*- coding: utf-8 -*-
from __future__ import annotations
"""端到端管线：输入一阶低通 → 前 10s 静止自标定 → 静止检测 → 滤波器步进。

对外的最外层封装，每拍喂 acc/gyro/mag 返回姿态/速度/位移/静止标志。
"""

import numpy as np

from .auto_calibration import AutoCalState, imu_auto_calibrator_run
from .math_utils import kfilter_inplace
from .motion import MotionState, motion_detector_detect
from .navigation_filter import NavigationFilter
from .parameters import Parameters


# 端到端管线（低通 → 自标定 → 检测 → 滤波）
# ---------------------------------------------------------------------------


class ImuPipeline:
    """每拍处理流程：输入一阶低通 -> (前 10s) 静止自标定安装矩阵/陀螺零偏 -> 静止检测 -> EKF 步进。"""

    def __init__(self, fs: float = 200.0, work_mode: int = 1, amode: int = 9, param: Parameters | None = None):
        self.param = param or Parameters()
        self.param.fs = fs
        self.param.work_mode = work_mode
        self.param.amode = amode
        self.dt = 1.0 / fs
        self.filter = NavigationFilter(self.param)
        self.ml_state = MotionState()
        self.cal_state = AutoCalState()
        self.acc_filt = np.zeros(3)
        self.gyro_filt = np.zeros(3)
        self.filt_init = False
        self.rotm_install = np.eye(3)
        self.gyro_bias = np.zeros(3)
        self.enable_auto_cal = True
        self.icount = 0

    def calculate(self, acc_raw, gyro_raw, mag_raw=None):
        acc = np.asarray(acc_raw, dtype=float).copy()
        gyro = np.asarray(gyro_raw, dtype=float).copy()
        mag = np.asarray(mag_raw, dtype=float).copy() if mag_raw is not None else np.zeros(3)

        if not self.filt_init:
            self.acc_filt = acc.copy()
            self.gyro_filt = gyro.copy()
            self.filt_init = True
        else:
            kfilter_inplace(self.acc_filt, acc, self.param.acc_filt_k)
            kfilter_inplace(self.gyro_filt, gyro, self.param.gyro_filt_k)

        gyro_corr = self.gyro_filt - self.gyro_bias
        acc_in = self.rotm_install @ self.acc_filt
        gyro_in = self.rotm_install @ gyro_corr

        if self.icount < int(self.param.fs) * 10:
            self.icount += 1
        if self.enable_auto_cal and self.icount < int(self.param.fs) * 10:
            res = imu_auto_calibrator_run(self.acc_filt, self.gyro_filt, self.cal_state, self.dt, self.param)
            if res.completed:
                self.rotm_install = res.imu_r
                self.gyro_bias = res.gyro_bias
                self.enable_auto_cal = False
                self.filter.reset()

        motionless_flag = motion_detector_detect(acc_in, gyro_in, self.ml_state, self.param, self.dt)
        self.filter.step(acc_in, gyro_in, mag, self.dt, motionless_flag)
        return {
            "euler_rad": self.filter.get_euler_angles_rad(),
            "quat": self.filter.get_attitude(),
            "vel": self.filter.get_velocity(),
            "pos": self.filter.get_position(),
            "motionless": motionless_flag,
        }
