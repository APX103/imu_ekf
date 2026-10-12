# -*- coding: utf-8 -*-
from __future__ import annotations
"""滤波器全部可调参数（一个类），默认值为 200Hz 档整定。

各字段含义见 README 参数表。
"""

import math


class Parameters:
    def __init__(self) -> None:
        # WitParameters
        self.fs = 200.0            # 采样率 Hz
        self.ipack_mean = 1        # 打包均值（每包样本数）
        self.work_mode = 0         # 0/1/2：位置积分手感开关的档位
        self.gyro_filt_k = 0.1     # 陀螺输入低通系数
        self.acc_filt_k = 0.1      # 加速度计输入低通系数
        self.k_bias = 0.1
        self.amode = 6             # 6=六轴(无磁) 9=九轴(磁航向融合)
        self.g0 = 9.795502520545487  # 默认重力常量（按当地纬度修改）
        # AutoCalParameters
        self.auto_cal_gyro_max_min = math.pi / 900.0   # 静止判定陀螺峰峰值 0.2°/s
        self.auto_cal_acc_max_min = 0.05               # 静止判定加计峰峰值 m/s^2
        # FusionParameters
        self.body_vel_gate = 10.0   # ZUPT 门限（配合运动档位缩放）
        self.mag_fuse_method = 2    # 2=启用磁航向融合
        self.mag_time_delay = 0.0
        self.mag_hdg_error = 0.1745   # 磁航heading 测量噪声 0.1745 rad²（约 10°）
        self.mag_hdg_gate = 5.0       # 磁航向创新门限（卡方式）
        self.mag_decl_deg = 0.0       # 磁偏角（度）
        self.ang_rate_bias_k = 0.04   # 陀螺零偏平滑系数
        # PredictionParameters
        self.ang_rate_noise = 0.01    # 陀螺角度增量噪声 (rad)
        self.accel_noise = 0.35       # 加计速度增量噪声 (m/s)
        # AlignmentParameters
        self.vel_err_ne = 5.0
        self.vel_err_d = 1.0
        self.quat_err = 0.1

    @staticmethod
    def create_default() -> "Parameters":
        return Parameters()


