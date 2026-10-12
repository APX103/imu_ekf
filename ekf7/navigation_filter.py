# -*- coding: utf-8 -*-
from __future__ import annotations
"""主滤波器：初始化状态机 + 机械编排预测 + 4 拍轮转观测调度。

调用一次 step() 处理一拍 IMU（acc/gyro/mag, dt, motionless_flag）。
初始化时间轴：0~0.1s 强制单位四元数 → 0.1~3s 静置均值累计 →
3~4s 粗对准 → 之后进入正式滤波。
"""

import math

import numpy as np

from .alignment import align_heading, init_covariance, init_states
from .observations import mag_heading_observation_fuse, velocity_observation_fuse
from .parameters import Parameters
from .prediction import covariance_predict, state_predict
from .quaternion import quat_to_euler


# 主滤波循环（NavigationFilter）
# ---------------------------------------------------------------------------


class NavigationFilter:
    def __init__(self, param: Parameters | None = None) -> None:
        self.param = param or Parameters()
        self.reset()

    def reset(self) -> None:
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.v = np.zeros(3)
        self.p = np.zeros((7, 7))
        self.flag_in = 0.0
        self.init_max_time = 0.0
        self.gyro_old = np.zeros(3)
        self.acc_sum = np.zeros(3)
        self.gyro_sum = np.zeros(3)
        self.del_ang_cov = np.zeros(3)
        self.dt_cov = 0.0
        self.flag_div = 0.0
        self.motionless_cnt = 0.0
        self.mag_run_time = 0.0
        self.gyro0_init_flag = 0.0
        self.del_vel_cov = np.zeros(3)
        self.acc_in_k = 0.0
        self.dt_run_time = 0.0
        self.gyro_in_bias = np.zeros(3)
        self.gyro_in_sum1 = np.zeros(3)
        self.gyro_in_cnt1 = 0.0
        self.gyro_in_sum2 = np.zeros(3)
        self.gyro_in_cnt2 = 0.0
        self.mag_sum = np.zeros(3)
        self.del_ang_bias = np.zeros(3)
        self.position = np.zeros(3)
        self.mag_in_old = np.zeros(3)
        self.debug_var = np.zeros(20)

    # -- 输出接口 -----------------------------------------------------------

    def get_euler_angles_rad(self) -> np.ndarray:
        return quat_to_euler(self.q)

    def get_attitude(self) -> np.ndarray:
        return self.q.copy()

    def get_velocity(self) -> np.ndarray:
        return self.v.copy()

    def get_position(self) -> np.ndarray:
        return self.position.copy()

    # -- 主步进 -------------------------------------------------------------

    def step(self, acc_in, gyro_in, mag_in, dt_imu: float, motionless_flag: int) -> None:
        prm = self.param
        n = int(prm.fs * prm.ipack_mean)
        gyro_norm = float(np.linalg.norm(gyro_in))
        mag = self.mag_in_old if motionless_flag > 3 else mag_in.copy()
        if motionless_flag <= 3:
            self.mag_in_old = mag_in.copy()
        n3 = n * 3

        if self.init_max_time < 0.1:
            # 阶段 0：前 0.1s 强制单位四元数
            self.init_max_time += dt_imu
            self.q = np.array([1.0, self.q[1], self.q[2], self.q[3]])
        elif self.init_max_time < 3.0:
            # 阶段 1：静置均值累计（陀螺逐拍波动 <0.02 rad/s 才计数）
            self.init_max_time += dt_imu
            if abs(self.flag_in - 0.0) < 1e-7:
                self.acc_sum = acc_in.copy()
                self.gyro_sum = gyro_in.copy()
                self.mag_sum = mag.copy()
                self.flag_in += 1.0
            elif self.flag_in < float(n3):
                self.acc_sum = self.acc_sum + acc_in
                self.gyro_sum = self.gyro_sum + gyro_in
                self.mag_sum = self.mag_sum + mag
                if (
                    abs(gyro_in[0] - self.gyro_old[0]) < 0.02
                    and abs(gyro_in[1] - self.gyro_old[1]) < 0.02
                    and abs(gyro_in[2] - self.gyro_old[2]) < 0.02
                ):
                    self.flag_in += 1.0
                else:
                    self.flag_in = 0.0
            if self.flag_in >= float(n):
                self.init_max_time = 3.0
        elif self.init_max_time < 4.0:
            # 阶段 2：粗对准（调平 + 磁航向），估计初始陀螺零偏
            self.init_max_time = 4.0
            if self.flag_in >= float(n):
                acc_mean = self.acc_sum / self.flag_in
                gyro_mean = self.gyro_sum / self.flag_in
                mag_mean = self.mag_sum / self.flag_in
                self.acc_in_k = prm.g0 / float(np.linalg.norm(acc_mean))
                acc_mean = self.acc_in_k * acc_mean
            else:
                acc_mean = np.array([0.0, 0.0, prm.g0])
                gyro_mean = np.array([1.0, 1.0, 1.0])
                mag_mean = mag.copy()
                self.acc_in_k = 1.0
            self.del_ang_cov = np.zeros(3)
            self.dt_cov = 0.0
            self.flag_div = 0.0
            self.q = init_states(acc_mean, gyro_mean, mag_mean, prm.g0, dt_imu, prm)
            self.p = init_covariance(prm, dt_imu, prm.amode)
            if all(abs(g) < 0.05 for g in gyro_mean):
                self.gyro_in_bias = gyro_mean.copy()
            else:
                gyro_mean = np.zeros(3)
                self.gyro_in_bias = gyro_mean.copy()
            self.del_ang_bias = gyro_mean * dt_imu
        else:
            # 阶段 3：正式滤波
            del_ang = gyro_in * dt_imu
            del_vel = self.acc_in_k * acc_in * dt_imu
            self.mag_run_time = min(100.0, self.mag_run_time + dt_imu)

            if abs(self.gyro0_init_flag - 0.0) < 1e-7:
                self.gyro0_init_flag = (
                    1.0
                    if abs(self.del_ang_bias[0]) + abs(self.del_ang_bias[1]) + abs(self.del_ang_bias[2]) < 1e-7
                    else 2.0
                )
            if self.gyro0_init_flag < 2.0 and motionless_flag > 3:
                self.del_ang_bias = del_ang.copy()
                self.gyro0_init_flag = 2.0

            # 静止分窗陀螺零偏估计（窗口 = 0.23*Fs 三段轮转，指数平滑）
            if motionless_flag == 4:
                self.motionless_cnt += 1.0
                n4 = float(n) * 0.23
                if self.motionless_cnt < n4 * 1.0:
                    self.gyro_in_sum1 = np.zeros(3)
                    self.gyro_in_cnt1 = 0.0
                    self.gyro_in_sum2 = np.zeros(3)
                    self.gyro_in_cnt2 = 0.0
                elif self.motionless_cnt < n4 * 2.0:
                    self.gyro_in_sum1 = self.gyro_in_sum1 + gyro_in
                    self.gyro_in_cnt1 += 1.0
                elif self.motionless_cnt < n4 * 3.0:
                    self.gyro_in_sum2 = self.gyro_in_sum2 + gyro_in
                    self.gyro_in_cnt2 += 1.0
                else:
                    mean1 = self.gyro_in_sum1 / self.gyro_in_cnt1
                    if (
                        abs(self.gyro_in_bias[0]) + abs(self.gyro_in_bias[1]) + abs(self.gyro_in_bias[2])
                        < 1e-5
                    ):
                        self.gyro_in_bias = mean1.copy()
                    k_bias = prm.ang_rate_bias_k
                    self.gyro_in_bias = self.gyro_in_bias * (1.0 - k_bias) + mean1 * k_bias
                    self.del_ang_bias = self.gyro_in_bias * dt_imu
                    self.gyro_in_sum1 = self.gyro_in_sum2.copy()
                    self.gyro_in_cnt1 = self.gyro_in_cnt2
                    self.gyro_in_sum2 = np.zeros(3)
                    self.gyro_in_cnt2 = 0.0
                    self.motionless_cnt = n4 * 2.0
            else:
                self.motionless_cnt = 0.0

            # 检测到完全静止时，用零偏增量替代真实增量（姿态冻结）
            if motionless_flag > 3:
                del_ang = self.del_ang_bias.copy()

            out_run_pos_flag = 0
            if prm.work_mode == 1:
                if gyro_norm > 50.0 / 573.0:
                    out_run_pos_flag = 1
            elif prm.work_mode == 2 and motionless_flag < 1:
                out_run_pos_flag = 1

            self.q, self.v, self.position, corr_ang, corr_vel = state_predict(
                self.q, self.v, del_ang, del_vel, dt_imu, self.del_ang_bias, out_run_pos_flag, self.position, prm.g0
            )
            self.dt_cov += dt_imu
            self.del_ang_cov = self.del_ang_cov + corr_ang
            self.del_vel_cov = self.del_vel_cov + corr_vel
            self.dt_run_time += dt_imu

            if self.dt_run_time >= 0.005:
                # 4 拍轮转：1/3 协方差传播，2 磁航向，0 ZUPT
                self.dt_run_time = 0.0
                self.flag_div += 1.0
                if self.flag_div >= 4.0:
                    self.flag_div = 0.0
                if abs(self.flag_div - 1.0) < 1e-7 or abs(self.flag_div - 3.0) < 1e-7:
                    self.p = covariance_predict(
                        self.p, self.del_ang_cov, self.del_vel_cov, self.q, self.dt_cov,
                        motionless_flag, prm.amode, prm,
                    )
                    self.del_ang_cov = np.zeros(3)
                    self.del_vel_cov = np.zeros(3)
                    self.dt_cov = 0.0
                elif abs(self.flag_div - 2.0) < 1e-7:
                    if prm.mag_fuse_method == 2 and float(np.linalg.norm(mag)) > 0.0 and prm.amode == 9:
                        if motionless_flag < 3:
                            if self.mag_run_time < 2.0:
                                r_mag = 100.0
                                mag_hdg_gate = prm.mag_hdg_gate
                            else:
                                r_mag = 10.0 * prm.mag_hdg_error * prm.mag_hdg_error
                                mag_hdg_gate = prm.mag_hdg_gate
                        else:
                            if self.mag_run_time > 10.0:
                                # 长时间静止后用磁航向重新粗对准并重置协方差
                                attitude = align_heading(
                                    self.q, mag, prm.mag_decl_deg * (math.pi / 180.0)
                                )
                                self.p = init_covariance(prm, dt_imu, prm.amode)
                                self.q = attitude
                            self.mag_run_time = 0.0
                            r_mag = 0.3 * prm.mag_hdg_error * prm.mag_hdg_error
                            mag_hdg_gate = prm.mag_hdg_gate
                        meas_dec = prm.mag_decl_deg * (math.pi / 180.0)
                        self.q, self.v, self.p, _nu = mag_heading_observation_fuse(
                            self.q, self.v, self.p, mag, meas_dec, mag_hdg_gate, r_mag
                        )
                    else:
                        self.mag_run_time = 0.0
                else:
                    # ZUPT：量测方差/门限按运动档位自适应
                    v_abs_sum = abs(self.v[0]) + abs(self.v[1]) + abs(self.v[2])
                    if motionless_flag == 0:
                        obs_var = 10000.0
                        gate = prm.body_vel_gate * 0.0001
                    elif motionless_flag == 1:
                        if v_abs_sum > 0.5:
                            self.v = np.zeros(3)
                        obs_var = 1.0
                        gate = prm.body_vel_gate
                    elif motionless_flag == 2:
                        if v_abs_sum > 0.5:
                            self.v = np.zeros(3)
                        obs_var = 0.010000000000000002
                        gate = prm.body_vel_gate * 10.0
                    else:
                        if v_abs_sum > 0.1:
                            self.v = np.zeros(3)
                        obs_var = 0.0001
                        gate = prm.body_vel_gate * 100.0
                    self.q, self.v, self.p, _innov, _s = velocity_observation_fuse(
                        self.q, self.v, self.p, obs_var, gate
                    )

        self.gyro_old = gyro_in.copy()
        if motionless_flag > 0:
            self.position = np.zeros(3)


# ---------------------------------------------------------------------------
