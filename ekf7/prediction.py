# -*- coding: utf-8 -*-
from __future__ import annotations
"""预测步：机械编排递推（姿态/速度/位置）与协方差传播 F P Fᵀ + Q。
"""

import numpy as np

from .jacobian import calc_f7, calc_q7
from .parameters import Parameters
from .quaternion import quat_from_rotation_vector, quat_multiply, quat_normalized, quat_to_dcm


def state_predict(state_q, state_v, del_ang, del_vel, dt, del_ang_bias, out_run_pos_flag, position, gravity):
    corrected_del_ang = del_ang - del_ang_bias
    dq = quat_from_rotation_vector(corrected_del_ang)
    q = quat_normalized(quat_multiply(state_q, dq))
    dcm = quat_to_dcm(q)
    dv_nav = dcm @ del_vel - np.array([0.0, 0.0, gravity]) * dt
    v_new = state_v + dv_nav
    pos = position.copy()
    if out_run_pos_flag > 0:
        pos = pos + 0.5 * dt * (state_v + v_new)
    return q, v_new, pos, corrected_del_ang, del_vel


def covariance_predict(p, delta_angle, delta_velocity, q, dt, motionless_flag, amode, param: Parameters):
    dax_var = dt * param.ang_rate_noise * (dt * param.ang_rate_noise)
    day_var = dax_var
    daz_var = dax_var
    dvx_var = dt * param.accel_noise * (dt * param.accel_noise)
    dvy_var = dvx_var
    dvz_var = dvx_var
    dvx, dvy, dvz = delta_velocity
    dax, day, daz = delta_angle
    q0, q1, q2, q3 = q
    if amode == 6:
        daz_var = 0.0
    if motionless_flag > 3:
        dax = 0.0
        day = 0.0
        daz = 0.0
    f = calc_f7(dax, day, daz, dvx, dvy, dvz, q0, q1, q2, q3)
    qm = calc_q7(dax_var, day_var, daz_var, dvx_var, dvy_var, dvz_var, q0, q1, q2, q3)
    p_new = f @ p @ f.T + qm
    p_new = 0.5 * (p_new + p_new.T)
    for i in range(7):
        if p_new[i, i] < 0.0:
            p_new[i, i] = 0.0
    return p_new


