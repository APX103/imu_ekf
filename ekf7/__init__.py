# -*- coding: utf-8 -*-
from __future__ import annotations
"""ekf7 — 七状态(四元数+速度)全量 EKF 姿态解算。

最常用入口::

    from ekf7 import ImuPipeline
    pipe = ImuPipeline(fs=200.0, work_mode=1, amode=9)  # amode: 6=六轴 9=九轴
    out = pipe.calculate(acc, gyro, mag)                # -> quat/euler_rad/vel/pos/motionless

需要更细粒度控制（自定义调度/观测）时，可直接使用 :class:`NavigationFilter`
与 :class:`MotionDetector` 等组件，各模块职责见包内 docstring。
"""

from .alignment import align_heading, align_tilt, init_covariance, init_states
from .auto_calibration import AutoCalResult, AutoCalState, imu_auto_calibrator_run
from .jacobian import calc_f7, calc_h_hdgc, calc_q7
from .math_utils import cross3, kfilter_inplace, wrap_to_pi
from .motion import MotionState, motion_detector_detect
from .navigation_filter import NavigationFilter
from .observations import mag_heading_observation_fuse, velocity_observation_fuse
from .parameters import Parameters
from .pipeline import ImuPipeline
from .prediction import covariance_predict, state_predict
from .quaternion import (
    quat_from_euler,
    quat_from_rotation_vector,
    quat_multiply,
    quat_normalized,
    quat_to_dcm,
    quat_to_euler,
)

__version__ = "1.0.0"
__all__ = [
    "ImuPipeline", "NavigationFilter", "Parameters",
    "MotionState", "motion_detector_detect",
    "AutoCalState", "AutoCalResult", "imu_auto_calibrator_run",
    "init_states", "init_covariance", "align_tilt", "align_heading",
    "state_predict", "covariance_predict",
    "velocity_observation_fuse", "mag_heading_observation_fuse",
    "calc_f7", "calc_q7", "calc_h_hdgc",
    "quat_multiply", "quat_normalized", "quat_to_dcm",
    "quat_from_rotation_vector", "quat_from_euler", "quat_to_euler",
    "wrap_to_pi", "cross3", "kfilter_inplace",
]
