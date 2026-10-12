/*
 * ekf7.h — 七状态(四元数+速度)全量 EKF 姿态解算，C99 移植
 *
 * 与 Python 参考实现（../ekf7/ 包）逐行对应；黑盒差分验证见 c/README.md。
 * 无动态分配、无外部依赖，可直接并入嵌入式工程（改 ekf7_real 为 float 即可，
 * 但需重新做差分验证评估精度损失）。
 *
 * 快速上手：
 *   ekf7_pipeline pipe;
 *   ekf7_pipeline_init(&pipe, 200.0, 1, 9);        // fs, work_mode, amode(6/9)
 *   ekf7_output out;
 *   ekf7_pipeline_calculate(&pipe, acc, gyro, mag, &out);  // 每拍一次
 *   // out.quat / out.euler_rad / out.vel / out.pos / out.motionless
 */
#ifndef EKF7_H
#define EKF7_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef double ekf7_real; /* 固件单精度 FPU 场景可改为 float（需重验差分） */

/* ---------------- 参数（默认 = 200Hz 档整定，见 ../README.md 参数表） ---------- */
typedef struct {
    double fs;                    /* 采样率 Hz */
    int    ipack_mean;            /* 打包均值 */
    int    work_mode;             /* 0/1/2 位置积分手感档位 */
    double gyro_filt_k, acc_filt_k; /* 输入一阶低通系数 */
    double k_bias;
    int    amode;                 /* 6=六轴(无磁) 9=九轴(磁航向融合) */
    double g0;                    /* 重力常量 */
    double auto_cal_gyro_max_min; /* 自标定静止陀螺峰峰值阈 */
    double auto_cal_acc_max_min;  /* 自标定静止加计峰峰值阈 */
    double body_vel_gate;         /* ZUPT 卡方门限基数 */
    int    mag_fuse_method;       /* 2=启用磁航向融合 */
    double mag_time_delay;
    double mag_hdg_error;         /* 磁航向测量噪声 */
    double mag_hdg_gate;          /* 磁航向卡方门限 */
    double mag_decl_deg;          /* 磁偏角（度） */
    double ang_rate_bias_k;       /* 陀螺零偏指数平滑系数 */
    double ang_rate_noise;        /* 过程噪声: 角度增量 (rad) */
    double accel_noise;           /* 过程噪声: 速度增量 (m/s) */
    double vel_err_ne, vel_err_d; /* 初始协方差: 速度 */
    double quat_err;              /* 初始协方差: 四元数 */
} ekf7_params;

ekf7_params ekf7_params_default(void);

/* ---------------- 三级静止检测器（0运动/1低角速率/2仅yaw静止/4完全静止） -------- */
typedef struct {
    double imu_sum_time_abs;
    double acc_raw_max_abs[3], acc_raw_min_abs[3];
    double gyro_raw_max_abs[3], gyro_raw_min_abs[3];
    double yaw_sum_time, yaw_max, yaw_min;
    double gyro_ml_sum_time;
    int    motionless_flag_out;
} ekf7_motion_state;

int ekf7_motion_detect(const double acc_in[3], const double gyro_in[3],
                       ekf7_motion_state *ml, const ekf7_params *p, double dt);

/* ---------------- 静止自标定（安装矩阵 + 陀螺零偏） ---------------- */
typedef struct {
    double run_flag, ml_error_flag, imu_sum_time_abs;
    double acc_raw_max_abs[3], acc_raw_min_abs[3];
    double gyro_raw_max_abs[3], gyro_raw_min_abs[3];
    double motionless_cnt;
    double acc_in_sum1[3], gyro_in_sum1[3];
    double imu_in_cnt1;
} ekf7_autocal_state;

typedef struct {
    double gyro_bias[3];
    double imu_r[9];   /* 行主序 3x3 安装矩阵 */
    bool   completed;
} ekf7_autocal_result;

void ekf7_autocal_run(const double acc_in[3], const double gyro_in[3],
                      ekf7_autocal_state *st, const ekf7_params *p, double dt,
                      ekf7_autocal_result *out);

/* ---------------- 主滤波器（NavigationFilter） ---------------- */
typedef struct {
    ekf7_params prm;               /* 参数副本 */
    double q[4];                  /* 姿态四元数 wxyz */
    double v[3];                  /* ENU 导航系速度 */
    double p[7][7];               /* 协方差 */
    double flag_in, init_max_time;
    double gyro_old[3], acc_sum[3], gyro_sum[3], mag_sum[3];
    double del_ang_cov[3], dt_cov, flag_div;
    double motionless_cnt, mag_run_time, gyro0_init_flag;
    double del_vel_cov[3], acc_in_k, dt_run_time;
    double gyro_in_bias[3], gyro_in_sum1[3], gyro_in_cnt1;
    double gyro_in_sum2[3], gyro_in_cnt2;
    double del_ang_bias[3], position[3], mag_in_old[3];
} ekf7_filter;

void   ekf7_filter_init(ekf7_filter *f, const ekf7_params *prm);
void   ekf7_filter_reset(ekf7_filter *f);
void   ekf7_filter_step(ekf7_filter *f, const double acc_in[3],
                        const double gyro_in[3], const double mag_in[3],
                        double dt_imu, int motionless_flag);
void   ekf7_filter_euler(const ekf7_filter *f, double rpy_rad[3]);

/* ---------------- 端到端管线（低通 → 自标定 → 检测 → 滤波） ---------------- */
typedef struct {
    ekf7_params     prm;
    double          dt;
    ekf7_filter     filt;
    ekf7_motion_state  ml;
    ekf7_autocal_state cal;
    double acc_filt[3], gyro_filt[3];
    bool   filt_init;
    double rotm_install[9];       /* 行主序 3x3 */
    double gyro_bias[3];
    bool   enable_autocal;
    int    icount;
} ekf7_pipeline;

typedef struct {
    double quat[4];
    double euler_rad[3];
    double vel[3];
    double pos[3];
    int    motionless;
} ekf7_output;

void ekf7_pipeline_init(ekf7_pipeline *pl, double fs, int work_mode, int amode);
/* mag 可传 NULL（六轴模式） */
void ekf7_pipeline_calculate(ekf7_pipeline *pl, const double acc_raw[3],
                             const double gyro_raw[3], const double mag_raw[3],
                             ekf7_output *out);

/* ---------------- 底层数学（公开便于单元测试/复用） ---------------- */
void   ekf7_quat_multiply(const double a[4], const double b[4], double out[4]);
void   ekf7_quat_normalized(const double q[4], double out[4]);
void   ekf7_quat_to_dcm(const double q[4], double r[9]);            /* 体->导航 */
void   ekf7_quat_from_rv(const double rv[3], double q[4]);          /* 旋转向量 */
void   ekf7_quat_from_euler(double phi, double theta, double psi, double q[4]);
void   ekf7_quat_to_euler(const double q[4], double rpy[3]);
double ekf7_wrap_pi(double angle);
void   ekf7_kfilter_inplace(double *prev, const double *x, double k, int n);

/* 雅可比（供研究/测试直接调用） */
void ekf7_calc_f7(double dax, double day, double daz,
                  double dvx, double dvy, double dvz,
                  double q0, double q1, double q2, double q3, double f[7][7]);
void ekf7_calc_q7(double dax_var, double day_var, double daz_var,
                  double dvx_var, double dvy_var, double dvz_var,
                  double q0, double q1, double q2, double q3, double q[7][7]);
void ekf7_calc_h_hdgc(double mx, double my, double mz,
                      double q0, double q1, double q2, double q3, double h[7]);

#ifdef __cplusplus
}
#endif
#endif /* EKF7_H */
