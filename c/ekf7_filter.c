/* ekf7_filter.c — 对准/预测/观测与主滤波循环
 * （对应 ../ekf7/{alignment,prediction,observations,navigation_filter}.py） */
#include <math.h>
#include <string.h>

#include "ekf7.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ekf7_params ekf7_params_default(void)
{
    ekf7_params p;
    p.fs = 200.0;
    p.ipack_mean = 1;
    p.work_mode = 0;
    p.gyro_filt_k = 0.1;
    p.acc_filt_k = 0.1;
    p.k_bias = 0.1;
    p.amode = 6;
    p.g0 = 9.795502520545487;
    p.auto_cal_gyro_max_min = M_PI / 900.0;
    p.auto_cal_acc_max_min = 0.05;
    p.body_vel_gate = 10.0;
    p.mag_fuse_method = 2;
    p.mag_time_delay = 0.0;
    p.mag_hdg_error = 0.1745;
    p.mag_hdg_gate = 5.0;
    p.mag_decl_deg = 0.0;
    p.ang_rate_bias_k = 0.04;
    p.ang_rate_noise = 0.01;
    p.accel_noise = 0.35;
    p.vel_err_ne = 5.0;
    p.vel_err_d = 1.0;
    p.quat_err = 0.1;
    return p;
}

/* ---------------- 对准（alignment.py） ---------------- */

static void align_tilt(const double init_accel[3], double q_out[4])
{
    const double ax = init_accel[0], ay = init_accel[1], az = init_accel[2];
    const double norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm > 5.0 && norm < 14.0) {
        const double horiz = sqrt(ax * ax + ay * ay);
        const double tilt = atan2(horiz, az);
        if (tilt > 0.001) {
            /* axis = accel × ẑ */
            const double c[3] = { ay * 1.0 - az * 0.0, az * 0.0 - ax * 1.0, ax * 0.0 - ay * 0.0 };
            const double cn = sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
            if (cn > 1e-12) {
                const double s = sin(0.5 * tilt);
                q_out[0] = cos(0.5 * tilt);
                q_out[1] = (c[0] / cn) * s;
                q_out[2] = (c[1] / cn) * s;
                q_out[3] = (c[2] / cn) * s;
                return;
            }
        }
    }
    /* 保持原值 */
    (void)q_out;
}

static void align_heading(const double q_in[4], const double mag[3],
                          double declination, double q_out[4])
{
    double e[3];
    ekf7_quat_to_euler(q_in, e);
    e[2] = 0.0;
    double q_level[4], r[9], mnav[3];
    ekf7_quat_from_euler(e[0], e[1], e[2], q_level);
    ekf7_quat_to_dcm(q_level, r);
    for (int i = 0; i < 3; i++)
        mnav[i] = r[3 * i] * mag[0] + r[3 * i + 1] * mag[1] + r[3 * i + 2] * mag[2];
    e[2] = declination + atan2(mnav[0], mnav[1]);
    ekf7_quat_from_euler(e[0], e[1], e[2], q_out);
}

static void init_states(const double acc_in[3], const double gyro_in[3],
                        const double mag_in[3], const ekf7_params *prm, double q_out[4])
{
    double q[4] = { 1.0, 0.0, 0.0, 0.0 };
    if (fabs(gyro_in[0]) < 0.05 && fabs(gyro_in[1]) < 0.05 && fabs(gyro_in[2]) < 0.05)
        align_tilt(acc_in, q);
    if (sqrt(mag_in[0] * mag_in[0] + mag_in[1] * mag_in[1] + mag_in[2] * mag_in[2]) > 0.0) {
        const double decl = prm->mag_decl_deg * (M_PI / 180.0);
        double qh[4];
        align_heading(q, mag_in, decl, qh);
        memcpy(q, qh, sizeof q);
    }
    memcpy(q_out, q, sizeof q);
}

static void init_covariance(const ekf7_params *prm, int amode, double p[7][7])
{
    double qd[4];
    const double one = prm->quat_err;
    qd[0] = one; qd[1] = one; qd[2] = one;
    qd[3] = (amode != 6) ? one : 0.0; /* 六轴：q3(yaw) 误差置 0 */
    const double vd[3] = { prm->vel_err_ne, prm->vel_err_ne, prm->vel_err_d };
    memset(p, 0, sizeof(double) * 49);
    for (int i = 0; i < 4; i++) p[i][i] = qd[i] * qd[i];
    for (int j = 0; j < 3; j++) p[4 + j][4 + j] = vd[j] * vd[j];
}

/* ---------------- 预测（prediction.py） ---------------- */

static void state_predict(const double q_in[4], const double v_in[3],
                          const double del_ang[3], const double del_vel[3],
                          double dt, const double del_ang_bias[3],
                          int out_run_pos_flag, const double pos_in[3],
                          double gravity,
                          double q_out[4], double v_out[3], double pos_out[3],
                          double corr_ang[3], double corr_vel[3])
{
    double dang[3], dq[4], q_tmp[4], r[9], gv[3], v_new[3];
    for (int i = 0; i < 3; i++)
        dang[i] = del_ang[i] - del_ang_bias[i];
    ekf7_quat_from_rv(dang, dq);
    ekf7_quat_multiply(q_in, dq, q_tmp);
    ekf7_quat_normalized(q_tmp, q_out);
    ekf7_quat_to_dcm(q_out, r);
    for (int i = 0; i < 3; i++) {
        gv[i] = r[3 * i] * del_vel[0] + r[3 * i + 1] * del_vel[1] + r[3 * i + 2] * del_vel[2];
        if (i == 2) gv[i] -= gravity * dt;
        v_new[i] = v_in[i] + gv[i];
    }
    for (int i = 0; i < 3; i++) {
        pos_out[i] = pos_in[i];
        if (out_run_pos_flag > 0)
            pos_out[i] += 0.5 * dt * (v_in[i] + v_new[i]);
        v_out[i] = v_new[i];
        corr_ang[i] = dang[i];
        corr_vel[i] = del_vel[i];
    }
}

static void covariance_predict(double p[7][7], const double delta_angle[3],
                               const double delta_velocity[3], const double q[4],
                               double dt, int motionless_flag, int amode,
                               const ekf7_params *prm)
{
    const double n_var = dt * prm->ang_rate_noise * (dt * prm->ang_rate_noise);
    double dax_var = n_var, day_var = n_var, daz_var = n_var;
    const double n2_var = dt * prm->accel_noise * (dt * prm->accel_noise);
    double dvx_var = n2_var, dvy_var = n2_var, dvz_var = n2_var;
    double dvx = delta_velocity[0], dvy = delta_velocity[1], dvz = delta_velocity[2];
    double dax = delta_angle[0], day = delta_angle[1], daz = delta_angle[2];
    double f[7][7], qm[7][7], tmp[7][7];

    if (amode == 6)
        daz_var = 0.0;
    if (motionless_flag > 3) {
        dax = 0.0; day = 0.0; daz = 0.0;
    }
    ekf7_calc_f7(dax, day, daz, dvx, dvy, dvz, q[0], q[1], q[2], q[3], f);
    ekf7_calc_q7(dax_var, day_var, daz_var, dvx_var, dvy_var, dvz_var,
                 q[0], q[1], q[2], q[3], qm);
    /* tmp = f * p * f^T + qm */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++) {
            double s = 0.0;
            for (int k = 0; k < 7; k++) s += f[i][k] * p[k][j];
            tmp[i][j] = s;
        }
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++) {
            double s = 0.0;
            for (int k = 0; k < 7; k++) s += tmp[i][k] * f[j][k];
            p[i][j] = s + qm[i][j];
        }
    for (int i = 0; i < 7; i++)
        for (int j = i + 1; j < 7; j++) {
            const double s = 0.5 * (p[i][j] + p[j][i]);
            p[i][j] = s; p[j][i] = s;
        }
    for (int i = 0; i < 7; i++)
        if (p[i][i] < 0.0) p[i][i] = 0.0;
}

/* ---------------- 观测（observations.py） ---------------- */

/* 3x3 逆（伴随式），供 ZUPT 的 inv(S) */
static void inv3(const double a[3][3], double out[3][3])
{
    const double c00 = a[1][1] * a[2][2] - a[1][2] * a[2][1];
    const double c01 = a[1][2] * a[2][0] - a[1][0] * a[2][2];
    const double c02 = a[1][0] * a[2][1] - a[1][1] * a[2][0];
    const double det = a[0][0] * c00 + a[0][1] * c01 + a[0][2] * c02;
    const double inv = 1.0 / det;
    out[0][0] = c00 * inv;
    out[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv;
    out[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv;
    out[1][0] = c01 * inv;
    out[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv;
    out[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv;
    out[2][0] = c02 * inv;
    out[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv;
    out[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv;
}

/* H = [0_3x4 | I_3] 的结构被显式利用，避免通用矩阵乘 */
static void velocity_observation_fuse(double q_in[4], double v_in[3], double p[7][7],
                                      double obs_var, double gate_size,
                                      double q_out[4], double v_out[3])
{
    double s[3][3];
    for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++)
            s[j][k] = p[4 + j][4 + k] + ((j == k) ? obs_var : 0.0);
    for (int i = 0; i < 3; i++) {
        if (v_in[i] * v_in[i] / (gate_size * gate_size * s[i][i]) > 1.0) {
            /* 拒绝本次观测：原样返回 */
            memcpy(q_out, q_in, sizeof(double) * 4);
            memcpy(v_out, v_in, sizeof(double) * 3);
            return;
        }
    }
    double sinv[3][3];
    inv3(s, sinv);
    /* K = P[:, 4:7] * inv(S)  (7x3) */
    double k[7][3];
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 3; j++) {
            double acc = 0.0;
            for (int m2 = 0; m2 < 3; m2++)
                acc += p[i][4 + m2] * sinv[m2][j];
            k[i][j] = acc;
        }
    double x[7];
    memcpy(x, q_in, sizeof(double) * 4);
    memcpy(x + 4, v_in, sizeof(double) * 3);
    for (int i = 0; i < 7; i++) {
        double d = 0.0;
        for (int j = 0; j < 3; j++) d += k[i][j] * v_in[j];
        x[i] -= d;
    }
    memcpy(q_out, x, sizeof(double) * 4);
    ekf7_quat_normalized(q_out, q_out);
    memcpy(v_out, x + 4, sizeof(double) * 3);
    /* P -= K * P[4:7, :] */
    for (int i = 0; i < 7; i++)
        for (int c = 0; c < 7; c++) {
            double d = 0.0;
            for (int j = 0; j < 3; j++) d += k[i][j] * p[4 + j][c];
            p[i][c] -= d;
        }
    for (int i = 0; i < 7; i++)
        for (int j = i + 1; j < 7; j++) {
            const double m2 = 0.5 * (p[i][j] + p[j][i]);
            p[i][j] = m2; p[j][i] = m2;
        }
    for (int i = 0; i < 7; i++)
        if (p[i][i] < 0.0) p[i][i] = 0.0;
}

static void mag_heading_observation_fuse(double q_in[4], double v_in[3], double p[7][7],
                                         const double mag[3], double meas_dec,
                                         double innov_gate, double r_mag,
                                         double q_out[4], double v_out[3])
{
    double r[9], mnav[3], h[7], ph[7], k[7];
    ekf7_quat_to_dcm(q_in, r);
    for (int i = 0; i < 3; i++)
        mnav[i] = r[3 * i] * mag[0] + r[3 * i + 1] * mag[1] + r[3 * i + 2] * mag[2];
    double nu = atan2(mnav[0], mnav[1]) - meas_dec;
    if (nu > M_PI) nu -= 2.0 * M_PI;
    else if (nu < -M_PI) nu += 2.0 * M_PI;
    ekf7_calc_h_hdgc(mag[0], mag[1], mag[2], q_in[0], q_in[1], q_in[2], q_in[3], h);
    for (int j = 0; j < 7; j++) {
        double acc = 0.0;
        for (int i = 0; i < 7; i++) acc += h[i] * p[i][j];
        ph[j] = acc;
    }
    double s = 0.0;
    for (int i = 0; i < 7; i++) s += ph[i] * h[i];
    s += r_mag;
    if (nu * nu / (innov_gate * innov_gate * s) > 1.0) {
        memcpy(q_out, q_in, sizeof(double) * 4);
        memcpy(v_out, v_in, sizeof(double) * 3);
        return;
    }
    for (int i = 0; i < 7; i++) {
        double acc = 0.0;
        for (int j = 0; j < 7; j++) acc += p[i][j] * h[j];
        k[i] = acc / s;
    }
    double x[7];
    memcpy(x, q_in, sizeof(double) * 4);
    memcpy(x + 4, v_in, sizeof(double) * 3);
    for (int i = 0; i < 7; i++) x[i] -= k[i] * nu;
    memcpy(q_out, x, sizeof(double) * 4);
    ekf7_quat_normalized(q_out, q_out);
    memcpy(v_out, x + 4, sizeof(double) * 3);
    for (int i = 0; i < 7; i++)
        for (int c = 0; c < 7; c++)
            p[i][c] -= k[i] * ph[c];
    for (int i = 0; i < 7; i++)
        for (int j = i + 1; j < 7; j++) {
            const double m2 = 0.5 * (p[i][j] + p[j][i]);
            p[i][j] = m2; p[j][i] = m2;
        }
    for (int i = 0; i < 7; i++)
        if (p[i][i] < 0.0) p[i][i] = 0.0;
}

/* ---------------- 主滤波循环（navigation_filter.py） ---------------- */

void ekf7_filter_init(ekf7_filter *f, const ekf7_params *prm)
{
    f->prm = *prm;
    ekf7_filter_reset(f);
}

void ekf7_filter_reset(ekf7_filter *f)
{
    ekf7_params prm = f->prm; /* 参数不在 reset 范围内（对应 Python 的独立 param 对象） */
    memset(f, 0, sizeof *f);
    f->prm = prm;
    f->q[0] = 1.0;
    /* 其余字段按 C# 语义默认全 0（含 min/max 锚点） */
}

void ekf7_filter_euler(const ekf7_filter *f, double rpy_rad[3])
{
    ekf7_quat_to_euler(f->q, rpy_rad);
}

void ekf7_filter_step(ekf7_filter *f, const double acc_in[3],
                      const double gyro_in[3], const double mag_in[3],
                      double dt_imu, int motionless_flag)
{
    const ekf7_params *prm = &f->prm;
    const int n = (int)(prm->fs * (double)prm->ipack_mean);
    const double gyro_norm = sqrt(gyro_in[0] * gyro_in[0] + gyro_in[1] * gyro_in[1]
                                  + gyro_in[2] * gyro_in[2]);
    double mag[3];
    if (motionless_flag > 3) {
        memcpy(mag, f->mag_in_old, sizeof mag);
    } else {
        memcpy(mag, mag_in, sizeof mag);
        memcpy(f->mag_in_old, mag_in, sizeof mag);
    }
    const double n3 = (double)(n * 3);

    if (f->init_max_time < 0.1) {
        /* 阶段 0：强制单位四元数 */
        f->init_max_time += dt_imu;
        f->q[0] = 1.0;
    } else if (f->init_max_time < 3.0) {
        /* 阶段 1：静置均值累计 */
        f->init_max_time += dt_imu;
        if (fabs(f->flag_in - 0.0) < 1e-7) {
            memcpy(f->acc_sum, acc_in, sizeof f->acc_sum);
            memcpy(f->gyro_sum, gyro_in, sizeof f->gyro_sum);
            memcpy(f->mag_sum, mag, sizeof f->mag_sum);
            f->flag_in += 1.0;
        } else if (f->flag_in < n3) {
            for (int i = 0; i < 3; i++) {
                f->acc_sum[i] += acc_in[i];
                f->gyro_sum[i] += gyro_in[i];
                f->mag_sum[i] += mag[i];
            }
            if (fabs(gyro_in[0] - f->gyro_old[0]) < 0.02
                && fabs(gyro_in[1] - f->gyro_old[1]) < 0.02
                && fabs(gyro_in[2] - f->gyro_old[2]) < 0.02)
                f->flag_in += 1.0;
            else
                f->flag_in = 0.0;
        }
        if (f->flag_in >= (double)n)
            f->init_max_time = 3.0;
    } else if (f->init_max_time < 4.0) {
        /* 阶段 2：粗对准 */
        double acc_mean[3], gyro_mean[3], mag_mean[3];
        f->init_max_time = 4.0;
        if (f->flag_in >= (double)n) {
            for (int i = 0; i < 3; i++) {
                acc_mean[i] = f->acc_sum[i] / f->flag_in;
                gyro_mean[i] = f->gyro_sum[i] / f->flag_in;
                mag_mean[i] = f->mag_sum[i] / f->flag_in;
            }
            const double an = sqrt(acc_mean[0] * acc_mean[0] + acc_mean[1] * acc_mean[1]
                                   + acc_mean[2] * acc_mean[2]);
            f->acc_in_k = prm->g0 / an;
            for (int i = 0; i < 3; i++) acc_mean[i] *= f->acc_in_k;
        } else {
            acc_mean[0] = 0.0; acc_mean[1] = 0.0; acc_mean[2] = prm->g0;
            gyro_mean[0] = gyro_mean[1] = gyro_mean[2] = 1.0;
            memcpy(mag_mean, mag, sizeof mag_mean);
            f->acc_in_k = 1.0;
        }
        memset(f->del_ang_cov, 0, sizeof f->del_ang_cov);
        f->dt_cov = 0.0;
        f->flag_div = 0.0;
        init_states(acc_mean, gyro_mean, mag_mean, prm, f->q);
        init_covariance(prm, prm->amode, f->p);
        if (fabs(gyro_mean[0]) < 0.05 && fabs(gyro_mean[1]) < 0.05
            && fabs(gyro_mean[2]) < 0.05) {
            memcpy(f->gyro_in_bias, gyro_mean, sizeof f->gyro_in_bias);
        } else {
            gyro_mean[0] = gyro_mean[1] = gyro_mean[2] = 0.0;
            memcpy(f->gyro_in_bias, gyro_mean, sizeof f->gyro_in_bias);
        }
        for (int i = 0; i < 3; i++)
            f->del_ang_bias[i] = gyro_mean[i] * dt_imu;
    } else {
        /* 阶段 3：正式滤波 */
        double del_ang[3], del_vel[3];
        for (int i = 0; i < 3; i++) {
            del_ang[i] = gyro_in[i] * dt_imu;
            del_vel[i] = f->acc_in_k * acc_in[i] * dt_imu;
        }
        f->mag_run_time = f->mag_run_time + dt_imu;
        if (f->mag_run_time > 100.0) f->mag_run_time = 100.0;

        if (fabs(f->gyro0_init_flag - 0.0) < 1e-7) {
            f->gyro0_init_flag =
                (fabs(f->del_ang_bias[0]) + fabs(f->del_ang_bias[1])
                 + fabs(f->del_ang_bias[2]) < 1e-7) ? 1.0 : 2.0;
        }
        if (f->gyro0_init_flag < 2.0 && motionless_flag > 3) {
            memcpy(f->del_ang_bias, del_ang, sizeof f->del_ang_bias);
            f->gyro0_init_flag = 2.0;
        }

        if (motionless_flag == 4) {
            f->motionless_cnt += 1.0;
            const double n4 = (double)n * 0.23;
            if (f->motionless_cnt < n4 * 1.0) {
                memset(f->gyro_in_sum1, 0, sizeof f->gyro_in_sum1);
                f->gyro_in_cnt1 = 0.0;
                memset(f->gyro_in_sum2, 0, sizeof f->gyro_in_sum2);
                f->gyro_in_cnt2 = 0.0;
            } else if (f->motionless_cnt < n4 * 2.0) {
                for (int i = 0; i < 3; i++) f->gyro_in_sum1[i] += gyro_in[i];
                f->gyro_in_cnt1 += 1.0;
            } else if (f->motionless_cnt < n4 * 3.0) {
                for (int i = 0; i < 3; i++) f->gyro_in_sum2[i] += gyro_in[i];
                f->gyro_in_cnt2 += 1.0;
            } else {
                double mean1[3];
                for (int i = 0; i < 3; i++) mean1[i] = f->gyro_in_sum1[i] / f->gyro_in_cnt1;
                if (fabs(f->gyro_in_bias[0]) + fabs(f->gyro_in_bias[1])
                    + fabs(f->gyro_in_bias[2]) < 1e-5)
                    memcpy(f->gyro_in_bias, mean1, sizeof f->gyro_in_bias);
                const double kb = prm->ang_rate_bias_k;
                for (int i = 0; i < 3; i++)
                    f->gyro_in_bias[i] = f->gyro_in_bias[i] * (1.0 - kb) + mean1[i] * kb;
                for (int i = 0; i < 3; i++)
                    f->del_ang_bias[i] = f->gyro_in_bias[i] * dt_imu;
                memcpy(f->gyro_in_sum1, f->gyro_in_sum2, sizeof f->gyro_in_sum1);
                f->gyro_in_cnt1 = f->gyro_in_cnt2;
                memset(f->gyro_in_sum2, 0, sizeof f->gyro_in_sum2);
                f->gyro_in_cnt2 = 0.0;
                f->motionless_cnt = n4 * 2.0;
            }
        } else {
            f->motionless_cnt = 0.0;
        }

        if (motionless_flag > 3)
            memcpy(del_ang, f->del_ang_bias, sizeof del_ang);

        int out_run_pos_flag = 0;
        if (prm->work_mode == 1) {
            if (gyro_norm > 50.0 / 573.0)
                out_run_pos_flag = 1;
        } else if (prm->work_mode == 2 && motionless_flag < 1) {
            out_run_pos_flag = 1;
        }

        double corr_ang[3], corr_vel[3];
        state_predict(f->q, f->v, del_ang, del_vel, dt_imu, f->del_ang_bias,
                      out_run_pos_flag, f->position, prm->g0,
                      f->q, f->v, f->position, corr_ang, corr_vel);
        f->dt_cov += dt_imu;
        for (int i = 0; i < 3; i++) {
            f->del_ang_cov[i] += corr_ang[i];
            f->del_vel_cov[i] += corr_vel[i];
        }
        f->dt_run_time += dt_imu;

        if (f->dt_run_time >= 0.005) {
            f->dt_run_time = 0.0;
            f->flag_div += 1.0;
            if (f->flag_div >= 4.0)
                f->flag_div = 0.0;
            if (fabs(f->flag_div - 1.0) < 1e-7 || fabs(f->flag_div - 3.0) < 1e-7) {
                covariance_predict(f->p, f->del_ang_cov, f->del_vel_cov, f->q,
                                   f->dt_cov, motionless_flag, prm->amode, prm);
                memset(f->del_ang_cov, 0, sizeof f->del_ang_cov);
                memset(f->del_vel_cov, 0, sizeof f->del_vel_cov);
                f->dt_cov = 0.0;
            } else if (fabs(f->flag_div - 2.0) < 1e-7) {
                const double mag_norm = sqrt(mag[0] * mag[0] + mag[1] * mag[1]
                                             + mag[2] * mag[2]);
                if (prm->mag_fuse_method == 2 && mag_norm > 0.0 && prm->amode == 9) {
                    double r_mag, mag_hdg_gate;
                    if (motionless_flag < 3) {
                        if (f->mag_run_time < 2.0) {
                            r_mag = 100.0;
                            mag_hdg_gate = prm->mag_hdg_gate;
                        } else {
                            r_mag = 10.0 * prm->mag_hdg_error * prm->mag_hdg_error;
                            mag_hdg_gate = prm->mag_hdg_gate;
                        }
                    } else {
                        if (f->mag_run_time > 10.0) {
                            double qa[4];
                            align_heading(f->q, mag, prm->mag_decl_deg * (M_PI / 180.0), qa);
                            init_covariance(prm, prm->amode, f->p);
                            memcpy(f->q, qa, sizeof f->q);
                        }
                        f->mag_run_time = 0.0;
                        r_mag = 0.3 * prm->mag_hdg_error * prm->mag_hdg_error;
                        mag_hdg_gate = prm->mag_hdg_gate;
                    }
                    const double meas_dec = prm->mag_decl_deg * (M_PI / 180.0);
                    double qo[4], vo[3];
                    mag_heading_observation_fuse(f->q, f->v, f->p, mag, meas_dec,
                                                 mag_hdg_gate, r_mag, qo, vo);
                    memcpy(f->q, qo, sizeof f->q);
                    memcpy(f->v, vo, sizeof f->v);
                } else {
                    f->mag_run_time = 0.0;
                }
            } else {
                /* ZUPT */
                const double v_abs = fabs(f->v[0]) + fabs(f->v[1]) + fabs(f->v[2]);
                double obs_var, gate;
                if (motionless_flag == 0) {
                    obs_var = 10000.0;
                    gate = prm->body_vel_gate * 0.0001;
                } else if (motionless_flag == 1) {
                    if (v_abs > 0.5) memset(f->v, 0, sizeof f->v);
                    obs_var = 1.0;
                    gate = prm->body_vel_gate;
                } else if (motionless_flag == 2) {
                    if (v_abs > 0.5) memset(f->v, 0, sizeof f->v);
                    obs_var = 0.010000000000000002;
                    gate = prm->body_vel_gate * 10.0;
                } else {
                    if (v_abs > 0.1) memset(f->v, 0, sizeof f->v);
                    obs_var = 0.0001;
                    gate = prm->body_vel_gate * 100.0;
                }
                double qo[4], vo[3];
                velocity_observation_fuse(f->q, f->v, f->p, obs_var, gate, qo, vo);
                memcpy(f->q, qo, sizeof f->q);
                memcpy(f->v, vo, sizeof f->v);
            }
        }
    }

    memcpy(f->gyro_old, gyro_in, sizeof f->gyro_old);
    if (motionless_flag > 0)
        memset(f->position, 0, sizeof f->position);
}
