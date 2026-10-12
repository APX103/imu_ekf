/* ekf7_pipeline.c — 端到端管线（对应 ../ekf7/pipeline.py）
 * 输入一阶低通 -> (前 10s) 静止自标定 -> 静止检测 -> 滤波器步进 */
#include <math.h>
#include <string.h>

#include "ekf7.h"

void ekf7_pipeline_init(ekf7_pipeline *pl, double fs, int work_mode, int amode)
{
    memset(pl, 0, sizeof *pl);
    pl->prm = ekf7_params_default();
    pl->prm.fs = fs;
    pl->prm.work_mode = work_mode;
    pl->prm.amode = amode;
    pl->dt = 1.0 / fs;
    ekf7_filter_init(&pl->filt, &pl->prm);
    pl->rotm_install[0] = pl->rotm_install[4] = pl->rotm_install[8] = 1.0;
    pl->enable_autocal = true;
}

void ekf7_pipeline_calculate(ekf7_pipeline *pl, const double acc_raw[3],
                             const double gyro_raw[3], const double mag_raw[3],
                             ekf7_output *out)
{
    double acc[3], gyro[3], mag[3];
    memcpy(acc, acc_raw, sizeof acc);
    memcpy(gyro, gyro_raw, sizeof gyro);
    if (mag_raw)
        memcpy(mag, mag_raw, sizeof mag);
    else
        memset(mag, 0, sizeof mag);

    if (!pl->filt_init) {
        memcpy(pl->acc_filt, acc, sizeof pl->acc_filt);
        memcpy(pl->gyro_filt, gyro, sizeof pl->gyro_filt);
        pl->filt_init = true;
    } else {
        ekf7_kfilter_inplace(pl->acc_filt, acc, pl->prm.acc_filt_k, 3);
        ekf7_kfilter_inplace(pl->gyro_filt, gyro, pl->prm.gyro_filt_k, 3);
    }

    double gyro_corr[3];
    for (int i = 0; i < 3; i++)
        gyro_corr[i] = pl->gyro_filt[i] - pl->gyro_bias[i];
    double acc_in[3], gyro_in[3];
    for (int i = 0; i < 3; i++) {
        acc_in[i] = pl->rotm_install[3 * i] * pl->acc_filt[0]
                  + pl->rotm_install[3 * i + 1] * pl->acc_filt[1]
                  + pl->rotm_install[3 * i + 2] * pl->acc_filt[2];
        gyro_in[i] = pl->rotm_install[3 * i] * gyro_corr[0]
                   + pl->rotm_install[3 * i + 1] * gyro_corr[1]
                   + pl->rotm_install[3 * i + 2] * gyro_corr[2];
    }

    const int cal_limit = (int)pl->prm.fs * 10;
    if (pl->icount < cal_limit)
        pl->icount++;
    if (pl->enable_autocal && pl->icount < cal_limit) {
        ekf7_autocal_result res;
        ekf7_autocal_run(pl->acc_filt, pl->gyro_filt, &pl->cal, &pl->prm, pl->dt, &res);
        if (res.completed) {
            memcpy(pl->rotm_install, res.imu_r, sizeof pl->rotm_install);
            memcpy(pl->gyro_bias, res.gyro_bias, sizeof pl->gyro_bias);
            pl->enable_autocal = false;
            ekf7_filter_reset(&pl->filt);
        }
    }

    const int motionless_flag = ekf7_motion_detect(acc_in, gyro_in, &pl->ml,
                                                   &pl->prm, pl->dt);
    ekf7_filter_step(&pl->filt, acc_in, gyro_in, mag, pl->dt, motionless_flag);

    memcpy(out->quat, pl->filt.q, sizeof out->quat);
    ekf7_quat_to_euler(pl->filt.q, out->euler_rad);
    memcpy(out->vel, pl->filt.v, sizeof out->vel);
    memcpy(out->pos, pl->filt.position, sizeof out->pos);
    out->motionless = motionless_flag;
}
