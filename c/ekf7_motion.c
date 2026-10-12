/* ekf7_motion.c — 三级静止检测器与静止自标定
 * （对应 ../ekf7/{motion,auto_calibration}.py） */
#include <math.h>
#include <string.h>

#include "ekf7.h"

int ekf7_motion_detect(const double acc_in[3], const double gyro_in[3],
                       ekf7_motion_state *ml, const ekf7_params *p, double dt)
{
    int still = 0;
    ml->imu_sum_time_abs += dt;
    for (int i = 0; i < 3; i++) {
        if (acc_in[i] > ml->acc_raw_max_abs[i]) ml->acc_raw_max_abs[i] = acc_in[i];
        if (acc_in[i] < ml->acc_raw_min_abs[i]) ml->acc_raw_min_abs[i] = acc_in[i];
        if (gyro_in[i] > ml->gyro_raw_max_abs[i]) ml->gyro_raw_max_abs[i] = gyro_in[i];
        if (gyro_in[i] < ml->gyro_raw_min_abs[i]) ml->gyro_raw_min_abs[i] = gyro_in[i];
    }
    if (!(fabs(ml->gyro_raw_max_abs[0] - ml->gyro_raw_min_abs[0]) < 0.010471204188481676
          && fabs(ml->gyro_raw_max_abs[1] - ml->gyro_raw_min_abs[1]) < 0.010471204188481676
          && fabs(ml->gyro_raw_max_abs[2] - ml->gyro_raw_min_abs[2]) < 0.010471204188481676
          && fabs(ml->acc_raw_max_abs[0] - ml->acc_raw_min_abs[0]) < 0.2
          && fabs(ml->acc_raw_max_abs[1] - ml->acc_raw_min_abs[1]) < 0.2
          && fabs(ml->acc_raw_max_abs[2] - ml->acc_raw_min_abs[2]) < 0.2)) {
        ml->imu_sum_time_abs = 0.0;
        memcpy(ml->acc_raw_max_abs, acc_in, sizeof ml->acc_raw_max_abs);
        memcpy(ml->acc_raw_min_abs, acc_in, sizeof ml->acc_raw_min_abs);
        memcpy(ml->gyro_raw_max_abs, gyro_in, sizeof ml->gyro_raw_max_abs);
        memcpy(ml->gyro_raw_min_abs, gyro_in, sizeof ml->gyro_raw_min_abs);
    }
    if (ml->imu_sum_time_abs >= 2.0) {
        ml->imu_sum_time_abs = 2.0;
        still = 1;
    }
    ml->yaw_sum_time += dt;
    int yaw_still = 0;
    if (gyro_in[2] > ml->yaw_max) ml->yaw_max = gyro_in[2];
    if (gyro_in[2] < ml->yaw_min) ml->yaw_min = gyro_in[2];
    if (!(fabs(ml->yaw_max - ml->yaw_min) < 5.0 / 573.0)) {
        ml->yaw_sum_time = 0.0;
        ml->yaw_max = gyro_in[2];
        ml->yaw_min = gyro_in[2];
    }
    if (ml->yaw_sum_time >= 1.0) {
        ml->yaw_sum_time = 1.0;
        yaw_still = 1;
    }
    const double gyro_norm = sqrt(gyro_in[0] * gyro_in[0] + gyro_in[1] * gyro_in[1]
                                  + gyro_in[2] * gyro_in[2]);
    ml->gyro_ml_sum_time += dt;
    int gyro_still = 0;
    double gyro_thresh, time_thresh;
    if (p->work_mode == 1) {
        gyro_thresh = 50.0 / 573.0;
        time_thresh = 0.5;
    } else {
        gyro_thresh = 0.02617801047120419;
        time_thresh = 0.2;
    }
    if (gyro_norm > gyro_thresh)
        ml->gyro_ml_sum_time = 0.0;
    if (ml->gyro_ml_sum_time >= time_thresh) {
        ml->gyro_ml_sum_time = time_thresh;
        gyro_still = 1;
    }
    int flag;
    if (still == 1) flag = 4;
    else if (yaw_still == 1) flag = 2;
    else if (gyro_still == 1) flag = 1;
    else flag = 0;
    ml->motionless_flag_out = flag;
    return flag;
}

void ekf7_autocal_run(const double acc_in[3], const double gyro_in[3],
                      ekf7_autocal_state *st, const ekf7_params *p, double dt,
                      ekf7_autocal_result *out)
{
    const int n = (int)(p->fs * (double)p->ipack_mean);
    memset(out, 0, sizeof *out);
    out->imu_r[0] = out->imu_r[4] = out->imu_r[8] = 1.0;

    st->imu_sum_time_abs += dt;
    int still = 0;
    for (int i = 0; i < 3; i++) {
        if (acc_in[i] > st->acc_raw_max_abs[i]) st->acc_raw_max_abs[i] = acc_in[i];
        if (acc_in[i] < st->acc_raw_min_abs[i]) st->acc_raw_min_abs[i] = acc_in[i];
        if (gyro_in[i] > st->gyro_raw_max_abs[i]) st->gyro_raw_max_abs[i] = gyro_in[i];
        if (gyro_in[i] < st->gyro_raw_min_abs[i]) st->gyro_raw_min_abs[i] = gyro_in[i];
    }
    if (!(fabs(st->gyro_raw_max_abs[0] - st->gyro_raw_min_abs[0]) < p->auto_cal_gyro_max_min
          && fabs(st->gyro_raw_max_abs[1] - st->gyro_raw_min_abs[1]) < p->auto_cal_gyro_max_min
          && fabs(st->gyro_raw_max_abs[2] - st->gyro_raw_min_abs[2]) < p->auto_cal_gyro_max_min
          && fabs(st->acc_raw_max_abs[0] - st->acc_raw_min_abs[0]) < p->auto_cal_acc_max_min * 2.0
          && fabs(st->acc_raw_max_abs[1] - st->acc_raw_min_abs[1]) < p->auto_cal_acc_max_min * 2.0
          && fabs(st->acc_raw_max_abs[2] - st->acc_raw_min_abs[2]) < p->auto_cal_acc_max_min * 2.0)) {
        st->imu_sum_time_abs = 0.0;
        memcpy(st->acc_raw_max_abs, acc_in, sizeof st->acc_raw_max_abs);
        memcpy(st->acc_raw_min_abs, acc_in, sizeof st->acc_raw_min_abs);
        memcpy(st->gyro_raw_max_abs, gyro_in, sizeof st->gyro_raw_max_abs);
        memcpy(st->gyro_raw_min_abs, gyro_in, sizeof st->gyro_raw_min_abs);
    }
    if (st->imu_sum_time_abs >= 2.0) {
        st->imu_sum_time_abs = 2.0;
        still = 1;
    }
    if (still == 1) {
        st->motionless_cnt += 1.0;
        const double n3 = (double)n * 0.5;
        if (st->motionless_cnt < n3 * 1.0) {
            memset(st->acc_in_sum1, 0, sizeof st->acc_in_sum1);
            memset(st->gyro_in_sum1, 0, sizeof st->gyro_in_sum1);
            st->imu_in_cnt1 = 0.0;
        } else if (st->motionless_cnt < n3 * 2.0) {
            for (int i = 0; i < 3; i++) {
                st->acc_in_sum1[i] += acc_in[i];
                st->gyro_in_sum1[i] += gyro_in[i];
            }
            st->imu_in_cnt1 += 1.0;
        } else if (!(st->motionless_cnt < n3 * 3.0)) {
            for (int i = 0; i < 3; i++)
                out->gyro_bias[i] = st->gyro_in_sum1[i] / st->imu_in_cnt1;
            double mean_acc[3];
            for (int i = 0; i < 3; i++)
                mean_acc[i] = st->acc_in_sum1[i] / st->imu_in_cnt1;
            /* align_tilt：与 ekf7_filter.c 内部实现一致，此处内联展开 */
            double q[4] = { 1.0, 0.0, 0.0, 0.0 };
            {
                const double ax = mean_acc[0], ay = mean_acc[1], az = mean_acc[2];
                const double norm = sqrt(ax * ax + ay * ay + az * az);
                if (norm > 5.0 && norm < 14.0) {
                    const double horiz = sqrt(ax * ax + ay * ay);
                    const double tilt = atan2(horiz, az);
                    if (tilt > 0.001) {
                        const double c[3] = { ay, -ax, 0.0 };
                        const double cn = sqrt(c[0] * c[0] + c[1] * c[1]);
                        if (cn > 1e-12) {
                            const double s = sin(0.5 * tilt);
                            q[0] = cos(0.5 * tilt);
                            q[1] = (c[0] / cn) * s;
                            q[2] = (c[1] / cn) * s;
                            q[3] = (c[2] / cn) * s;
                        }
                    }
                }
            }
            ekf7_quat_to_dcm(q, out->imu_r);
            st->motionless_cnt = 0.0;
            st->run_flag = 1.0;
            out->completed = true;
        }
    } else {
        st->motionless_cnt = 0.0;
    }
}
