/* ekf7_math.c — 四元数与基础数学（对应 ../ekf7/quaternion.py + math_utils.py） */
#include <math.h>
#include <string.h>

#include "ekf7.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double ekf7_wrap_pi(double angle)
{
    angle = fmod(angle, 2.0 * M_PI);
    if (angle > M_PI)
        angle -= 2.0 * M_PI;
    else if (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

void ekf7_kfilter_inplace(double *prev, const double *x, double k, int n)
{
    const double km1 = 1.0 - k;
    for (int i = 0; i < n; i++)
        prev[i] = prev[i] * km1 + x[i] * k;
}

void ekf7_quat_multiply(const double a[4], const double b[4], double out[4])
{
    const double w = a[0], x = a[1], y = a[2], z = a[3];
    const double w2 = b[0], x2 = b[1], y2 = b[2], z2 = b[3];
    out[0] = w * w2 - x * x2 - y * y2 - z * z2;
    out[1] = w * x2 + x * w2 + y * z2 - z * y2;
    out[2] = w * y2 - x * z2 + y * w2 + z * x2;
    out[3] = w * z2 + x * y2 - y * x2 + z * w2;
}

void ekf7_quat_normalized(const double q[4], double out[4])
{
    const double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-12) {
        out[0] = 1.0; out[1] = out[2] = out[3] = 0.0;
        return;
    }
    const double inv = 1.0 / n;
    for (int i = 0; i < 4; i++)
        out[i] = q[i] * inv;
}

void ekf7_quat_to_dcm(const double q[4], double r[9]) /* 行主序，体->导航 */
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double w2 = w * w, x2 = x * x, y2 = y * y, z2 = z * z;
    const double xy = x * y, xz = x * z, yz = y * z;
    const double wx = w * x, wy = w * y, wz = w * z;
    r[0] = w2 + x2 - y2 - z2;  r[1] = 2.0 * (xy - wz);  r[2] = 2.0 * (xz + wy);
    r[3] = 2.0 * (xy + wz);    r[4] = w2 - x2 + y2 - z2;r[5] = 2.0 * (yz - wx);
    r[6] = 2.0 * (xz - wy);    r[7] = 2.0 * (yz + wx);  r[8] = w2 - x2 - y2 + z2;
}

void ekf7_quat_from_rv(const double rv[3], double q[4])
{
    const double n2 = rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2];
    double w, s;
    if (n2 < 1e-8) {
        w = 1.0 - n2 * (0.125 - n2 / 384.0);
        s = 0.5 - n2 * (1.0 / 48.0 - n2 / 3840.0);
    } else {
        const double n = sqrt(n2);
        w = cos(n * 0.5);
        s = sin(n * 0.5) / n;
    }
    q[0] = w; q[1] = s * rv[0]; q[2] = s * rv[1]; q[3] = s * rv[2];
}

void ekf7_quat_from_euler(double phi, double theta, double psi, double q[4])
{
    const double cr = cos(phi * 0.5), sr = sin(phi * 0.5);
    const double cp = cos(theta * 0.5), sp = sin(theta * 0.5);
    const double cy = cos(psi * 0.5), sy = sin(psi * 0.5);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

void ekf7_quat_to_euler(const double q[4], double rpy[3])
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    double num = 2.0 * (x * z - w * y);
    if (num < -1.0) num = -1.0;
    if (num > 1.0) num = 1.0;
    rpy[0] = atan2(2.0 * (y * z + w * x), w * w - x * x - y * y + z * z);
    rpy[1] = -asin(num);
    rpy[2] = atan2(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z);
}
