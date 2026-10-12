/* test_runner.c — 差分验证驱动：CSV 进 CSV 出，格式与 test/run_py.py 完全一致。
 * 用法: ./ekf7_test <in.csv> <out.csv> <amode> <fs>
 * 输入列: t,ax,ay,az,gx,gy,gz,mx,my,mz    输出列: k,q0..q3,roll,pitch,yaw,vx..vz,px..pz,flag */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ekf7.h"

int main(int argc, char **argv)
{
    if (argc < 5) {
        fprintf(stderr, "usage: %s <in.csv> <out.csv> <amode> <fs>\n", argv[0]);
        return 1;
    }
    FILE *fin = fopen(argv[1], "r");
    if (!fin) { perror(argv[1]); return 1; }
    FILE *fout = fopen(argv[2], "w");
    if (!fout) { perror(argv[2]); fclose(fin); return 1; }
    const int amode = atoi(argv[3]);
    const double fs = atof(argv[4]);

    ekf7_pipeline pipe;
    ekf7_pipeline_init(&pipe, fs, 1, amode); /* work_mode=1，与验证数据集一致 */

    fprintf(fout, "k,q0,q1,q2,q3,roll,pitch,yaw,vx,vy,vz,px,py,pz,flag\n");
    char line[512];
    long k = 0;
    if (fgets(line, sizeof line, fin) == NULL) return 1; /* 表头 */
    while (fgets(line, sizeof line, fin)) {
        double v[10];
        int got = 0;
        char *cur = line;
        for (int i = 0; i < 10; i++) {
            char *end;
            v[i] = strtod(cur, &end);
            if (end == cur) break;
            cur = end + 1; /* 跳过逗号 */
            got++;
        }
        if (got < 10) continue;
        ekf7_output o;
        ekf7_pipeline_calculate(&pipe, &v[1], &v[4], &v[7], &o);
        fprintf(fout, "%ld,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d\n",
                k++, o.quat[0], o.quat[1], o.quat[2], o.quat[3],
                o.euler_rad[0], o.euler_rad[1], o.euler_rad[2],
                o.vel[0], o.vel[1], o.vel[2],
                o.pos[0], o.pos[1], o.pos[2], o.motionless);
    }
    fclose(fin); fclose(fout);
    fprintf(stderr, "C(本移植): %ld 拍 -> %s (amode=%d, fs=%g)\n", k, argv[2], amode, fs);
    return 0;
}
