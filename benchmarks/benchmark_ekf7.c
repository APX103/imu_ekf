/* ekf7 C99 单帧处理延时基准测试。
 *
 * 编译（在仓库根目录执行）：
 *     cc -O2 -std=c99 -o /tmp/bench_ekf7_c \
 *         benchmarks/benchmark_ekf7.c \
 *         c/ekf7_math.c c/ekf7_jacobian.c c/ekf7_filter.c \
 *         c/ekf7_motion.c c/ekf7_pipeline.c -lm
 *     /tmp/bench_ekf7_c
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "../c/ekf7.h"

#define FS 200.0
#define WARMUP 100
#define N 200000

static double frand(void) { return (double)rand() / RAND_MAX; }

int main(void) {
    srand(42);

    for (int amode = 6; amode <= 9; amode += 3) {
        ekf7_pipeline pipe;
        ekf7_pipeline_init(&pipe, FS, 1, amode);
        ekf7_output out;
        volatile double sink = 0.0;

        for (int i = 0; i < WARMUP; i++) {
            double acc[3] = {frand(), frand(), 9.0 + frand()};
            double gyro[3] = {frand() * 0.1, frand() * 0.1, frand() * 0.1};
            double mag[3] = {frand(), frand(), -frand()};
            ekf7_pipeline_calculate(&pipe, acc, gyro, amode == 9 ? mag : NULL, &out);
        }

        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        for (int i = 0; i < N; i++) {
            double acc[3] = {frand(), frand(), 9.0 + frand()};
            double gyro[3] = {frand() * 0.1, frand() * 0.1, frand() * 0.1};
            double mag[3] = {frand(), frand(), -frand()};
            ekf7_pipeline_calculate(&pipe, acc, gyro, amode == 9 ? mag : NULL, &out);
            sink += out.quat[0] + out.vel[0] + out.pos[0];
        }
        clock_gettime(CLOCK_MONOTONIC, &t1);

        double elapsed = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
        double per_frame_us = elapsed / N * 1e6;
        double fps = N / elapsed;
        double cpu_200hz = (elapsed / (N / FS)) * 100;

        printf("amode=%d (sink=%g):\n", amode, sink);
        printf("  elapsed      : %.3f s\n", elapsed);
        printf("  per frame    : %.3f us\n", per_frame_us);
        printf("  fps          : %.0f\n", fps);
        printf("  CPU @ 200Hz  : %.4f%%\n", cpu_200hz);
        printf("\n");
    }
    return 0;
}
