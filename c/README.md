# c/ — C99 移植（可直接进固件）

Python 参考实现（`../ekf7/`）的逐行 C 移植。**零依赖、无动态分配、单文件编译**，
设计目标就是被别的工程直接抄走或扩展。

## 文件

| 文件 | 职责 | 对应 Python |
|---|---|---|
| `ekf7.h` | 全部公共 API：参数/状态结构体、滤波器、管线 | 包接口 |
| `ekf7_math.c` | 四元数运算、wrap、标量低通 | quaternion.py + math_utils.py |
| `ekf7_jacobian.c` | F7 / Q7 / 磁航向梯度（解析雅可比） | jacobian.py |
| `ekf7_filter.c` | 对准/预测/观测/主滤波循环 | alignment + prediction + observations + navigation_filter |
| `ekf7_motion.c` | 三级静止检测、静止自标定 | motion.py + auto_calibration.py |
| `ekf7_pipeline.c` | 端到端管线（低通→自标定→检测→滤波） | pipeline.py |
| `test_runner.c` | 差分验证驱动（CSV 进出，格式与 run_py.py 一致） | — |

## 用法

```c
#include "ekf7.h"

ekf7_pipeline pipe;
ekf7_pipeline_init(&pipe, 200.0, /*work_mode=*/1, /*amode=*/9); // amode: 6=六轴 9=九轴

ekf7_output out;
ekf7_pipeline_calculate(&pipe, acc, gyro, mag, &out);  // 每拍一次; mag 可传 NULL
// out.quat[4] / out.euler_rad[3] / out.vel[3] / out.pos[3] / out.motionless
```

需要更细粒度控制（自定义调度/观测）时，直接用 `ekf7_filter_*` / `ekf7_motion_detect`
等底层 API，结构体字段全部公开。

## 差分验证（vs 参考实现原始输出，2026-10-12）

```bash
make
./ekf7_test ../test/data_1000.csv /tmp/out_c.csv 9 200
python3 ../test/compare.py <参考输出.csv> /tmp/out_c.csv
```

| 场景 | 拍数 | 最大绝对差 | 静止判别序列 |
|---|---|---|---|
| amode=9 | 1000 | 3.8e-10 | 完全一致 |
| amode=9 | 3000 | 1.1e-8 | 完全一致 |
| amode=6 | 3000 | 3.2e-8 | 完全一致 |

差异来自 3×3 求逆方式（伴随式 vs LU）与求和顺序，属同一算法的浮点噪声量级
（远低于任何物理显著性阈值）。

## 嵌入式集成注意

1. **精度**：整体用 `double` 实现。目标 MCU 只有单精度 FPU（Cortex-M4F /
   ESP32-S3）时可全局替换为 `float`（含 `ekf7_calc_*` 的中间变量；算法自带
   对称化/对角截断/四元数归一化等数值卫生），但**必须重新跑差分验证**评估
   精度损失。算力参考：全算法 ~1-2 kFLOP/拍 @200Hz，M4F @170MHz 占用约 1%。
2. **已知行为怪癖保留**（与参考实现一致，见 ../README.md「已知局限」）：
   峰峰值式静止检测在**匀速旋转下会误判静止**（yaw 冻结+零偏污染）。产品化建议
   在 `ekf7_motion_detect` 里把峰峰值判据换成瞬时阈值（‖ω‖ 与 |‖acc‖−g|）。
3. `ekf7_filter_reset()` 不清参数（参数嵌在结构体内但受保护）——自标定完成会
   调它，别手工 memset 整个 filter 结构体，除非之后重新 init。
