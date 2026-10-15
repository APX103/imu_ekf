# ekf7 性能基准测试报告

> 测试日期：2026-10-15  
> 测试目的：给出 ekf7 在当前开发机（Apple M4）上的单帧处理延时、等效帧率，以及 200Hz 实时场景下的 CPU 占用估算。

---

## 1. 测试环境

| 项目 | 值 |
|------|-----|
| 机型 | Mac mini (Apple M4) |
| OS | macOS 15.x (Darwin 25.6.0, ARM64) |
| Python | 3.9.6 |
| NumPy | 1.26.4（以实际运行输出为准） |
| C 编译器 | Apple clang / cc |
| C 编译选项 | `-O2 -std=c99` |

---

## 2. 测试方法

- **Python**：直接调用 `ImuPipeline.calculate()`，预热 100 帧后计时 20,000 帧。
- **C99**：直接调用 `ekf7_pipeline_calculate()`，预热 100 帧后计时 200,000 帧。
- 输入为固定/轻微随机化的 IMU 数据，避免编译器过度优化。
- C 代码使用 `volatile double sink` 消费输出，防止整段循环被优化掉。

脚本位置：

- Python：`benchmarks/benchmark_ekf7.py`
- C99：`benchmarks/benchmark_ekf7.c`

---

## 3. 实测结果

### 3.1 Python 实现

| 模式 | 单帧耗时 | 等效帧率 | 200Hz CPU 占用 |
|------|---------|---------|---------------|
| amode=6（六轴无磁） | **~32.8 µs** | ~30,500 fps | **~0.66%** |
| amode=9（九轴带磁） | **~35.4 µs** | ~28,200 fps | **~0.71%** |

### 3.2 C99 实现

| 模式 | 单帧耗时 | 等效帧率 | 200Hz CPU 占用 |
|------|---------|---------|---------------|
| amode=6（六轴无磁） | **~0.16–0.22 µs** | ~4.6–6.2 Mfps | **~0.003–0.004%** |
| amode=9（九轴带磁） | **~0.16–0.22 µs** | ~4.6–6.2 Mfps | **~0.003–0.004%** |

> 注：C99 版本在 M4 上极快，九轴与六轴差距很小，是因为 4 拍轮转调度里磁航向更新只占 1/4 节拍，且 M4 的超标量流水线对随机分支数据非常友好。

---

## 4. 延时分析

### 4.1 单帧处理延时

- **Python**：约 30~40 µs，相对于 200Hz 的 5ms 采样间隔可忽略。
- **C99**：约 0.15~0.20 µs，几乎无感。

### 4.2 算法端到端延时

| 来源 | 延时 | 说明 |
|------|------|------|
| 初始化阶段 | **4s** | 0~4s 为强制单位四元数 → 静置累计 → 粗对准，4s 后才进入正式 EKF |
| 观测调度 | **≤5ms** | 每 5ms 才做一次观测更新（协方差 / 磁航向 / ZUPT） |
| 静止检测 flag=4 | **~2s** | 完全静止需要三轴陀螺 + 三轴加计连续 2s 峰峰值未突破阈值 |
| 静止检测 flag=2 | **~1s** | 仅偏航静止需要 z 轴陀螺连续 1s 峰峰值未突破阈值 |

因此：

- **纯 processing latency**：Python 几十 µs，C 亚 µs。
- **从静止发生到 ZUPT 生效的最坏延时**：约 **2s + 5ms**。
- **从上电到稳定输出的 startup 延时**：约 **4s**。

---

## 5. 理论估算交叉验证

README 中给出全算法约 **1~2 kFLOP/拍**。

- 200Hz 下总算力需求：200~400 kFLOP/s。
- Apple M4 峰值算力在数十 GFLOP/s 量级，纯算力角度 CPU 占用应 < 0.01%。
- 实测 C99 ~0.003%，吻合。
- Python 慢约 200 倍，主要来自 NumPy 小矩阵开销、函数调用、对象访问，而非算法本身复杂。

对于嵌入式场景（如 README 提到的 Cortex-M4F @170MHz）：

- 1~2 kFLOP/拍 × 200Hz = 200~400 kFLOP/s。
- M4F 单精度 FPU 约 170 MFLOP/s 峰值。
- 考虑内存访问、分支、标量运算后，README 给出的占用约 **1%** 是合理的工程估算。

---

## 6. 复现步骤

### Python

```bash
cd /Volumes/ssd/main_link/work/quat-zupt-ekf
python3 benchmarks/benchmark_ekf7.py
```

### C99

```bash
cd /Volumes/ssd/main_link/work/quat-zupt-ekf
cc -O2 -std=c99 -o /tmp/bench_ekf7_c \
    benchmarks/benchmark_ekf7.c \
    c/ekf7_math.c c/ekf7_jacobian.c c/ekf7_filter.c \
    c/ekf7_motion.c c/ekf7_pipeline.c -lm
/tmp/bench_ekf7_c
```

---

## 7. 结论

- **200Hz 实时性**：Python 和 C99 都完全满足，且留有巨大余量。
- **低延时场景**：C99 单帧处理延时约 0.2 µs 量级；算法本身的 startup/静止检测延时是主要瓶颈，而非 CPU。
- **嵌入式可行性**：C99 版本在低端 MCU 上也能轻松跑 200Hz，与 README 的估算一致。
