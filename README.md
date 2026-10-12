# quat-zupt-ekf

七状态全量 EKF 姿态解算的 Python 实现（四元数 + 导航系速度，ZUPT 零速观测 + 磁航向观测），
含静止检测、陀螺零偏在线估计与安装矩阵自动标定。按职责拆分为 `ekf7/` 包
（11 个模块，最大 273 行），仅依赖 numpy，可直接用也可 `pip install`。

## 这是什么

一个 IMU（加速度计 + 陀螺 ± 磁力计）姿态解算滤波器的 Python 实现与验证工程：

- **状态**：`x = [q0, q1, q2, q3, vE, vN, vU]`（四元数 4 + ENU 导航系速度 3），
  7×7 协方差直接在全量状态上传播（SINS-EKF 全量式，非现代误差状态式）；
- **预测**：陀螺角增量 → 四元数右乘传播；比力旋转到导航系积分减重力；
- **观测 1（ZUPT）**：静止时速度为零的伪观测，`H=[0₃ₓ₄, I₃]`，
  量测方差/门限按四级运动状态自适应（R 从 10⁴ 到 10⁻⁴）；
- **观测 2（磁航向，可选）**：`h(q) = atan2((R·m)_E, (R·m)_N) − D_ec` 一维观测，
  解析雅可比 + 卡方式门控 + 静止超时重对准；
- **配套机制**：三级静止检测器、静止分窗陀螺零偏估计、安装矩阵（调平）自标定、
  三段式粗对准（静置均值 → 加计调平 → 磁航向）。

## 验证

实现经两层验证：

1. **行为验证**（`examples/demo_synthetic.py`）：合成 200Hz 数据（初始倾斜 5°/−3°、陀螺零偏、
   往复偏航运动）下，横滚/俯仰稳态误差 < 0.05°，九轴模式 yaw 收敛（磁航向提供绝对参考），
   六轴模式 yaw 纯陀螺积分缓漂——符合该类算法的可观测性设计。
2. **差分黑盒验证**（`test/`）：同一份 CSV 分别喂给两份实现逐拍比对，
   三个场景（1000/3000 拍 × 九轴/六轴）最大绝对差 5.1~8.5 × 10⁻¹⁵（机器精度），
   静止判别序列完全一致。差分测试还抓出并修正过一处初始化默认值偏差。
3. **C99 移植验证**（`c/`）：同一差分方法验证，三场景最大绝对差
   3.8 × 10⁻¹⁰ ~ 3.2 × 10⁻⁸（求逆方式与求和顺序差异），静止判别完全一致。
   C 版零依赖、无动态分配，供嵌入式固件直接集成。

## 快速开始

```bash
# 方式一：免安装直接跑（仓库根目录）
PYTHONPATH=. python3 examples/demo_synthetic.py

# 方式二：安装为包
pip install . && python3 -c "import ekf7; print(ekf7.__version__)"
```

```python
from ekf7 import ImuPipeline

pipe = ImuPipeline(fs=200.0, work_mode=1, amode=9)   # amode: 6=六轴(无磁) 9=九轴
for acc, gyro, mag in samples:                        # 体坐标系, m/s², rad/s
    out = pipe.calculate(acc, gyro, mag)
    out["quat"]        # [w,x,y,z]
    out["euler_rad"]   # [roll, pitch, yaw]
    out["vel"]         # 导航系速度
    out["pos"]         # 短时相对位移（静止即清零，非 PDR 级）
    out["motionless"]  # 静止标志 0/1/2/4
```

## 已知局限

1. **持续匀速旋转会被静止检测器误判为静止**（峰峰值+锚点重置式检测的固有缺陷），
   导致 yaw 冻结、零偏被污染——往复运动场景无碍，匀速转弯场景慎用；
2. 全量四元数协方差传播在大机动下数值稳定性弱于现代 ES-EKF；
3. 位置输出仅是短时相对演示量；
4. 默认重力常量为 9.7955 m/s²（按当地纬度修改 `Parameters.g0`）。

## 目录

```
ekf7/                       算法包，每个模块一个职责（按数据流向读）
├── math_utils.py           基础工具：角度包裹 / 叉积 / 标量低通      30 行
├── quaternion.py           四元数运算（wxyz，含小角度级数）          89 行
├── parameters.py           全部可调参数（默认 = 200Hz 档整定）       45 行
├── jacobian.py             F7 / Q7 / 磁航向梯度的解析雅可比        213 行
├── alignment.py            粗对准：调平 / 磁航向 + 初始状态与协方差  66 行
├── prediction.py           机械编排递推 + 协方差传播                 51 行
├── observations.py         ZUPT 与磁航向观测更新                     63 行
├── motion.py               三级静止检测器                            87 行
├── auto_calibration.py     静止自标定（安装矩阵 + 陀螺零偏）         91 行
├── navigation_filter.py    主滤波器：初始化状态机 + 4 拍观测调度    273 行
├── pipeline.py             端到端管线 ImuPipeline                    76 行
└── __init__.py             公共 API 导出                             46 行

examples/demo_synthetic.py  合成数据行为验证
test/                       差分测试（数据生成 / 驱动 / 比对 / 对端搭建说明）
c/                          C99 移植（零依赖，可直接进固件；差分验证 3.8e-10~3.2e-8）
pyproject.toml              包定义，可 pip install
```

## License

MIT — 见 [LICENSE](LICENSE)。
