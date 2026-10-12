# 差分测试的对端一侧

差分黑盒测试的思路：同一份合成数据喂给**两份实现**，逐拍比对全部输出。
本仓库一侧是 `run_py.py`；对端可以是任意等价实现（另一种语言/另一次重构），
只要按相同的调用链和输出格式驱动即可。

## 步骤

1. `python3 ../gen_data.py` 生成确定性的合成数据（`data_1000.csv` / `data_3000.csv`）；
2. 对端：写一个驱动程序，按「输入一阶低通(k=0.1) → 前10s静止自标定 →
   静止检测 → 滤波器步进」的调用链逐拍运行对端实现，输出
   `k,q0..q3,roll,pitch,yaw,vx..vz,px..pz,flag` CSV；
3. `PYTHONPATH=.. python3 ../run_py.py data_1000.csv out_py.csv 9 200` 跑本仓库实现；
4. `python3 ../compare.py out_peer.csv out_py.csv` 逐拍比对。

对端驱动核心（伪代码）：

```text
param 按 README 参数表初始化（fs=200, work_mode=1, amode=9/6）
acc_filt/gyro_filt 一阶低通: x ← (1−0.1)x + 0.1·input
前 10s 运行静止自标定；完成时: 安装矩阵←调平矩阵, 陀螺零偏←静置均值, 滤波器重启
每拍: flag = 静止检测(acc, gyro); filter.step(acc, gyro, mag, dt, flag)
记录 q/欧拉角/速度/位移/flag
```

验收标准：最大绝对差 < 1e-9（我们实测 5.1~8.5e-15，差异来自矩阵库求和顺序），
静止判别序列逐拍一致。
