# T11 - 关节控制验证

- **阶段**: 2 - Robot 封装
- **里程碑**: M5
- **依赖**: T10
- **状态**: [x] 已完成（headless / raw position JointController）

## 目标

验证 SO-101 robot 子类的各个 arm joint 能正常受控运动。

## 详细步骤

### 11.1 创建关节测试脚本

创建 `scripts/m5_test_so101_robot.py`：

```python
# 加载 OmniGibson 环境，使用 SO101 robot
# 逐个关节测试 ±0.5 rad 运动
# 记录关节位置响应
```

### 11.2 逐关节测试

对每个 arm joint 执行：

1. 读取当前关节位置
2. 发送目标位置 = 当前 + 0.5 rad
3. 运行 100 step
4. 读取最终关节位置
5. 检查是否接近目标

测试关节：
- [x] shoulder_pan: ±0.5 rad
- [x] shoulder_lift: ±0.5 rad
- [x] elbow_flex: ±0.5 rad
- [x] wrist_flex: ±0.5 rad
- [x] wrist_roll: ±0.5 rad

### 11.3 检查关节限位

- [x] 关节不超过 URDF 定义的 limit
- [x] 接近 limit 时不爆炸
- [x] 从 limit 附近能正常返回

### 11.4 连续运动稳定性

- 连续 step 1000 帧
- 检查无漂移（关节在零力矩下保持位置）
- 检查无 NaN

### 11.5 controller 参数调优

如果关节响应不好：
- `kp`（比例增益）太大 → 硬冲、震荡
- `kp` 太小 → 到不了目标
- `damping_ratio` 建议 0.8
- 当前 URDF `effort=10` 可能太大，如有问题调小

## 验收标准

- [x] 所有 5 个 arm joint 能独立控制
- [x] 关节响应合理（误差 < 0.05 rad）
- [x] 无关节限位违反
- [x] 1000 step 连续运行无漂移
- [x] 无物理爆炸或抖动

## 输出文件

```
scripts/m5_test_so101_robot.py
```

## 完成记录

- 新增 `scripts/m5_test_so101_robot.py`，加载 T10 的 `SO101` robot，并显式使用
  raw position `JointController`：
  - `action_normalize=False`
  - `arm_0`: 5 维绝对关节目标
  - `gripper_0`: 1 维绝对夹爪目标
- 验证命令：
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m5_test_so101_robot.py`
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m5_test_so101_robot.py --headless --steps-per-target 100 --hold-steps 1000 --log-every 250`
- 结果：
  - 5 个 arm joints 各自 `+0.5/-0.5 rad` 均成功到达目标。
  - 每次最终误差均为 `0.00000 rad`（脚本输出精度）。
  - 1000 step hold：`joint_drift_rad=0.000000`，`base_drift_m=0.000000`。
  - 无 NaN、无 limit violation、无物理爆炸。

## 备注

- `effort=10` 是占位值，如果发现关节"硬冲"，调小 controller `kp` 到 50
- SO-101 整机仅 ~0.5kg，关节驱动力不需要太大
- 如果关节完全不响应，检查 controller config 中的 joint index 映射
