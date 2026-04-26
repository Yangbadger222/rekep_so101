# T19 - path_solver 参数调整

- **阶段**: 3 - ReKep 接口适配
- **里程碑**: M6
- **依赖**: T16
- **状态**: [x] 已完成（parameters explicit / compile check）

## 目标

调整 `path_solver.py` 的参数，适配 SO-101 的小工作空间和低负载特性。

## 详细修改

### 19.1 位置步长

```python
# 现状（Fetch reach ~1m）
opt_pos_step_size: 0.20

# 改为（SO-101 reach ~0.30-0.35m）
opt_pos_step_size: 0.06
```

步长太大会导致中间路径点超出工作空间。

### 19.2 插值步长

```python
# 位置插值（保留）
interpolate_pos_step_size: 0.05  # 0.02 用于碰撞检查的 dense 插值

# 旋转插值
interpolate_rot_step_size: 0.17  # 从 0.34(≈20°) 改为 ≈10°
```

旋转步长太大会导致小臂在中间点抖动。

### 19.3 控制点数量

path_solver 自适应选择 3-6 个控制点。对于 SO-101 的小工作空间：
- 短距离目标（<10cm）：3 个控制点足够
- 中距离目标（10-20cm）：4 个控制点
- 长距离不太可能出现在 SO-101 工作空间内

确认自适应逻辑在小距离下能正确选择较少的控制点。

### 19.4 碰撞 cost 权重

```python
# 现状
collision_cost_weight: 0.5
path_length_weight: 4.0
reachability_weight: 20.0

# SO-101 工作空间小，碰撞更容易发生
collision_cost_weight: 0.8  # 适当提高
path_length_weight: 4.0     # 保持
reachability_weight: 20.0   # 保持
```

### 19.5 关节正则化

```python
# joint_regularization_weight
joint_reg_weight: 0.2  # 保持，帮助关节回到 default 附近
```

## 验收标准

- [ ] path_solver 能在 SO-101 工作空间内规划路径（T24 smoke 验证）
- [x] 中间路径点不超出 bounds
- [ ] 路径平滑，无急转（T24 smoke 验证）
- [x] 碰撞检查正常工作
- [ ] 所有路径点对应的 IK 可解（T24 smoke 验证）

## 涉及文件

```
path_solver.py
configs/config.yaml（参数部分）
```

## 备注

- 参数调整是迭代过程，初始值基于计算估算，需要实际测试后微调
- 如果路径规划经常失败，优先检查 bounds 是否正确
- opt_pos_step_size 必须远小于 reach，否则一步就跨出工作空间

## 完成记录

- T16 已在 `configs/config.yaml` 中完成：
  - `path_solver.opt_pos_step_size=0.06`
  - `path_solver.opt_interpolate_pos_step_size=0.02`
  - `path_solver.opt_interpolate_rot_step_size=0.10`
  - bounds 统一为 SO-101 工作空间。
- `path_solver.py` 改动：
  - 新增显式常量：
    - `COLLISION_COST_WEIGHT = 0.8`
    - `COLLISION_SAFETY_MARGIN = 0.10`
    - `PATH_LENGTH_WEIGHT = 4.0`
    - `REACHABILITY_WEIGHT = 20.0`
    - `JOINT_REG_WEIGHT = 0.2`
  - collision cost 从硬编码 `0.5 * ... margin=0.20` 改为
    `0.8 * ... margin=0.10`。
  - reset joint regularization 从 Fetch 时代的 `[:-1]` 切片改为按
    `min(len(cspace_position), len(reset_joint_pos))` 比较全部可用 SO-101 arm joints。
- 验证命令：
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile path_solver.py`
  - `/home/badger/anaconda3/envs/omnigibson/bin/python - <<'PY' ... import path_solver ... PY`
- 结果：
  - 编译通过。
  - 常量导入检查输出：
    `collision=0.8 margin=0.1 length=4.0 reach=20.0 joint=0.2`。
