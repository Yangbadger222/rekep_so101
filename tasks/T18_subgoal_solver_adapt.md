# T18 - subgoal_solver 抓取 cost 适配

- **阶段**: 3 - ReKep 接口适配
- **里程碑**: M6
- **依赖**: T13
- **状态**: [x] 已完成（axis explicit / weight adjusted）

## 目标

修改 `subgoal_solver.py` 中的抓取质量 cost，适配 SO-101 的 EEF 坐标系朝向。

## 背景

`subgoal_solver.py:71-76` 写死了 top-down grasp 偏好：

```python
preferred_dir = np.array([0, 0, -1])
grasp_cost = -np.dot(opt_pose_homo[:3, 0], preferred_dir) + 1
```

假设 **EE 的 X 轴指向夹爪开合方向**（Fetch/Franka 习惯）。SO-101 的 `gripper_frame_link` 可能不同。

## 详细步骤

### 18.1 确认 SO-101 EEF 坐标系

从 T13 获取 SO-101 EEF 坐标系信息：

- EEF X 轴方向 = 相对 `gripper_link` 为 `[-1, 0, 0]`
- EEF Y 轴方向 = 相对 `gripper_link` 为 `[0, 1, 0]`
- EEF Z 轴方向 = 相对 `gripper_link` 为 `[0, 0, -1]`
- 夹爪开合方向 = 由 gripper/jaw 几何决定，不直接作为 approach axis
- "指向物体"方向 = EEF +X（与 `Main._execute_grasp_action()` 的 `grasp_depth` 推进方向一致）

### 18.2 修改 grasp cost

**文件**: `subgoal_solver.py`  
**行号**: 71-76

```python
# 现状（假设 X 轴指向夹爪开合方向）
grasp_cost = -np.dot(opt_pose_homo[:3, 0], preferred_dir) + 1

# 如果 SO-101 EE 的"指向物体"方向是 Z 轴：
grasp_cost = -np.dot(opt_pose_homo[:3, 2], preferred_dir) + 1

# 如果是 Y 轴：
grasp_cost = -np.dot(opt_pose_homo[:3, 1], preferred_dir) + 1
```

### 18.3 调整 grasp cost 权重

```python
# 现状
grasp_cost_weight = 10.0

# 如果需要侧面抓取（垃圾躺在桌面），降低权重
grasp_cost_weight = 2.0  # 让 IK 反推决定姿态
```

### 18.4 调整 grasp_depth

```python
# 现状（Fetch finger 较长）
grasp_depth: 0.10

# 改为（SO-101 finger 长度 ~30mm）
grasp_depth: 0.04
```

否则 grasp pose 会沉到桌面以下。

### 18.5 调整 preferred_dir

对于桌面垃圾任务，top-down 抓取通常合理：

```python
preferred_dir = np.array([0, 0, -1])  # 从上往下抓
```

如果 SO-101 的工作空间限制导致 top-down 不可行，可以改为侧面抓取。

## 验收标准

- [x] grasp cost 使用正确的 EEF 轴
- [x] grasp_depth 适配 SO-101 finger 长度
- [ ] subgoal_solver 能为桌面垃圾找到合理的 grasp pose（T24 端到端 smoke 验证）
- [ ] grasp pose 不沉入桌面以下（T24 端到端 smoke 验证）
- [ ] 优化结果在 SO-101 工作空间内（T24 端到端 smoke 验证）

## 涉及文件

```
subgoal_solver.py
configs/config.yaml（grasp_depth）
```

## 备注

- 具体修改依赖 T13 的 EEF 坐标系确认结果
- 如果不确定轴向，可以在 Isaac Sim 中可视化 EEF frame
- grasp_depth 太大会导致 approach 动作超出工作空间
- grasp_depth 太小会导致夹爪没合到物体就宣布完成

## 完成记录

- `subgoal_solver.py` 改动：
  - 新增显式常量：
    - `GRASP_APPROACH_AXIS = 0`
    - `GRASP_PREFERRED_DIR = np.array([0, 0, -1])`
    - `GRASP_COST_WEIGHT = 2.0`
  - grasp stage cost 改为读取 `opt_pose_homo[:3, GRASP_APPROACH_AXIS]`，
    并在 debug dict 记录 `grasp_axis='eef_x'`。
  - 权重从硬编码 `10.0` 降到 `2.0`，让 SO-101 小工作空间内的 IK / 约束有更多调整余地。
- `configs/config.yaml` 中 `grasp_depth=0.04` 已在 T16 完成。
- 验证命令：
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile subgoal_solver.py`
  - `/home/badger/anaconda3/envs/omnigibson/bin/python - <<'PY' ... import subgoal_solver ... PY`
- 结果：
  - 编译通过。
  - 常量导入检查输出：`GRASP_AXIS=0 WEIGHT=2.0 DIR=[0, 0, -1]`。
