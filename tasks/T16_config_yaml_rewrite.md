# T16 - config.yaml 工作空间与控制器重写

- **阶段**: 3 - ReKep 接口适配
- **里程碑**: M6
- **依赖**: T09
- **状态**: [x] 已完成（YAML load / static check）

## 目标

重写 `configs/config.yaml` 中的工作空间 bounds、robot 控制器配置和相关优化参数，适配 SO-101。

## 详细修改清单

### 16.1 工作空间 bounds 重算

**行号**: 第 9-10 行

```yaml
# 现状（Fetch 工作空间）
bounds_min: [-0.45, -0.75, 0.698]
bounds_max: [ 0.10,  0.60, 1.200]

# 改为（SO-101 reach ~0.30-0.35m，假设 base 在 (0,0,0.75)）
bounds_min: [-0.30, -0.30, 0.75]
bounds_max: [ 0.30,  0.30, 1.10]
```

所有引用 bounds 的地方都要同步：
- `main` section
- `env` section
- `keypoint_proposer` section
- `visualizer` section

### 16.2 Robot 控制器配置

**行号**: 第 32-55 行

```yaml
# 现状（Fetch 控制器）
robot:
  type: Fetch
  controller_config:
    base: DifferentialDriveController
    arm_0: OperationalSpaceController
    gripper_0: MultiFingerGripperController

# 改为（SO-101，无 base）
robot:
  type: SO101
  controller_config:
    arm_0:
      name: OperationalSpaceController
      kp: 50           # 降低，避免硬冲
      damping_ratio: 0.8
    gripper_0:
      name: MultiFingerGripperController
      mode: binary     # 开/关
  grasping_mode: assisted
```

### 16.3 优化参数调整

```yaml
# grasp_depth: SO-101 finger 长度 ~30mm
grasp_depth: 0.04  # 从 0.10 缩小

# interpolation 步长
interpolate_pos_step_size: 0.05  # 保留
interpolate_rot_step_size: 0.17  # 从 0.34(≈20°) 缩小到 ≈10°，减少抖动
```

### 16.4 subgoal_solver 参数

```yaml
subgoal_solver:
  opt_pos_step_size: 0.06  # 从 0.20 缩小，匹配 SO-101 reach
```

### 16.5 keypoint_proposer 参数

```yaml
keypoint_proposer:
  min_dist_bt_keypoints: 0.03  # 从 0.06 缩小，否则小垃圾关键点被合并
  max_mask_ratio: 0.5          # 保留
```

### 16.6 physics 参数

```yaml
og_sim:
  physics_frequency: 120  # 从 60 提高，避免 0.5kg 小臂震荡
  action_frequency: 30    # 对应调整
```

## 对照表

| 参数 | Fetch 原值 | SO-101 新值 | 原因 |
|------|-----------|-------------|------|
| bounds_min | [-0.45,-0.75,0.698] | [-0.30,-0.30,0.75] | reach 缩小 |
| bounds_max | [0.10,0.60,1.200] | [0.30,0.30,1.10] | reach 缩小 |
| grasp_depth | 0.10 | 0.04 | finger 更短 |
| interpolate_rot_step_size | 0.34 | 0.17 | 减少小臂抖动 |
| opt_pos_step_size | 0.20 | 0.06 | 匹配 reach |
| min_dist_bt_keypoints | 0.06 | 0.03 | 小垃圾检测 |
| physics_frequency | 60 | 120 | 轻小臂稳定性 |
| kp (controller) | 高 | 50 | 避免硬冲 |

## 验收标准

- [x] bounds 在 SO-101 工作空间范围内
- [x] robot type 指向 SO101
- [x] controller config 无 DifferentialDrive
- [x] grasping_mode 设为 assisted
- [x] 所有 bounds 引用一致
- [x] config.yaml 能被正确加载（YAML 无语法错误）

## 涉及文件

```
configs/config.yaml
```

## 备注

- bounds 值需要根据实际 IK 测试结果微调
- controller kp/damping 需要根据物理稳定性测试调优
- physics_frequency 提高会降低仿真速度，如果太慢再降回

## 完成记录

- `configs/config.yaml` 改动：
  - bounds 统一改为 `[-0.30, -0.30, 0.75]` 到 `[0.30, 0.30, 1.10]`。
  - `interpolate_rot_step_size` 从 `0.34` 改为 `0.17`。
  - `grasp_depth` 从 `0.10` 改为 `0.04`。
  - `env.og_sim.physics_frequency` 从 `60` 改为 `120`，
    `action_frequency` 从 `15` 改为 `30`。
  - robot config 改为 `name: so101`、`type: SO101`、`position: [0.0, -0.15, 0.75]`、
    `obs_modalities: []`、`action_normalize: False`、`grasping_mode: assisted`。
  - 移除 Fetch base / DifferentialDrive / camera controller 配置。
  - `arm_0` 改为 absolute position `JointController`，与 T15 的 Lula IK
    joint-target action 路径对齐。
  - `gripper_0` 改为 `MultiFingerGripperController`，`mode: binary`，
    `inverted: true`，保持高层 `-1=open`、`1=close` 语义。
  - `path_solver.opt_pos_step_size` 从 `0.20` 改为 `0.06`。
  - `keypoint_proposer.min_dist_bt_keypoints` 从 `0.06` 改为 `0.03`。
- 验证命令：
  - `/home/badger/anaconda3/envs/omnigibson/bin/python - <<'PY' ... get_config('./configs/config.yaml') ... PY`
  - `rg "Fetch|DifferentialDrive|trunk|OperationalSpaceController" configs/config.yaml -n`
- 结果：
  - YAML 加载成功。
  - `robot_type=SO101`
  - `action_normalize=False`
  - `bounds_main` 与 `bounds_env` 一致。
  - `physics_frequency=120`
  - `action_frequency=30`
  - `arm_controller=JointController`
  - `gripper_controller=MultiFingerGripperController`
  - Fetch / DifferentialDrive / trunk / OperationalSpaceController 静态检查无匹配。
