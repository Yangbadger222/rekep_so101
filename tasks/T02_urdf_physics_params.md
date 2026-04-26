# T02 - URDF 物理参数校验

- **阶段**: 1 - 资产验证
- **里程碑**: M1
- **依赖**: 无
- **状态**: [x] 已完成（静态校验）

## 目标

校验 `so101_new_calib.urdf` 中的关节参数、质量、惯性等物理属性，确认其可用于物理仿真。

## 详细步骤

### 2.1 确认关节信息

从 URDF 中读取并记录以下参数：

| 参数 | 预期值 | 实际值 |
|------|--------|--------|
| DOF（不含夹爪） | 5 | 5 |
| arm joints | shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll | 一致 |
| gripper joint | gripper (revolute) | 一致 |
| EE link | gripper_frame_link (fixed joint) | 一致 |
| finger link | moving_jaw_so101_v1_link | 一致 |

### 2.2 确认关节限位

| Joint | lower (rad) | upper (rad) | 近似角度 |
|-------|-------------|-------------|----------|
| shoulder_pan | -1.91986 | 1.91986 | ±110° |
| shoulder_lift | -1.74533 | 1.74533 | ±100° |
| elbow_flex | -1.69000 | 1.69000 | ±97° |
| wrist_flex | -1.65806 | 1.65806 | ±95° |
| wrist_roll | -2.74385 | 2.84121 | -157°~+163° |
| gripper | -0.17453 | 1.74533 | -10°~+100° |

### 2.3 确认质量与惯性

- [x] base_link: 0.147 kg
- [x] shoulder_link: 0.100006 kg
- [x] 其余实体 link: 0.012-0.104 kg（`moving_jaw_so101_v1_link` 为 0.012 kg）
- [x] 整机: 0.632006001 kg（含 1e-9 kg dummy EE link）
- [x] 实体 link 的 inertia tensor 非全零
- [x] `gripper_frame_link` 是 fixed dummy EE frame，mass=1e-9、inertia=0，作为例外接受

### 2.4 确认 effort 与 velocity

当前 URDF 中 `effort=10`, `velocity=10` 是 onshape-to-robot 默认占位值。

**STS3215 实际参数**（如需更真实仿真）：
- stall torque ≈ 30 kg·cm ≈ 2.94 N·m
- 建议后续根据 datasheet 修正

### 2.5 确认 mesh 路径

URDF 中 mesh 引用格式为：

```xml
<mesh filename="assets/xxx.stl"/>
```

确认所有 mesh 文件存在且路径相对于 `assert/SO101/` 目录。

## 验收标准

- [x] 所有 joint 名称与文档一致
- [x] joint 限位值正确
- [x] 所有实体 link 有合理的 mass（非零）
- [x] 实体 link inertia tensor 非全零；`gripper_frame_link` 为 fixed dummy 例外
- [x] mesh 文件路径全部可解析
- [x] link 链结构正确：base_link → shoulder_link → upper_arm_link → lower_arm_link → wrist_link → gripper_link → gripper_frame_link

## 涉及文件

```
assert/SO101/so101_new_calib.urdf
assert/SO101/joints_properties.xml
assert/SO101/assets/*.stl
```

## 备注

- `so101_new_calib.urdf`：关节零位在范围中间（推荐用此版本）
- `so101_old_calib.urdf`：关节零位在完全伸展水平位
- effort/velocity 暂时不改，阶段 2 再决定是否修正

## Codex 验证记录（2026-04-25）

运行：

```bash
python3 scripts/m1_check_urdf.py
```

结果：

```text
links: 8
joints: 7
total_mass_kg: 0.632006001
mesh_refs: 34 (13 unique)
dummy_links_with_zero_inertia: gripper_frame_link
PASS: SO-101 URDF physical parameters are internally consistent
```
