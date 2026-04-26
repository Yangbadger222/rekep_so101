# T08 - Lula robot_descriptor.yaml 生成

- **阶段**: 2 - Robot 封装
- **里程碑**: M4
- **依赖**: T02
- **状态**: [x] 已完成（headless / Lula load 验证）

## 目标

为 SO-101 创建 Lula `robot_descriptor.yaml` 文件，这是 IK Solver 的必要输入。

## 背景

ReKep 的 `ik_solver.py` 使用 `lula.load_robot(robot_description_path, robot_urdf_path)` 加载机器人描述。当前项目中没有 SO-101 的 Lula descriptor，这是**关键缺失项**。

## 详细步骤

### 8.1 选择生成方式

**方式 A（推荐）：Isaac Sim Lula Robot Description Editor**

1. 在 Isaac Sim 中打开 SO-101 URDF
2. 使用 `Extensions → Lula Robot Description Editor`
3. 导入 URDF，配置 cspace joints
4. 导出 `robot_descriptor.yaml`

**方式 B：手写最小 yaml**

基于 URDF 参数手动编写。

### 8.2 创建文件

目标路径：`assert/SO101/lula/so101_robot_descriptor.yaml`

```bash
mkdir -p assert/SO101/lula
```

### 8.3 最小可用模板

```yaml
api_version: 1.0

cspace:
  - shoulder_pan
  - shoulder_lift
  - elbow_flex
  - wrist_flex
  - wrist_roll

root_link: base_link

default_q: [0.0, -0.5, 1.0, 0.0, 0.0]

acceleration_limits: [10.0, 10.0, 10.0, 10.0, 10.0]
jerk_limits: [1000, 1000, 1000, 1000, 1000]

cspace_to_urdf_joint_names:
  shoulder_pan: shoulder_pan
  shoulder_lift: shoulder_lift
  elbow_flex: elbow_flex
  wrist_flex: wrist_flex
  wrist_roll: wrist_roll

collision_spheres:
  base_link:
    - { "center": [0, 0, 0.02], "radius": 0.04 }
  shoulder_link:
    - { "center": [0, 0, 0], "radius": 0.03 }
  upper_arm_link:
    - { "center": [0.05, 0, 0], "radius": 0.025 }
  lower_arm_link:
    - { "center": [0.05, 0, 0], "radius": 0.025 }
  wrist_link:
    - { "center": [0, 0, 0], "radius": 0.02 }
  gripper_link:
    - { "center": [0.02, 0, 0], "radius": 0.02 }
```

### 8.4 参数说明

| 字段 | 说明 | 来源 |
|------|------|------|
| cspace | 控制空间关节列表（不含 gripper） | URDF |
| root_link | 运动链起点 | URDF |
| default_q | 默认关节角度（rad） | 需调试 |
| acceleration_limits | 加速度限制 | STS3215 datasheet 或估计 |
| jerk_limits | 加加速度限制 | 估计值 |
| cspace_to_urdf_joint_names | cspace 到 URDF joint 的映射 | URDF |
| collision_spheres | 各 link 的碰撞球（用于自碰撞检测） | 需在 Isaac Sim 量取 |

### 8.5 collision_spheres 调整

碰撞球的 center 和 radius 需要根据实际 mesh 大小调整：

1. 在 Isaac Sim 中测量各 link 的实际尺寸
2. 调整碰撞球中心和半径使其包裹 link 几何体
3. 不要太大（导致虚假碰撞）也不要太小（漏检碰撞）

## 验收标准

- [x] `assert/SO101/lula/so101_robot_descriptor.yaml` 文件存在
- [x] YAML 格式正确，无语法错误
- [x] cspace joints 与 URDF 一致
- [x] `lula.load_robot()` 能成功加载（不报错）
- [x] collision_spheres 覆盖所有主要 link

## 完成记录

- 使用方式 B 手写最小 descriptor，但按本机 Isaac/Lula 示例修正为实际 schema：`collision_spheres` 使用 list-of-maps，未使用任务初稿里的 `cspace_to_urdf_joint_names` mapping。
- `cspace` 为 5 个手臂关节：`shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`；`gripper` 通过 `cspace_to_urdf_rules` 固定为 `0.0`。
- 在 OmniGibson headless 环境启动后，`lazy.lula.load_robot("lula/so101_robot_descriptor.yaml", "so101_new_calib.urdf")` 成功；裸 Python 下 `lazy.lula` 不可用，需先启动 Isaac/OG 扩展环境。

## 输出文件

```
assert/SO101/lula/so101_robot_descriptor.yaml
```

## 备注

- 如果 Lula Editor 不可用，手写方式足够起步
- `default_q` 影响 IK 求解的初始猜测，需要后续调优
- collision_spheres 精度影响自碰撞检测，初版可用粗略值
- 后续如果 Lula IK 效果不好，可考虑替换为 cuRobo
