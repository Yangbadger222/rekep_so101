# T10 - SO101 ManipulationRobot 子类实现

- **阶段**: 2 - Robot 封装
- **里程碑**: M5
- **依赖**: T02, T08
- **状态**: [x] 已完成（headless / M5 load smoke）

## 目标

实现 `SO101(ManipulationRobot)` 子类，使 SO-101 能作为 OmniGibson 可控机器人使用。

## 背景

OmniGibson 没有 SO-101 的内建支持。必须自己实现一个子类。推荐参考 `Franka` 类（单臂 + 平行夹爪 + 无 trunk/base，结构最接近 SO-101）。

## 详细步骤

### 10.1 创建文件

创建 `so101_robot.py`（项目根目录或新建 `robots/` 目录）。

### 10.2 实现必要属性与方法

```python
from omnigibson.robots import ManipulationRobot

class SO101(ManipulationRobot):
    """SO-101 5-DOF robot arm with parallel gripper."""
    
    @property
    def model_name(self):
        return "SO101"
    
    @property
    def usd_path(self):
        return "assert/SO101/so101_new_calib/so101_new_calib.usd"
    
    @property
    def urdf_path(self):
        return "assert/SO101/so101_new_calib.urdf"
    
    # === Arm ===
    @property
    def arm_link_names(self):
        return {
            "arm_0": [
                "shoulder_link",
                "upper_arm_link",
                "lower_arm_link",
                "wrist_link",
                "gripper_link",
                "gripper_frame_link",
            ]
        }
    
    @property
    def arm_joint_names(self):
        return {
            "arm_0": [
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
            ]
        }
    
    # === Gripper ===
    @property
    def gripper_link_names(self):
        return {
            "arm_0": [
                "gripper_link",
                "moving_jaw_so101_v1_link",
            ]
        }
    
    @property
    def gripper_joint_names(self):
        return {
            "arm_0": ["gripper"]
        }
    
    # === EE & Fingers ===
    @property
    def eef_link_names(self):
        return {"arm_0": "gripper_frame_link"}
    
    @property
    def finger_link_names(self):
        return {"arm_0": ["moving_jaw_so101_v1_link"]}
    
    # === Lula descriptor ===
    @property
    def robot_arm_descriptor_yamls(self):
        return {"arm_0": "assert/SO101/lula/so101_robot_descriptor.yaml"}
    
    # === Joint positions ===
    @property
    def default_joint_pos(self):
        return [0.0, -0.5, 1.0, 0.0, 0.0, 0.5]  # 5 arm + 1 gripper
    
    @property
    def reset_joint_pos(self):
        return self.default_joint_pos
    
    # === Controller config ===
    def _default_controller_config(self):
        # 参考 Franka 的 controller config，适配 SO-101
        ...
```

### 10.3 Controller 配置

SO-101 需要的 controller：
- **Arm**: `OperationalSpaceController` 或 `JointController`
- **Gripper**: `MultiFingerGripperController`

**关键区别**（vs Fetch）：
- 无 trunk joints
- 无 base wheels（不需要 DifferentialDriveController）
- action_dim = 5 (arm) + 1 (gripper) = **6**（Fetch 是 12）

### 10.4 Grasping mode

**必须显式设置**：

```python
grasping_mode: "assisted"  # 或 "sticky"
```

SO-101 夹爪开口仅 ~30mm、夹力弱，物理 grasping 在 OG 中几乎一定不稳。

### 10.5 assisted grasp 配置

在 `_default_controller_config` 中显式列出 finger links：

```python
"assisted_grasp_finger_links": ["moving_jaw_so101_v1_link"]
```

否则 assisted grasping 可能检测不到 finger-object contact。

> 实测说明（2026-04-25）：当前 OG `MultiFingerGripperController` 不接受
> `assisted_grasp_finger_links` 作为 controller kwargs，否则 controller 初始化会报错。
> 本实现改为在 `SO101` 类上暴露 `assisted_grasp_finger_links`，并实现
> `assisted_grasp_start_points` / `assisted_grasp_end_points`。此外，为满足 OG assisted
> grasp 至少两个 finger contact 的判断，`finger_link_names` 同时包含固定侧
> `gripper_link` 与活动侧 `moving_jaw_so101_v1_link`。

## 验收标准

- [x] `SO101` 类能被正确导入
- [x] `env.robots[0]` 是 `SO101` 实例
- [x] `robot.action_dim == 6`（5 arm + 1 gripper）
- [x] 所有 property 返回正确值
- [x] Controller config 无报错
- [x] `grasping_mode` 设为 `assisted`

## 输出文件

```
so101_robot.py
scripts/m5_load_so101_robot.py
```

## 完成记录

- 新增 `so101_robot.py`：本地注册 `SO101(ManipulationRobot)`，使用
  `so101_new_calib_og_usdobject.usd` wrapper 作为 robot USD，默认固定底座、
  `JointController` arm、`MultiFingerGripperController` gripper、`grasping_mode="assisted"`。
- 新增 `scripts/m5_load_so101_robot.py`：创建 OG headless empty scene，验证 registry、
  `env.robots[0]` 类型、root link、controller keys、joint/property 映射、`action_dim=6`。
- 额外实现 `aabb` fallback：当前 wrapper 作为 `BaseRobot` 初始化时，OG 对 link
  collision boundary points 可能返回空列表；fallback 只用于 robot 初始化的
  `reset_joint_pos_aabb_extent`，避免修改 OG 源码。
- 验证：
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile so101_robot.py` 通过。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m5_load_so101_robot.py` 通过。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m5_load_so101_robot.py --headless --steps 10 --log-every 5` 通过：
    `type=SO101`、`root_link=base_link`、`controllers=['arm_0', 'gripper_0']`、
    `dofs=['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']`、
    `action_dim=6`。

## 涉及参考文件

```
# OmniGibson 内置 Franka 类（结构最接近 SO-101）
# 可通过以下方式找到：
# python -c "import omnigibson; print(omnigibson.__path__)"
# 然后查看 robots/franka.py
```

## 备注

- link/joint 名称必须与 URDF 完全一致
- 先只实现必要属性，验证通过后再完善
- 如果 OG 的 ManipulationRobot 接口有变化，以实际 API 为准
- 参考 Franka 类时，删除 trunk 相关代码
