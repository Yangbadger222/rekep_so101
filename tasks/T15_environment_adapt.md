# T15 - environment.py 接口适配

- **阶段**: 3 - ReKep 接口适配
- **里程碑**: M6
- **依赖**: T10
- **状态**: [x] 已完成（SO101 interface smoke）

## 目标

将 `environment.py` 中所有 Fetch 特有的接口和逻辑替换为 SO-101 兼容版本。

## 详细修改清单

### 15.1 移除 Fetch 猴补丁

**行号**: 第 12 行、第 23 行、第 55 行

```python
# 现状：Fetch._initialize = ... (monkey patch)
# SO-101 不需要 trunk hack
# 删除所有 Fetch._initialize 猴补丁代码
```

### 15.2 修改 trunk + arm 控制索引

**行号**: 第 75-77 行

```python
# 现状
control_idx = np.concatenate([robot.trunk_control_idx, robot.arm_control_idx["arm_0"]])

# 改为（SO-101 没有 trunk）
control_idx = robot.arm_control_idx["arm_0"]
```

### 15.3 保留 grasping_mode 断言

**行号**: 第 97 行

```python
# 保留此断言，但确保 config.yaml 中设了 assisted
assert grasping_mode == "assisted"
```

### 15.4 修改 get_arm_joint_positions

**行号**: 第 290-294 行

```python
# 现状：包含 trunk + arm joints
# 改为：仅 arm joints（5个）
def get_arm_joint_postions(self):
    return self.robot.get_joint_positions()[self.arm_control_idx]
```

### 15.5 修改 gripper action

**行号**: 第 296-316 行

```python
# 现状：12 维 action，action[10:] 是 gripper
# 改为：6 维 action，action[5:] 是 gripper

def get_gripper_open_action(self):
    action = np.zeros(self.robot.action_dim)  # 6 维
    action[5:] = -1.0  # 最后一维是 gripper，-1=open
    return action

def get_gripper_close_action(self):
    action = np.zeros(self.robot.action_dim)  # 6 维
    action[5:] = 1.0   # 1=close
    return action

def get_gripper_null_action(self):
    action = np.zeros(self.robot.action_dim)
    action[5:] = 0.0
    return action
```

### 15.6 修改 _move_to_waypoint action 布局

**行号**: 第 480-484 行

```python
# 现状：
# action[4:7] = position
# action[7:10] = orientation
# action[10:] = gripper

# 改为（按 SO-101 controller 布局）：
# 需要根据 SO-101 的 OperationalSpaceController 配置确定
# 如果用 joint 控制器：action[0:5] = joint targets, action[5] = gripper
# 如果用 OSC：按 OSC action dim 重新映射
```

**注意**：这里的具体修改取决于 T10 中 controller config 的设计。

### 15.7 修改 reset 中的 EE 偏移

**行号**: 第 265-273 行

```python
# 现状（Fetch reach）
ee_pose[:3] += [0, -0.2, -0.1]

# 改为（SO-101 reach 更小）
ee_pose[:3] += [0, -0.05, -0.03]
```

### 15.8 修改 Fetch import

移除对 Fetch 的导入引用（如果有）。

## 验收标准

- [x] 无 Fetch 相关 import
- [x] 无 trunk 相关代码
- [x] action 维度正确（6 维）
- [x] gripper open/close/null action 正确
- [x] `get_arm_joint_positions` 返回 5 个值
- [x] `_move_to_waypoint` action 布局正确
- [x] reset 偏移量适合 SO-101

## 涉及文件

```
environment.py
```

## 备注

- 这是改动最多的文件，建议逐项修改、逐项测试
- action 布局直接取决于 controller 配置（T10）
- 先确认 `robot.action_dim` 的具体值和各维含义，再改 action 相关代码

## 完成记录

- `environment.py` 改动：
  - 移除 `Fetch` import、`Fetch._initialize` monkey patch 和 trunk control index 假设。
  - `ReKepOGEnv` 初始化时要求 `env.robots[0]` 是 `SO101`。
  - 缓存 `arm_control_idx` / `arm_action_idx` / `gripper_action_idx`：
    - arm action index: `[0, 1, 2, 3, 4]`
    - gripper action index: `[5]`
  - `reset_joint_pos` 改为 5 维 SO-101 arm joint reset。
  - `get_arm_joint_postions()` 返回 5 维 arm qpos。
  - `open_gripper()` / `close_gripper()` 使用 6 维 robot action，并保持当前 arm qpos
    作为 no-op arm command。
  - `_move_to_waypoint()` 移除 Fetch 12D OSC action 布局，改为用 Lula IK 求解
    5 维 SO-101 arm joint target，再写入 `action[0:5]`；夹爪保持
    `action[5]` 的 last command。
  - `reset()` 的视野避让偏移从 `[0, -0.2, -0.1]` 改为 `[0, -0.05, -0.03]`。
  - `scene_file` 仅在非空时写入 scene config，避免普通 `Scene` smoke 被空路径污染。
- 验证命令：
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile environment.py main.py`
  - `rg "from omnigibson\\.robots\\.fetch|Fetch\\._initialize|isinstance\\(self\\.robot, Fetch\\)|trunk_control_idx|np\\.zeros\\(12\\)|action\\[10:|this action space is only for fetch" environment.py -n`
  - 最小 SO-101 `ReKepOGEnv` headless smoke（空 `Scene`，无 camera，显式 `og.shutdown()`）。
- 结果：
  - 编译通过。
  - Fetch import / monkey patch / trunk / 12D action 静态检查无匹配。
  - smoke 输出：
    - `robot=SO101`
    - `action_dim=6`
    - `arm_idx=[0, 1, 2, 3, 4]`
    - `gripper_idx=[5]`
    - `reset_shape=(5,)`
    - `arm_joint_positions_shape=(5,)`
    - `open=-1.0 close=1.0 null=0.0`
