# SO-101 OmniGibson + ReKep 仿真接入方案

本文档记录如何把 `SO-101` 资产接入 OmniGibson，并最终配合 ReKep 做“固定在桌子上清理垃圾”的物理仿真。

当前资产目录：

```text
assert/
  SO101/
    so101_new_calib/
      so101_new_calib.usd
      configuration/
        so101_new_calib_base.usd
        so101_new_calib_physics.usd
        so101_new_calib_robot.usd
        so101_new_calib_sensor.usd
    so101_new_calib.urdf
    so101_new_calib.xml
    assets/
      *.stl
```

默认 USD 路径：

```text
/home/badger/Desktop/Rekep/assert/SO101/so101_new_calib/so101_new_calib.usd
```

> 注：目录名现在是 `assert`。如果后续整理项目，建议改成 `assets`，但改名前需要同步所有引用路径（USD 内部相对引用、urdf 中的 mesh 路径、Python 配置中的硬编码路径）。**第一阶段不要改名**，避免破坏 git diff 与 USD 内部引用。

## SO-101 关键物理参数（已从 `assert/SO101/so101_new_calib.urdf` 读出）

```text
DOF（不含夹爪）         : 5
arm joint 名称（顺序）  : shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll
gripper joint 名称      : gripper（单 revolute，平行夹爪由 moving_jaw 实现）
arm link 名称           : base_link → shoulder_link → upper_arm_link → lower_arm_link → wrist_link → gripper_link
end-effector link       : gripper_frame_link（fixed joint 挂在 gripper_link 上，是真正的 EE 参考系）
finger link             : moving_jaw_so101_v1_link

joint limits（rad，URDF 实测）
  shoulder_pan   : [-1.91986, 1.91986]   ≈ ±110°
  shoulder_lift  : [-1.74533, 1.74533]   ≈ ±100°
  elbow_flex     : [-1.69000, 1.69000]   ≈ ±97°
  wrist_flex     : [-1.65806, 1.65806]   ≈ ±95°
  wrist_roll     : [-2.74385, 2.84121]   ≈ -157°…+163°
  gripper        : [-0.17453, 1.74533]   ≈ -10°…+100°（开 = 大角度）
joint effort/velocity   : 全部 effort=10 N·m, velocity=10 rad/s（URDF 默认占位，非真实 STS3215 参数）

物理质量（URDF）
  base_link        : 0.147 kg
  shoulder_link    : 0.100 kg
  其余 link        : 量级 ~0.05–0.10 kg
  整机             : ~0.5 kg

经验值（需 Isaac Sim 量取最终确认）
  最大夹爪开口     : ~30–40 mm（依据 moving_jaw STL 几何）
  工作空间半径     : ~0.30–0.35 m（shoulder→elbow→wrist 累计长度）
  推荐 payload     : ≤ 100 g（受 STS3215 舵机扭矩限制）
```

**直接影响**：
- `configs/config.yaml` 的 `bounds_min/max` 必须按 SO-101 reach 重算（详见 3.2）。
- 桌面"垃圾"物体最大尺寸 ≤ 30 mm，否则永远抓不住。
- payload ≤ 100 g 决定可抓物的密度与体积。
- URDF 的 `effort=10`/`velocity=10` 是 onshape-to-robot 默认占位值，**不是 STS3215 实际参数**；如果要做接近真实的物理仿真，建议查 STS3215 datasheet 后修正（典型 stall torque ≈ 30 kg·cm ≈ 2.94 N·m）。

## 目标

1. 在 OmniGibson 中加载 SO-101 模型。
2. 保留物理碰撞，用于真实仿真。
3. 将 SO-101 固定在桌子上。
4. 添加桌子、垃圾、垃圾桶等物体。
5. 最终让 ReKep 控制 SO-101 清理桌面垃圾。

## 总体路线

推荐分三阶段推进：

```text
阶段 1：SO-101 作为 USDObject 加入场景，验证显示、比例、碰撞
阶段 2：SO-101 封装成 OmniGibson Robot，验证关节、夹爪、action、IK
阶段 3：接入 ReKep，完成桌面垃圾清理任务
```

不要一开始就直接接 ReKep。否则后续问题会混在一起，难以判断是资产物理问题、机器人控制问题，还是 ReKep 任务逻辑问题。

## 阶段 1：资产物理验证

### 1.1 检查 USD 引用

确认 `so101_new_calib.usd` 内部引用是稳定的相对路径，例如：

```text
@configuration/so101_new_calib_robot.usd@
@configuration/so101_new_calib_physics.usd@
```

不要依赖临时绝对路径，否则 OmniGibson 加载时可能找不到 mesh、材质或 physics 配置。

### 1.2 检查坐标和单位

在 Isaac Sim / USD Composer 中打开：

```text
assert/SO101/so101_new_calib/so101_new_calib.usd
```

确认：

- 单位是 meter。
- Z-up。
- base 坐标合理。
- 模型没有整体大 100 倍或小 100 倍。
- 模型原点最好位于机械臂底座安装点附近。

### 1.3 检查碰撞

因为要物理仿真，不能只加载 visual mesh。每个需要参与碰撞的 link 都应该有 collider。

建议：

- 机械臂 link：使用 convex hull 或 convex decomposition。
- 夹爪 finger：使用简单 convex collider，避免过薄。
- 桌子：使用 box collider 或简化 mesh collider。
- 垃圾：使用 convex hull。
- 垃圾桶：使用简化 mesh 或多个 primitive collider。

避免：

- 给复杂 STL 原始三角网格直接做动态碰撞。
- 初始状态下 link 之间互相穿透。
- 垃圾初始位置和桌面/夹爪/机械臂穿透。

### 1.4 检查物理属性

SO-101 如果要作为可控机械臂，USD/URDF 应该包含：

- articulation root
- link mass
- inertia
- joint axis
- joint limit
- joint drive / stiffness / damping
- gripper joints
- end-effector link

如果暂时只验证碰撞，可以先不控制机械臂，只把它作为固定物体加载。

## 阶段 1A：作为 USDObject 加入 OmniGibson

先用 `USDObject` 验证 SO-101 能否出现在桌面上，并且能参与碰撞。

OmniGibson 当前版本中，普通 USD 资产使用：

```python
"type": "USDObject"
```

SO-101 对象配置示例：

```python
{
    "type": "USDObject",
    "name": "so101",
    "usd_path": "/home/badger/Desktop/Rekep/assert/SO101/so101_new_calib/so101_new_calib.usd",
    "category": "robot",
    "fixed_base": True,
    "visual_only": False,
    "kinematic_only": False,
    "position": [0.0, 0.0, 0.75],
    "orientation": [0.0, 0.0, 0.0, 1.0],
    "scale": 1.0,
}
```

字段含义：

- `fixed_base=True`：固定底座，适合桌面安装。
- `visual_only=False`：保留碰撞和物理。
- `kinematic_only=False`：让物体走物理系统。
- `position`：需要根据桌面高度调整。
- `orientation`：四元数，默认 `[x, y, z, w]`。

注意：`USDObject` 只能把 SO-101 当作场景物体加载。它不能直接被 ReKep 控制。

> ⚠️ **API 提示**：当前 OmniGibson 版本对 `"type": "USDObject"` 字符串解析方式可能与文档不一致。推荐先在 Python 中用最小脚本调用：
>
> ```python
> from omnigibson.objects import USDObject
> obj = USDObject(name="so101", usd_path="...", fixed_base=True, visual_only=False)
> og.sim.import_object(obj)
> ```
>
> 验证可加载后，再决定是否写进场景 JSON 的 `objects_info`。**不要**直接把 dict 拷进未验证的字段。

## 桌面场景建议

最小可用场景：

```text
table       固定，有碰撞
so101       固定在桌面，有碰撞
trash_0     动态，有碰撞，可抓取
trash_1     动态，有碰撞，可抓取
trash_bin   固定或动态，有碰撞
robot       后续替换为 SO-101 robot
```

桌子配置原则：

```python
"fixed_base": True
```

垃圾配置原则：

```python
"fixed_base": False
"visual_only": False
```

垃圾桶配置原则：

```python
"fixed_base": True
"visual_only": False
```

## 固定在桌子上的方案

第一版推荐使用“位姿固定”，不要先做 table-to-robot fixed joint。

做法：

1. 桌子 `fixed_base=True`。
2. SO-101 `fixed_base=True`。
3. 调整 SO-101 的 `position`，让底座刚好贴在桌面上。
4. 确认仿真开始后 SO-101 不掉落、不抖动、不爆炸。

这种方案足够用于 ReKep 桌面任务。

如果后续需要更严格的机械连接，再在 USD 中添加 fixed joint，把 SO-101 base 固定到桌子 prim 上。

## 碰撞验证清单

在接 ReKep 前，必须先通过以下检查：

- SO-101 加载后位置正确。
- SO-101 不掉落。
- SO-101 不抖动。
- SO-101 不与桌子初始穿透。
- 垃圾掉到桌面后能停住。
- 垃圾不会穿过桌子。
- 垃圾不会被碰撞弹飞。
- SO-101 link / gripper 靠近垃圾时不会直接穿模。
- 场景能连续 step 至少 100 步。

如果出现穿模，优先检查：

- USD 是否真的有 collision。
- collision 是否被 disabled。
- collider 是否过薄。
- collider 是否太复杂。
- scale 是否正确。
- 初始位置是否穿透。
- physics timestep 是否太大。

## 阶段 2：封装成 OmniGibson Robot

ReKep 不能控制普通 `USDObject`。如果要让 SO-101 执行清理垃圾，需要把它接成 OmniGibson robot。

### 2.1 必备资产与描述文件

```text
robot name
usd_path
base link name
end-effector link name
arm joint names
gripper joint names
joint lower limits
joint upper limits
joint velocity limits
default joint positions
controller config
```

> 🔥 **关键缺失**：当前资产里**没有** Lula `robot_descriptor.yaml`（也没有 cuRobo 配置）。
> 而 ReKep 的 `IKSolver` (`ik_solver.py`) 和 OG 的 `Fetch.robot_arm_descriptor_yamls`
> 都依赖这个文件。**必须先生成**：
>
> - 方式 A：在 Isaac Sim 里用 `Lula Robot Description Editor` 基于 SO-101 URDF 导出。
> - 方式 B：手写一份最小 yaml（`cspace_to_urdf_joint_names`、`default_q`、`acceleration_limits`）。
>
> 没有这个 yaml，阶段 3 的 IK 反推无法工作，subgoal_solver 的 reachability cost 也算不出来。

### 2.2 SO-101 Robot 子类

OmniGibson 当前没有 SO-101 内建支持。需要自己实现一个 `SO101(ManipulationRobot)` 子类，
最小要实现的属性/方法：

```text
@property arm_link_names / arm_joint_names / arm_control_idx
@property gripper_link_names / gripper_joint_names / gripper_control_idx
@property eef_link_names / finger_link_names
@property robot_arm_descriptor_yamls   # 指向 2.1 中生成的 yaml
@property urdf_path
@property default_joint_pos / reset_joint_pos
_default_controller_config
```

> 也可以参考 OG 内置 `Franka` 类（结构最接近 SO-101，单臂 + 平行夹爪 + 无 trunk/base wheels），
> 复制后改 link/joint 名比从零写要快。

### 2.3 Grasping mode

SO-101 夹爪开口仅 ~30 mm、夹力弱，**物理 grasping 在 OG 中几乎一定不稳**。必须显式设：

```yaml
grasping_mode: assisted   # 或 sticky
```

否则 `environment.py:97` 的 `assert` 会通过，但实际抓取一定失败。

### 2.4 验证项

- `env.robots[0]` 是 SO-101。
- `robot.action_dim` 正常（应该是 5 + 1 = 6，远小于 Fetch 的 12）。
- arm joints 能动。
- gripper 能开合并能 `is_grasping` 返回 TRUE。
- 末端位姿能读取（`get_eef_position/orientation`）。
- IK 能解目标位姿（用 2.1 的 yaml）。
- 简单 action 不会导致机器人爆炸或穿模。
- 连续 step 1000 帧不漂移。

## 阶段 3：接入 ReKep

ReKep 当前代码硬编码了 Fetch 接口。下面是**精确的接口替换清单**（带文件 + 行号），改之前先用 `git diff` 留底。

### 3.1 必改文件清单

| 文件 | 行/位置 | 现状（Fetch） | SO-101 改法 |
|------|---------|---------------|-------------|
| `main.py` | 第 14 行 `from omnigibson.robots.fetch import Fetch` | 导入 Fetch | 改成 SO-101 子类 |
| `main.py` | 第 42 行 `assert isinstance(self.env.robot, Fetch)` | 强断言 Fetch | 改成 `isinstance(..., SO101)` |
| `main.py` | 第 44–48 行 `IKSolver(...)` 参数 | 用 Fetch 描述 | 用 SO-101 yaml + urdf + eef link |
| `environment.py` | 第 12 行、第 23 行、第 55 行 | `Fetch._initialize = ...` 猴补丁 | SO-101 不需要 trunk hack，删掉 |
| `environment.py` | 第 75–77 行 `trunk_control_idx + arm_control_idx` | Fetch 有 trunk | SO-101 没 trunk，仅 `arm_control_idx` |
| `environment.py` | 第 97 行 grasping_mode assert | OK | 保留，但 yaml 必须设 assisted |
| `environment.py` | 第 290–294 行 `get_arm_joint_postions` | trunk + arm | 仅 arm |
| `environment.py` | 第 296–316 行 `open/close_gripper` | 12 维 action，`action[10:]` | 改为 6 维，`action[5:]` 或 SO-101 实际 dim |
| `environment.py` | 第 480–484 行 `_move_to_waypoint` action 布局 | `action[4:7]/[7:10]/[10:]` | 按 SO-101 controller 重写 |
| `environment.py` | 第 265–273 行 `reset` 中的 `ee_pose[:3] += [0,-0.2,-0.1]` | Fetch reach | 缩成 `[0,-0.05,-0.03]` |
| `configs/config.yaml` | 第 9–10 行 `bounds_min/max` | Fetch 工作空间 | 缩成 SO-101 reach（见 3.2） |
| `configs/config.yaml` | 第 32–55 行 robot block | Fetch 控制器 | 改成 SO-101 controller_config |
| `configs/config.yaml` | 第 57–70 行 camera | Fetch 头部相机视角 | 重新校准到 SO-101 桌面（见 3.3） |
| `ik_solver.py` | 整体 | 假设 Lula yaml 存在 | 加载 2.1 生成的 SO-101 yaml |

### 3.2 工作空间 bounds 重算

Fetch 当前的 bounds：

```yaml
bounds_min: [-0.45, -0.75, 0.698]
bounds_max: [ 0.10,  0.60, 1.200]
```

SO-101 reach 仅 ~0.30–0.35 m。假设 SO-101 base 安装在桌面 `(0, 0, 0.75)`，建议：

```yaml
bounds_min: [-0.30, -0.30, 0.75]
bounds_max: [ 0.30,  0.30, 1.10]
```

否则 subgoal_solver 会反复求解机器人完全够不着的位姿，而 path_solver 的 `opt_pos_step_size: 0.20`
也需要相应缩小到 `0.05–0.08`。

### 3.3 相机配置（重要）

SO-101 没有头部/腕部内置相机，ReKep 的 `keypoint_proposer` 与 `vlm_camera` 必须依赖**外部固定 RGB-D 相机**：

- 在 `config.yaml` `camera` 块新增一个**俯视桌面**的相机（例如 `position=[0, 0, 1.4]`，朝下 60°–90°）。
- 该相机要能看到：SO-101 base、整个桌面、所有垃圾、垃圾桶。
- `main.config['vlm_camera']` 指向它的 id。
- 同时保留一个录制相机（侧视角，用于 `save_video`）。
- 注意 `_step` 中 `cam_obs[1]['rgb']` 取的是 id=1 录制相机，若改 id 编号也要同步。

### 3.4 抓取相关 cost（深入到 subgoal_solver 实现）

`subgoal_solver.py:71–76` 写死了 top-down grasp 偏好：

```python
preferred_dir = np.array([0, 0, -1])
grasp_cost = -np.dot(opt_pose_homo[:3, 0], preferred_dir) + 1
```

即假设 **EE 的 X 轴指向夹爪开合方向**（Fetch/Franka 习惯）。SO-101 的 `gripper_frame_link`
经过若干 fixed joint，其 X/Y/Z 朝向需要在 Isaac Sim 里读出 quaternion 确认。

可能的修正：
- 若 SO-101 EE 的"指向物体"方向是 Z 轴而非 X 轴，把 `opt_pose_homo[:3, 0]` 改成 `opt_pose_homo[:3, 2]`。
- 若需要侧面抓取（垃圾躺在桌面），把 `preferred_dir` 改为 `[0, 0, -1]` 不变，但权重 `10.0` 调小到 `2.0`，让 IK 反推决定姿态。
- `grasp_depth: 0.10` → `0.04`（SO-101 finger 长度 ~30 mm，否则 grasp pose 会沉到桌面以下）。
- `interpolate_pos_step_size: 0.05` 保留；`interpolate_rot_step_size: 0.34`（≈20°）对小臂稍大，建议 `0.17`（≈10°）以减少抖动。
- `path_solver.opt_pos_step_size: 0.20` → `0.06`，匹配 SO-101 reach。

### 3.5 IK solver Lula yaml 模板（最小可用版）

`ik_solver.py:21` 调用 `lazy.lula.load_robot(robot_description_path, robot_urdf_path)`，
后者已存在（`assert/SO101/so101_new_calib.urdf`），前者需要新建。建议路径：

```text
assert/SO101/lula/so101_robot_descriptor.yaml
```

最小骨架（需在 Isaac Sim 的 Lula Robot Description Editor 里完善 `acceleration_limits` 与
`self_collision` 配置）：

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
jerk_limits:        [1000, 1000, 1000, 1000, 1000]
cspace_to_urdf_joint_names:
  shoulder_pan:  shoulder_pan
  shoulder_lift: shoulder_lift
  elbow_flex:    elbow_flex
  wrist_flex:    wrist_flex
  wrist_roll:    wrist_roll
collision_spheres:
  base_link:     [{ "center": [0, 0, 0.02], "radius": 0.04 }]
  shoulder_link: [{ "center": [0, 0, 0],    "radius": 0.03 }]
  upper_arm_link:[{ "center": [0.05, 0, 0], "radius": 0.025 }]
  lower_arm_link:[{ "center": [0.05, 0, 0], "radius": 0.025 }]
  wrist_link:    [{ "center": [0, 0, 0],    "radius": 0.02 }]
  gripper_link:  [{ "center": [0.02, 0, 0], "radius": 0.02 }]
```

`IKSolver` 创建时：
```python
ik_solver = IKSolver(
    robot_description_path="assert/SO101/lula/so101_robot_descriptor.yaml",
    robot_urdf_path="assert/SO101/so101_new_calib.urdf",
    eef_name="gripper_frame_link",   # 必须与 URDF 中 link 名一致
    reset_joint_pos=env.reset_joint_pos,
    world2robot_homo=env.world2robot_homo,
)
```

⚠️ **URDF mesh 路径**：当前 URDF 用 `filename="assets/xxx.stl"` 相对路径。Lula 加载时
working dir 必须是 `assert/SO101/`，否则 mesh 找不到。建议在 `IKSolver` 调用前 `os.chdir`
或改 URDF 为绝对路径副本。

### 3.6 任务 prompt 与 keypoint

- keypoint proposal 是否能看到桌面垃圾——取决于 3.3 的相机视角与 `max_mask_ratio: 0.5`（垃圾过小可能被过滤）。
- `keypoint_proposer.min_dist_bt_keypoints: 0.06` 对小桌面太大，建议 `0.03`，否则多块垃圾会被合并成 1 个关键点。
- constraint generation prompt 在 `vlm_query/prompt_template.txt`，需要新增"清理垃圾到桶"示例段落（参考已有 "put red block on top of blue block" 模板）。
- 参考 `vlm_query/pen` 目录的输出结构，新建 `vlm_query/trash_cleanup`，提供至少一份 cached query 用于断网/无 API key 调试。
- `task_list` 在 `main.py` 第 368 行附近，需要新增 `trash` 任务条目（scene_file / instruction / rekep_program_dir）。
- 多个垃圾时，建议 instruction 仍只描述"一次抓一个"，让 ReKep 在 backtrack 机制中重复运行（更鲁棒）。

建议任务描述：

```text
Use the fixed SO-101 robot mounted on the table to pick up one piece of trash
on the tabletop and place it into the trash bin.
```

### 3.7 垃圾物体的尺寸约束

由于 SO-101 夹爪开口仅 ~30 mm：

- 单个垃圾的最长边 ≤ 30 mm。
- 推荐物体类型：纸团、瓶盖、糖果、小积木、橡皮擦。
- 不要用 BEHAVIOR-1K 里的瓶子、罐头等大物体。
- 物体密度调小（`density ≤ 200 kg/m³`）以保证 ≤ 100 g。
- 垃圾桶口径建议 ≥ 100 mm，给 release pose 留容差。

## 风险登记与回退路径

| ID | 风险 | 影响 | 缓解 / Plan B |
|----|------|------|---------------|
| R1 | URDF mass/inertia 是 onshape 默认值，物理可能漂浮或抖 | 阶段 2 | 用 `<mass>` 手动注入合理值，或在 OG 中 `kinematic_only=True` 跑视觉验证 |
| R2 | `effort=10` 占位太大，关节会"硬冲"目标位置 | 阶段 2 | 调小 controller `kp` 到 `50`，`damping_ratio=0.8` |
| R3 | Lula yaml 写不对，IK 永远 `success=False` | 阶段 2/3 | 先用 cuRobo（更现代）替换 Lula；或临时禁用 reachability cost（subgoal_solver.py:55 把 `ik_cost=0`） |
| R4 | SO-101 reach 太短，subgoal_solver 找不到可行解 | 阶段 3 | 缩小 bounds、把 trash 放在 base 30 cm 范围内；或把 trash bin 放近一些 |
| R5 | assisted grasping 检测不到 finger-object contact | 阶段 3 | 在 SO101 子类的 `_default_controller_config` 里把 `assisted_grasp_finger_links` 显式列出 |
| R6 | DINOv2 在小垃圾上分不出 patch（patch=14×14 px） | 阶段 3 | 提高相机 resolution 到 720，或镜头 zoom-in；垃圾使用高对比度颜色 |
| R7 | VLM 输出的 stage 数与 trash 数耦合，cache 后无法泛化 | 阶段 3 | cached query 只覆盖 1 个 trash 的 3-stage 流程，多 trash 由 main loop 重启实现 |
| R8 | OG `physics_frequency: 60` 对 0.5 kg 小臂偏大，舵机会震荡 | 全程 | 临时调到 `120`，`action_frequency: 30` |

## 调试脚本清单（建议创建在 `scripts/` 目录）

每个脚本对应一个里程碑，方便回归测试：

```text
scripts/
  m1_open_usd.py             # 仅 og.sim.import_object(USDObject(...))，跑 1000 step
  m2_table_and_robot.py      # m1 + 桌子 + 调位置
  m3_table_trash_bin.py      # m2 + 垃圾 + 桶
  m4_test_ik.py              # 加载 lula yaml，对若干 target 求解 IK，打印成功率
  m5_test_so101_robot.py     # 用 SO101 子类，env.robots[0]，逐关节 ±0.5 rad 运动
  m6_test_gripper.py         # open/close 100 次，验证 is_grasping
  m7_run_rekep_cached.py     # 等价 main.py --task trash --use_cached_query
```

**共同要求**：每个脚本独立、可单独运行、有 `--headless` 选项、在终端打印关键指标
（IK success rate、average step time、漂移距离）。

## 验收 / 端到端 Demo 标准

完成本方案后，以下命令应一次性跑通且生成 `videos/<timestamp>.mp4`：

```bash
python main.py --task trash --use_cached_query
```

视频里能看到：
1. SO-101 在桌面上稳定不漂；
2. 末端先移动到垃圾上方，gripper 闭合抓取；
3. 末端搬运到 trash bin 上方，gripper 打开释放；
4. 垃圾掉入桶内或桶口；
5. 整个过程 < 60 s，无穿模、无 NaN、无 segfault。

## 与原方案的差异说明（变更日志）

- 2025-04-25 v2：补充 SO-101 实测 URDF 参数、Lula yaml 模板、subgoal_solver 内部 cost 解读、风险登记、调试脚本清单、端到端验收标准，并修复阶段 3 章节编号重复。

## 推荐实施顺序

1. 用 Isaac Sim / USD Composer 打开 `so101_new_calib.usd`，检查 mesh、scale、collision、articulation。
2. **量取并填回"SO-101 关键物理参数"表格**（夹爪开口、reach、EE link 名）。
3. 在 OmniGibson 中用 `USDObject` 加载 SO-101（最小脚本，先不进 main.py）。
4. 加桌子，调整 SO-101 到桌面安装位置。
5. 加 1 个垃圾物体，验证掉落和桌面碰撞。
6. 加多个垃圾和垃圾桶，验证场景稳定（连续 1000 step 不抖）。
7. **生成 SO-101 的 Lula `robot_descriptor.yaml`**（阶段 2.1）。
8. 写 `SO101(ManipulationRobot)` 子类（阶段 2.2），最小可跑通 `env.robots[0]`。
9. 单独脚本测试：joint 控制、gripper 开合、`is_grasping`、`get_eef_pose`、IK 求解。
10. 按"3.1 必改文件清单"逐项替换 ReKep 接口（建议每改一项跑一次 smoke test）。
11. 重新校准 `bounds_min/max`、`grasp_depth`、相机位姿。
12. 准备一份 cached query (`vlm_query/trash_cleanup`)，跑 `python main.py --task trash --use_cached_query`。
13. 调通 cached 后，再跑 live VLM query（确认 prompt_template 已加垃圾任务示例）。

## 关键里程碑（DoD）

每个阶段必须满足下列 Definition of Done 才能进入下一阶段：

| 里程碑 | 验证命令 / 方式 | 通过标准 |
|--------|----------------|----------|
| M1 USD 加载 | Isaac Sim 打开 | mesh/scale/collision 全部正常 |
| M2 USDObject in OG | 最小 python 脚本 + 1000 step | 不爆炸、不漂移 |
| M3 桌面场景 | 加桌+垃圾+桶后 1000 step | 物体物理稳定 |
| M4 Lula yaml | `IKSolver(...).solve(target_pose)` | 在 reach 内能解 |
| M5 SO101 robot 子类 | `env.robots[0].action_dim == 6` 且 `is_grasping` 可返回 TRUE | 通过 |
| M6 接口替换 | `python main.py --task trash --use_cached_query` 不报 AttributeError | 通过 |
| M7 端到端 | 上述命令能完成 grasp→move→release 至少一次 | 通过 |

## 常见问题

### 模型看得见但没有碰撞

通常是 USD 只有 visual mesh，没有 collider，或 collider 被禁用。需要在 Isaac Sim 中给每个 link 添加 collision approximation。

### 一开始就爆炸或飞走

常见原因：

- 初始位置穿透桌面。
- link collider 互相穿透。
- mass / inertia 不合理。
- joint drive 太硬。
- collider 过于复杂。

### SO-101 能加载但 ReKep 控制不了

这是正常的。`USDObject` 不是 robot。必须封装成 OmniGibson robot，并提供 action space、joint names、EE link、gripper interface。

### 桌面垃圾穿过桌子

检查桌子是否有 collision，垃圾是否有 collision，二者是否都不是 `visual_only`。

### 终端 warning 很多

OmniGibson / Isaac Sim 启动时经常有插件和 particle warning。只要没有 Python traceback、segmentation fault，并且环境能正常 step，通常可以先忽略。

## 当前下一步

当前最合理的下一步是：

```text
用 USDObject 加载 SO-101 + 桌子 + 一个垃圾物体，验证 100-150 step 内物理稳定。
```

通过后，再进入 SO-101 robot 封装。
