# T13 - 末端执行器位姿读取验证

- **阶段**: 2 - Robot 封装
- **里程碑**: M5
- **依赖**: T10
- **状态**: [x] 已完成（headless / URDF FK 对照）

## 目标

验证 SO-101 robot 子类的末端执行器（EEF）位姿读取接口正确工作。

## 详细步骤

### 13.1 EEF 位姿读取

验证以下接口：

```python
# 位置 [x, y, z]
ee_pos = robot.get_eef_position()

# 方向 [qx, qy, qz, qw]
ee_ori = robot.get_eef_orientation()

# 完整位姿 [x, y, z, qx, qy, qz, qw]
ee_pose = np.concatenate([ee_pos, ee_ori])
```

### 13.2 正运动学一致性

1. 设置已知关节角度
2. 读取 EEF 位姿
3. 对比 URDF 正运动学计算结果
4. 误差应 < 1mm（位置）和 < 0.01 rad（旋转）

### 13.3 EEF 坐标系朝向确认

这是后续 subgoal_solver 抓取 cost 的关键。

在 Isaac Sim 中读取 `gripper_frame_link` 的 quaternion，确认：
- [x] EEF 的 X 轴方向是什么？（Fetch/Franka 中 X 轴指向夹爪开合方向）
- [x] EEF 的 Z 轴方向是什么？（通常指向"指向物体"方向）
- [x] 记录 SO-101 的实际 EEF 坐标系朝向

**直接影响 T18**：`subgoal_solver.py:71-76` 中 `opt_pose_homo[:3, 0]` 假设 X 轴指向夹爪开合方向。

### 13.4 world2robot_homo 变换

当 SO-101 安装在桌面上时，需要正确计算 world-to-robot 变换矩阵：

```python
# SO-101 base 在世界坐标系中的位置
robot_base_pos = [0.0, -0.15, 0.75]  # 示例
robot_base_ori = [0.0, 0.0, 0.0, 1.0]

# 构建变换矩阵
world2robot_homo = np.linalg.inv(robot2world_homo)
```

### 13.5 动态跟踪测试

1. 缓慢移动关节
2. 逐帧读取 EEF 位姿
3. 确认位姿平滑变化（无跳变）
4. 确认位姿在工作空间范围内

## 验收标准

- [x] `get_eef_position()` 返回合理的 3D 位置
- [x] `get_eef_orientation()` 返回合理的四元数
- [x] EEF 位姿与正运动学计算一致（误差 < 1mm）
- [x] EEF 坐标系朝向已记录（X/Y/Z 轴含义）
- [x] 动态跟踪平滑无跳变

## 输出文件

无新脚本，可在 `scripts/m5_test_so101_robot.py` 中追加测试。

## 备注

- EEF link 是 `gripper_frame_link`（fixed joint 挂在 `gripper_link` 上）
- 四元数格式需确认是 (x,y,z,w) 还是 (w,x,y,z)，ReKep 统一用 (x,y,z,w)
- EEF 朝向信息将直接影响 T18 中 grasp cost 的适配

## 完成记录

- 在 `scripts/m5_test_so101_robot.py` 中追加 EEF pose / FK 检查，可用 `--eef-only`
  单独运行，不改变原 T11 默认关节 smoke 行为。
- FK 对照方式：
  - 解析 `assert/SO101/so101_new_calib.urdf` 中从 `base_link` 到
    `gripper_frame_link` 的链：
    `shoulder_pan -> shoulder_lift -> elbow_flex -> wrist_flex -> wrist_roll -> gripper_frame_joint`
  - 用 URDF joint origin / axis 计算 base-to-EEF FK。
  - 用 `base_link` 当前 world pose 左乘 FK，与 `robot.get_eef_position()` /
    `robot.get_eef_orientation()` 对比。
- 验证命令：
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m5_test_so101_robot.py scripts/m6_test_gripper.py so101_robot.py`
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m5_test_so101_robot.py --headless --eef-only`
  - 回归：`/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m5_test_so101_robot.py --headless --steps-per-target 5 --hold-steps 5 --settle-steps 2 --log-every 2`
- 结果：
  - `world2robot_check identity_err=0.000000000`。
  - 三个 EEF 目标位姿均通过 FK 对照，最大静态误差约
    `fk_pos_err_m=0.000000166`、`fk_rot_err_rad=0.000000657`。
  - `get_eef_*` 与 `gripper_frame_link` 直接 link pose 完全一致：
    `link_pos_err_m=0.000000000`、`link_rot_err_rad=0.000000000`。
  - 四元数格式为 `(x, y, z, w)`，norm 约为 1。
  - 动态跟踪 80 steps：`max_step_pos_m=0.001621794`、
    `max_step_rot_rad=0.009289596`，无跳变。
  - EEF 相对 `gripper_link` 的固定轴向：
    - X = `[-1, 0, 0]`
    - Y = `[0, 1, 0]`
    - Z = `[0, 0, -1]`
    即 `gripper_frame_joint` 相对 `gripper_link` 翻转 X/Z 轴。
