# T12 - 夹爪开合与抓取验证

- **阶段**: 2 - Robot 封装
- **里程碑**: M5
- **依赖**: T10
- **状态**: [x] 已完成（headless / assisted grasp smoke）

## 目标

验证 SO-101 的夹爪能正常开合，`is_grasping` 在 assisted mode 下能正确返回抓取状态。

## 详细步骤

### 12.1 创建夹爪测试脚本

创建 `scripts/m6_test_gripper.py`：

```python
# 测试夹爪开合
# 测试 is_grasping 状态
# 测试 assisted grasping 与物体交互
```

### 12.2 基础开合测试

```python
# 打开夹爪
robot.apply_action(open_gripper_action)  # gripper action = -1.0
for _ in range(50):
    og.sim.step()

# 关闭夹爪
robot.apply_action(close_gripper_action)  # gripper action = 1.0
for _ in range(50):
    og.sim.step()
```

重复 100 次，验证：
- [x] 夹爪每次都能完全打开
- [x] 夹爪每次都能完全关闭
- [x] 开合动作平滑，无抖动

### 12.3 抓取状态测试

1. 在夹爪前方放一个小物体
2. 关闭夹爪
3. 检查 `robot.is_grasping()` 返回值

```python
# 放置物体在夹爪附近
trash.set_position(gripper_position + offset)

# 关闭夹爪
robot.apply_action(close_gripper_action)
for _ in range(100):
    og.sim.step()

# 检查抓取状态
grasping = robot.is_grasping()
print(f"is_grasping: {grasping}")  # 期望 True
```

### 12.4 Assisted grasping 配置检查

确认以下配置正确：

```python
grasping_mode: "assisted"
assisted_grasp_finger_links: ["gripper_link", "moving_jaw_so101_v1_link"]
```

如果 `is_grasping` 始终返回 False：
1. 检查 `assisted_grasp_finger_links` 是否正确列出
2. 检查 finger link 是否有碰撞体
3. 检查物体是否在夹爪接触范围内
4. 检查 `assisted` 模式的接触阈值

### 12.5 抓取后搬运测试

1. 抓取物体
2. 移动 arm joints
3. 检查物体是否跟随移动（assisted mode 应该会保持）
4. 打开夹爪
5. 检查物体是否被释放

## 验收标准

- [x] 夹爪能正常开合
- [x] 100 次开合无故障
- [x] `is_grasping` 在抓住物体时返回 True
- [x] `is_grasping` 在未抓住时返回 False
- [x] 抓取后物体跟随末端移动
- [x] 释放后物体掉落

## 输出文件

```
scripts/m6_test_gripper.py
```

## 备注

- SO-101 夹爪开口仅 ~30mm，测试物体必须小于此尺寸
- 物理 grasping（非 assisted）几乎一定不稳，不要尝试
- `gripper` joint 范围: [-0.17453, 1.74533]（-10°~+100°）
- 本地 OG controller 实测使用 `open_qpos=-0.174533`、`closed_qpos=1.74533`；
  normalized gripper action `-1.0` 打开，`1.0` 闭合。

## 完成记录

- 新增 `scripts/m6_test_gripper.py`：
  - 加载 T10 的 `SO101` robot，`grasping_mode="assisted"`。
  - 100 次执行 `open_command=-1.0` / `close_command=1.0`，校验 controller open/closed qpos。
  - 使用 2 cm primitive cube 验证 assisted grasp；脚本先尝试 `gripper_link`
    局部确定性放置 `(0.010, 0.000, -0.030)`，再 fallback 到 assisted grasp ray alpha 扫描。
  - 抓取后移动 arm，验证 cube 跟随；打开夹爪并开启重力，验证 cube 掉落。
- `so101_robot.py` 的 assisted grasp 配置使用两个 finger links：
  - `gripper_link`
  - `moving_jaw_so101_v1_link`
  这是 OG rigid assisted grasp 对“两指接触”判定的要求。
- 验证命令：
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m6_test_gripper.py so101_robot.py`
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m6_test_gripper.py --headless --log-every-cycles 20`
- 结果：
  - 100 次开合全部通过，`max_open_err=0.00000`，`max_close_err=0.00000`。
  - 初始未抓取状态为 `FALSE`。
  - `source=ray ray=0 alpha=0.35 step=0` 触发 assisted grasp，`is_grasping=TRUE`。
  - 搬运测试：`eef_motion_m=0.22489`，`cube_motion_m=0.19582`。
  - 释放测试：`drop_m=0.82942`。
  - 脚本输出 `PASS: SO101 gripper open/close and assisted grasp verified`。
