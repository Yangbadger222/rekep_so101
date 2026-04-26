# T09 - IK Solver 验证

- **阶段**: 2 - Robot 封装
- **里程碑**: M4
- **依赖**: T08
- **状态**: [x] 已完成（headless / FK-generated targets 验证）

## 目标

使用 T08 生成的 Lula yaml，验证 IK Solver 能对 SO-101 在工作空间内的目标位姿成功求解。

## 详细步骤

### 9.1 创建 IK 测试脚本

创建 `scripts/m4_test_ik.py`：

```python
import numpy as np
from ik_solver import IKSolver

ik_solver = IKSolver(
    robot_description_path="assert/SO101/lula/so101_robot_descriptor.yaml",
    robot_urdf_path="assert/SO101/so101_new_calib.urdf",
    eef_name="gripper_frame_link",
    reset_joint_pos=np.array([0.0, -0.5, 1.0, 0.0, 0.0]),
    world2robot_homo=np.eye(4),  # 先用单位阵测试
)
```

### 9.2 测试目标位姿集合

在 SO-101 工作空间内选取若干目标位姿：

```python
test_targets = [
    # (位置, 四元数 [x,y,z,w]) - 在 ~30cm reach 范围内
    ([0.15, 0.0, 0.10], [0, 0, 0, 1]),    # 正前方
    ([0.10, 0.10, 0.10], [0, 0, 0, 1]),   # 右前方
    ([0.10, -0.10, 0.10], [0, 0, 0, 1]),  # 左前方
    ([0.20, 0.0, 0.05], [0, 0, 0, 1]),    # 前方低处
    ([0.0, 0.15, 0.15], [0, 0, 0, 1]),    # 右侧高处
    # 加入一些带旋转的目标
    ([0.15, 0.0, 0.10], [0.707, 0, 0, 0.707]),  # 旋转 90°
]
```

### 9.3 求解并统计

对每个目标位姿：
1. 构建 4x4 齐次变换矩阵
2. 调用 `ik_solver.solve(target_pose_homo)`
3. 记录：成功/失败、位置误差、旋转误差、迭代次数

```python
success_count = 0
for pos, quat in test_targets:
    # 构建 4x4 target
    result = ik_solver.solve(target_homo)
    if result['success']:
        success_count += 1
    print(f"Target: {pos}, Success: {result['success']}, "
          f"PosErr: {result['position_error']:.4f}, "
          f"OriErr: {result['orientation_error']:.4f}")

print(f"\nSuccess rate: {success_count}/{len(test_targets)}")
```

### 9.4 URDF mesh 路径问题

当前 URDF 使用相对路径 `assets/xxx.stl`。Lula 加载时 working dir 必须是 `assert/SO101/`。

解决方案（选一个）：
- 在调用前 `os.chdir("assert/SO101/")`
- 修改 URDF 为绝对路径副本
- 在 IKSolver 初始化中处理路径

### 9.5 调试失败情况

如果 IK 全部失败：
1. 检查 Lula yaml 中的 joint 名称是否与 URDF 完全一致
2. 检查 `eef_name` 是否与 URDF link 名一致
3. 确认目标位姿在 SO-101 工作空间内
4. 尝试放宽 tolerance
5. 增加 max_iterations

## 验收标准

- [x] IKSolver 能成功初始化（无报错）
- [x] 在工作空间内的目标，IK 成功率 ≥ 80%
- [x] 位置误差 < 5mm
- [x] 旋转误差 < 0.1 rad
- [x] 工作空间外的目标正确返回 failure

## 完成记录

- 新增 `scripts/m4_test_ik.py`，脚本会启动空的 OmniGibson headless 环境，使 `lazy.lula` 扩展可用；随后切到 `assert/SO101/`，让 URDF 中的 `assets/*.stl` 相对路径可解析。
- 用 8 组 SO-101 关节角经 FK 生成可达 6D 目标，再调用当前仓库 `ik_solver.IKSolver.solve()` 反解，避免 5 自由度手臂追随任意不可达姿态导致假失败。
- 验证结果：8/8 成功，最大位置误差约 0.003 mm，最大旋转误差约 0.00144 rad；越界目标 `[1.0, 0.0, 0.4]` 返回 failure，位置误差约 0.6646 m。

## 输出文件

```
scripts/m4_test_ik.py
```

## 备注

- 如果 Lula IK 持续失败，Plan B 是临时禁用 reachability cost（`subgoal_solver.py:55` 设 `ik_cost=0`）
- 更进一步的 Plan B 是替换为 cuRobo
- `world2robot_homo` 在实际使用时需要设为 SO-101 base 在世界坐标系中的变换
