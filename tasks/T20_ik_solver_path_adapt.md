# T20 - ik_solver.py SO-101 路径适配

- **阶段**: 3 - ReKep 接口适配
- **里程碑**: M6
- **依赖**: T08, T09
- **状态**: [x] 已完成（IKSolver 内部路径解析 / smoke 通过）

## 目标

修改 `ik_solver.py` 的初始化参数和路径处理，确保 SO-101 的 URDF 和 Lula yaml 能被正确加载。

## 详细步骤

### 20.1 URDF mesh 路径问题

**核心问题**：当前 URDF 使用相对路径 `filename="assets/xxx.stl"`。Lula 加载时 working dir 必须是 `assert/SO101/`，否则 mesh 找不到。

**解决方案（选其一）**：

#### 方案 A：运行时切换 working dir

```python
import os
original_dir = os.getcwd()
os.chdir("assert/SO101")
lula_robot = lazy.lula.load_robot(robot_description_path, robot_urdf_path)
os.chdir(original_dir)
```

#### 方案 B：创建绝对路径 URDF 副本

```python
# 生成一份 URDF，将所有 mesh 路径改为绝对路径
# assert/SO101/so101_new_calib_absolute.urdf
```

#### 方案 C：修改 IKSolver 类（已采用）

在 `IKSolver.__init__` 中处理路径：

```python
class IKSolver:
    def __init__(self, robot_description_path, robot_urdf_path, ...):
        # 确保 URDF 中的 mesh 路径可解析
        urdf_dir = os.path.dirname(os.path.abspath(robot_urdf_path))
        ...
```

### 20.2 修改加载路径

**文件**: `ik_solver.py`  
**行号**: 21

```python
# 现状
self.robot = lazy.lula.load_robot(robot_description_path, robot_urdf_path)

# 已添加路径处理
robot_description_path = os.path.abspath(robot_description_path)
robot_urdf_path = os.path.abspath(robot_urdf_path)
```

### 20.3 确认 EEF link 名称

```python
# 现状（可能是 Fetch 的 EEF link）
eef_name = "gripper_link"  # Fetch

# 改为
eef_name = "gripper_frame_link"  # SO-101
```

### 20.4 确认 IK 参数

```python
# IK 求解参数可能需要调整
max_iterations = 300        # 可能需要增加（5DOF 约束更紧）
position_tolerance = 0.005  # 5mm
orientation_tolerance = 0.05 # rad
```

SO-101 只有 5 DOF，某些 6DOF 目标位姿可能无解。IK solver 需要能正确处理这种情况。

## 验收标准

- [x] `IKSolver` 初始化无报错
- [x] URDF mesh 路径正确解析
- [x] Lula yaml 正确加载
- [x] EEF link 名称正确（`gripper_frame_link`）
- [x] 在 SO-101 工作空间内的目标能成功求解

## 涉及文件

```
ik_solver.py
assert/SO101/so101_new_calib.urdf
assert/SO101/lula/so101_robot_descriptor.yaml
```

## 备注

- 5DOF 机器人的 IK 本身就比 7DOF（Fetch）难解，预期成功率会低一些
- 如果 Lula IK 效果不好，Plan B 是使用 cuRobo
- mesh 路径问题是常见坑，务必先解决再测试 IK

## 完成记录

- `ik_solver.py` 改动：
  - 新增 `_resolve_path()`：支持 absolute path、相对当前 cwd、以及相对 URDF 所在目录的 descriptor / URDF 路径。
  - 新增 `_temporary_cwd()` 和 `_path_from_cwd()`：只在调用 `lazy.lula.load_robot()` 时临时切到 URDF 所在目录，确保 URDF 内 `assets/*.stl` 相对 mesh 路径可解析，并在结束后恢复原 cwd。
  - `IKSolver` 记录 `robot_description_path`、`robot_urdf_path`、`robot_asset_root`，便于 smoke / 后续调试确认。
- `environment.py` 改动：
  - 移除外层 `_temporary_cwd()` workaround，直接把 SO101 robot 暴露的 descriptor / URDF 绝对路径交给 `IKSolver`。
- `scripts/m4_test_ik.py` 改动：
  - 不再 `os.chdir(assert/SO101)`；直接用 descriptor / URDF 绝对路径调用 `IKSolver`，让 M4 smoke 覆盖 T20 路径处理。
- 验证命令：
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile ik_solver.py environment.py main.py scripts/m4_test_ik.py`
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m4_test_ik.py --headless`
  - 最小 `ReKepOGEnv` 初始化 smoke（empty `Scene`、无相机）：确认 `robot=SO101`、`action_dim=6`、`ik_asset_root=/home/badger/Desktop/Rekep/assert/SO101`、`cwd_restored=True`、`eef=gripper_frame_link`。
- 结果：
  - 编译通过。
  - M4 IK smoke 通过：8/8 个 FK-generated reach 内目标成功，越界目标 failure。
  - 删除 `environment.py` 外层 `chdir` 后，`ReKepOGEnv` 内部 IK 初始化仍通过。
