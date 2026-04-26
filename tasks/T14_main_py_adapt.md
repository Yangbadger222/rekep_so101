# T14 - main.py Fetch 引用替换

- **阶段**: 3 - ReKep 接口适配
- **里程碑**: M6
- **依赖**: T10
- **状态**: [x] 已完成（入口层替换；full run 依赖后续 T15/T16/T21）

## 目标

将 `main.py` 中所有 Fetch 机器人的硬编码引用替换为 SO-101。

## 详细修改清单

### 14.1 导入替换

**文件**: `main.py`  
**行号**: 第 14 行

```python
# 现状
from omnigibson.robots.fetch import Fetch

# 改为
from so101_robot import SO101
```

### 14.2 类型断言替换

**文件**: `main.py`  
**行号**: 第 42 行

```python
# 现状
assert isinstance(self.env.robot, Fetch)

# 改为
assert isinstance(self.env.robot, SO101)
```

### 14.3 IKSolver 参数替换

**文件**: `main.py`  
**行号**: 第 44-48 行

```python
# 现状（使用 Fetch 的描述文件）
self.ik_solver = IKSolver(
    robot_description_path=...,  # Fetch yaml
    robot_urdf_path=...,         # Fetch urdf
    eef_name=...,                # Fetch EEF
    ...
)

# 改为
self.ik_solver = IKSolver(
    robot_description_path="assert/SO101/lula/so101_robot_descriptor.yaml",
    robot_urdf_path="assert/SO101/so101_new_calib.urdf",
    eef_name="gripper_frame_link",
    reset_joint_pos=self.env.reset_joint_pos,
    world2robot_homo=self.env.world2robot_homo,
)
```

### 14.4 任务列表更新

**文件**: `main.py`  
**行号**: 第 368 行附近

添加 `trash` 任务条目：

```python
task_list = {
    ...
    "trash": {
        "scene_file": "configs/og_scene_file_trash.json",
        "instruction": "Use the fixed SO-101 robot mounted on the table to pick up one piece of trash on the tabletop and place it into the trash bin.",
        "rekep_program_dir": "vlm_query/trash_cleanup",
    },
}
```

## 验收标准

- [x] `from so101_robot import SO101` 无报错
- [x] `isinstance` 检查通过
- [x] IKSolver 用 SO-101 参数初始化
- [x] `trash` 任务在 task_list 中注册
- [x] `python main.py --help` 不报 ImportError；`--task trash` 已接入 task_list

## 涉及文件

```
main.py
```

## 备注

- 每改一项跑一次 smoke test
- 先用 `git diff` 留底再改
- IKSolver 的 URDF mesh 路径问题需要同时处理（参见 T20）

## 完成记录

- `main.py` 改动：
  - `from omnigibson.robots.fetch import Fetch` 替换为 `from so101_robot import SO101`。
  - IK 初始化前的类型断言改为 `isinstance(self.env.robot, SO101)`。
  - IKSolver 继续读取 `self.env.robot.robot_arm_descriptor_yamls`、
    `self.env.robot.urdf_path` 和 `self.env.robot.eef_link_names`；由于 T10 的
    `SO101` 已暴露这些属性，因此不会在 `main.py` 中重复硬编码路径。
  - 新增 `trash` task 条目。
  - 修复入口原本忽略 `args.task`、总是运行 `pen` 的问题，现在使用
    `task = task_list[args.task]` 并对未知 task 抛出清晰错误。
- 验证命令：
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile main.py`
  - `/home/badger/anaconda3/envs/omnigibson/bin/python main.py --help`
  - `rg "from so101_robot import SO101|isinstance\\(self.env.robot, SO101\\)|'trash'|task = task_list\\[args.task\\]" main.py -n`
- 结果：
  - `main.py --help` 正常显示 CLI，无 ImportError。
  - 静态检查确认 SO101 import、SO101 isinstance、trash task 和 `args.task`
    选择逻辑均已存在。
- 未执行完整 `python main.py --task trash`，因为后续 T15/T16/T21 尚未完成：
  - `environment.py` 仍有 Fetch/trunk/action-space 假设。
  - `configs/config.yaml` 仍指向 Fetch。
  - `configs/og_scene_file_trash.json` 由 T21 创建。
