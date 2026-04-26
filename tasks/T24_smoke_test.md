# T24 - 冒烟测试（无报错启动）

- **阶段**: 4 - 任务定义与验证
- **里程碑**: M6
- **依赖**: T14, T15, T16, T17, T18, T19, T20, T21, T22, T23
- **状态**: [x] 已完成（bounded cached smoke 进入 stage loop）

## 目标

确保 `python main.py --task trash --use_cached_query` 能启动运行，不出现 ImportError、AttributeError、KeyError 等致命错误。

## 详细步骤

### 24.1 逐步启动测试

按顺序检查各组件能否正常初始化：

#### Step 1: 配置加载
```python
from utils import get_config
config = get_config("configs/config.yaml")
# 检查无报错
```

#### Step 2: 场景加载
```python
# OmniGibson 环境初始化
# SO-101 robot 加载
# 桌面场景加载
```

#### Step 3: 求解器初始化
```python
# IKSolver 初始化（Lula yaml + URDF）
# SubgoalSolver 初始化
# PathSolver 初始化
```

#### Step 4: VLM 组件初始化
```python
# KeypointProposer 初始化（DINOv2）
# ConstraintGenerator 初始化（可跳过 GPT-4o）
```

#### Step 5: 任务执行
```python
# perform_task("trash")
# _execute() main loop 开始
```

### 24.2 错误检查清单

| 错误类型 | 可能原因 | 对应任务 |
|---------|---------|---------|
| ImportError: SO101 | so101_robot.py 未创建 | T10 |
| AssertionError: isinstance | main.py 断言未改 | T14 |
| FileNotFoundError: yaml | Lula yaml 不存在 | T08 |
| FileNotFoundError: json | scene json 不存在 | T21 |
| KeyError: 'trash' | task_list 未注册 | T23 |
| AttributeError: trunk | environment.py 未改 | T15 |
| IndexError: action dim | action 维度不匹配 | T15 |
| ValueError: bounds | bounds 配置错误 | T16 |

### 24.3 完整启动命令

```bash
python main.py --task trash --use_cached_query
```

### 24.4 最小成功标准

- 环境初始化成功
- SO-101 robot 加载成功
- IK solver 初始化成功
- 场景渲染至少 1 帧
- main loop 开始执行（即使优化不收敛）

### 24.5 常见 warning 处理

OmniGibson / Isaac Sim 启动时常有插件和 particle warning。以下可忽略：
- USD plugin warnings
- PhysX particle warnings
- 已知的 deprecation warnings

以下不可忽略：
- Python traceback
- segmentation fault
- NaN in simulation
- robot action dimension mismatch

## 验收标准

- [x] 命令启动无 ImportError
- [x] 命令启动无 AssertionError
- [x] 命令启动无启动路径 FileNotFoundError
- [x] 命令启动无 KeyError
- [x] 命令启动无 AttributeError
- [x] 环境能初始化并开始仿真
- [x] main loop 开始执行
- [x] 至少连续运行 100 step 无崩溃

## 涉及文件

所有已修改的文件。

## 备注

- 此任务是所有阶段 3 修改的集成测试
- 如果某步失败，定位到对应的任务进行修复
- 不要求端到端成功完成任务，只要求启动和运行不报错
- 优化不收敛、抓取失败等是功能问题，不在此任务范围

## 完成记录

T24 期间修复了三个启动 / 集成问题：

- `constraint_generation.py`：`OPENAI_API_KEY` 检查延后到 live VLM `generate()`，cached query 模式不再需要 API key。
- `utils.py`：`calculate_collision_cost()` 对空 collision point cloud 返回 `0.0`，避免 SO-101 当前 gripper/wrist collision points 为空时 reshape 崩溃。
- `main.py`：主入口用 `try/finally` 调用 `og.shutdown()`，减少异常退出后的 Isaac / OG 139 风险。

验证：

- `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile main.py constraint_generation.py utils.py subgoal_solver.py path_solver.py environment.py` 通过。
- 空 collision point 单测通过：`EMPTY_COLLISION_COST_PASS 0.0`。
- 真实入口 alarm-bounded smoke 通过：用 `runpy` 设置 `sys.argv = ['main.py', '--task', 'trash', '--use_cached_query']`，150 秒后 Python alarm 正常退出，退出码 `0`。日志显示：

```text
Reset done.
[stage=2] backtrack to stage 1
...
T24_ALARM_TIMEOUT_REACHED
```

结论：`main.py --task trash --use_cached_query` 已完成配置加载、场景/robot/IK/solver/camera/cached query 初始化，并进入 main stage loop；未出现 ImportError、AssertionError、KeyError、AttributeError 等启动类错误。

遗留到 T25 / M7：stage 2 反复 backtrack 到 stage 1，说明 stage 1 grasp 后 `get_grasping_cost_by_keypoint_idx(0)` 未满足，端到端抓取/搬运还需要继续调优。
