# T23 - main.py 任务入口注册

- **阶段**: 4 - 任务定义与验证
- **里程碑**: M6
- **依赖**: T14, T21, T22
- **状态**: [x] 已完成（T14 已注册，T23 复核通过）

## 目标

在 `main.py` 中注册 `trash` 任务入口，使 `python main.py --task trash` 能正确运行。

## 详细步骤

### 23.1 添加任务条目

**文件**: `main.py`  
**位置**: `task_list` 字典（第 368 行附近）

```python
task_list = {
    # ... 已有任务 ...
    "trash": {
        "scene_file": "configs/og_scene_file_trash.json",
        "instruction": (
            "Use the fixed SO-101 robot mounted on the table to pick up "
            "one piece of trash on the tabletop and place it into the trash bin."
        ),
        "rekep_program_dir": "vlm_query/trash_cleanup",
    },
}
```

### 23.2 确认参数引用

确认 `scene_file`、`instruction`、`rekep_program_dir` 指向正确的文件：

- [x] `configs/og_scene_file_trash.json` 存在（T21）
- [x] `vlm_query/trash_cleanup/` 目录存在且有 cached query（T22）
- [x] instruction 文本与 prompt_template 示例一致

### 23.3 确认命令行参数

```bash
python main.py --task trash --use_cached_query
```

确认 argparse 能正确识别 `--task trash`。

### 23.4 确认 cached query 加载路径

当 `--use_cached_query` 时，`main.py` 会从 `rekep_program_dir` 加载：
- `metadata.json`
- `stage{i}_subgoal_constraints.txt`
- `stage{i}_path_constraints.txt`

确认路径拼接正确。

## 验收标准

- [x] `--task trash` 不报 KeyError
- [x] `--use_cached_query` 能加载 trash_cleanup 的 cached 文件
- [x] instruction 文本正确传递给 constraint_generator
- [x] scene_file 能被 OmniGibson 加载

## 涉及文件

```
main.py
```

## 备注

- 单 trash 任务描述足够，多 trash 由 main loop 的 backtrack 机制处理
- 如果需要支持多个 trash 任务变体，可以注册 `trash_1`, `trash_2` 等

## 完成记录

`main.py` 的 `trash` task entry 已在 T14 添加；T23 未新增代码路径，只做复核：

- `scene_file = './configs/og_scene_file_trash.json'`
- `instruction` 包含 SO-101、tabletop trash、trash bin 描述
- `rekep_program_dir = './vlm_query/trash_cleanup'`

验证：

- `/home/badger/anaconda3/envs/omnigibson/bin/python main.py --help` 通过，argparse 识别 `--task` / `--use_cached_query`。
- AST + 文件存在性复核通过：`T23_TRASH_ENTRY_PASS`，`num_stages=3`，三阶段 subgoal/path constraint 文件均存在。
- `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile main.py` 通过。
- `scene_file` 实际 OG 加载已由 T21 验证；cached query loader 已由 T22 验证。
