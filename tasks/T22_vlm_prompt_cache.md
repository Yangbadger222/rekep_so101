# T22 - VLM prompt 模板与 cached query 准备

- **阶段**: 4 - 任务定义与验证
- **里程碑**: M6
- **依赖**: 无
- **状态**: [x] 已完成（prompt 示例 + cached query）

## 目标

准备垃圾清理任务的 VLM prompt 模板和 cached query 文件，用于断网/无 API key 调试。

## 详细步骤

### 22.1 更新 prompt_template.txt

**文件**: `vlm_query/prompt_template.txt`

在已有模板中新增"清理垃圾到桶"的示例段落：

```text
Example: Use the fixed SO-101 robot mounted on the table to pick up one piece 
of trash on the tabletop and place it into the trash bin.

Stage 1 (move to trash): Move end-effector above the trash.
  - subgoal constraint: ee should be directly above the trash keypoint
  - path constraint: none

Stage 2 (grasp): Close gripper to grasp the trash.
  - subgoal constraint: ee should be at trash keypoint position
  - path constraint: ee should stay above table surface
  - grasp_keypoints: [trash keypoint index]

Stage 3 (move to bin): Move grasped trash above the trash bin.
  - subgoal constraint: ee should be above the trash bin opening
  - path constraint: grasped trash should stay above table surface
  - release_keypoints: [trash bin keypoint index]
```

### 22.2 创建 cached query 目录

```bash
mkdir -p vlm_query/trash_cleanup
```

### 22.3 创建 metadata.json

```json
{
  "init_keypoint_positions": [
    [0.15, 0.0, 0.7625],
    [-0.15, 0.215, 0.84],
    [-0.15, 0.085, 0.84],
    [-0.085, 0.15, 0.84],
    [-0.215, 0.15, 0.84]
  ],
  "num_keypoints": 5,
  "num_stages": 3,
  "grasp_keypoints": [0, -1, -1],
  "release_keypoints": [-1, -1, 0]
}
```

说明：
- 3 个阶段：grasp → lift → move-to-bin+release
- `grasp_keypoints: [0, -1, -1]`：在 stage 1 结束时抓取 keypoint 0（垃圾）
- `release_keypoints: [-1, -1, 0]`：在 stage 3 结束时释放已抓取的 keypoint 0
- 当前 cached query 使用 1 个垃圾关键点 + 4 个桶口边缘关键点；桶口中心在 constraint 内由四个 rim keypoint 的均值计算，避免把空中点注册到不稳定位置。

### 22.4 创建 stage constraint 文件

**`vlm_query/trash_cleanup/stage1_subgoal_constraints.txt`**

```python
def stage1_subgoal_constraint1(end_effector, keypoints):
    """Align the end-effector with the trash piece so the grasp action can close around it."""
    trash_pos = keypoints[0]
    cost = np.linalg.norm(end_effector - trash_pos)
    return cost
```

**`vlm_query/trash_cleanup/stage1_path_constraints.txt`**

```python
# No path constraints for the initial grasp stage.
```

**`vlm_query/trash_cleanup/stage2_subgoal_constraints.txt`**

```python
def stage2_subgoal_constraint1(end_effector, keypoints):
    """Lift the grasped trash safely above the tabletop before moving toward the bin."""
    trash_pos = keypoints[0]
    target_height = 0.84
    cost = np.maximum(0.0, target_height - trash_pos[2])
    return cost
```

**`vlm_query/trash_cleanup/stage2_path_constraints.txt`**

```python
def stage2_path_constraint1(end_effector, keypoints):
    """The robot must still be grasping the trash piece."""
    return get_grasping_cost_by_keypoint_idx(0)

def stage2_path_constraint2(end_effector, keypoints):
    """The grasped trash should stay above the tabletop during lifting."""
    trash_pos = keypoints[0]
    table_clearance = 0.78
    cost = np.maximum(0.0, table_clearance - trash_pos[2])
    return cost
```

**`vlm_query/trash_cleanup/stage3_subgoal_constraints.txt`**

```python
def stage3_subgoal_constraint1(end_effector, keypoints):
    """Place the grasped trash above the center of the trash bin opening."""
    trash_pos = keypoints[0]
    bin_opening = np.mean(keypoints[1:5], axis=0)
    target_pos = bin_opening + np.array([0.0, 0.0, 0.07])
    cost = np.linalg.norm(trash_pos - target_pos)
    return cost
```

**`vlm_query/trash_cleanup/stage3_path_constraints.txt`**

```python
def stage3_path_constraint1(end_effector, keypoints):
    """The robot must still be grasping the trash piece until it reaches the bin."""
    return get_grasping_cost_by_keypoint_idx(0)
```

### 22.5 验证 cached query

```python
from utils import load_functions_from_txt
funcs = load_functions_from_txt(
    "vlm_query/trash_cleanup/stage1_subgoal_constraints.txt",
    lambda idx: 0.0,
)
print(f"Loaded {len(funcs)} constraint functions")
```

## 验收标准

- [x] `vlm_query/trash_cleanup/` 目录存在
- [x] `metadata.json` 格式正确
- [x] 所有 stage constraint 文件存在
- [x] constraint 函数可被 `load_functions_from_txt` 正确加载
- [x] `vlm_query/prompt_template.txt` 包含垃圾任务示例
- [x] `--use_cached_query` 的 cached program 格式可被当前 loader 静态加载

## 输出文件

```
vlm_query/prompt_template.txt（更新）
vlm_query/trash_cleanup/
  metadata.json
  stage1_subgoal_constraints.txt
  stage1_path_constraints.txt
  stage2_subgoal_constraints.txt
  stage2_path_constraints.txt
  stage3_subgoal_constraints.txt
  stage3_path_constraints.txt
```

## 备注

- constraint 函数的具体实现需要根据实际 keypoint 位置调整
- 参考 `vlm_query/pen/` 目录的输出结构
- cached query 只覆盖 1 个 trash 的 3-stage 流程
- 多个 trash 由 main loop 重启机制实现
- 后续接 live VLM 时，确认 prompt_template 已包含垃圾任务示例

## 完成记录

Codex 实现：

- 更新 `vlm_query/prompt_template.txt`，加入 tabletop trash → trash bin 的 3-stage 示例。
- 新增 `vlm_query/trash_cleanup/metadata.json` 与 stage constraint 文件。
- 更新 `.gitignore`，放行 `vlm_query/trash_cleanup/`，否则 cached query 会被 `vlm_query/*` 忽略规则隐藏。
- cached query 使用 5 个 keypoint：`trash_0` + 四个 bin rim 点；release 的对象 keypoint 为 `0`，符合 `main.py` 对 `release_keypoints` 必须释放已抓取 keypoint 的约定。

验证：

```bash
/home/badger/anaconda3/envs/omnigibson/bin/python - <<'PY'
# json metadata + load_functions_from_txt smoke
PY
```

输出：

```text
stage1_subgoal_constraints.txt 1
stage1_path_constraints.txt 0
stage2_subgoal_constraints.txt 1
stage2_path_constraints.txt 2
stage3_subgoal_constraints.txt 1
stage3_path_constraints.txt 1
T22_CACHE_STATIC_PASS
```
