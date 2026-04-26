# T21 - 场景 JSON 文件创建

- **阶段**: 4 - 任务定义与验证
- **里程碑**: M6
- **依赖**: T06, T16, T17
- **状态**: [x] 已完成（primitive scene JSON + camera smoke）

## 目标

创建 `configs/og_scene_file_trash.json`，定义垃圾清理任务的完整 OmniGibson 场景。

## 详细步骤

### 21.1 参考现有场景文件

参考 `configs/og_scene_file_pen.json` 的结构，创建垃圾清理场景。

### 21.2 场景组成

```json
{
  "scene": {
    "type": "Scene"
  },
  "robots": [
    {
      "type": "SO101",
      "name": "so101",
      "fixed_base": true,
      "position": [0.0, -0.15, 0.75],
      "orientation": [0, 0, 0, 1],
      "controller_config": {
        "arm_0": { ... },
        "gripper_0": { ... }
      },
      "grasping_mode": "assisted"
    }
  ],
  "objects": [
    {
      "type": "DatasetObject",
      "name": "table",
      "category": "breakfast_table",
      "model": "rjgmmy",
      "fixed_base": true,
      "position": [0.0, 0.0, 0.0]
    },
    {
      "type": "PrimitiveObject",
      "name": "trash_0",
      "primitive_type": "Cube",
      "size": 0.025,
      "fixed_base": false,
      "visual_only": false,
      "rgba": [1.0, 0.0, 0.0, 1.0],
      "position": [0.15, 0.0, 0.77]
    },
    {
      "type": "DatasetObject",
      "name": "trash_bin",
      "category": "waste_basket",
      "model": "...",
      "fixed_base": true,
      "position": [-0.15, 0.15, 0.75]
    }
  ],
  "cameras": [
    {
      "name": "vlm_camera",
      "position": [0.0, 0.0, 1.4],
      "orientation": [...],
      "resolution": [720, 720]
    },
    {
      "name": "record_camera",
      "position": [0.5, -0.5, 1.2],
      "orientation": [...],
      "resolution": [640, 480]
    }
  ]
}
```

### 21.3 物体放置原则

- 所有垃圾在 SO-101 工作空间内（距 base ≤ 30cm）
- 垃圾桶在工作空间边缘但可达
- 初始位置无穿透
- 垃圾从桌面上方 1-2cm 位置开始（让其自然落到桌面）

### 21.4 验证场景文件

```bash
# 使用 test.py 或最小脚本加载场景
python -c "
import json
with open('configs/og_scene_file_trash.json') as f:
    config = json.load(f)
print('Scene loaded:', config.keys())
"
```

## 验收标准

- [x] JSON 文件语法正确
- [x] 能被 OmniGibson 正确加载
- [x] 所有物体正确出现在场景中
- [x] 相机视角覆盖整个任务区域
- [x] 物体初始位置无穿透

## 输出文件

```
configs/og_scene_file_trash.json
```

## 备注

- 场景文件的具体字段格式以 OmniGibson 当前版本的 API 为准
- 物体 model id 需要查 OmniGibson 资产库确认
- 如果内置物体不合适，可以用 PrimitiveObject 替代

## 完成记录

Codex 实现：新增 `configs/og_scene_file_trash.json`，采用 M3 已验证的 primitive 几何：

- `table`：固定 `PrimitiveObject(Cube)`，尺寸 `0.8 x 0.6 x 0.05 m`，桌面高度 `0.75 m`。
- `trash_0` / `trash_1`：动态 25 mm 高对比小方块，初始中心高度 `0.7775 m`，自然落到桌面。
- `trash_bin_base` + 四个 `trash_bin_wall_*`：固定 primitive 开口垃圾桶，中心 `[-0.15, 0.15, 0.755]`。

为兼容 OmniGibson semantic segmentation，JSON 中 `category` 使用 OG 已知类别：
`breakfast_table` / `paper_towel` / `pencil_holder`；物体 name 仍保留
`trash_0`、`trash_1`、`trash_bin_*`，便于 ReKep 和任务逻辑识别。

`configs/config.yaml` 的 scene 改为轻量 `Scene`，并启用不可见 floor plane；具体物体由
`scene_file` 加载，避免继续依赖 `Rs_int` traversable scene。

验证：

- `/home/badger/anaconda3/envs/omnigibson/bin/python - <<'PY' ... json.load/yaml.safe_load ... PY` 通过：`json_yaml_ok`。
- 最小 `ReKepOGEnv` scene load + 120 step settle 通过：对象列表包含 `so101`、`table`、两块垃圾和五个垃圾桶部件；`trash_0` / `trash_1` 最终中心高度均约 `0.7625 m`。
- 相机 smoke 通过：cam 0 输出 RGB `(720, 720, 3)`、depth `(720, 720)`、seg `(720, 720)` 且 unique semantic id 为 5、points `(720, 720, 3)`；cam 1 输出 RGB `(640, 640, 3)`、depth `(640, 640)`、seg `(640, 640)` 且 unique semantic id 为 5、points `(640, 640, 3)`。

已知环境噪声：OG headless 启动仍会打印 OI-10 的 `Failed to create change watch ... errno=28`，但本阶段验证退出码均为 0。
