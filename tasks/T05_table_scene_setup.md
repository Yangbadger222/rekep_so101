# T05 - 桌面场景搭建

- **阶段**: 1 - 资产验证
- **里程碑**: M3
- **依赖**: T04
- **状态**: [x] 已完成（primitive table / headless 验证）

## 目标

在 OmniGibson 中搭建桌子 + SO-101 的基础场景，调整 SO-101 位置使其"安装"在桌面上。

## 详细步骤

### 5.1 添加桌子

使用 OmniGibson 内置桌子资产或自定义桌子：

```python
table_config = {
    "type": "DatasetObject",
    "name": "table",
    "category": "breakfast_table",
    "model": "rjgmmy",
    "fixed_base": True,
    "position": [0.0, 0.0, 0.0],
}
```

备选：如果内置桌子不合适，可用简单 box primitive 代替。

Codex 实现：为避免依赖 OG 数据集资产，本阶段使用 `PrimitiveObject(Cube)` 作为
0.8 m × 0.6 m × 0.05 m 的固定桌面，桌面高度设为 0.75 m。

### 5.2 调整 SO-101 位置

核心：SO-101 底座需要刚好贴在桌面上。

1. 获取桌面高度（桌子 position.z + 桌面厚度/2）
2. 设置 SO-101 position.z = 桌面高度
3. SO-101 水平位置根据任务需要调整（建议桌子中心偏后）

```python
# 假设桌面高度 ~0.75m
so101.set_position([0.0, -0.15, 0.75])  # 桌面中心偏后
so101.set_orientation([0.0, 0.0, 0.0, 1.0])  # 默认朝向
```

当前验证位姿：

```text
table center = [0.0, 0.0, 0.725]
table top    = 0.75 m
SO-101 pos   = [0.0, -0.15, 0.75]
```

### 5.3 创建验证脚本

创建 `scripts/m2_table_and_robot.py`：

- 加载桌子（fixed_base=True）
- 加载 SO-101（fixed_base=True）
- 调整位姿使 SO-101 底座贴桌面
- 连续 1000 step
- 打印 SO-101 位置变化

### 5.4 验证固定方案

采用"位姿固定"方案（第一版，不做 fixed joint）：

- [x] 桌子 `fixed_base=True`
- [x] SO-101 `fixed_base=True`
- [x] 仿真开始后 SO-101 不掉落、不抖动
- [x] SO-101 底座不穿入桌面（按 table top height 对齐；GUI 目视未验证）

## 验收标准

- [x] 桌子正常加载（primitive table，GUI 显示未人工目视确认）
- [x] SO-101 安装在桌面高度上，headless 位姿合理
- [x] SO-101 底座不穿透桌面（按 z 高度对齐）
- [x] 1000 step 后两者位置稳定
- [x] 无物理抖动或爆炸

## 输出文件

```
scripts/m2_table_and_robot.py
```

## 备注

- 桌面高度取决于所用桌子资产，需要实际量取
- 如果桌子表面不平，SO-101 可能需要微调 Z 位置
- 后续如需更严格连接，可在 USD 中添加 fixed joint（但此阶段不需要）

## Codex 验证记录（2026-04-25）

- 新增 `scripts/m2_table_and_robot.py`，加载 primitive table + SO-101
  `so101_new_calib_og_usdobject.usd` wrapper。
- `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m2_table_and_robot.py`
  通过。
- `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m2_table_and_robot.py --headless --steps 1000 --log-every 250`
  通过：table 和 SO-101 在 step 0/250/500/750/999 的 drift 均为 0。
