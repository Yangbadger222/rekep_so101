# T06 - 垃圾与垃圾桶物体添加

- **阶段**: 1 - 资产验证
- **里程碑**: M3
- **依赖**: T05
- **状态**: [x] 已完成（primitive trash / open-top primitive bin）

## 目标

在桌面场景中添加可抓取的垃圾物体和垃圾桶，为后续抓取任务做准备。

## 详细步骤

### 6.1 垃圾物体选择

SO-101 夹爪开口 ~30mm，payload ≤ 100g。物体必须满足：

- **最长边 ≤ 30mm**
- **重量 ≤ 100g**
- **密度 ≤ 200 kg/m³**

推荐物体类型：
- 纸团、瓶盖、糖果、小积木、橡皮擦
- **不要用** BEHAVIOR-1K 的瓶子、罐头等大物体

### 6.2 添加垃圾物体

可选方案：

**方案 A：使用 OmniGibson 内置小物体**
```python
trash_0 = DatasetObject(
    name="trash_0",
    category="eraser",  # 或其他小物体
    model="...",
    fixed_base=False,
    visual_only=False,
)
```

**方案 B：使用 primitive 形状**
```python
from omnigibson.objects import PrimitiveObject
trash_0 = PrimitiveObject(
    name="trash_0",
    primitive_type="Cube",
    size=0.025,  # 25mm 边长
    fixed_base=False,
    visual_only=False,
    rgba=[1.0, 0.0, 0.0, 1.0],  # 红色，高对比度
)
```

Codex 实现：采用方案 B，创建 2 个 25 mm `PrimitiveObject(Cube)` 垃圾块：
`trash_0` 红色，`trash_1` 黄色，均为动态物体。

### 6.3 添加垃圾桶

垃圾桶口径建议 ≥ 100mm，给 release 留容差。

```python
trash_bin = DatasetObject(
    name="trash_bin",
    category="waste_basket",
    model="...",
    fixed_base=True,  # 固定不动
    visual_only=False,
)
```

桶放在 SO-101 工作范围内（距 base 30cm 内）。

### 6.4 设置初始位置

```python
# 垃圾放在桌面上，SO-101 可达范围内
trash_0.set_position([0.15, 0.0, 0.76])   # 桌面上方
trash_1.set_position([0.10, 0.10, 0.76])   # 第二个垃圾

# 垃圾桶放在桌面上或桌旁
trash_bin.set_position([-0.15, 0.15, 0.75])
```

Codex 实现：垃圾桶使用 primitive box 组合成开口容器（底板 + 四面墙），外尺寸约
0.14 m × 0.14 m × 0.09 m，中心位于 `[-0.15, 0.15]`，固定在桌面上。

### 6.5 创建验证脚本

创建 `scripts/m3_table_trash_bin.py`：

- 加载桌子 + SO-101 + 2 个垃圾 + 1 个垃圾桶
- 垃圾从桌面上方 1-2cm 掉落
- 验证垃圾落到桌面后停住
- 连续 1000 step

## 验收标准

- [x] 垃圾物体正常加载（GUI 显示未人工目视确认）
- [x] 垃圾掉到桌面后能停住（最终高度为桌面上方 12.5 mm）
- [x] 垃圾不被碰撞弹飞
- [x] 垃圾桶正常加载且固定
- [x] 所有物体尺寸在 SO-101 夹爪可抓范围内（25 mm）
- [x] 使用高对比度颜色（红、黄垃圾，蓝色垃圾桶）

## 输出文件

```
scripts/m3_table_trash_bin.py
```

## 备注

- 垃圾使用高对比度颜色（红、蓝、黄），避免 DINOv2 分不出 patch
- 如果用 primitive 物体，确保 collision 已启用
- 初始位置不要与桌面/SO-101 穿透

## Codex 验证记录（2026-04-25）

- 新增 `scripts/m3_table_trash_bin.py`，加载 table + SO-101 + 2 个小垃圾 + 开口垃圾桶。
- 垃圾初始中心高度为 `table_top + trash_size/2 + drop_height = 0.7775 m`，
  最终中心高度为 `0.7625 m`，对应桌面上方 12.5 mm。
- `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m3_table_trash_bin.py`
  通过。
- `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m3_table_trash_bin.py --headless --steps 1000 --log-every 250`
  通过：垃圾稳定落桌，未穿透，未弹飞。
