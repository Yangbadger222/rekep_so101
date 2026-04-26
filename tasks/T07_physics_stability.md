# T07 - 场景物理稳定性验证

- **阶段**: 1 - 资产验证
- **里程碑**: M3
- **依赖**: T06
- **状态**: [x] 已完成（headless 1000 step 指标验证）

## 目标

验证完整桌面场景（桌子 + SO-101 + 垃圾 + 垃圾桶）在连续物理仿真中的稳定性。

## 详细步骤

### 7.1 稳定性检查清单

在完整场景中连续运行至少 1000 step，逐项检查：

- [x] SO-101 加载后位置正确
- [x] SO-101 不掉落
- [x] SO-101 不抖动（位置漂移 < 1mm）
- [x] SO-101 不与桌子初始穿透
- [x] 垃圾掉到桌面后能停住
- [x] 垃圾不穿过桌子
- [x] 垃圾不被碰撞弹飞
- [ ] SO-101 link/gripper 靠近垃圾时不穿模（尚未控制机械臂接近垃圾）
- [x] 场景能连续 step 至少 1000 步
- [x] 无 NaN 值出现
- [x] 无 segfault

### 7.2 定量指标采集

脚本应输出以下指标：

```text
SO-101 位置漂移（1000 step 后）: x.xxx mm
垃圾 0 最终位置偏移: x.xxx mm
垃圾 1 最终位置偏移: x.xxx mm
平均 step time: x.xxx ms
最大 step time: x.xxx ms
NaN 计数: 0
```

### 7.3 穿模问题排查

如果出现穿模，按优先级检查：

1. USD 是否有 collision
2. collision 是否被 disabled
3. collider 是否过薄
4. collider 是否太复杂（三角网格碰撞不适合动态物体）
5. scale 是否正确
6. 初始位置是否穿透
7. physics timestep 是否太大（当前 60Hz，可能需要调到 120Hz）

### 7.4 physics 参数调优

如果稳定性有问题，尝试：

```yaml
physics_frequency: 120  # 从 60 提高到 120
action_frequency: 30    # 对应调整
```

## 验收标准

- [x] 完整场景连续 1000 step 无物理错误
- [x] SO-101 位置漂移 < 1mm
- [x] 垃圾物体物理行为合理
- [x] 无桌面穿模现象
- [x] 无 NaN 或 segfault
- [x] 输出定量稳定性指标

## 输出文件

```
scripts/m3_table_trash_bin.py  （更新，加入指标输出）
```

## 备注

- 这是进入阶段 2 的门槛，物理稳定性不通过不要继续
- 如果 OG `physics_frequency: 60` 对 0.5kg 小臂偏大，舵机会震荡，需调到 120
- 如果模型持续不稳定，可临时设 `kinematic_only=True` 跑视觉验证

## Codex 验证记录（2026-04-25）

运行：

```bash
/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m3_table_trash_bin.py --headless --steps 1000 --log-every 250
```

结果：

```text
SO-101 位置漂移（1000 step 后）: 0.000 mm
桌子位置漂移（1000 step 后）: 0.000 mm
垃圾桶最大位置漂移: 0.000 mm
垃圾 0 最终位置偏移: 14.319 mm
垃圾 1 最终位置偏移: 14.319 mm
垃圾 0/1 最终高度: 桌面上方 12.500 mm
平均 step time: 11.900 ms
最大 step time: 47.802 ms
NaN 计数: 0
```
