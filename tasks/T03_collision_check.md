# T03 - 碰撞体检查与添加

- **阶段**: 1 - 资产验证
- **里程碑**: M1
- **依赖**: T01
- **状态**: [x] 已完成（静态 / headless 验证）

## 目标

确保 SO-101 的每个需要参与碰撞的 link 都有正确的碰撞体（collider），可在物理仿真中正常工作。

## 详细步骤

### 3.1 在 Isaac Sim 中检查现有碰撞体

打开 USD 文件，逐个检查每个 link prim 是否有 collision approximation：

| Link | 需要碰撞 | 推荐碰撞类型 | 当前状态 |
|------|---------|-------------|---------|
| base_link | 是 | convex hull | 4 个 `convexHull` collider，enabled |
| shoulder_link | 是 | convex hull | 3 个 `convexHull` collider，enabled |
| upper_arm_link | 是 | convex hull | 2 个 `convexHull` collider，enabled |
| lower_arm_link | 是 | convex hull | 3 个 `convexHull` collider，enabled |
| wrist_link | 是 | convex hull | 2 个 `convexHull` collider，enabled |
| gripper_link | 是 | convex hull | 2 个 `convexHull` collider，enabled |
| gripper_frame_link | 否（无几何） | 无 | 0 个 collider，符合 dummy EE frame 预期 |
| moving_jaw_so101_v1_link | 是 | 简单 convex | 1 个 `convexHull` collider，enabled |

### 3.2 添加缺失的碰撞体

在 Isaac Sim 中为缺少碰撞体的 link 添加 collision approximation：

1. 选中目标 prim
2. Add → Physics → Collision
3. 选择 Collision Approximation：
   - 机械臂 link：`Convex Hull` 或 `Convex Decomposition`
   - 夹爪 finger：`Convex Hull`（避免过薄的 mesh collider）

Codex 检查结果：`so101_new_calib_physics.usd` 已通过 `/colliders/<link>` prototype
和 `/so101_new_calib/<link>/collisions` instance reference 为所有实体 link 配置碰撞体；
本轮无需新增或重写 USD collision。

### 3.3 检查碰撞体质量

- [x] 碰撞体不过于复杂（均为 `convexHull`，不是动态三角网格碰撞）
- [x] 夹爪碰撞体不过薄（moving jaw 使用单独 `convexHull` collider）
- [x] 默认姿态 headless raw stage 1000 step 无漂移 / NaN
- [x] 碰撞体没有被 disabled

### 3.4 保存修改

本轮未发现缺失碰撞体，因此没有保存 / 重写 USD 文件。

## 验收标准

- [x] 所有需要碰撞的 link 都有 collider
- [x] collider 类型合理（convex hull / convex decomposition）
- [x] 默认姿态 headless raw stage 1000 step 无漂移 / NaN
- [ ] Isaac Sim GUI 中碰撞体可视化未人工目视确认
- [x] USD 重新加载正常（metadata + raw stage smoke 均通过）

## 涉及文件

```
assert/SO101/so101_new_calib/so101_new_calib.usd
assert/SO101/so101_new_calib/configuration/so101_new_calib_physics.usd
```

## 备注

- `so101_new_calib_physics.usd` 可能已包含部分碰撞配置，先检查再添加
- 如果 base collision mesh 被移除（README 提到），需要手动添加简单碰撞体
- 碰撞体过于复杂会导致仿真性能下降和不稳定

## Codex 验证记录（2026-04-25）

- 新增 `scripts/m1_check_colliders.py`，用 USD API 展开 instance prototype 并检查
  `PhysicsCollisionAPI` / `PhysicsMeshCollisionAPI`、`physics:approximation`、enabled 状态和 mesh descendant。
- `PYTHONPATH=/tmp/codex-usd-core python3 scripts/m1_check_colliders.py --detail` 通过：
  7 个实体 link 共 17 个 collider，全部为 `convexHull`；`gripper_frame_link` 为 0 个 collider。
- `PYTHONPATH=/tmp/codex-usd-core python3 scripts/m1_open_usd.py --metadata-only` 通过。
- `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m1_open_usd.py --headless --steps 1000 --log-every 250`
  通过：step 0/250/500/750/999 的 `base_link` drift 均为 0。
