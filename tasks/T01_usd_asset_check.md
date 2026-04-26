# T01 - USD 资产检查与修复

- **阶段**: 1 - 资产验证
- **里程碑**: M1
- **依赖**: 无
- **状态**: [x] 已完成（headless / static 验证）

## 目标

确认 `so101_new_calib.usd` 能被正确加载，内部引用、坐标系、单位、比例均正常。

## 详细步骤

### 1.1 检查 USD 内部引用

打开 `assert/SO101/so101_new_calib/so101_new_calib.usd`，确认内部引用为**相对路径**：

```text
@configuration/so101_new_calib_robot.usd@
@configuration/so101_new_calib_physics.usd@
@configuration/so101_new_calib_base.usd@
@configuration/so101_new_calib_sensor.usd@
```

如果是绝对路径，需要改成相对路径，否则 OmniGibson 加载时找不到子文件。

### 1.2 检查坐标与单位

在 Isaac Sim / USD Composer 中打开 USD 文件，确认：

- [x] 单位是 **meter**（不是 cm 或 mm）
- [x] 坐标系为 **Z-up**
- [x] base 坐标原点在机械臂底座安装点附近
- [x] 模型整体大小合理（不会大 100 倍或小 100 倍）
- [ ] 各 link 之间无明显缝隙或重叠

### 1.3 检查 mesh 文件引用

确认 USD 中引用的 STL mesh 路径都能被正确解析。当前 STL 文件位于：

```text
assert/SO101/assets/*.stl
```

实际为 13 个 STL 文件，另有 13 个同名 `.part` 源文件。

### 1.4 检查 configuration 子文件

确认以下文件存在且内容合理：

```text
assert/SO101/so101_new_calib/configuration/
  so101_new_calib_base.usd
  so101_new_calib_physics.usd
  so101_new_calib_robot.usd
  so101_new_calib_sensor.usd
```

## 验收标准

- [x] USD 在 Isaac Sim 中能正常打开（headless raw stage）
- [ ] 所有 mesh 正确显示（GUI 目视未验证）
- [x] 模型大小合理（底座 ~5-10 cm 宽）
- [x] Z-up，单位 meter
- [x] 内部引用均为相对路径

## 涉及文件

```
assert/SO101/so101_new_calib/so101_new_calib.usd
assert/SO101/so101_new_calib/configuration/*.usd
assert/SO101/assets/*.stl
```

## 备注

- 如果模型是 cm 单位，需要在加载时设 `scale=0.01` 或在 USD 中修正。
- 不要在此阶段修改目录名 `assert` → `assets`，避免破坏现有引用。

## Codex 验证记录（2026-04-25）

- 根 USD metadata：`metersPerUnit=1.0`，`upAxis=Z`，`defaultPrim=/so101_new_calib`。
- 根 USD 内部引用均为相对路径：`configuration/so101_new_calib_{base,physics,robot,sensor}.usd`。
- 修复 `configuration/so101_new_calib_base.usd` 中缺失的 `/visuals/gripper_frame_link` 空 prim，消除 stage 打开时的 unresolved reference。
- `assert/SO101/assets/` 下共有 13 个 `.stl`，URDF 引用的 mesh 文件均存在。
- 用 `scripts/m1_open_usd.py --headless --steps 1000 --log-every 250` 通过 raw stage smoke test，`base_link` drift = 0。
- 尚未做人工 GUI 目视检查；`--load-mode usdobject` 仍被 OG 多 root link 推断阻塞，留到 T04 / T10 处理。
