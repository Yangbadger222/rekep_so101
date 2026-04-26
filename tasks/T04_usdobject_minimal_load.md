# T04 - USDObject 最小加载脚本

- **阶段**: 1 - 资产验证
- **里程碑**: M2
- **依赖**: T01, T02, T03
- **状态**: [x] 已完成（headless / USDObject wrapper 验证）

## 目标

用 OmniGibson 的 `USDObject` 加载 SO-101 模型，验证其能正常显示并参与物理仿真，连续 1000 step 不爆炸。

## 详细步骤

### 4.1 创建最小加载脚本

创建 `scripts/m1_open_usd.py`：

```python
import omnigibson as og
from omnigibson.objects import USDObject

cfg = {
    "scene": {"type": "Scene"},
    "objects": [],
    "robots": [],
}

env = og.Environment(configs=cfg)

so101 = USDObject(
    name="so101",
    usd_path="/home/badger/Desktop/Rekep/assert/SO101/so101_new_calib/so101_new_calib_og_usdobject.usd",
    fixed_base=True,
    visual_only=False,
)

env.scene.add_object(so101)
so101.set_position([0.0, 0.0, 0.5])

for i in range(1000):
    og.sim.step()
    if i % 100 == 0:
        pos = so101.get_position()
        print(f"Step {i}: position = {pos}")

og.sim.stop()
```

Codex 实现说明：当前脚本默认使用
`assert/SO101/so101_new_calib/so101_new_calib_og_usdobject.usd` 作为
`USDObject` 兼容 wrapper。原始 USD 的 joints 位于 `/so101_new_calib/joints`，
OmniGibson 1.1.1 只扫描 link 直接子 joint 来推断 root link，因此直接传原始 USD
会触发多 root link 断言。wrapper 引用原始资产，停用原 `/joints` / `/root_joint`，
并把 7 个 joint spec 复制到对应 parent link 下；visual、collision、mass、mesh
仍来自原资产。

### 4.2 运行并检查

```bash
python scripts/m1_open_usd.py
```

观察：
- [ ] 模型是否正常显示（GUI 目视未验证）
- [x] 大小是否合理（headless 加载位置与 T01 bbox/scale 一致）
- [x] 1000 step 后位置是否漂移
- [x] 是否有 segfault 或 NaN 错误
- [x] 终端是否有致命 warning

### 4.3 调试常见问题

| 问题 | 检查项 |
|------|--------|
| 看不到模型 | scale 是否正确、position 是否在视野内 |
| 模型掉落 | `fixed_base=True` 是否生效 |
| 模型抖动 | mass/inertia 是否异常、碰撞体是否穿透 |
| 模型爆炸 | 初始碰撞体穿透、joint drive 过硬 |
| 报错找不到文件 | USD 内部引用路径问题 |

### 4.4 添加 headless 支持

脚本应支持 `--headless` 参数用于无 GUI 测试。

## 验收标准

- [x] 脚本能独立运行
- [ ] SO-101 模型正常显示（GUI 目视未验证）
- [x] 1000 step 后位置漂移 < 1mm
- [x] 无 segfault、NaN、脚本级 Python traceback
- [x] 支持 `--headless` 参数

## 输出文件

```
scripts/m1_open_usd.py
scripts/m2_make_usdobject_wrapper.py
assert/SO101/so101_new_calib/so101_new_calib_og_usdobject.usd
```

## 涉及文件

```
assert/SO101/so101_new_calib/so101_new_calib.usd
assert/SO101/so101_new_calib/so101_new_calib_og_usdobject.usd
```

## 备注

- 这是最简场景测试，只有 SO-101 一个物体
- 不在 `main.py` 中测试，避免引入其他复杂依赖
- 如果 `USDObject` 不能直接用 dict 配置，以 Python API 调用为准

## Codex 验证记录（2026-04-25）

- 新增 `scripts/m2_make_usdobject_wrapper.py`，生成 OG 兼容 wrapper USD。
- 更新 `scripts/m1_open_usd.py`：默认 `--load-mode usdobject`，默认使用 wrapper；
  `--load-mode raw` 仍加载原始 USD；`--metadata-only` 仍检查原始 USD metadata。
- wrapper 静态检查：有效 root link 推断为 `base_link`；`/so101_new_calib/joints`
  与 `/so101_new_calib/root_joint` 均 inactive。
- `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m1_open_usd.py --headless --steps 1000 --log-every 250`
  通过：`root_link=base_link`，step 0/250/500/750/999 drift 均为 0。
- `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m1_open_usd.py --headless --load-mode raw --steps 1000 --log-every 250`
  回归通过：原始 USD raw stage drift 均为 0。
