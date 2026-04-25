# 项目交接文档 — Rekep × SO-101

> 本文档是 agent 之间的实时交接日志。**任何 agent 改完代码必须更新本文件**。
> 强制约束见仓库根目录 `.agent` 文件。

---

## 1. 项目一句话目标
让 SO-101 机械臂在 OmniGibson 仿真中，被 ReKep 框架驱动，完成"桌面清理垃圾到垃圾桶"的闭环操作任务。

## 2. 必读文档
| 文档 | 用途 |
|------|------|
| `.agent` | 强制约束（必须先读） |
| `docs/SO101_OMNIGIBSON_REKEP_PLAN.md` | 总体技术方案（M1–M7 路线、风险表、Lula 模板） |
| `docs/HANDOFF.md` ←本文件 | 实时交接状态 |
| `README.md` | ReKep 原始说明 |

---

## 3. Current State（请每次更新）

最后更新：2025-04-25
最后更新人：planning-agent

### 3.1 里程碑进度
| ID | 里程碑 | 状态 | 备注 |
|----|--------|------|------|
| M0 | 方案文档 + agent 约束 | ✅ Done | `docs/SO101_OMNIGIBSON_REKEP_PLAN.md` v2 + `.agent` v1 已就绪 |
| M1 | USD 在 Isaac Sim 中可加载 | ⬜ Not Started | 需要人工在 Isaac Sim 中打开验证 |
| M2 | USDObject 在 OG 中稳定 1000 step | ⬜ Not Started | 需要 `scripts/m1_open_usd.py` |
| M3 | 桌+垃圾+桶物理稳定 | ⬜ Not Started | 需要 `scripts/m3_table_trash_bin.py` |
| M4 | Lula yaml + IK 成功率 ≥ 80% | ⬜ Not Started | yaml 模板已在方案文档；脚本 `m4_test_ik.py` 待写 |
| M5 | SO101 robot 子类 | ⬜ Not Started | 推荐参考 OG 内置 Franka |
| M6 | ReKep 接口替换不报错 | ⬜ Not Started | 详见方案 3.1 表 |
| M7 | 端到端 Demo 视频 | ⬜ Not Started | 验收标准见方案末尾 |

### 3.2 文件改动摘要（自项目启动以来）
| 文件 | 状态 | 说明 |
|------|------|------|
| `docs/SO101_OMNIGIBSON_REKEP_PLAN.md` | 新增 → v2 | 三轮迭代后定稿 |
| `.agent` | 新增 | agent 强制约束 |
| `docs/HANDOFF.md` | 新增 | 本文档 |
| 其他源代码 | 未改动 | — |

---

## 4. In Progress（开工前在此登记，收工后清空或转 Done）

> 当前无人在做。
>
> 如你正在做，请追加：
> - **agent**：xxx
> - **意图**：要改什么、为什么
> - **预计触达文件**：…
> - **开始时间**：…

---

## 5. Next Steps（建议下一位 agent 优先做的事）

按优先级排列：

1. **写 `scripts/m1_open_usd.py`**：最小脚本验证 USD 在 OG 中能加载、不爆炸、连续 1000 step。
2. **量取 SO-101 真实几何参数**：在 Isaac Sim 中量夹爪开口、reach、`gripper_frame_link` 朝向，回填到方案文档。
3. **生成 Lula `robot_descriptor.yaml`**：用方案 3.5 的模板，路径 `assert/SO101/lula/so101_robot_descriptor.yaml`；写完用 `scripts/m4_test_ik.py` 跑通 IK。
4. **写 SO101(ManipulationRobot) 子类**：参考 OG 内置 `Franka`，最小可用即可。

⚠️ 不要跳过 1-3 直接做 4。每一步都有独立 smoke test。

---

## 6. Open Issues / Known Bugs / Tech Debt

| ID | 描述 | 严重度 | 提出时间 | 提出人 |
|----|------|--------|----------|--------|
| OI-1 | 资产目录拼写为 `assert`（应为 `assets`），暂不改名 | low | 2025-04-25 | planning-agent |
| OI-2 | URDF 中 `effort=10`/`velocity=10` 是占位值，非 STS3215 真实参数 | medium | 2025-04-25 | planning-agent |
| OI-3 | `assert/SO101/` 下没有 Lula `robot_descriptor.yaml`，IK 无法直接工作 | high | 2025-04-25 | planning-agent |
| OI-4 | URDF mesh 路径是相对的（`assets/xxx.stl`），Lula 加载需注意 working dir | medium | 2025-04-25 | planning-agent |
| OI-5 | `main.py:42` 硬断言 `Fetch`，必须替换为 SO101 后才能跑 | high | 2025-04-25 | planning-agent |
| OI-6 | `configs/config.yaml` 的 `bounds_min/max` 是 Fetch 工作空间，对 SO-101 太大 | high | 2025-04-25 | planning-agent |
| OI-7 | `subgoal_solver.py:71-76` 的 grasp cost 假设 EE X 轴朝外，SO-101 需确认 | medium | 2025-04-25 | planning-agent |

---

## 7. Change Log（追加式，最新在最上）

格式：
```
### YYYY-MM-DD — <agent 标签 / 用户>
- **Files**: <文件:行号区间>
- **Summary**: <一句话>
- **Validation**: <跑了什么 + 结果>
- **Milestone impact**: <对哪个 M 有进展>
- **Notes**: <可选>
```

---

### 2025-04-25 — planning-agent
- **Files**:
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`（新增并迭代到 v2）
  - `.agent`（新增）
  - `docs/HANDOFF.md`（新增，即本文件）
- **Summary**: 完成总体方案设计、agent 强制约束、交接文档骨架。源代码未动。
- **Validation**:
  - 文档已通读自检；与 `assert/SO101/so101_new_calib.urdf` 实测参数对齐。
  - 未运行任何 Python 代码（不涉及代码改动）。
- **Milestone impact**: M0 完成，M1-M7 全部 Not Started。
- **Notes**: 下一位接手请按 §5 Next Steps 推进；务必先读 `.agent`。

---

## 8. 提示给下一位 agent

- 跑任何脚本前确认 `OPENAI_API_KEY` 是否需要（cached query 不需要）。
- OmniGibson 启动慢（30–60 s），不要因为没立刻输出就 Ctrl+C。
- 如果 Isaac Sim / OG 没装好，先在 HANDOFF.md 登记 OI-X 并停手，不要绕开。
- 改完务必回到 §4 把 In Progress 清空、§3 更新里程碑、§7 加 Change Log。
- 任何对方案的修订必须同步到 `docs/SO101_OMNIGIBSON_REKEP_PLAN.md` 末尾"变更日志"。
