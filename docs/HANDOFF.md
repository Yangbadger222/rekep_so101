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

最后更新：2026-04-26
最后更新人：claude (opus 4.7) — T27 暂停于 register_keypoints mismatch

### 3.1 里程碑进度
| ID | 里程碑 | 状态 | 备注 |
|----|--------|------|------|
| M0 | 方案文档 + agent 约束 | ✅ Done | `docs/SO101_OMNIGIBSON_REKEP_PLAN.md` v2 + `.agent` v1 已就绪 |
| M1 | USD / URDF 静态资产校验 | ✅ Done (headless/static) | T01 raw stage 1000 step drift=0；T02 URDF 静态校验通过；T03 collision 静态检查通过；GUI 目视未人工确认 |
| M2 | USDObject 在 OG 中稳定 1000 step | ✅ Done (headless/static) | T04 wrapper 模式 `root_link=base_link`；USDObject 1000 step drift=0；GUI 目视未人工确认 |
| M3 | 桌+垃圾+桶物理稳定 | ✅ Done (headless/static) | T05/T06/T07 primitive scene 1000 step 通过；SO-101/table/bin drift=0，垃圾稳定落桌 |
| M4 | Lula yaml + IK 成功率 ≥ 80% | ✅ Done (headless) | T08/T09 完成；`m4_test_ik.py` 8/8 reach 内 FK target 成功，越界 target failure |
| M5 | SO101 robot 子类 + 控制验证 | ✅ Done (headless) | T10/T11/T12/T13 完成；`SO101` 可加载，joint/gripper/EEF smoke 均通过 |
| M6 | ReKep 接口替换不报错 | ✅ Done (bounded cached smoke) | T24 alarm-bounded `main.py --task trash --use_cached_query` 完成 reset 并进入 stage loop，无启动类 ImportError/AttributeError/KeyError |
| M7 | 端到端 Demo 视频 | ✅ Done (headless cached demo) | T25 完成；`main.py --task trash --use_cached_query` 退出码 0，生成 `videos/2026-04-26-11-11-18.mp4` |
| M8 | live VLM API 端到端 | ✅ Done (headless live demo) | T26 完成；修复 `KeypointProposer` 的 kmeans NaN 死循环；`main.py --task trash`（无 cache）退出码 0，gpt-4o 返回完整 3-stage program，生成 `videos/2026-04-26-13-46-28.mp4`（1001 帧 / 33.4 s） |
| M9 | 动作质量 / sim2real ready | 🟡 In Progress (blocked) | T27 部分完成；入口归一、grasp/release 删写死改 closed-loop、轨迹平滑参数、VLM 升级；当前阻塞：`register_keypoints` 把 cached keypoint 0 错配到 SO-101 wrapper 的某个 visual mesh，IK xyz=[0.48,1.18,0.01] 无解 |

### 3.2 文件改动摘要（自项目启动以来）
| 文件 | 状态 | 说明 |
|------|------|------|
| `docs/SO101_OMNIGIBSON_REKEP_PLAN.md` | 新增 → v2 | 三轮迭代后定稿 |
| `.agent` | 新增 | agent 强制约束 |
| `docs/HANDOFF.md` | 新增 | 本文档 |
| `scripts/m1_open_usd.py` | 新增 | M1/M2 raw / metadata / USDObject wrapper smoke test；默认 `USDObject` 模式 |
| `scripts/m1_check_urdf.py` | 新增 | M1 URDF joint/link/mass/inertia/mesh 静态检查 |
| `scripts/m1_check_colliders.py` | 新增 | M1 USD collider 静态检查，展开 instance prototype 检查 collision API / approximation / enabled 状态 |
| `scripts/m2_make_usdobject_wrapper.py` | 新增 | 生成 SO-101 的 OG `USDObject` 兼容 wrapper |
| `scripts/m2_table_and_robot.py` | 新增 | M3 桌面 + SO-101 wrapper 基础场景 smoke test |
| `scripts/m3_table_trash_bin.py` | 新增 | M3 桌 + SO-101 + 垃圾 + 开口垃圾桶完整场景稳定性指标脚本 |
| `scripts/m4_test_ik.py` | 新增/修改 | M4/T20 Lula descriptor + `IKSolver` headless 验证脚本；T20 后不再手动 `chdir(assert/SO101)` |
| `so101_robot.py` | 新增/修改 | SO-101 的 OmniGibson `ManipulationRobot` 子类，本地注册 `SO101`；含 assisted grasp finger links / contact fallback；T25 调整 grasp ray 与 arm controller 输出限制 |
| `scripts/m5_load_so101_robot.py` | 新增 | M5/T10 robot class load smoke：验证 registry、`env.robots[0]`、controller keys、`action_dim=6` |
| `scripts/m5_test_so101_robot.py` | 新增 | M5/T11 arm joint control smoke：逐关节 ±0.5 rad + 1000 step hold；`--eef-only` 验证 T13 EEF / FK |
| `scripts/m6_test_gripper.py` | 新增 | M5/T12 gripper smoke：100 次开合、assisted grasp、抓取搬运、释放掉落 |
| `assert/SO101/lula/so101_robot_descriptor.yaml` | 新增 | SO-101 Lula robot descriptor：5 arm joints，`gripper` fixed，粗略 collision spheres |
| `assert/SO101/so101_new_calib/so101_new_calib_og_usdobject.usd` | 新增 | OG `USDObject` wrapper：引用原资产，停用原 `/joints` / `/root_joint`，复制 joints 到 parent link 下 |
| `assert/SO101/so101_new_calib/configuration/so101_new_calib_base.usd` | 修改 | 补 `/visuals/gripper_frame_link` 空 prim，消除 unresolved reference |
| `main.py` | 修改 | T14：入口切到 SO101，新增 `trash` task，修复 `args.task` 选择；T25 新增 SO101 cached trash grasp/release baseline |
| `constraint_generation.py` | 修改 | T24：live VLM API key lazy init，cached query 不再要求 `OPENAI_API_KEY` |
| `utils.py` | 修改 | T24：空 collision point cloud 的 collision cost 返回 0 |
| `environment.py` | 修改 | T15：SO101 6D action / 5D arm joints / Lula IK waypoint 适配 |
| `configs/config.yaml` | 修改 | T16/T17：SO101 robot/controller/workspace/外部相机配置 |
| `configs/og_scene_file_trash.json` | 新增 | T21：primitive table / trash / open-top bin ReKep trash scene |
| `.gitignore` | 修改 | T22：放行 `vlm_query/trash_cleanup/` cached query 目录 |
| `vlm_query/prompt_template.txt` | 修改 | T22：新增 tabletop trash → trash bin 示例 |
| `vlm_query/trash_cleanup/` | 新增 | T22：trash cached query metadata + stage constraints |
| `subgoal_solver.py` | 修改 | T18：SO101 EEF approach axis 显式化，grasp cost 权重调低 |
| `path_solver.py` | 修改 | T19：小工作空间路径 / 碰撞 / 关节正则参数显式化 |
| `ik_solver.py` | 修改 | T20：descriptor / URDF 路径解析内聚，Lula load 期间临时切到 URDF 目录解析 mesh |
| `scripts/t25_debug_rekep_grasp.py` | 新增 | T25 grasp/release 诊断脚本：打印 keypoint/object、assisted grasp ray、finger contact、`_ag_obj_in_hand` 与 drop qpos |
| `keypoint_proposal.py` | 修改 | T26：在 `_cluster_features` 入口过滤 NaN points + 像素数门槛 + safe min-max 归一化，避免 `kmeans_pytorch` 在天空/远处 mask 上死循环；live VLM 路径解锁 |
| `tasks/T26_keypoint_kmeans_nan_fix.md` | 新增 | T26 任务记录：根因分析、改动说明、单测 + live 端到端验证日志 |
| `main.py` | 修改 | T27：顶部 bootstrap（auto-exec omnigibson conda env / OMNIGIBSON_HEADLESS / OPENAI_API_KEY from ~/.zshrc）、`--gui` flag、删 `SO101_GRASP_FINAL_QPOS / SO101_DROP_QPOS` 写死、grasp/release 走 closed-loop top-down primitive、新增 `_drive_to_pose` 直接 IK 驱动避免 spline interpolate |
| `environment.py` | 修改 | T27：`execute_action` intermediate rot_threshold 5°→15°，`_move_to_waypoint` 容忍 ik_fail_streak < 3 |
| `configs/config.yaml` | 修改 | T27：bounds ±0.30→±0.25，grasp_depth 0.005→0.02，constraint_tolerance 0.10→0.05，action_steps_per_iter 5→50，path_solver opt_pos/rot_step 收紧，VLM 模型升级到 `gpt-4o-2024-11-20` |
| `subgoal_solver.py` | 修改 | T27：`GRASP_PREFERRED_DIR` 改为 top-down `[0,0,-1]`，`GRASP_COST_WEIGHT` 10→1（soft hint 不再支配 IK pos cost） |
| `vlm_query/prompt_template.txt` | 修改 | T27：加 6 条 "Critical correctness rules" 段，禁止 constant cost / self-referential offset 这类 stage2 backtrack 死循环根因 |
| `vlm_query/trash_cleanup/stage{1,3}_subgoal_constraints.txt` | 修改 | T27：stage1 简化为原版 ReKep `align EEF↔keypoint`；stage3 release z offset 0.07→0.04（reach 友好） |
| `tasks/T27_action_quality_sim2real.md` | 新增 | T27 任务记录：入口归一 + 动作质量 / sim2real 改造，含完整 v1→v8 验证日志和 register_keypoints mismatch 阻塞根因 |
| `scripts/run_trash_live.sh` | 删除 | T27：所有逻辑搬到 main.py 顶部 |

---

## 4. In Progress（开工前在此登记，收工后清空或转 Done）

> **T27 暂停**（用户主动叫停以保存进度）：动作质量 / sim2real 改造已完成主要 7 项（入口归一、grasp/release 真闭环、参数平滑、VLM 升级、prompt 加严、cached 约束修正），但 cached 端到端跑通被 `register_keypoints` 错配阻塞——cached metadata 的 keypoint 0（trash 中心）在 scene 中被匹配到 (0.48, 1.18, 0.01) 这个非 trash mesh 上，IK 必然无解。详见 `tasks/T27_action_quality_sim2real.md` §阻塞根因。

---

## 5. Next Steps（建议下一位 agent 优先做的事）

按优先级排列：

1. **解阻塞 T27**：在 [environment.py:209](environment.py#L209) `register_keypoints` 加 `print(f'keypoint {idx} -> {closest_obj.name} @ {closest_prim_path}')`，跑一次 cached trash，定位 keypoint 0 被错配到的 mesh，把它加到 `exclude_names`；或者改用 trust-metadata 路径（直接用 `init_keypoint_positions` 不再 re-sample mesh）。
2. **跑通 cached + 录视频**：T27 阻塞解开后跑 `python3 main.py --task trash --use_cached_query`，目标：grasp + lift + drop 全程闭环、视频生成、退出码 0。
3. **3 次 live VLM 成功率统计**：cached 跑通后跑 `python3 main.py --task trash` 三次，记录每次 stage 进展和最终 grasp/release 是否成功，给出量化成功率。
4. **GUI 录制最终 demo**：成功率 >2/3 后跑 `--gui` 录制更清晰版本。

备注：T25/T26/T27 都未伪造 grasp 状态；T27 把 SO101 写死的 set_joint_positions 全部替换为 IK + assisted_grasp 真闭环。grasp 用 top-down EEF +Z→世界 -Z（已 IK probe 验证可达），release 同 quat 在 release keypoint 上方 6 cm hover 后 open。

---

## 6. Open Issues / Known Bugs / Tech Debt

| ID | 描述 | 严重度 | 提出时间 | 提出人 |
|----|------|--------|----------|--------|
| OI-1 | 资产目录拼写为 `assert`（应为 `assets`），暂不改名 | low | 2025-04-25 | planning-agent |
| OI-2 | URDF 中 `effort=10`/`velocity=10` 是占位值，非 STS3215 真实参数 | medium | 2025-04-25 | planning-agent |
| OI-4 | URDF mesh 路径是相对的（`assets/xxx.stl`），Lula 加载需注意 working dir | medium | 2025-04-25 | planning-agent |
| OI-5 | `main.py:42` 硬断言 `Fetch`，必须替换为 SO101 后才能跑 | high | 2025-04-25 | planning-agent |
| OI-6 | `configs/config.yaml` 的 `bounds_min/max` 是 Fetch 工作空间，对 SO-101 太大 | high | 2025-04-25 | planning-agent |
| OI-7 | `subgoal_solver.py:71-76` 的 grasp cost 假设 EE X 轴朝外，SO-101 需确认 | medium | 2025-04-25 | planning-agent |
| OI-8 | 本次开工时 `git status --short` 显示 `tasks/` 为未跟踪目录；按用户要求作为任务输入使用，暂不清理 | low | 2026-04-25 | codex |
| OI-12 | 本次接手 T12 时工作树已有前序里程碑改动：`assert/SO101/so101_new_calib/configuration/so101_new_calib_base.usd`、`docs/HANDOFF.md`、`docs/SO101_OMNIGIBSON_REKEP_PLAN.md` 为 modified，`assert/SO101/lula/`、wrapper USD、`scripts/`、`so101_robot.py`、`tasks/` 为未跟踪；按用户要求继续阶段开发，不回滚 | low | 2026-04-25 | codex |
| OI-13 | 开始 T13 前 `git status --short` 仍显示前序阶段改动与 T12 文档 / 脚本更新混在同一 dirty worktree：`assert/SO101/so101_new_calib/configuration/so101_new_calib_base.usd`、`docs/HANDOFF.md`、`docs/SO101_OMNIGIBSON_REKEP_PLAN.md` 为 modified，`assert/SO101/lula/`、wrapper USD、`scripts/`、`so101_robot.py`、`tasks/` 为未跟踪；继续开发但不回滚任何现有改动 | low | 2026-04-25 | codex |
| OI-14 | 开始 T14 前 dirty worktree 仍为前序阶段累计状态：`assert/SO101/so101_new_calib/configuration/so101_new_calib_base.usd`、`docs/HANDOFF.md`、`docs/SO101_OMNIGIBSON_REKEP_PLAN.md` 为 modified，`assert/SO101/lula/`、wrapper USD、`scripts/`、`so101_robot.py`、`tasks/` 为未跟踪；继续开发但不回滚任何现有改动 | low | 2026-04-25 | codex |
| OI-15 | 开始 T15 前 dirty worktree 新增 `main.py` modified（T14），其余仍为前序阶段累计改动；继续开发但不回滚任何现有改动 | low | 2026-04-25 | codex |
| OI-16 | 开始 T16 前 dirty worktree 新增 `environment.py` modified（T15）和 `main.py` modified（T14），其余仍为前序阶段累计改动；继续开发但不回滚任何现有改动 | low | 2026-04-25 | codex |
| OI-17 | 开始 T17 前 dirty worktree 新增 `configs/config.yaml` modified（T16），以及 `environment.py` / `main.py` modified；继续开发但不回滚任何现有改动 | low | 2026-04-25 | codex |
| OI-18 | 回填记录：开始 T18 前 dirty worktree 已包含 T17 的 `configs/config.yaml` 更新，以及前序累计改动；继续开发但不回滚任何现有改动 | low | 2026-04-25 | codex |
| OI-19 | 回填记录：开始 T19 前 dirty worktree 新增 `subgoal_solver.py` modified（T18），其余仍为前序阶段累计改动；继续开发但不回滚任何现有改动 | low | 2026-04-25 | codex |
| OI-20 | 开始 T20 前 dirty worktree：`assert/SO101/so101_new_calib/configuration/so101_new_calib_base.usd`、`configs/config.yaml`、`docs/HANDOFF.md`、`docs/SO101_OMNIGIBSON_REKEP_PLAN.md`、`environment.py`、`main.py`、`path_solver.py`、`subgoal_solver.py` 为 modified；`assert/SO101/lula/`、wrapper USD、`scripts/`、`so101_robot.py`、`tasks/` 为未跟踪；继续开发但不回滚任何现有改动 | low | 2026-04-25 | codex |
| OI-21 | 开始 T21 前 dirty worktree 新增 `ik_solver.py` modified（T20）以及 `scripts/m4_test_ik.py` / `tasks/T20_ik_solver_path_adapt.md` 更新（位于未跟踪 `scripts/` / `tasks/` 内）；其余仍为前序阶段累计改动；继续开发但不回滚任何现有改动 | low | 2026-04-25 | codex |
| OI-22 | 开始 T22 前 dirty worktree 新增 `configs/og_scene_file_trash.json` untracked、`tasks/T21_scene_json.md` 更新（位于未跟踪 `tasks/` 内），以及 `configs/config.yaml` / `docs/HANDOFF.md` / `docs/SO101_OMNIGIBSON_REKEP_PLAN.md` 的 T21 更新；其余仍为前序阶段累计改动；继续开发但不回滚任何现有改动 | low | 2026-04-25 | codex |
| OI-23 | 开始 T23 前 dirty worktree 新增 `.gitignore` / `vlm_query/prompt_template.txt` modified，`vlm_query/trash_cleanup/` untracked，以及 `tasks/T22_vlm_prompt_cache.md` 更新（位于未跟踪 `tasks/` 内）；其余仍为前序阶段累计改动；继续开发但不回滚任何现有改动 | low | 2026-04-25 | codex |
| OI-24 | 开始 T24 前 dirty worktree 新增 `tasks/T23_task_entry_register.md` 更新（位于未跟踪 `tasks/` 内）以及 `docs/HANDOFF.md` / `docs/SO101_OMNIGIBSON_REKEP_PLAN.md` 的 T23 更新；其余仍为前序阶段累计改动；继续开发但不回滚任何现有改动 | low | 2026-04-25 | codex |
| OI-25 | 开始 T25 前 dirty worktree 新增 `constraint_generation.py` / `utils.py` modified（T24 修复）以及 `tasks/T24_smoke_test.md` 更新（位于未跟踪 `tasks/` 内）；其余仍为前序阶段累计改动；继续开发但不回滚任何现有改动 | low | 2026-04-25 | codex |
| OI-27 | T25 headless cached demo 使用 SO101 任务专用 grasp/drop joint baseline；live VLM query 与更通用的 6D grasp/release planner 仍是后续增强 | medium | 2026-04-26 | codex |
| OI-28 | `og_utils.pixel_to_3d_points` 对天空 / floor_plane 远端的 inf depth 反投影出 NaN points（cam0 ~30% 像素），目前 T26 在 KeypointProposer 内过滤；源头未修 | medium | 2026-04-26 | claude |
| OI-29 | live VLM trash 跑通后 OSC waypoint 中段 rot_error 经常 >30°，触发 `OSC pose not reached`；SO-101 工作空间小 + path_solver 旋转步进偏大；不影响完成抓放但轨迹抖动 | medium | 2026-04-26 | claude |
| OI-30 | T26 改前 dirty worktree 已经包含前序所有阶段累计改动（`assert/SO101/...usd`、`configs/`、`docs/`、`environment.py`、`ik_solver.py`、`main.py`、`subgoal_solver.py`、`path_solver.py`、`utils.py`、`constraint_generation.py` modified；`scripts/`、`so101_robot.py`、`tasks/`、`vlm_query/trash_cleanup/` 未跟踪）；按用户授权继续开发，不回滚 | low | 2026-04-26 | claude |
| OI-31 | `environment.register_keypoints` 把 cached metadata 的 keypoint 0（trash 中心）通过 `mesh.sample(1000)` 错配到了一个非 trash 的 visual mesh（log 显示 IK target xyz=`[0.48, 1.18, 0.01]`），导致 cached trash 任务 stage 1 IK 永远无解；建议加 verbose log 定位 mesh 后扩 `exclude_names`，或直接 trust cached metadata 不 re-sample | high | 2026-04-26 | claude |
| OI-32 | OG 启动后任何未捕获 exception 都会被 Kit 静默吞掉（traceback 不打印或冲到 stderr 末尾），T27 已在 `main.py` 入口加 `try/except BaseException` 强制 `traceback.print_exc(stderr)` 缓解；但根本上是 OG 全局 excepthook 的问题 | low | 2026-04-26 | claude |
| OI-33 | SO-101 5-DOF 不是任意 6D pose 都可达：`/tmp/rekep_eef_probe.py` 验证 EEF +X→世界 -Z 不可达（pos_err=0.14），EEF +Z→世界 -Z 可达（pos_err=0）；T27 已硬编码后者作为 grasp/release 用的 top-down quat | medium | 2026-04-26 | claude |
| OI-10 | Isaac / OG headless 启动期间大量 `Failed to create change watch ... errno=28/No space left on device`，1000 step 仍通过；疑似 inotify watch 上限或系统资源配置问题 | medium | 2026-04-25 | codex |
| OI-11 | `tasks/T01_usd_asset_check.md` 原写“26 个 STL”，实际为 13 个 `.stl` + 13 个 `.part` | low | 2026-04-25 | codex |

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

### 2026-04-26 — claude (opus 4.7) [T27 paused mid-task]
- **Files**:
  - `main.py`（顶部 bootstrap、入口异常打印、grasp/release 真闭环、新增 `_drive_to_pose`）
  - `environment.py`（`execute_action` intermediate threshold 放宽，`_move_to_waypoint` 容忍 IK fail streak）
  - `configs/config.yaml`（bounds、grasp_depth、constraint_tolerance、interpolate steps、action_steps_per_iter、path_solver opt_rot/pos_step、VLM 模型升级）
  - `subgoal_solver.py`（`GRASP_PREFERRED_DIR`、`GRASP_COST_WEIGHT` 调整）
  - `vlm_query/prompt_template.txt`（加 6 条 critical correctness rules）
  - `vlm_query/trash_cleanup/stage{1,3}_subgoal_constraints.txt`
  - `tasks/T27_action_quality_sim2real.md` (新增)
  - `docs/HANDOFF.md`
  - 删除：`scripts/run_trash_live.sh`
- **Summary**: T27 入口归一 + sim2real ready 改造；`python main.py --task trash` 现在可直接运行（自动 conda env / OMNIGIBSON_HEADLESS / OPENAI_API_KEY），SO101 grasp/release 全部走真闭环 IK + assisted_grasp。**当前阻塞**：`register_keypoints` 把 cached keypoint 0 错配到非 trash mesh（IK target xyz=[0.48,1.18,0.01] 无解），导致 cached 端到端没跑通。
- **Validation**:
  - EEF reachability probe（`/tmp/rekep_eef_probe.py`）：top-down EEF +Z→world -Z 可达，EEF +X→world -Z 不可达；硬编码前者作为 grasp/release quat。
  - `python -m py_compile` 通过（main.py、environment.py、subgoal_solver.py、keypoint_proposal.py、path_solver.py、utils.py、so101_robot.py、ik_solver.py、constraint_generation.py）。
  - Cached run 跑了 v1→v8 8 个版本，每个版本看到的失败模式都不同（path 抽不空 / IK rot_error 飙到 156° / register_keypoints 错配）；第 8 版定位到 `register_keypoints` 是当前 root cause，详见 `tasks/T27_action_quality_sim2real.md` §端到端 cached run 表格。
- **Milestone impact**: M9 In Progress (blocked)。
- **Notes**: 用户主动叫停以保存进度；下一位接手按 §5 Next Steps 第 1 步先解 register_keypoints 阻塞即可继续。

---

### 2026-04-26 — claude (opus 4.7)
- **Files**:
  - `keypoint_proposal.py`
  - `tasks/T26_keypoint_kmeans_nan_fix.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T26 / M8；定位并修复 `KeypointProposer._cluster_features` 在 live VLM 路径下 `kmeans_pytorch` 死循环的根因（cam0 ~30% 像素 depth=NaN 注入 mask），live `main.py --task trash` 端到端跑通。
- **Validation**:
  - 诊断脚本 `/tmp/rekep_diag2.py`：cam0 unique seg uid 5 个；其中 uid `764121901` 占 29.58% 且 `pts_finite=0`（全 NaN），是死循环源头。
  - KeypointProposer 单测 `/tmp/rekep_kp_check.py`：`get_keypoints` 3.41 s 返回 12 个 keypoints，3 次 kmeans 在 8–12 iter 内 tol 1e-4 收敛。
  - live 端到端：`OPENAI_API_KEY=… OMNIGIBSON_HEADLESS=1 python main.py --task trash` 退出码 0；gpt-4o 流式返回完整 3-stage program（`num_stages=3 grasp_keypoints=[9,-1,-1] release_keypoints=[-1,-1,9]`）；SubgoalSolver/PathSolver 进入 stage 1→2 循环；`videos/2026-04-26-13-46-28.mp4` 1001 帧 / 33.4 s / 640x640 / 2.85 MB；`[423.117s] Simulation App Shutting Down` 干净退出。
  - `python -m py_compile keypoint_proposal.py` 通过。
- **Milestone impact**: M8 Done (headless live VLM demo)。
- **Notes**: 根因和具体改动详见 `tasks/T26_keypoint_kmeans_nan_fix.md`。NaN 在 OG depth 渲染源头未修（OI-28），仅在 KeypointProposer 内过滤；`kmeans_pytorch` 库本身没有 `iter_limit`，所以必须从输入端杜绝 NaN/Inf。

---

### 2026-04-26 — codex
- **Files**:
  - `main.py`
  - `so101_robot.py`
  - `scripts/t25_debug_rekep_grasp.py`
  - `tasks/T25_e2e_demo.md`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T25 / M7；SO101 cached trash demo 可真实建立 assisted grasp、移动到 bin、释放并保存视频。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile main.py scripts/t25_debug_rekep_grasp.py so101_robot.py` 通过。
  - grasp 诊断通过：`_execute_grasp_action()` 后 `is_grasping=True`，`_ag_obj_in_hand=trash_0`，finger contact/raycast 命中 `trash_0`。
  - release 诊断通过：打开 gripper 后 `is_grasping=False`、`_ag_obj_in_hand=None`，`trash_0` 最终位置约 `[-0.12883, 0.11154, 0.77250]`，位于 bin footprint 内。
  - `OMNIGIBSON_HEADLESS=1 /home/badger/anaconda3/envs/omnigibson/bin/python main.py --task trash --use_cached_query` 退出码 0，生成 `videos/2026-04-26-11-11-18.mp4`。
  - OpenCV 验证视频可打开，`640x640`、`102` 帧、`30 fps`、抽样帧非空。
- **Milestone impact**: M7 Done (headless cached demo)。
- **Notes**: 未删除 grasping constraint，未手动写 `_ag_obj_in_hand`；live VLM query 和通用 6D release planner 留作后续增强。

---

### 2026-04-25 — codex
- **Files**:
  - `constraint_generation.py`
  - `utils.py`
  - `main.py`
  - `tasks/T24_smoke_test.md`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T24 / M6 smoke；cached `trash` 启动路径可在无 API key 情况下运行，进入 main stage loop。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile main.py constraint_generation.py utils.py subgoal_solver.py path_solver.py environment.py` 通过。
  - 空 collision point 单测通过：`EMPTY_COLLISION_COST_PASS 0.0`。
  - alarm-bounded 真实入口 smoke 通过：以 `sys.argv=['main.py','--task','trash','--use_cached_query']` 运行 150 秒，退出码 0；日志包含 `Reset done` 与多次 `[stage=2] backtrack to stage 1`。
- **Milestone impact**: M6 Done；下一步进入 T25 / M7 端到端抓取与释放调优。
- **Notes**: `timeout` 强杀 Isaac 会产生临时文件 cleanup traceback，因此最终 smoke 使用 Python `SIGALRM` 让入口自然展开并调用 `og.shutdown()`。当前功能阻塞是 grasp 未建立后触发 backtrack，不属于 T24 启动类错误。

---

### 2026-04-25 — codex
- **Files**:
  - `tasks/T23_task_entry_register.md`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T23；复核 `main.py` 的 `trash` task entry、scene file、cached query 目录与 CLI 参数链路。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python main.py --help` 通过，argparse 识别 `--task` / `--use_cached_query`。
  - AST/file existence 检查通过：`T23_TRASH_ENTRY_PASS`，`scene_file=./configs/og_scene_file_trash.json`，`rekep_program_dir=./vlm_query/trash_cleanup`，`num_stages=3`。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile main.py` 通过。
- **Milestone impact**: M6 In Progress；T23 Done，下一步进入 T24 cached smoke。
- **Notes**: `scene_file` 实际 OG 加载已由 T21 验证；cached query loader 已由 T22 验证。

---

### 2026-04-25 — codex
- **Files**:
  - `.gitignore`
  - `vlm_query/prompt_template.txt`
  - `vlm_query/trash_cleanup/metadata.json`
  - `vlm_query/trash_cleanup/stage1_subgoal_constraints.txt`
  - `vlm_query/trash_cleanup/stage1_path_constraints.txt`
  - `vlm_query/trash_cleanup/stage2_subgoal_constraints.txt`
  - `vlm_query/trash_cleanup/stage2_path_constraints.txt`
  - `vlm_query/trash_cleanup/stage3_subgoal_constraints.txt`
  - `vlm_query/trash_cleanup/stage3_path_constraints.txt`
  - `tasks/T22_vlm_prompt_cache.md`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T22；新增 trash cleanup cached query，并在 prompt template 中加入 tabletop trash → bin 的 3-stage 示例。
- **Validation**:
  - metadata JSON load 通过：`num_keypoints=5`，`num_stages=3`，`grasp_keypoints=[0,-1,-1]`，`release_keypoints=[-1,-1,0]`。
  - `load_functions_from_txt` 静态加载通过：stage1 subgoal 1 / path 0，stage2 subgoal 1 / path 2，stage3 subgoal 1 / path 1。
  - `vlm_query/prompt_template.txt` 包含 `pick up tabletop trash and place it into a trash bin` 示例。
- **Milestone impact**: M6 In Progress；T22 Done，下一步进入 T23 task entry 复核。
- **Notes**: `.gitignore` 已放行 `vlm_query/trash_cleanup/`，否则该 cached query 会被 `vlm_query/*` 忽略规则隐藏。

---

### 2026-04-25 — codex
- **Files**:
  - `configs/config.yaml`
  - `configs/og_scene_file_trash.json`
  - `tasks/T21_scene_json.md`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T21；ReKep trash 任务改用轻量 `Scene` + primitive table / trash / open-top bin scene file，并修复 OG semantic segmentation 不接受 `trash` / `trash_bin` category 的问题。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python - <<'PY' ... json.load/yaml.safe_load ... PY` 通过：`json_yaml_ok`。
  - 最小 `ReKepOGEnv` scene load + 120 step settle 通过：对象列表包含 `so101`、`table`、两块垃圾和五个垃圾桶部件；`trash_0` / `trash_1` 最终中心高度均约 `0.7625 m`。
  - 相机 smoke 通过：cam 0 RGB `(720, 720, 3)` / depth `(720, 720)` / seg `(720, 720)` / points `(720, 720, 3)`；cam 1 RGB `(640, 640, 3)` / depth `(640, 640)` / seg `(640, 640)` / points `(640, 640, 3)`；两路 semantic segmentation unique id 均为 5。
- **Milestone impact**: M6 In Progress；T21 Done，下一步进入 T22 cached query。
- **Notes**: JSON 中 object `name` 保留 `trash_*` / `trash_bin_*`，但 `category` 使用 OG 已知的 `paper_towel` / `pencil_holder`；OG 启动仍有 OI-10 的 change-watch `errno=28` 环境告警，但验证退出码为 0。

---

### 2026-04-25 — codex
- **Files**:
  - `ik_solver.py`
  - `environment.py`
  - `scripts/m4_test_ik.py`
  - `tasks/T20_ik_solver_path_adapt.md`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T20；`IKSolver` 内部解析 descriptor / URDF 路径，并在 Lula load 期间自动切到 URDF 所在目录解析 SO-101 `assets/*.stl`。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile ik_solver.py environment.py main.py scripts/m4_test_ik.py` 通过。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m4_test_ik.py --headless` 通过：不再手动 `chdir(assert/SO101)`，8/8 个 FK-generated reach 内目标成功，越界目标 failure。
  - 最小 `ReKepOGEnv` empty `Scene` smoke 通过：`robot=SO101`，`action_dim=6`，`ik_asset_root=/home/badger/Desktop/Rekep/assert/SO101`，`cwd_restored=True`，`eef=gripper_frame_link`。
- **Milestone impact**: M6 In Progress；T20 Done，下一步进入 T21 scene JSON。
- **Notes**: OG 启动仍有 OI-10 的 change-watch `errno=28` 环境告警，但测试退出码为 0。

---

### 2026-04-25 — codex
- **Files**:
  - `path_solver.py`
  - `tasks/T19_path_solver_adjust.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T19；把 SO-101 小工作空间的 path solver 碰撞、路径长度、IK reachability、关节正则参数显式化。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile path_solver.py` 通过。
  - 常量导入检查通过：`collision=0.8 margin=0.1 length=4.0 reach=20.0 joint=0.2`。
- **Milestone impact**: M6 In Progress；T19 Done，下一步进入 T20 `ik_solver.py` 路径适配。
- **Notes**: `configs/config.yaml` 的 `path_solver.opt_pos_step_size=0.06` 等参数已在 T16 完成；完整路径质量留到 T24 trash scene smoke 验证。

---

### 2026-04-25 — codex
- **Files**:
  - `subgoal_solver.py`
  - `tasks/T18_subgoal_solver_adapt.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T18；SO-101 抓取 approach axis 使用 EEF +X 显式常量，grasp cost 权重从 10 降到 2。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile subgoal_solver.py` 通过。
  - 常量导入检查通过：`GRASP_AXIS=0 WEIGHT=2.0 DIR=[0, 0, -1]`。
- **Milestone impact**: M6 In Progress；T18 Done，下一步进入 T19 path solver 参数调整。
- **Notes**: T13 已确认 `gripper_frame_link` pose / FK 一致；`Main._execute_grasp_action()` 沿 EEF +X 推进，因此 T18 保持 X 轴但降低姿态偏好权重。

---

### 2026-04-25 — codex
- **Files**:
  - `configs/config.yaml`
  - `tasks/T17_camera_config.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T17 静态配置；新增 SO-101 桌面任务外部 VLM 俯视相机和侧上方录制相机。
- **Validation**:
  - `get_config('./configs/config.yaml')` 加载通过；`main.vlm_camera=0`。
  - cam 0 / cam 1 四元数 norm 均为 `1.000000`。
  - `rg "cam_0|cam_1|Fetch|head|wrist" configs/config.yaml environment.py main.py -n` 未发现旧 Fetch 相机配置残留。
- **Milestone impact**: M6 In Progress；T17 Done，下一步进入 T18 subgoal solver 抓取 cost 适配。
- **Notes**: T21 已完成实际 RGB/depth/semantic segmentation/point cloud smoke；完整任务流程仍留到 T24。

---

### 2026-04-25 — codex
- **Files**:
  - `configs/config.yaml`
  - `tasks/T16_config_yaml_rewrite.md`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T16；重写 SO-101 workspace、robot config、controller、grasp 和优化参数。
- **Validation**:
  - `get_config('./configs/config.yaml')` 加载通过：`robot_type=SO101`，`action_normalize=False`，bounds main/env 一致，`physics_frequency=120`，`action_frequency=30`，arm controller 为 `JointController`，gripper controller 为 `MultiFingerGripperController`。
  - `rg "Fetch|DifferentialDrive|trunk|OperationalSpaceController" configs/config.yaml -n` 无匹配。
- **Milestone impact**: M6 In Progress；T16 Done，下一步进入 T17 camera config。
- **Notes**: T16 controller 与 T15 的 Lula IK → 5D absolute joint target 路径对齐；未在此阶段创建 trash scene JSON。

---

### 2026-04-25 — codex
- **Files**:
  - `environment.py`
  - `tasks/T15_environment_adapt.md`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T15；`environment.py` 移除 Fetch/trunk/12D action 假设，改为 SO101 6D action 与 Lula IK waypoint。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile environment.py main.py` 通过。
  - `rg "from omnigibson\\.robots\\.fetch|Fetch\\._initialize|isinstance\\(self\\.robot, Fetch\\)|trunk_control_idx|np\\.zeros\\(12\\)|action\\[10:|this action space is only for fetch" environment.py -n` 无匹配。
  - 最小 SO101 `ReKepOGEnv` headless smoke 通过：`robot=SO101`，`action_dim=6`，`arm_idx=[0,1,2,3,4]`，`gripper_idx=[5]`，`reset_shape=(5,)`，`arm_joint_positions_shape=(5,)`，`open=-1.0 close=1.0 null=0.0`。
- **Milestone impact**: M6 In Progress；T15 Done，下一步进入 T16 config rewrite。
- **Notes**: `scene_file` 现在只在非空时写入 scene config，避免普通 `Scene` smoke 被空路径污染。完整 trash 任务仍需 T16/T17/T21/T24。

---

### 2026-04-25 — codex
- **Files**:
  - `main.py`
  - `tasks/T14_main_py_adapt.md`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T14；`main.py` 从 Fetch 入口切到 SO101，并新增 `trash` task 注册。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile main.py` 通过。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python main.py --help` 通过，无 ImportError。
  - `rg "from so101_robot import SO101|isinstance\\(self.env.robot, SO101\\)|'trash'|task = task_list\\[args.task\\]" main.py -n` 确认 SO101 import / assert、trash task 和 `args.task` 选择逻辑存在。
- **Milestone impact**: M6 In Progress；T14 Done，下一步进入 T15 `environment.py`。
- **Notes**: 完整 `main.py --task trash` 仍依赖 T15/T16/T21；当前只完成入口层 Fetch→SO101 替换。

---

### 2026-04-25 — codex
- **Files**:
  - `scripts/m5_test_so101_robot.py`
  - `tasks/T13_eef_pose_verify.md`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T13 / M5；在现有关节 smoke 中追加 `--eef-only`，验证 `gripper_frame_link` EEF pose 与 URDF FK 一致，并记录坐标轴。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m5_test_so101_robot.py scripts/m6_test_gripper.py so101_robot.py` 通过。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m5_test_so101_robot.py --headless --eef-only` 通过：最大静态 FK 误差约 `0.000000166 m` / `0.000000657 rad`；`get_eef_*` 与 `gripper_frame_link` 直接 pose 完全一致；动态 80 steps 最大单步位移 `0.001621794 m`、旋转 `0.009289596 rad`。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m5_test_so101_robot.py --headless --steps-per-target 5 --hold-steps 5 --settle-steps 2 --log-every 2` 通过，确认默认 T11 joint smoke 路径未被破坏。
- **Milestone impact**: M5 Done (headless)；下一步进入 M6 / T14。
- **Notes**: EEF 相对 `gripper_link` 的固定轴向为 X=`[-1,0,0]`、Y=`[0,1,0]`、Z=`[0,0,-1]`，即 `gripper_frame_joint` 翻转 X/Z 轴。

---

### 2026-04-25 — codex
- **Files**:
  - `so101_robot.py`
  - `scripts/m6_test_gripper.py`
  - `tasks/T12_gripper_verify.md`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T12；验证 SO-101 夹爪 100 次开合、assisted grasp、抓取搬运与释放。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m6_test_gripper.py so101_robot.py` 通过。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m6_test_gripper.py --headless --cycles 1 --grasp-steps 12 --move-steps 5 --release-steps 20 --log-every-cycles 1` 通过：抓取、跟随、释放完整 PASS。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m6_test_gripper.py --headless --log-every-cycles 20` 通过：100 次开合 `max_open_err=0.00000`、`max_close_err=0.00000`；`source=ray ray=0 alpha=0.35 step=0` 触发 `is_grasping=TRUE`；`eef_motion_m=0.22489`、`cube_motion_m=0.19582`、`drop_m=0.82942`。
- **Milestone impact**: M5 In Progress；T12 Done，下一步进入 T13 EEF pose 验证。
- **Notes**: OG rigid assisted grasp 对当前 SO-101 需要两个 finger links，因此使用 `gripper_link` + `moving_jaw_so101_v1_link`。当 OG `get_contact_pairs` 与 `get_contact_data` 因 contact data 上限不一致时，`SO101._establish_grasp_rigid` 对 contact point 使用 object link pose fallback，避免已找到 ag_data 后被 assertion 中断。

---

### 2026-04-25 — codex
- **Files**:
  - `scripts/m5_test_so101_robot.py`
  - `tasks/T11_joint_control_verify.md`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T11；新增 SO-101 arm joint raw position control smoke，验证 5 个 arm joints 可独立控制。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m5_test_so101_robot.py` 通过。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m5_test_so101_robot.py --headless --steps-per-target 100 --hold-steps 1000 --log-every 250` 通过：5 个 arm joints 各自 `+0.5/-0.5 rad` 均到达目标，输出误差均为 `0.00000 rad`；1000 step hold 的 `joint_drift_rad=0.000000`、`base_drift_m=0.000000`。
- **Milestone impact**: M5 In Progress；T11 Done，下一步进入 T12 gripper / assisted grasp 验证。
- **Notes**: T11 使用 `action_normalize=False` + raw position `JointController`，便于直接验证 joint index mapping 和驱动响应；不改变 T10 的默认 controller 配置。

---

### 2026-04-25 — codex
- **Files**:
  - `so101_robot.py`
  - `scripts/m5_load_so101_robot.py`
  - `tasks/T10_so101_robot_class.md`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T10；新增本地 `SO101(ManipulationRobot)` 子类，并验证可被 OmniGibson registry 创建为 `env.robots[0]`。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile so101_robot.py` 通过。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m5_load_so101_robot.py` 通过。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m5_load_so101_robot.py --headless --steps 10 --log-every 5` 通过：`type=SO101`，`root_link=base_link`，controller keys 为 `arm_0/gripper_0`，DOF 顺序为 5 arm + `gripper`，`action_dim=6`，`grasping_mode=assisted`。
- **Milestone impact**: M5 In Progress；T10 Done，下一步进入 T11 arm joint control。
- **Notes**: `SO101.usd_path` 使用 T04 生成的 `so101_new_calib_og_usdobject.usd` wrapper。OG `BaseRobot` 初始化对当前 wrapper 的 collision boundary points 可能为空，因此 `SO101.aabb` 提供保守 fallback，避免改 OG 源码。

---

### 2026-04-25 — codex
- **Files**:
  - `assert/SO101/lula/so101_robot_descriptor.yaml`
  - `scripts/m4_test_ik.py`
  - `tasks/T08_lula_descriptor.md`
  - `tasks/T09_ik_solver_verify.md`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md`
  - `docs/HANDOFF.md`
- **Summary**: 完成 T08/T09 / M4；新增 SO-101 Lula descriptor，并用当前 `ik_solver.IKSolver` 在 OG headless 环境中验证 IK 成功率。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m4_test_ik.py` 通过。
  - YAML 静态检查通过：5 个 cspace joints 与 URDF 一致，`root_link=base_link`。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m4_test_ik.py --headless` 通过：`IKSolver` 初始化成功，8/8 个 FK-generated reach 内目标成功，最大位置误差约 0.003 mm，最大旋转误差约 0.00144 rad；越界目标 `[1.0, 0.0, 0.4]` 返回 failure。
- **Milestone impact**: M4 Done (headless)；下一步进入 T10 SO101 `ManipulationRobot` 子类。
- **Notes**: 裸 Python 下 `omnigibson.lazy.lula` 不可用；需先启动 Isaac/OG 扩展环境。URDF mesh 相对路径通过脚本切换工作目录到 `assert/SO101/` 处理。

---

### 2026-04-25 — codex
- **Files**:
  - `scripts/m3_table_trash_bin.py:1-293`
  - `tasks/T06_trash_objects.md:6, 54-55, 84-85, 98-125`
  - `tasks/T07_physics_stability.md:6, 18-28, 66-71, 85-105`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md:549, 617-620`
  - `docs/HANDOFF.md:32, 49-51, 56-67, 104-121`
- **Summary**: 完成 T06/T07；完整 primitive 桌面清理场景包含 SO-101、2 个 25 mm 高对比垃圾块和开口垃圾桶，headless 1000 step 稳定。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m3_table_trash_bin.py` 通过。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m3_table_trash_bin.py --headless --steps 1000 --log-every 250` 通过：SO-101/table/bin drift=0；trash_0/trash_1 最终高度均为桌面上方 12.5 mm；平均 step time 11.900 ms，最大 47.802 ms；NaN=0；进程退出码 0。
- **Milestone impact**: M3 Done (headless/static)；下一步进入 M4 Lula / IK。
- **Notes**: `SO-101 link/gripper 靠近垃圾时不穿模` 尚未通过运动控制验证，将随 T11/T12/T13 处理。

---

### 2026-04-25 — codex
- **Files**:
  - `scripts/m2_table_and_robot.py:1-175`
  - `tasks/T05_table_scene_setup.md:6, 31-32, 48-53, 70-81, 95-102`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md:549, 617-620`
  - `docs/HANDOFF.md:32, 48-50, 55-67, 104-118`
- **Summary**: 完成 T05；新增 primitive table + SO-101 wrapper 基础场景，桌面 top=0.75 m，SO-101 安装位姿 `[0, -0.15, 0.75]`，headless 1000 step 稳定。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m2_table_and_robot.py` 通过。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m2_table_and_robot.py --headless --steps 1000 --log-every 250` 通过：table 和 SO-101 在 step 0/250/500/750/999 的 drift 均为 0，进程退出码 0。
- **Milestone impact**: M3 In Progress；T05 Done，T06/T07 待做。
- **Notes**: 桌子使用 `PrimitiveObject(Cube)`，尺寸 0.8 × 0.6 × 0.05 m，避免依赖 OG DatasetObject 资产；GUI 目视仍未人工确认。

---

### 2026-04-25 — codex
- **Files**:
  - `scripts/m1_open_usd.py:1-268`
  - `scripts/m2_make_usdobject_wrapper.py:1-90`
  - `assert/SO101/so101_new_calib/so101_new_calib_og_usdobject.usd`（binary USD wrapper）
  - `tasks/T04_usdobject_minimal_load.md:6, 32, 37, 49-55, 64-68, 86-90, 95-123`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md:167, 193, 198-202, 519-520, 549, 617-620`
  - `docs/HANDOFF.md:31, 44-49, 54-67, 83-86, 104-121`
- **Summary**: 完成 T04 / M2；通过 OG 兼容 wrapper 解决 `USDObject` 多 root link 推断问题，`root_link=base_link`，USDObject headless 1000 step drift=0。
- **Validation**:
  - `PYTHONPATH=/tmp/codex-usd-core python3 -m py_compile scripts/m1_open_usd.py scripts/m2_make_usdobject_wrapper.py` 通过。
  - `PYTHONPATH=/tmp/codex-usd-core python3 scripts/m2_make_usdobject_wrapper.py` 通过，生成 wrapper。
  - wrapper 静态检查通过：有效 root link 为 `base_link`，原 `/so101_new_calib/joints` 和 `/so101_new_calib/root_joint` inactive。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m1_open_usd.py --headless --steps 1000 --log-every 250` 通过：step 0/250/500/750/999 drift 均为 0，进程退出码 0。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m1_open_usd.py --headless --load-mode raw --steps 1000 --log-every 250` 回归通过：原始 USD raw stage drift 均为 0，进程退出码 0。
- **Milestone impact**: M2 Done (headless/static)；解除 OI-9，下一步进入 T05 桌面场景。
- **Notes**: `USDObject` 使用 `env.scene.add_object(obj)`；OG 的 `add_asset_to_stage` 要求 wrapper 文件后缀为 `.usd`，`.usda` 会触发 assert。

---

### 2026-04-25 — codex
- **Files**:
  - `scripts/m1_check_colliders.py:1-176`
  - `tasks/T03_collision_check.md:6, 20-27, 39-52, 56-60, 75-83`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md:205-208, 549-550`
  - `docs/HANDOFF.md:30, 46-48, 53-66, 104-121`
- **Summary**: 完成 T03 collision 静态检查；确认现有 USD 已为 7 个实体 link 配好 17 个 `convexHull` collider，`gripper_frame_link` 无 collider 且符合 dummy EE frame 预期。
- **Validation**:
  - `PYTHONPATH=/tmp/codex-usd-core python3 -m py_compile scripts/m1_check_colliders.py` 通过。
  - `PYTHONPATH=/tmp/codex-usd-core python3 scripts/m1_check_colliders.py --detail` 通过：7 个实体 link 共 17 个 collider，全部为 `convexHull`，无 disabled collider。
  - `PYTHONPATH=/tmp/codex-usd-core python3 scripts/m1_open_usd.py --metadata-only` 通过。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m1_open_usd.py --headless --steps 1000 --log-every 250` 通过：step 0/250/500/750/999 drift 均为 0，进程退出码 0。
- **Milestone impact**: M1 T03 Done (headless/static)；M2 仍被 OI-9 阻塞。
- **Notes**: 本轮未改写 USD collision；GUI 碰撞体可视化仍未人工确认。`usd-core` 仅临时安装到 `/tmp/codex-usd-core` 用于检查，收工前清理。

---

### 2026-04-25 — codex
- **Files**:
  - `assert/SO101/so101_new_calib/configuration/so101_new_calib_base.usd`（binary USD；新增 `/visuals/gripper_frame_link` 空 prim）
  - `scripts/m1_open_usd.py:1-230`
  - `scripts/m1_check_urdf.py:1-153`
  - `tasks/T01_usd_asset_check.md:6, 31-35, 45, 61-65, 80-87`
  - `tasks/T02_urdf_physics_params.md:6, 20-24, 39-44, 66-71, 87-104`
  - `docs/SO101_OMNIGIBSON_REKEP_PLAN.md:53-55, 198-203, 544-545, 607-610`
  - `docs/HANDOFF.md:23-121`
- **Summary**: 完成 T01/T02 静态检查与 M1 headless smoke；修复缺失 visual prim；新增 M1 USD/URDF 检查脚本。
- **Validation**:
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m1_open_usd.py` 通过。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python -m py_compile scripts/m1_check_urdf.py` 通过。
  - `PYTHONPATH=/tmp/codex-usd-core python3 scripts/m1_open_usd.py --metadata-only` 通过：`metersPerUnit=1.0`、`upAxis=Z`、4 个 configuration 引用均为相对路径。
  - `python3 scripts/m1_check_urdf.py` 通过：8 links、7 joints、总质量 0.632006001 kg、34 个 mesh refs / 13 unique。
  - `/home/badger/anaconda3/envs/omnigibson/bin/python scripts/m1_open_usd.py --headless --steps 1000 --log-every 250` 通过：step 0/250/500/750/999 drift 均为 0，进程退出码 0。
- **Milestone impact**: M1 Done (headless/static)；M2 仍 Not Started，`USDObject` 包装阻塞已登记 OI-9。
- **Notes**: GUI 目视检查未执行；headless 启动存在 OI-10 的 change-watch 环境告警，但未导致 1000 step 失败。

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
