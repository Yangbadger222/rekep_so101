# T27 — Action quality / sim2real overhaul

## 目标（用户对齐）

1. **入口归一**：让 `python main.py --task trash` 直接能跑（自动 conda env / OMNIGIBSON_HEADLESS / OPENAI_API_KEY），与原版 ReKep UX 一致。
2. **提高仿真任务成功率**（用户选定优先级 1）。
3. **Sim2real ready**：删除 `set_joint_positions` 写死，全程走真实控制器。
4. **VLM 鲁棒性**：升级模型 + prompt 加严示例。

## 完成情况

| 项 | 状态 |
|---|---|
| `python main.py --task trash` 直接可跑（自动从 ~/.zshrc 读 OPENAI_API_KEY，自动 re-exec 到 omnigibson conda env） | ✅ |
| `--gui` flag 可关闭 headless | ✅ |
| 删除 `SO101_GRASP_FINAL_QPOS` / `SO101_DROP_QPOS` 写死 | ✅ |
| Grasp / release 走 closed-loop 直接 IK + assisted_grasp（已不再 set_joint_positions） | ✅ |
| Path solver 平滑：rot 步进 0.78→0.40，pos 步进 0.06→0.04；env intermediate rot threshold 5°→15°；IK fail tolerate streak | ✅ |
| Bounds 收口：±0.30 → ±0.25 (m), z 上界 1.10→1.05 | ✅ |
| `action_steps_per_iter` 5→50（之前永远抽不空 path） | ✅ |
| `constraint_tolerance` 0.10→0.05；`grasp_depth` 0.005→0.02 | ✅ |
| VLM 升级到 `gpt-4o-2024-11-20`，prompt 加 6 条 critical correctness rules | ✅ |
| Cached trash_cleanup constraints 简化（stage1 直接 align EEF↔trash；stage3 z offset 0.07→0.04） | ✅ |
| **端到端 cached 跑通（视频生成）** | 🔴 阻塞 — 见 §阻塞根因 |
| 端到端 live VLM 跑通 | ⏸ 未验证（cached 阻塞先解决） |

## 关键改动一览

### `main.py`
- 顶部 bootstrap：(a) 不在 omnigibson conda env 时 `os.execv` 切过去；(b) 默认 `OMNIGIBSON_HEADLESS=1`（除非 `--gui`）；(c) `OPENAI_API_KEY` 缺失时从 `~/.zshrc` 解析 codex JSON 行。
- 删除 `SO101_GRASP_FINAL_QPOS / SO101_DROP_QPOS` 常量。
- main loop：grasp / release stage 跳过 SubgoalSolver/PathSolver，直接调 `_execute_grasp_action` / `_execute_release_action`。
- `_execute_grasp_action`：4 phase（pre-grasp 7 cm 上方 → descend 到 grasp_depth/2 → close + 等 assisted grasp 触发 → lift 6 cm），全部用新增的 `_drive_to_pose` 直接 IK 驱动 joint。
- `_execute_release_action`：hover 6 cm 上方 → open。
- 新增 `_drive_to_pose(target_xyz, target_quat, gripper_action, ...)`：一次 IK + 多步 step，不做 spline 中间帧，避免 5-DOF wrist 跟不上。
- Top-down quat 用 `[[1,0,0],[0,-1,0],[0,0,-1]]`（EEF +Z 朝世界 -Z），由 `/tmp/rekep_eef_probe.py` 的 IK reachability 测试得出（EEF +X 朝 -Z 不可达）。
- 入口 `try / except BaseException` 把 OG 吞掉的 traceback 强制打印到 stderr（避免再出现 "卡了之后 Shutting Down 不知道为啥" 的现象）。

### `environment.py`
- `execute_action`：intermediate `rot_threshold 5°→15°`，`pos_threshold 0.10→0.05`，中间 max_steps=8。
- `_move_to_waypoint`：IK fail 不再立即 break，允许 `ik_fail_streak < 3` 继续尝试（hold 上一个 joint target 走一步让仿真 settle，下一轮 IK 用新 seed）。

### `configs/config.yaml`
- bounds、grasp_depth、constraint_tolerance、interpolate steps、action_steps_per_iter、path_solver opt_pos/rot_step、constraint_generator.model 等。

### `subgoal_solver.py`
- `GRASP_PREFERRED_DIR [-0.726,-0.255,0.638] → [0,0,-1]`（top-down soft hint）。
- `GRASP_COST_WEIGHT 10.0 → 1.0`（让它不再支配 IK pos error）。

### `vlm_query/prompt_template.txt`
- 加 6 条 "Critical correctness rules" 段，明确禁止 "constant cost" / "self-referential offset"（直接对应上一轮 stage2 backtrack 死循环 root cause）。

### `vlm_query/trash_cleanup/stage{1,3}_*`
- stage1 删除 SO-101-specific `grasp_window_offset`，回到原版 ReKep "align EEF with keypoint"。
- stage3 release `+0.07` → `+0.04` z offset。

### 删除
- `scripts/run_trash_live.sh`（所有逻辑都搬到 main.py）。

## 验证日志

### EEF 可达性 probe（`/tmp/rekep_eef_probe.py`）

| 候选 quat | reachable? | pos_err |
|---|---|---|
| 当前 reset orient | ❌ | 0.14 |
| identity | ❌ | 0.0 (ori miss) |
| topdown EEF +X→-Z（最初用） | ❌ | 0.14 |
| **topdown EEF +Z→-Z（采用）** | ✅ | 0.0 |
| approach -Y from +Y | ❌ | 0.10 |
| approach +X (forward) | ❌ | 0.14 |

### 端到端 cached run（v1 → v8）

| 版本 | 改动 | 现象 |
|---|---|---|
| v1 | 仅 main.py grasp/release 真闭环 | stage1 path 永远抽不空（5 步/轮），stage 永不前进 |
| v3 | `action_steps_per_iter` 5→50 | stage1 path 抽空，进入 stage 2，但立刻 backtrack（grasp 没建立） |
| v4 | grasp 时 EE 推到 keypoint 而非沿前一 pose | 还是 backtrack；IK rot_error 飙到 172° |
| v5 | `GRASP_PREFERRED_DIR=[0,0,-1]` | 还是 backtrack；rot_error 0→72° 增长 |
| v6 | top-down `+X→-Z` | rot_error 0→100° 增长（quat 不可达） |
| v7 | top-down `+Z→-Z` + grasp/release 跳过 solver | rot_error 仍涨（execute_action interpolate 中间帧不可达） |
| v8 | 新增 `_drive_to_pose`（直接 IK，不 interpolate） | **新阻塞**：register_keypoints 把 keypoint 0 匹配到 `[0.48, 1.18, 0.01]`（远离 trash 实际位置），IK 当然无解 |

## 阻塞根因（v8 暴露的新问题）

[environment.py:182-212](environment.py#L182) `register_keypoints` 把 cached metadata 中的 keypoint 0（trash 中心 `[0.15, 0, 0.7625]`）通过 `mesh.sample(1000)` 找最近 visual mesh 点。当前 exclude list 是 `['wall', 'floor', 'ceiling', 'table', 'fetch', 'robot', 'so101']`——但 SO-101 wrapper USD 的某个 visual mesh 在 (0.48, 1.18, 0.01) 区域被 sample 到，距离 keypoint 0 比 trash_0 mesh 还近。

证据：log 里 `_drive_to_pose: IK could not solve for xyz=[0.48061871 1.18361089 0.01      ]`——这些数值不在 SO-101 ±0.32 m reach 内（y=1.18 远超），所以 IK 必然失败。

**修法**（下一轮接手要做的）：
1. 加 verbose log 在 [environment.py:209](environment.py#L209) 打印 `closest_prim_path` 和 `closest_obj.name`，确认是哪个 mesh 抢走了 keypoint。
2. 把那个 mesh 的 obj name substring 加到 `exclude_names`；或者改用 `keypoint2object` 时显式按 category 过滤（trash 应该只匹配 `paper_towel` category）。
3. 另一种思路：`init_keypoint_positions` 直接用作 keypoint 位置（信任 cached metadata），不再 re-sample mesh。这是更 sim2real-friendly 的做法。

## 给下一位接手的 quick start

```bash
# 直接跑（任意 shell / 任意 python）：
python3 main.py --task trash --use_cached_query   # cached
python3 main.py --task trash                       # live VLM
python3 main.py --task trash --gui                 # 带窗口
```

入口已自包含。但 cached 当前会卡在 register_keypoints 错配上（§阻塞根因）。建议先在 `register_keypoints` 加打印，确认是哪个 mesh 抢了 keypoint 0，再决定加 exclude 名还是用 trust-metadata 路径。

## 文件改动

- 修改：`main.py`、`environment.py`、`configs/config.yaml`、`subgoal_solver.py`、`vlm_query/prompt_template.txt`、`vlm_query/trash_cleanup/stage1_subgoal_constraints.txt`、`vlm_query/trash_cleanup/stage3_subgoal_constraints.txt`
- 删除：`scripts/run_trash_live.sh`
- 新增：`tasks/T27_action_quality_sim2real.md`（本文件）

