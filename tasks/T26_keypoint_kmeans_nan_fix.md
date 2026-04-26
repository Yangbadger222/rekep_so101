# T26 — KeypointProposer kmeans NaN deadlock fix (live VLM unblock)

## 背景

T25 完成的是 cached query baseline；切到 live VLM API（`main.py --task trash` 不带
`--use_cached_query`）时，`KeypointProposer._cluster_features` 调用 `kmeans_pytorch`
会无限 loop，整条 pipeline 卡死在 keypoint 提取阶段，VLM 请求根本发不出去。

## 根因

诊断脚本（`/tmp/rekep_diag2.py`）打印 `cam[0]` 各 mask 的统计：

| seg uid (instance) | 像素数 | 占比 | `max_mask_ratio=0.5` 跳过？ | points 有效？ |
|---|---|---|---|---|
| 468506626 | 1324 | 0.26% | ❌ | ✅ 1324 |
| 667506563 | 26207 | 5.06% | ❌ | ✅ |
| **764121901** | **153346** | **29.58%** | **❌** | **🔥 0 个 finite** |
| 1662940987 | 23699 | 4.57% | ❌ | ✅ |
| 1949122937 | 313824 | 60.54% | ✅ | — |

mask `764121901` 是天空 / 远处 floor_plane，OG 渲染出来 depth 为 inf/0，
`og_utils.pixel_to_3d_points` 算出全 NaN points。它占比 29.58% 没被
`max_mask_ratio` 跳过，传进 `kmeans_pytorch`：

* `kmeans_pytorch.kmeans` 是 `while True: ... if center_shift**2 < tol: break`，
  没有 `iter_limit` 参数。
* NaN 进入 centroid → `center_shift = NaN` → `NaN < tol` 永远 False → 无限循环。

附带还有两个潜在 NaN 源：
1. PCA 后某维 `min == max`（小 mask / 纯色）→ `(x-min)/(max-min) = 0/0`。
2. 点云某轴 `min == max`（mask 内点共面）→ 同样 `0/0`。

## 改动

仅 `keypoint_proposal.py`，新增两个安全网，不动其他模块：

1. **NaN 像素过滤**：`_cluster_features` 入口先算 `valid_point_mask =
   np.isfinite(points).all(axis=-1)`，每个 binary_mask 与之 AND，把渲染失败
   的像素从 mask 里剔掉。
2. **像素数门槛**：`mask_pixel_count < num_candidates_per_mask` 时整个 mask
   跳过，避免 kmeans 在样本数少于 cluster 数时崩或退化。
3. **`_safe_minmax_normalize`**：min-max 归一化，`denom = max - min` 为 0 时
   替换为 1（常量列 collapse 到 0 而不是 NaN/Inf）。features_pca 与
   feature_points 都改用它。
4. **最终守卫**：`torch.isfinite(X).all()` 不通过则 continue。
5. **空 cluster 防御**：cluster 内 member_idx.sum()==0 时跳过该 cluster
   （否则 `member_points[closest_idx]` 会越界）。

## 验证

### 1. KeypointProposer 单测（`/tmp/rekep_kp_check.py`）

```
[diag] env reset ok
[diag] got cam obs; rgb=(720, 720, 3) points_finite=0.7042
[diag] get_keypoints returned in 3.41s; n_keypoints=12 projected_shape=(720, 720, 3)
```

3 次 kmeans 全部在 8–12 iter 内 tol 1e-4 收敛，无死循环。

### 2. live VLM 端到端

```
OPENAI_API_KEY=<key> OMNIGIBSON_HEADLESS=1 \
  /home/badger/anaconda3/envs/omnigibson/bin/python main.py --task trash
```

* keypoint 提取：`Got 12 proposed keypoints`
* gpt-4o 流式返回完整 3-stage 程序，写入
  `vlm_query/2026-04-26_13-40-10_use_the_fixed_so-101_robot_..._trash_bin./`
  （`metadata.json` `num_stages=3 grasp_keypoints=[9,-1,-1] release_keypoints=[-1,-1,9]`）
* SubgoalSolver / PathSolver 正常迭代，stage 1→2 转换通过。
* 进程退出码 0，`videos/2026-04-26-13-46-28.mp4` 生成（1001 帧 / 33.4 s / 640x640 / 2.85 MB）。
* `[423.117s] Simulation App Shutting Down` 干净退出。

## 已知遗留

* SO-101 工作空间小，OSC waypoint 经常 `pos_error≈0.05~0.20` / `rot_error>30°` 触发
  `OSC pose not reached`；不影响最终是否拾起垃圾，但路径轨迹不平滑。后续可调
  `path_solver` 的 IK pos_error 权重或工作空间上下界。
* live demo 视频未做人工目视复核，建议接手者播放 `videos/2026-04-26-13-46-28.mp4`
  确认 grasp / drop 视觉正确。
* gpt-4o 选了 keypoint 9 为 trash，使用 keypoints 0/1/2/3/4 平均作为 bin opening
  中心；若关键点编号在不同 reset 中变化，结果会随之变化（这是 ReKep 设计本身）。

## 文件

* 修改：`keypoint_proposal.py`（+31/-8 行）
* 新增：`tasks/T26_keypoint_kmeans_nan_fix.md`（本文件）
* 新增（运行产物）：`vlm_query/2026-04-26_13-40-10_use_the_fixed_so-101_robot.../`
* 新增（运行产物）：`videos/2026-04-26-13-46-28.mp4`
