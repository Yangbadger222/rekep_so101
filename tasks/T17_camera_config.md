# T17 - 相机配置（外部俯视相机）

- **阶段**: 3 - ReKep 接口适配
- **里程碑**: M6
- **依赖**: T05
- **状态**: [x] 已完成（config static check）

## 目标

为 SO-101 桌面场景配置外部 RGB-D 相机，替换 Fetch 的头部/腕部相机。

## 背景

SO-101 没有内置相机。ReKep 的 `keypoint_proposer` 和 `vlm_camera` 需要外部固定 RGB-D 相机提供观测。

## 详细步骤

### 17.1 相机需求分析

需要两个相机：

| ID | 用途 | 位置 | 朝向 |
|----|------|------|------|
| cam 0 | VLM 观测（keypoint 检测） | 桌面俯视 | 朝下 60-90° |
| cam 1 | 录制视频（save_video） | 侧面 | 观察全场景 |

### 17.2 VLM 相机配置（cam 0）

```yaml
camera_0:
  name: "vlm_camera"
  position: [0.0, 0.0, 1.4]   # 桌面正上方
  orientation: [...]            # 朝下 ~60-90°
  resolution: [720, 720]       # 提高分辨率，帮助 DINOv2 检测小垃圾
  fov: 60
```

必须能看到：
- [ ] SO-101 base 和全部 arm（T24 实图确认）
- [ ] 整个桌面（T24 实图确认）
- [ ] 所有垃圾物体（T24 实图确认）
- [ ] 垃圾桶（T24 实图确认）

### 17.3 录制相机配置（cam 1）

```yaml
camera_1:
  name: "record_camera"
  position: [0.5, -0.5, 1.2]  # 侧面偏上
  orientation: [...]            # 朝向桌面中心
  resolution: [640, 480]
  fov: 60
```

### 17.4 修改 config.yaml 相机块

**行号**: 第 57-70 行

```yaml
# 现状（Fetch 头部相机）
camera:
  - position: [...]  # Fetch 头部位置
    orientation: [...]
    ...

# 改为（外部固定相机）
camera:
  - position: [0.0, 0.0, 1.4]
    orientation: [...计算朝下四元数...]
    resolution: [720, 720]
    name: "vlm_camera"
  - position: [0.5, -0.5, 1.2]
    orientation: [...计算侧视四元数...]
    resolution: [640, 480]
    name: "record_camera"
```

### 17.5 修改 main.py 相机引用

确认 `main.config['vlm_camera']` 指向 cam 0 (id=0)。

### 17.6 修改 _step 中的相机引用

`environment.py` 中 `cam_obs[1]['rgb']` 取的是 id=1 录制相机。确认 id 编号与配置一致。

### 17.7 相机视角调试

1. 加载场景后截图
2. 确认截图中所有任务相关物体可见
3. 确认垃圾物体在图像中占足够像素（DINOv2 patch=14x14 px，垃圾至少要占 20x20 px）
4. 如果垃圾太小，提高分辨率或 zoom-in

## 验收标准

- [ ] VLM 相机能看到桌面全景（T24 实图确认）
- [ ] 垃圾物体在 VLM 相机图像中清晰可见（T24 实图确认）
- [ ] 录制相机能看到 SO-101 操作全过程（T24 实图确认）
- [x] `vlm_camera` id 配置正确
- [ ] 两个相机的 RGB/depth/segmentation 输出正常

## 涉及文件

```
configs/config.yaml
environment.py（相机 id 引用）
main.py（vlm_camera 配置）
og_utils.py（相机初始化）
```

## 备注

- 相机四元数计算：朝下 90° 对应旋转 X 轴 -90°
- 如果 DINOv2 检测不到小垃圾，优先提高分辨率到 720
- 垃圾使用高对比度颜色有助于检测

## 完成记录

- `configs/config.yaml` 相机块改为外部固定相机：
  - cam 0 / `vlm_camera`：`position=[0.0, -0.05, 1.42]`，
    `orientation=[0.0, 0.0, 0.0, 1.0]`，`resolution=720`。
    该相机为桌面上方俯视视角，保持 `main.vlm_camera=0`。
  - cam 1 / `record_camera`：`position=[0.55, -0.65, 1.15]`，
    `orientation=[0.520686, 0.202538, 0.300668, 0.772957]`，
    `resolution=640`。该相机为侧面偏上视角，用于 `save_video()`。
- `environment.py` 保持 `_step()` 使用 `cam_obs[1]["rgb"]` 作为录制帧，
  与 cam 1 配置一致。
- 验证命令：
  - `/home/badger/anaconda3/envs/omnigibson/bin/python - <<'PY' ... get_config('./configs/config.yaml') ... PY`
  - `rg "cam_0|cam_1|Fetch|head|wrist" configs/config.yaml environment.py main.py -n`
- 结果：
  - YAML 加载通过。
  - `vlm_camera=0`。
  - cam 0 quaternion norm = `1.000000`。
  - cam 1 quaternion norm = `1.000000`。
  - 未发现旧 `cam_0` / `cam_1` / Fetch head camera 配置。
- 尚未做 RGB/depth/segmentation 实图检查；需要 T21 创建 trash scene 后在 T24 smoke 中验证。
