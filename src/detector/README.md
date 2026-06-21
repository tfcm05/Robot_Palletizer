# detector

基于 YOLO + ByteTrack 的 2D 目标检测与跟踪节点。订阅 RGB 图像话题，运行模型推理，输出经时序聚合的稳定 BoundingBoxes。

## 功能

- **YOLO 推理**：调用 ultralytics 加载模型（默认 yolo11n.pt），支持 CUDA 加速
- **跨帧跟踪**：可选择 ByteTrack 或 BotSort 跟踪器，为同一物体分配一致的 track_id
- **时序聚合**：对单帧检测结果做 EMA 平滑、漏检剔除与近邻匹配，消除逐帧抖动，按固定频率发布稳定快照

## 话题

### 订阅

| 话题 | 类型 | 说明 |
|---|---|---|
| `~image_topic` | sensor_msgs/Image | RGB 输入图像，默认 `/kinect2/qhd/image_color_rect` |

### 发布

| 话题 | 类型 | 说明 |
|---|---|---|
| `~bounding_box_topic` | detector/BoundingBoxes | 聚合后的 2D 检测框列表 |

## 消息

**BoundingBox**

| 字段 | 类型 | 说明 |
|---|---|---|
| x_min, y_min, x_max, y_max | int64 | 边界框像素坐标 |
| class_id | int32 | 类别 ID（0=green_large, 1=green_small, 2=red_large, 3=red_small） |
| track_id | int32 | 跟踪 ID，-1 表示未跟踪 |
| confidence | float32 | 检测置信度 |

**BoundingBoxes**

| 字段 | 类型 | 说明 |
|---|---|---|
| header | std_msgs/Header | 时间戳与坐标系 |
| bounding_boxes | BoundingBox[] | 当前存活的 2D 检测框列表 |

## 参数

所有参数从节点私有命名空间 `~` 加载，由 yaml 配置文件提供。

### 模型参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| model | string | yolo11n.pt | 模型文件路径 |
| confidence_threshold | float | 0.25 | 置信度阈值 |
| iou_threshold | float | 0.45 | NMS IoU 阈值 |
| device | string | cpu | 推理设备 |
| classes | list | [0,1,2,3] | 检测类别 |

### 跟踪参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| use_track | bool | false | 是否启用跨帧跟踪 |
| tracker_type | string | bytetrack | 跟踪器类型 |
| track_persist | bool | true | 帧间持久化跟踪状态 |

### 话题参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| image_topic | string | /kinect2/qhd/image_color_rect | 订阅图像话题 |
| bounding_box_topic | string | /detector/bounding_boxes | 发布 bbox 话题 |

### 聚合参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| bbox_alpha | float | 0.4 | EMA 平滑系数 |
| max_missed_frames | int | 5 | 连续漏检帧数阈值 |
| match_distance | float | 50.0 | 近邻匹配像素距离 |
| publish_rate | float | 10.0 | 发布频率（Hz） |

### 可视化

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| show_window | bool | true | 是否显示 OpenCV 窗口 |

## 启动

```bash
roslaunch detector detector.launch
```
