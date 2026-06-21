# object_detection

3D 物体位姿估计节点。同步订阅 detector 的 BoundingBoxes、深度图与相机内参，生成目标点云，通过 PCA 分析与质心校正计算每个物体的三维位姿，并发布 DetectionObjects。

## 功能

- **时间同步**：使用 ApproximateTime 策略同步 bbox、depth、camera_info 三路消息
- **点云生成**：在 bbox 区域内用深度图生成 XYZ 点云，经范围过滤和深度一致性掩码剔除背景
- **PCA 分析**：对点云做主成分分析，提取物体三个主轴方向
- **class_id 修正**：置信度低于阈值时从点云 extent 推断真实尺寸，修正检测器的尺寸分类错误（颜色一般不易错）
- **质心校正**：按 PCA 主方向投影，若某方向可见完整尺寸则取中点作为中心，若只看到单面则沿已知边长偏移 L/2 估计几何中心
- **TF 变换**：将相机系下位姿变换到目标坐标系（默认 base_link）

## 话题

### 订阅

| 话题 | 类型 | 说明 |
|---|---|---|
| `~bbox_topic` | detector/BoundingBoxes | 2D 检测框，默认 `/detector/bounding_boxes` |
| `~depth_topic` | sensor_msgs/Image | 深度图（32FC1），默认 `/kinect2/qhd/image_depth_rect` |
| `~camera_info_topic` | sensor_msgs/CameraInfo | 相机内参，默认 `/kinect2/qhd/camera_info` |

### 发布

| 话题 | 类型 | 说明 |
|---|---|---|
| `~object_detection_topic` | object_detection/DetectionObjects | 3D 检测结果 |

## 消息

**DetectionObject**

| 字段 | 类型 | 说明 |
|---|---|---|
| class_id | int32 | 透传自 detector，低置信度时可能被 correctClassId 修正 |
| track_id | int32 | 透传自 detector 的跟踪 ID |
| pose | geometry_msgs/PoseStamped | 物体三维位姿（frame_id 取决于 TF） |
| cloud | sensor_msgs/PointCloud2 | 物体点云（可选，默认关闭） |

**DetectionObjects**

| 字段 | 类型 | 说明 |
|---|---|---|
| header | std_msgs/Header | 时间戳（来自 camera_info）与坐标系 |
| objects | DetectionObject[] | 当前帧所有 3D 物体 |

## 参数

### 话题参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| bbox_topic | string | /detector/bounding_boxes | 订阅的 bbox 话题 |
| depth_topic | string | /kinect2/qhd/image_depth_rect | 订阅的深度图话题 |
| camera_info_topic | string | /kinect2/qhd/camera_info | 订阅的相机内参话题 |
| object_detection_topic | string | /object_detection/detected_objects | 发布的检测结果话题 |

### 同步参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| sync_queue_size | int | 10 | 同步队列长度 |
| sync_slop | float | 0.08 | 同步容忍时间（秒） |

### TF 参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| tf_cache_time | float | 10.0 | TF 监听器缓存时间（秒） |

### 深度处理参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| min_depth | float | 0.1 | 最小有效深度（米） |
| max_depth | float | 5.0 | 最大有效深度（米） |
| depth_tolerance | float | 0.08 | 前景深度一致性容差（米） |
| center_sample_ratio | float | 0.2 | 中心采样窗口占 ROI 比例 |
| size_confidence_threshold | float | 0.6 | 尺寸推断置信度阈值（低于此值用点云 extent 修正 class_id） |

### 输出参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| pose_frame | string | base_link | 输出位姿的目标坐标系 |
| publish_point_cloud | bool | false | 是否在检测结果中附带点云 |

## 算法说明

### correctClassId

当检测置信度低于阈值时，从点云在两个较大 PCA 主方向上的 extent 推断真实尺寸，修正 class_id（颜色一般不易错，尺寸分类更容易错）。

### computeObjectCentroid

1. 由 class_id 确定边长 L
2. 将点云投影到三个 PCA 主方向，得到每个方向的可见范围 [min_proj, max_proj]
3. 若 extent 接近 L（大于 0.7×L），说明看到完整尺寸，用中点作为中心
4. 若只看单面，从可见表面向远离相机方向偏移 L/2，估计几何中心

### sampleDepth

在 bbox 中心区域采样深度值，取有限有效值的中位数作为前景基准深度，用于构建深度一致性掩码过滤背景点。

## 启动

```bash
roslaunch object_detection object_detection.launch
```
