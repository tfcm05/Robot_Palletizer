# object_detection

ROS 物体检测后端节点包。接收 2D 边界框、深度图和相机内参，通过深度图生成前景点云，计算每个检测物体的三维位姿，并发布带位姿的检测结果。

---

## 1. 功能概述

`object_detection_node` 作为检测后端，主要完成以下工作：

1. 同步订阅 `detector/BoundingBoxes`、`sensor_msgs/Image`（深度图）和 `sensor_msgs/CameraInfo`。
2. 对每个边界框对应的 ROI 区域，基于深度图生成相机坐标系下的点云。
3. 计算点云质心，得到物体在相机坐标系下的位姿。
4. 通过 TF 将位姿转换到目标坐标系（默认 `base_link`）。
5. 可选：对低置信度检测框，使用点云包围盒对角线长度进行类别修正。
6. 发布自定义消息 `DetectionObjects`，其中每个物体包含类别 ID、`geometry_msgs/PoseStamped` 位姿和可选点云。

---

## 2. 依赖

- ROS（roscpp、rospy）
- cv_bridge
- tf
- message_filters
- pcl_ros / pcl_conversions
- OpenCV
- PCL
- detector（提供 `detector/BoundingBoxes` 消息）

---

## 3. 自定义消息

### 3.1 DetectionObject.msg

```
int32 class_id
geometry_msgs/PoseStamped pose
sensor_msgs/PointCloud2 cloud
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `class_id` | `int32` | 物体类别 ID |
| `pose` | `geometry_msgs/PoseStamped` | 物体三维位姿（含 Header，坐标系为目标坐标系 `pose_frame`） |
| `cloud` | `sensor_msgs/PointCloud2` | 物体点云（仅在 `publish_point_cloud=true` 时填充） |

### 3.2 DetectionObjects.msg

```
Header header
DetectionObject[] objects
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `Header` | 消息头，时间戳与相机内参一致，frame_id 为相机坐标系 |
| `objects` | `DetectionObject[]` | 检测到的物体列表 |

---

## 4. 节点：object_detection_node

### 4.1 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/detector/bounding_boxes` | `detector/BoundingBoxes` | 检测器输出的 2D 边界框 |
| `/kinect2/qhd/image_depth_rect` | `sensor_msgs/Image` | 对齐后的深度图 |
| `/kinect2/qhd/camera_info` | `sensor_msgs/CameraInfo` | 深度相机内参 |

> 话题名可通过配置文件修改，详见第 5 节。

### 4.2 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/object_detection/detected_objects` | `object_detection/DetectionObjects` | 检测到的物体列表（带位姿） |

### 4.3 坐标变换

- 输入深度图和边界框位于相机坐标系。
- 节点内部通过 `tf::TransformListener::transformPose()` 将位姿转换到 `pose_frame` 参数指定的坐标系。
- 若 TF 转换失败，则退而使用相机坐标系下的位姿，并输出警告。

---

## 5. 参数配置

参数文件位于 `config/object_detection_config.yaml`：

```yaml
# 订阅的话题
bbox_topic: /detector/bounding_boxes
depth_topic: /kinect2/qhd/image_depth_rect
camera_info_topic: /kinect2/qhd/camera_info

# 发布的话题
object_detection_topic: /object_detection/detected_objects

# 同步参数
sync_queue_size: 10      # 消息同步队列大小
tf_cache_time: 10.0      # TF 缓存时间（秒）
sync_slop: 0.08          # 时间同步容忍（秒）

# 深度过滤参数
min_depth: 0.1           # 有效最小深度（米）
max_depth: 6.0           # 有效最大深度（米）
depth_tolerance: 0.10    # 前景过滤深度容差（米）

# 类别修正参数
confidence_threshold: 0.5            # 低于该阈值时启用点云形状修正
large_object_diagonal_threshold: 0.18 # 点云包围盒对角线阈值（米）
min_points_for_shape: 30             # 形状判断最少点数

# 坐标系
pose_frame: "base_link"  # 输出位姿的目标坐标系

# 是否发布点云
publish_point_cloud: false
```

---

## 6. 启动文件

### 6.1 object_detection.launch

单独启动 `object_detection_node` 并加载默认配置：

```bash
roslaunch object_detection object_detection.launch
```

### 6.2 wpb_simple.launch

启动 Gazebo 仿真环境（`wpb_simple.world`）并加载机器人模型，通常用于配合检测算法进行仿真测试：

```bash
roslaunch object_detection wpb_simple.launch
```

---

## 7. 编译

在 catkin 工作空间根目录执行：

```bash
catkin_make
source devel/setup.bash
```

---

## 8. 使用示例

```bash
# 1. 启动仿真环境（可选）
roslaunch object_detection wpb_simple.launch

# 2. 启动检测器（例如 detector 包提供的节点）
# roslaunch detector detector.launch

# 3. 启动 object_detection 节点
roslaunch object_detection object_detection.launch

# 4. 查看检测结果
rostopic echo /object_detection/detected_objects
```

---

## 9. 注意事项

- `DetectionObject.pose` 类型为 `geometry_msgs/PoseStamped`，包含 `header` 和 `pose`，下游节点使用时请注意通过 `pose.pose.position` 访问位置信息。
- 若 `publish_point_cloud` 设为 `true`，每条检测结果都会附带物体点云，消息体积会显著增大，可能影响通信效率。
- 点云质心计算对立方体类物体（class_id 0~3）会沿 z 轴偏移半个点云深度，以估计物体顶部中心位置。

---

## 10. 目录结构

```
object_detection/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── object_detection_config.yaml
├── launch/
│   ├── object_detection.launch
│   └── wpb_simple.launch
├── msg/
│   ├── DetectionObject.msg
│   └── DetectionObjects.msg
└── src/
    └── object_detection_node.cpp
```
