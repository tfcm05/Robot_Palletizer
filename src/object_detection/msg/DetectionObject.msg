# 物体类别 ID（透传自 detector）
int32 class_id

# 跨帧跟踪 ID（透传自 detector）
int32 track_id

# 物体 3D 位姿（取决于 TF）
geometry_msgs/PoseStamped pose

# 物体点云（可选）
sensor_msgs/PointCloud2 cloud
