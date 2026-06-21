# 边界框坐标（像素）
int64 x_min
int64 y_min
int64 x_max
int64 y_max

# 物体类别 ID
int32 class_id

# 跨帧跟踪 ID（ByteTrack 分配，-1 表示未跟踪）
int32 track_id

# 检测置信度
float32 confidence
