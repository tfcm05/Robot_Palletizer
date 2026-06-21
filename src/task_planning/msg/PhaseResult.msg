# 阶段类型常量
uint8 GRASP = 0
uint8 PLACE = 1

# 当前完成的阶段类型
uint8 phase_type

# 轮次编号（每次 resetState 后递增）
int32 round_index

# 本轮内成功的 grasp_command_index 列表
int32[] indices_completed

# 本轮内失败的 grasp_command_index 列表
int32[] indices_failed
