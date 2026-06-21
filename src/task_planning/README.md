# task_planning

任务规划节点，位于 object_detection 与 motion_control 之间，实现带抓取重试的状态机。接收 3D 检测结果，按策略逐个规划目标，发布 GraspCommand，监控执行反馈，完成后发布 PhaseResult。

## 功能

- **状态机**：IDLE → PLANNING → WAITING → FIN，由 robot_core 的 Info 消息触发进入 PLANNING 状态
- **目标选择**：按 planning_strategy 从检测队列中选取抓取目标（默认 nearest_first）
- **重试机制**：抓取失败后自动重试，重试耗尽（max_retry_count）后标记该 track_id 为失败并跳过
- **阶段报告**：每轮结束后发布 PhaseResult，列出成功/失败的指令编号

## 状态机

```
IDLE ──(TASK_SCHEDULE 触发)──> PLANNING
PLANNING ──(无物体 / 全部失败)──> FIN
PLANNING ──(发布 GraspCommand)──> WAITING
WAITING ──(成功)──> PLANNING（选择下一目标）
WAITING ──(失败, retry < max)──> WAITING（重发同一目标）
WAITING ──(失败, retry >= max)──> PLANNING（标记失败，跳过此 track_id）
FIN ──(发布 PhaseResult)──> IDLE（resetState）
```

## 话题

### 订阅

| 话题 | 类型 | 说明 |
|---|---|---|
| `~info_topic` | robot_core/Info | 机器人模式触发，默认 `/ctrl/info` |
| `~detection_topic` | object_detection/DetectionObjects | 3D 检测结果，默认 `/object_detection/detected_objects` |
| `~grasp_result_topic` | task_planning/GraspResult | 抓取执行反馈，默认 `/motion_control/grasp_result` |

### 发布

| 话题 | 类型 | 说明 |
|---|---|---|
| `~grasp_command_topic` | task_planning/GraspCommand | 抓取指令，默认 `/task_planning/grasp_command` |
| `~phase_result_topic` | task_planning/PhaseResult | 阶段完成报告，默认 `/task_planning/phase_result` |

## 消息

**GraspCommand**

| 字段 | 类型 | 说明 |
|---|---|---|
| target_class_id | int32 | 目标类别 ID |
| target_track_id | int32 | 目标跟踪 ID |
| grasp_command_index | int32 | 指令编号（本轮递增，重试不变） |

**GraspResult**

| 字段 | 类型 | 说明 |
|---|---|---|
| result | uint8 | 0=成功, 1=失败 |
| grasp_command_index | int32 | 对应的指令编号 |

**PhaseResult**

| 字段 | 类型 | 说明 |
|---|---|---|
| phase_type | uint8 | 阶段类型（0=抓取, 1=放置） |
| round_index | int32 | 轮次编号 |
| indices_completed | int32[] | 本轮成功的指令编号列表 |
| indices_failed | int32[] | 本轮失败的指令编号列表 |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| planning_strategy | string | nearest_first | 目标选择策略 |
| loop_rate | float | 10.0 | 主循环频率（Hz） |
| max_retry_count | int | 2 | 单个目标最大重试次数 |
| info_topic | string | /ctrl/info | 订阅模式触发话题 |
| detection_topic | string | /object_detection/detected_objects | 订阅检测结果话题 |
| grasp_result_topic | string | /motion_control/grasp_result | 订阅抓取反馈话题 |
| grasp_command_topic | string | /task_planning/grasp_command | 发布抓取指令话题 |
| phase_result_topic | string | /task_planning/phase_result | 发布阶段报告话题 |

## 依赖

- ROS Noetic
- robot_core（消息包）
- object_detection（消息包）
- motion_control（执行包）

## 启动

```bash
roslaunch task_planning task_planning.launch
```
