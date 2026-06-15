# task_planning

task_planning 作为高层决策层，负责接收 `object_detection` 发布的检测结果，按策略逐个规划子任务，并将子任务指令下发给 `motion_control` 执行。

---

## 1. 概述

`task_planning_node` 维护一个四状态状态机：

- **IDLE**：等待任务触发信号。
- **PLANNING**：获取一次检测结果，选择目标物体并发布 `SubTask`。
- **WAITING_RESULT**：等待 `motion_control` 返回的 `SubTaskResult`。
- **FIN**：本轮任务结束，发布汇总后的 `Task` 消息。

当前默认规划策略为 `nearest_first`：选择距离原点最近的物体作为下一个执行目标。

---

## 2. 消息定义

### 2.1 Task.msg（发布）

```
# 子任务数量
int32 sub_task_count

# 子任务列表
int32[] sub_task_indices_completed
int32[] sub_task_indices_failed

# 任务编号
int32 task_index

# 是否终止
bool is_finish
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `sub_task_count` | `int32` | 本轮已执行子任务总数（成功 + 失败） |
| `sub_task_indices_completed` | `int32[]` | 成功完成的子任务编号列表 |
| `sub_task_indices_failed` | `int32[]` | 执行失败的子任务编号列表 |
| `task_index` | `int32` | 当前任务编号 |
| `is_finish` | `bool` | 是否全部完成，`true` 表示场景中无剩余物体 |

### 2.2 SubTask.msg（发布）

```
# 目标物体类别
int32 target_class_id

# 目标物体位姿
geometry_msgs/PoseStamped target_pose

# 子任务编号
int32 sub_task_index
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `target_class_id` | `int32` | 目标物体类别 ID |
| `target_pose` | `geometry_msgs/PoseStamped` | 目标物体三维位姿（含 Header，坐标系与 `DetectionObject.pose` 一致） |
| `sub_task_index` | `int32` | 子任务编号 |

> `SubTask.target_pose` 和 `DetectionObject.pose` 均为 `geometry_msgs/PoseStamped` 类型，下游节点访问位置时请使用 `target_pose.pose.position.x/y/z`。

---

## 3. 订阅与发布话题

### 3.1 发布

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/task_planning/task` | `task_planning/Task` | 下发任务指令（每轮任务结束时发布） |
| `/task_planning/sub_task` | `task_planning/SubTask` | 下发子任务指令 |

### 3.2 订阅

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/object_detection/detected_objects` | `object_detection/DetectionObjects` | 检测到的物体列表 |
| `/motion_control/sub_task_result` | `motion_control/SubTaskResult` | 子任务执行结果回执 |
| `/ctrl/info` | `robot_core/Info` | 机器人状态信息，用于触发任务调度 |

---

## 4. 配置参数

参数文件位于 `config/task_planning_config.yaml`：

```yaml
# 子任务列表数量上限
max_sub_task_count: 3

# 规划策略
planning_strategy: "nearest_first"  # 可选：nearest_first, custom_logic

# 循环频率（Hz）
loop_rate: 10

# 订阅话题
detection_topic: /object_detection/detected_objects
sub_task_result_topic: /motion_control/sub_task_result
info_topic: /ctrl/info

# 发布话题
task_topic: /task_planning/task
sub_task_topic: /task_planning/sub_task
```

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_sub_task_count` | `int` | `3` | 单轮任务中最多成功执行多少个子任务，达到上限后进入 FIN 状态 |
| `planning_strategy` | `string` | `"nearest_first"` | 目标选择策略，目前仅实现 `nearest_first` |
| `loop_rate` | `double` | `10.0` | 主循环频率（Hz） |
| `detection_topic` | `string` | `/object_detection/detected_objects` | 检测结果话题 |
| `sub_task_result_topic` | `string` | `/motion_control/sub_task_result` | 子任务执行结果话题 |
| `info_topic` | `string` | `/ctrl/info` | 机器人状态信息话题 |
| `task_topic` | `string` | `/task_planning/task` | 任务指令发布话题 |
| `sub_task_topic` | `string` | `/task_planning/sub_task` | 子任务指令发布话题 |

---

## 5. 核心逻辑

### 5.1 状态机说明

#### IDLE（准备阶段）

等待 `/ctrl/info` 中的 `TASK_SCHEDULE` 模式触发信号。收到触发后，状态切换为 PLANNING。

#### PLANNING（规划阶段）

1. 等待并获取一次 `DetectionObjects` 检测结果。
2. 若场景中无物体，直接进入 FIN 状态。
3. 否则按 `planning_strategy` 选择目标物体，构造并发布 `SubTask`，状态切换为 WAITING_RESULT。

#### WAITING_RESULT（等待结果阶段）

等待 `motion_control` 返回 `SubTaskResult`：

- **成功**：将子任务编号加入完成列表。
  - 若完成数量达到 `max_sub_task_count`，进入 FIN 状态。
  - 否则回到 PLANNING，继续规划下一个子任务。
- **失败**：将子任务编号加入失败列表，跳过当前任务，回到 PLANNING。

#### FIN（结束阶段）

再次获取一次检测结果，判断本轮任务是否真正结束：

- 若场景中无剩余物体，发布 `is_finish=true` 的 `Task` 消息。
- 若场景中仍有物体，发布 `is_finish=false` 的 `Task` 消息，等待下一轮触发。

随后重置状态并回到 IDLE。

### 5.2 状态转移图

```
[IDLE] --(TASK_SCHEDULE 触发)--> [PLANNING]

[PLANNING] --(无物体)--> [FIN]
[PLANNING] --(发布 SubTask)--> [WAITING_RESULT]

[WAITING_RESULT] --(成功，未达上限)--> [PLANNING]
[WAITING_RESULT] --(成功，达到上限)--> [FIN]
[WAITING_RESULT] --(失败)--> [PLANNING]

[FIN] --(发布 Task 后)--> [IDLE]
```

---

## 6. 编译

在 catkin 工作空间根目录执行：

```bash
catkin_make
source devel/setup.bash
```

---

## 7. 使用示例

```bash
# 1. 启动 object_detection 节点
roslaunch object_detection object_detection.launch

# 2. 启动 task_planning 节点
roslaunch task_planning task_planning.launch

# 3. 查看下发的子任务
rostopic echo /task_planning/sub_task

# 4. 查看任务汇总
rostopic echo /task_planning/task
```

---

## 8. 注意事项

- `SubTask.target_pose` 为 `geometry_msgs/PoseStamped` 类型，`motion_control` 等下游节点应通过 `sub_task.target_pose.pose` 获取位姿，通过 `sub_task.target_pose.header.frame_id` 获取坐标系。
- 使用 `rostopic echo` 查看 `Task` 消息时，可通过 `sub_task_indices_completed` 字段查看成功完成的子任务编号列表。
- 当前 `nearest_first` 策略计算的是 `DetectionObject.pose.pose.position` 到原点的欧氏距离。

---

## 9. 模块目录结构

```
task_planning/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── task_planning_config.yaml
├── launch/
│   └── task_planning.launch
├── msg/
│   ├── SubTask.msg
│   └── Task.msg
└── src/
    └── task_planning_node.cpp
```
