# task_planning 模块设计文档

## 1. 概述

task_planning 位于 object_detection 与 motion_control 之间，作为高层决策层。负责接收检测结果，逐个规划任务并发布给 motion_control 执行，支持数量限制和任务回执驱动。

## 2. 消息定义

### 2.1 TaskCommand.msg（发布）

```
int32 target_class_id             # 目标物体类别ID
geometry_msgs/Pose target_pose    # 目标位姿
int32 task_index                  # 当前任务序号（从0开始）
int32 total_count                 # 总任务数
```

## 3. 订阅与发布话题

### 发布

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/task_planning/task_command` | `task_planning/TaskCommand` | 下发任务指令 |

### 订阅

| 话题 | 消息类型 | 来源 | 说明 |
|---|---|---|---|
| `/object_detection/detected_objects` | `object_detection/DetectionObjects` | object_detection | 检测到的物体列表 |
| `/motion_control/task_result` | `motion_control/TaskResult` | motion_control | 任务执行结果回执 |
| `/robot_core/msg/info` | `robot_core/Info` | robot_core | 机器人状态信息（可选） |

## 4. 配置参数 (config/task_planning_config.yaml)

```yaml
# 任务列表数量上限
max_task_count: 3
# 策略
planning_strategy: "nearest_first"  # 可选：nearest_first（最近优先）, custom_logic（自定义逻辑）
# 循环频率（Hz）控制规划节奏，过快可能导致过度规划，过慢可能响应迟钝
loop_rate: 10

# 订阅话题
detection_topic: /object_detection/detected_objects
task_result_topic: /motion_control/task_result
robot_info_topic: /robot_core/msg/info

# 发布话题
command_topic: /task_planning/task_command
```

## 5. 核心逻辑

### 5.1 两阶段设计

#### 阶段一：规划执行（planning & execution）

通过robot_navigation的状态变化触发规划流程：

1. 接收到触发信号后，订阅一次 `DetectionObjects`
2. 将场景物体按策略排序，如果没有物体则进入 IDLE 状态等待下一次触发
3. 规划并执行
4. 每完成一个任务，等待 `TaskResult` 回执
    - 成功：重复步骤 1，继续规划下一个任务，如果已达 `max_task_count` 则停止
    - 失败：记录日志，跳过当前任务继续下一个

#### 阶段二：完成

### 5.3 状态转移图

```
[IDLE] --(触发)--> [PLANNING] --(发布 TaskCommand)--> [WAITING_RESULT] --(TaskResult: completed)--> [PLANNING]
[WAITING_RESULT] --(TaskResult: error)--> [PLANNING] (跳过当前任务)

[PLANNING] --(到达最大任务数/无物体)--> [RUNNING] (等待总任务完成)--> [IDLE]

```

### 5.2 消息流时序

```
task_planning                          motion_control
    │                                       │
    ├─ TaskCommand(task_idx=0) ──────→      │
    │←────── TaskResult(completed) ─────┤   │
    │                                       │
    ├─ TaskCommand(task_idx=1) ──────→      │
    │←────── TaskResult(completed) ─────┤   │
    │                                       │
    ├─ TaskCommand(task_idx=2) ──────→      │
    │←────── TaskResult(completed) ─────┤   │
    │                                       │
    ├─ TaskCommand(task_idx=3) ──────→      │  (跳过出错，继续下一个)
    │←────── TaskResult(completed) ─────┤   │
    │                                       │
    
    │←────── TaskResult(completed) ─────┤   │
```

## 6. 模块目录结构

```
task_planning/
├── CMakeLists.txt
├── package.xml
├── README.md
├── msg/
│  └── TaskCommand.msg
│   
├── config/
│   └── task_planning_config.yaml
├── launch/
│   └── task_planning.launch
└── src/
    └── task_planning_node.cpp
```
