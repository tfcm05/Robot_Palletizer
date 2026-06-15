#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <motion_control/SubTaskResult.h>
#include <object_detection/DetectionObject.h>
#include <object_detection/DetectionObjects.h>
#include <robot_core/Info.h>
#include <task_planning/SubTask.h>
#include <task_planning/Task.h>

/**
 * @class TaskPlanningNode
 * @brief 任务规划节点
 *
 * 作为高层决策层，负责接收检测结果，逐个规划子任务并发布给 motion_control 执行。
 * 采用三阶段状态机：准备（IDLE） -> 规划执行（PLANNING / WAITING_RESULT） -> 结束（FIN）。
 */
class TaskPlanningNode {
public:
    /**
     * @brief 构造函数：加载参数、初始化 ROS 订阅者与发布者
     */
    TaskPlanningNode()
        // ROS 节点句柄
        : private_nh_("~")   ///< 私有节点句柄，用于读取参数
        , public_nh_()       ///< 公共节点句柄，用于订阅和发布话题
        // 配置参数
        , rate_(ros::Rate(private_nh_.param("loop_rate", 10.0)))          ///< 主循环频率
        , max_sub_task_count_(private_nh_.param("max_sub_task_count", 3)) ///< 单轮任务中成功子任务数量上限
        , planning_strategy_(private_nh_.param("planning_strategy", std::string("nearest_first"))) ///< 规划策略
        , loop_rate_(private_nh_.param("loop_rate", 10.0))                ///< 主循环频率（Hz）
        // ROS 订阅者
        , info_sub_(public_nh_.subscribe(
              private_nh_.param("info_topic", std::string("/ctrl/info")), 5,
              &TaskPlanningNode::infoCallback, this))   ///< 机器人状态信息订阅者
        , detection_sub_(public_nh_.subscribe(
              private_nh_.param("detection_topic", std::string("/object_detection/detected_objects")), 5,
              &TaskPlanningNode::detectionCallback, this))   ///< 检测结果订阅者
        , sub_task_result_sub_(public_nh_.subscribe(
              private_nh_.param("sub_task_result_topic", std::string("/motion_control/sub_task_result")), 5,
              &TaskPlanningNode::subTaskResultCallback, this))   ///< 子任务执行结果订阅者
        // ROS 发布者
        , task_pub_(public_nh_.advertise<task_planning::Task>(
              private_nh_.param("task_topic", std::string("/task_planning/task")), 1))   ///< 任务指令发布者
        , sub_task_pub_(public_nh_.advertise<task_planning::SubTask>(
              private_nh_.param("sub_task_topic", std::string("/task_planning/sub_task")), 1)) { ///< 子任务指令发布者
        ROS_INFO("TaskPlanningNode initialized. Strategy: %s, Max sub-tasks: %d, Rate: %.1f Hz",
                 planning_strategy_.c_str(), max_sub_task_count_, loop_rate_);
    }

    /**
     * @brief 主循环：根据当前状态调用对应的状态处理函数
     */
    void run() {
        while (ros::ok()) {
            ros::spinOnce();

            switch (state_) {
                case IDLE:
                    handleIdle();
                    break;
                case PLANNING:
                    handlePlanning();
                    break;
                case WAITING_RESULT:
                    handleWaitingResult();
                    break;
                case FIN:
                    handleFin();
                    break;
                default:
                    ROS_ERROR("Unknown state!");
                    break;
            }

            rate_.sleep();
        }
    }

private:
    /**
     * @brief 状态枚举
     *
     * IDLE：准备阶段，等待触发信号
     * PLANNING：规划阶段，订阅检测结果并发布子任务
     * WAITING_RESULT：等待子任务执行结果
     * FIN：结束阶段，发布任务完成信息
     */
    enum State { IDLE, PLANNING, WAITING_RESULT, FIN };

    // ROS 节点句柄
    ros::NodeHandle private_nh_;
    ros::NodeHandle public_nh_;

    // 循环频率控制器
    ros::Rate rate_;

    // 配置参数
    const int max_sub_task_count_;               ///< 单轮任务中成功子任务数量上限
    const std::string planning_strategy_;        ///< 规划策略
    const double loop_rate_;                     ///< 主循环频率（Hz）

    // ROS 订阅者
    ros::Subscriber info_sub_;            ///< 机器人状态信息订阅者
    ros::Subscriber detection_sub_;       ///< 检测结果订阅者
    ros::Subscriber sub_task_result_sub_; ///< 子任务执行结果订阅者

    // ROS 发布者
    ros::Publisher task_pub_;    ///< 任务指令发布者
    ros::Publisher sub_task_pub_; ///< 子任务指令发布者

    // 当前状态
    State state_ = IDLE;

    // 任务相关索引
    int task_index_ = 0;     ///< 当前任务编号
    int sub_task_index_ = 0; ///< 下一个待发布子任务的编号

    // 子任务执行结果记录
    std::vector<int32_t> sub_task_indices_completed_; ///< 成功完成的子任务编号列表
    std::vector<int32_t> sub_task_indices_failed_;    ///< 执行失败的子任务编号列表

    // 数据接收标志与缓存
    bool got_detection_ = false;
    object_detection::DetectionObjects::ConstPtr latest_detection_; ///< 最新检测结果

    bool got_result_ = false;
    motion_control::SubTaskResult::ConstPtr latest_result_; ///< 最新子任务执行结果

    bool triggered_ = false; ///< 是否收到任务触发信号

    /**
     * @brief 检测结果回调函数
     * @param msg 检测到的物体列表
     */
    void detectionCallback(const object_detection::DetectionObjects::ConstPtr &msg) {
        latest_detection_ = msg;
        got_detection_ = true;
    }

    /**
     * @brief 子任务执行结果回调函数
     * @param msg 子任务执行结果
     */
    void subTaskResultCallback(const motion_control::SubTaskResult::ConstPtr &msg) {
        latest_result_ = msg;
        got_result_ = true;
    }

    /**
     * @brief 机器人状态信息回调函数
     * @param msg 机器人状态信息
     *
     * 当机器人处于 TASK_SCHEDULE 模式时，若当前为 IDLE 状态，则触发任务规划。
     */
    void infoCallback(const robot_core::Info::ConstPtr &msg) {
        if (msg->mode == robot_core::Info::TASK_SCHEDULE) {
            if (state_ == IDLE) {
                triggered_ = true;
                ROS_INFO("TASK_SCHEDULE trigger received, entering PLANNING state");
            }
        }
    }

    /**
     * @brief IDLE 状态处理函数
     *
     * 等待触发信号，收到后进入 PLANNING 状态。
     */
    void handleIdle() {
        if (triggered_) {
            state_ = PLANNING;
            ROS_INFO("State: IDLE -> PLANNING");
        }
    }

    /**
     * @brief PLANNING 状态处理函数
     *
     * 1. 等待一次检测结果；
     * 2. 若场景中无物体，进入 FIN 状态；
     * 3. 否则按策略选择目标物体，发布 SubTask，进入 WAITING_RESULT 状态。
     */
    void handlePlanning() {
        if (!got_detection_ || !latest_detection_) {
            ROS_INFO_THROTTLE(5.0, "PLANNING: Waiting for detection data...");
            return;
        }

        // 场景中无物体，结束本轮任务
        if (latest_detection_->objects.empty()) {
            ROS_WARN("PLANNING: No objects detected. Going to FIN.");
            got_detection_ = false;
            state_ = FIN;
            return;
        }

        // 按规划策略选择目标物体
        const auto &obj = selectTargetObject(latest_detection_);

        // 构造并发布子任务
        task_planning::SubTask sub_task;
        sub_task.target_class_id = obj.class_id;
        sub_task.target_pose = obj.pose;
        sub_task.sub_task_index = sub_task_index_;

        sub_task_pub_.publish(sub_task);
        ROS_INFO("Published SubTask: class_id=%d, index=%d",
                 sub_task.target_class_id, sub_task.sub_task_index);

        sub_task_index_++;
        got_detection_ = false;
        state_ = WAITING_RESULT;
    }

    /**
     * @brief WAITING_RESULT 状态处理函数
     *
     * 等待子任务执行结果回执：
     * - 成功：记录到完成列表；若成功数量达到上限则进入 FIN，否则回到 PLANNING 继续规划下一个子任务。
     * - 失败：记录到失败列表，跳过当前任务，回到 PLANNING 继续规划下一个子任务。
     */
    void handleWaitingResult() {
        if (!got_result_ || !latest_result_) {
            ROS_INFO_THROTTLE(5.0, "WAITING_RESULT: Waiting for sub-task result...");
            return;
        }

        got_result_ = false;

        if (latest_result_->result == motion_control::SubTaskResult::SUCCESS) {
            sub_task_indices_completed_.push_back(latest_result_->task_index);
            ROS_INFO("SubTask %d succeeded", latest_result_->task_index);

            // 成功子任务数量达到上限，结束本轮任务
            if (static_cast<int>(sub_task_indices_completed_.size()) >= max_sub_task_count_) {
                ROS_INFO("Reached max sub-task count (%d). Going to FIN.", max_sub_task_count_);
                got_detection_ = false;
                state_ = FIN;
            } else {
                state_ = PLANNING;
            }
        } else {
            sub_task_indices_failed_.push_back(latest_result_->task_index);
            ROS_ERROR("SubTask %d failed. Skipping to next.", latest_result_->task_index);
            state_ = PLANNING;
        }
    }

    /**
     * @brief FIN 状态处理函数
     *
     * 再次订阅一次检测结果，判断任务是否真正结束：
     * - 若场景中无剩余物体，发布 is_finish=true 的 Task 消息，进入 IDLE。
     * - 若场景中仍有物体，发布 is_finish=false 的 Task 消息，进入 IDLE 等待下一轮触发。
     */
    void handleFin() {
        if (!got_detection_ || !latest_detection_) {
            ROS_INFO_THROTTLE(5.0, "FIN: Waiting for detection data...");
            return;
        }

        // 构造并发布任务指令
        task_planning::Task task;
        task.task_index = task_index_;
        task.sub_task_count = static_cast<int32_t>(sub_task_indices_completed_.size() + sub_task_indices_failed_.size());
        task.sub_task_indices_completed = sub_task_indices_completed_;
        task.sub_task_indices_failed = sub_task_indices_failed_;

        if (latest_detection_->objects.empty()) {
            task.is_finish = true;
            ROS_INFO("No objects remaining. Task finished (is_finish=true).");
        } else {
            task.is_finish = false;
            ROS_INFO("Objects still remaining. Task cycle done but more work to do (is_finish=false).");
        }

        task_pub_.publish(task);
        ROS_INFO("Published Task: task_index=%d, sub_task_count=%d, completed=%d, failed=%d, is_finish=%s",
                 task.task_index, task.sub_task_count,
                 static_cast<int>(sub_task_indices_completed_.size()),
                 static_cast<int>(sub_task_indices_failed_.size()),
                 task.is_finish ? "true" : "false");

        resetState();
    }

    /**
     * @brief 重置任务状态与计数器
     *
     * 回到 IDLE，清空本轮任务数据，任务编号递增，准备下一轮任务规划。
     */
    void resetState() {
        state_ = IDLE;
        triggered_ = false;
        got_detection_ = false;
        got_result_ = false;
        latest_detection_ = nullptr;
        latest_result_ = nullptr;
        sub_task_index_ = 0;
        sub_task_indices_completed_.clear();
        sub_task_indices_failed_.clear();
        task_index_++;
    }

    /**
     * @brief 计算物体位姿到原点的欧氏距离
     * @param pose 物体位姿
     * @return 欧氏距离
     */
    double calculateDistance(const geometry_msgs::PoseStamped &pose) {
        double dx = pose.pose.position.x;
        double dy = pose.pose.position.y;
        double dz = pose.pose.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    /**
     * @brief 根据规划策略从检测结果中选择目标物体
     * @param detection 检测结果
     * @return 被选中的目标物体
     *
     * - nearest_first：选择距离原点最近的物体。
     * - 其他策略：当前默认返回第一个检测到的物体，可后续扩展。
     */
    object_detection::DetectionObject selectTargetObject(const object_detection::DetectionObjects::ConstPtr &detection) {
        if (planning_strategy_ == "nearest_first") {
            double min_distance = 10.0;
            object_detection::DetectionObject nearest_obj;
            for (const auto &obj : detection->objects) {
                double distance = calculateDistance(obj.pose);
                if (distance < min_distance) {
                    min_distance = distance;
                    nearest_obj = obj;
                }
            }
            return nearest_obj;
        } else {
            return detection->objects.front();
        }
    }
};

/**
 * @brief 主函数
 * @param argc 参数个数
 * @param argv 参数列表
 * @return 程序退出码
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "task_planning_node");
    TaskPlanningNode node;
    node.run();
    return 0;
}
