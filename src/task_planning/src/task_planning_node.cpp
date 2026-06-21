#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <object_detection/DetectionObject.h>
#include <object_detection/DetectionObjects.h>
#include <robot_core/Info.h>
#include <task_planning/GraspCommand.h>
#include <task_planning/GraspResult.h>
#include <task_planning/PhaseResult.h>

/**
 * @class TaskPlanningNode
 * @brief 任务规划节点（含抓取重试状态机）
 *
 * 作为高层决策层，接收检测结果，逐个规划抓取目标并发布 GraspCommand 给 motion_control 执行。
 * 抓取失败时自动重试，重试耗尽后标记该 track_id 为失败并跳过。
 *
 * 状态机：
 *   IDLE ──(TASK_SCHEDULE 触发)──> PLANNING
 *   PLANNING ──(无物体 / 全部失败)──> FIN
 *   PLANNING ──(发布 GraspCommand)──> WAITING
 *   WAITING ──(成功)──> PLANNING
 *   WAITING ──(失败, retry < max)──> WAITING（重发同一目标）
 *   WAITING ──(失败, retry >= max)──> PLANNING（标记 track_id 失败）
 *   FIN ──(发布 PhaseResult)──> IDLE（resetState）
 */
class TaskPlanningNode {
public:
    /**
     * @brief 构造函数：加载参数、初始化 ROS 订阅者与发布者
     */
    TaskPlanningNode()
        : private_nh_("~")   ///< 私有节点句柄，用于读取参数
        , public_nh_()       ///< 公共节点句柄，用于订阅和发布话题
        , rate_(ros::Rate(private_nh_.param("loop_rate", 10.0)))          ///< 主循环频率
        , max_retry_count_(private_nh_.param("max_retry_count", 2))       ///< 单个目标最大重试次数
        , planning_strategy_(private_nh_.param("planning_strategy", std::string("nearest_first"))) ///< 目标选择策略
        , loop_rate_(private_nh_.param("loop_rate", 10.0))                ///< 主循环频率（Hz）
        // ROS 订阅者
        , info_sub_(public_nh_.subscribe(
              private_nh_.param("info_topic", std::string("/ctrl/info")), 5,
              &TaskPlanningNode::infoCallback, this))   ///< 机器人模式触发
        , detection_sub_(public_nh_.subscribe(
              private_nh_.param("detection_topic", std::string("/object_detection/detected_objects")), 5,
              &TaskPlanningNode::detectionCallback, this))   ///< 3D 检测结果
        , grasp_result_sub_(public_nh_.subscribe(
              private_nh_.param("grasp_result_topic", std::string("/motion_control/grasp_result")), 5,
              &TaskPlanningNode::graspResultCallback, this))   ///< 抓取执行反馈
        // ROS 发布者
        , phase_result_pub_(public_nh_.advertise<task_planning::PhaseResult>(
              private_nh_.param("phase_result_topic", std::string("/task_planning/phase_result")), 1)) ///< 阶段完成报告
        , grasp_command_pub_(public_nh_.advertise<task_planning::GraspCommand>(
              private_nh_.param("grasp_command_topic", std::string("/task_planning/grasp_command")), 1)) { ///< 抓取指令
        ROS_INFO("TaskPlanningNode initialized. Strategy: %s, Max retries: %d, Rate: %.1f Hz",
                 planning_strategy_.c_str(), max_retry_count_, loop_rate_);
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
                case WAITING:
                    handleWaiting();
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
     * IDLE：     准备阶段，等待 TASK_SCHEDULE 触发信号
     * PLANNING： 规划阶段，选择目标并发布 GraspCommand
     * WAITING：  等待抓取结果，失败时重试
     * FIN：      结束阶段，发布 PhaseResult 并重置
     */
    enum State { IDLE, PLANNING, WAITING, FIN };

    // ROS 节点句柄
    ros::NodeHandle private_nh_;
    ros::NodeHandle public_nh_;

    // 循环频率控制器
    ros::Rate rate_;

    // 配置参数
    const int max_retry_count_;               ///< 单个抓取目标的最大重试次数
    const std::string planning_strategy_;      ///< 目标选择策略
    const double loop_rate_;                   ///< 主循环频率（Hz）

    // ROS 订阅者
    ros::Subscriber info_sub_;            ///< 机器人模式触发
    ros::Subscriber detection_sub_;       ///< 3D 检测结果
    ros::Subscriber grasp_result_sub_;    ///< 抓取执行反馈

    // ROS 发布者
    ros::Publisher phase_result_pub_;     ///< 阶段完成报告
    ros::Publisher grasp_command_pub_;    ///< 抓取指令（发给 motion_control）

    // 当前状态
    State state_ = IDLE;

    // 轮次与指令跟踪
    int round_index_ = 0;          ///< 当前轮次编号（每次 resetState 后递增）
    int grasp_command_index_ = 0;  ///< 本轮内下一个抓取指令编号
    int current_retry_count_ = 0;  ///< 当前目标的重试计数

    // 结果跟踪
    std::vector<int32_t> grasp_indices_completed_; ///< 成功完成的 grasp_command_index 列表
    std::vector<int32_t> grasp_indices_failed_;    ///< 执行失败的 grasp_command_index 列表
    std::set<int32_t> failed_track_ids_;           ///< 重试耗尽的 track_id 集合

    // 当前抓取目标
    object_detection::DetectionObject current_target_; ///< 正在尝试抓取的目标物体
    bool has_current_target_ = false;                  ///< 是否持有当前目标

    // 检测结果缓存
    bool got_detection_ = false;
    object_detection::DetectionObjects::ConstPtr latest_detection_; ///< 最新检测结果快照

    // 抓取结果缓存
    bool got_result_ = false;
    task_planning::GraspResult::ConstPtr latest_result_; ///< 最新抓取结果

    bool triggered_ = false; ///< 是否收到 TASK_SCHEDULE 触发信号

    /**
     * @brief 检测结果回调函数
     * @param msg 检测到的物体列表
     */
    void detectionCallback(const object_detection::DetectionObjects::ConstPtr &msg) {
        latest_detection_ = msg;
        got_detection_ = true;
    }

    /**
     * @brief 抓取结果回调函数
     * @param msg 抓取执行结果
     */
    void graspResultCallback(const task_planning::GraspResult::ConstPtr &msg) {
        latest_result_ = msg;
        got_result_ = true;
    }

    /**
     * @brief 机器人状态信息回调函数
     * @param msg 机器人模式信息
     *
     * 当机器人处于 TASK_SCHEDULE 模式且当前为 IDLE 状态时，
     * 设置 triggered_ 标志以进入 PLANNING 状态。
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
     * 1. 等待检测结果；
     * 2. 若场景中无物体，进入 FIN 状态；
     * 3. 若所有物体均已失败，进入 FIN 状态；
     * 4. 否则按策略选择目标物体，发布 GraspCommand，进入 WAITING 状态。
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

        // 按规划策略选择目标物体（跳过已失败的 track_id）
        const auto *target = selectTargetObject(latest_detection_);
        if (target == nullptr) {
            ROS_WARN("PLANNING: No valid target (all failed). Going to FIN.");
            got_detection_ = false;
            state_ = FIN;
            return;
        }

        current_target_ = *target;
        has_current_target_ = true;
        current_retry_count_ = 0;

        publishGraspCommand();

        got_detection_ = false;
        state_ = WAITING;
        ROS_INFO("State: PLANNING -> WAITING (grasp_command_index=%d)", grasp_command_index_);
    }

    /**
     * @brief WAITING 状态处理函数
     *
     * 等待抓取执行结果：
     * - 成功：记录到完成列表，进入 FIN 状态；
     * - 失败：重试计数递增；
     *   - 若 retry < max：重发同一 GraspCommand，保持在 WAITING。
     *   - 若 retry >= max：记录到失败列表，标记 track_id，回到 PLANNING。
     */
    void handleWaiting() {
        if (!got_result_ || !latest_result_) {
            ROS_INFO_THROTTLE(5.0, "WAITING: Waiting for grasp result...");
            return;
        }

        got_result_ = false;

        if (latest_result_->result == task_planning::GraspResult::SUCCESS) {
            grasp_indices_completed_.push_back(latest_result_->grasp_command_index);
            ROS_INFO("GraspCommand %d succeeded", latest_result_->grasp_command_index);

            grasp_command_index_++;
            has_current_target_ = false;
            state_ = FIN;
            ROS_INFO("State: WAITING -> FIN");
        } else {
            current_retry_count_++;
            ROS_ERROR("GraspCommand %d failed (retry %d/%d)",
                      latest_result_->grasp_command_index, current_retry_count_, max_retry_count_);

            if (current_retry_count_ < max_retry_count_) {
                // 重试同一目标
                publishGraspCommand();
                ROS_INFO("Retrying GraspCommand %d", grasp_command_index_);
            } else {
                // 重试耗尽，标记为失败
                grasp_indices_failed_.push_back(latest_result_->grasp_command_index);
                failed_track_ids_.insert(current_target_.track_id);
                ROS_WARN("GraspCommand %d exhausted retries. Marking track_id=%d as failed.",
                         latest_result_->grasp_command_index, current_target_.track_id);

                grasp_command_index_++;
                has_current_target_ = false;
                state_ = PLANNING;
            }
        }
    }

    /**
     * @brief FIN 状态处理函数
     *
     * 构造并发布 PhaseResult 消息（phase_type=GRASP），
     * 包含轮次编号、成功与失败的指令编号列表，然后重置状态回到 IDLE。
     */
    void handleFin() {
        task_planning::PhaseResult phase_result;
        phase_result.phase_type = task_planning::PhaseResult::GRASP;
        phase_result.round_index = round_index_;
        phase_result.indices_completed = grasp_indices_completed_;
        phase_result.indices_failed = grasp_indices_failed_;

        phase_result_pub_.publish(phase_result);

        ROS_INFO("Published PhaseResult: phase=GRASP, round=%d, completed=%d, failed=%d",
                 phase_result.round_index,
                 static_cast<int>(phase_result.indices_completed.size()),
                 static_cast<int>(phase_result.indices_failed.size()));
        ROS_INFO("State: FIN -> IDLE");   
        resetState();
    }

    /**
     * @brief 发布当前目标的 GraspCommand
     *
     * 构造 GraspCommand 消息，填入当前目标的 class_id、track_id 和指令编号。
     * motion_control 通过 track_id 从 detection 话题获取位姿。
     */
    void publishGraspCommand() {
        task_planning::GraspCommand cmd;
        cmd.target_class_id = current_target_.class_id;
        cmd.target_track_id = current_target_.track_id;
        cmd.grasp_command_index = grasp_command_index_;
        grasp_command_pub_.publish(cmd);
        ROS_INFO("Published GraspCommand: class_id=%d, track_id=%d, index=%d",
                 cmd.target_class_id, cmd.target_track_id, cmd.grasp_command_index);
    }

    /**
     * @brief 重置状态与计数器
     *
     * 清空本轮任务数据，重置所有索引和标志，
     * 轮次编号递增，回到 IDLE 准备下一轮任务规划。
     */
    void resetState() {
        state_ = IDLE;
        triggered_ = false;
        got_detection_ = false;
        got_result_ = false;
        latest_detection_ = nullptr;
        latest_result_ = nullptr;
        grasp_command_index_ = 0;
        current_retry_count_ = 0;
        has_current_target_ = false;
        grasp_indices_completed_.clear();
        grasp_indices_failed_.clear();
        failed_track_ids_.clear();
        round_index_++;
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
     * @return 被选中的目标物体指针，无有效候选时返回 nullptr
     *
     * 过滤掉 failed_track_ids_ 中的物体：
     * - nearest_first：选择距离原点最近的物体。
     * - 其他策略
     */
    const object_detection::DetectionObject *selectTargetObject(
        const object_detection::DetectionObjects::ConstPtr &detection) {

        std::vector<const object_detection::DetectionObject *> candidates;
        for (const auto &obj : detection->objects) {
            if (failed_track_ids_.find(obj.track_id) == failed_track_ids_.end()) {
                candidates.push_back(&obj);
            }
        }

        if (candidates.empty()) {
            return nullptr;
        }

        if (planning_strategy_ == "nearest_first") {
            double min_distance = 10.0;
            const object_detection::DetectionObject *nearest = nullptr;
            for (const auto *obj : candidates) {
                double distance = calculateDistance(obj->pose);
                if (distance < min_distance) {
                    min_distance = distance;
                    nearest = obj;
                }
            }
            return nearest;
        } else {
            return candidates.front();
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
