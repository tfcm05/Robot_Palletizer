#include <cmath>
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <robot_core/Info.h>
#include <object_detection/DetectionObjects.h>
#include <object_detection/DetectionObject.h>
#include <task_planning/TaskCommand.h>
#include <task_planning/TaskResult.h>

class TaskPlanningNode {
public:
    TaskPlanningNode() :
        nh_(),
        rate_(10.0)
    {
        // 最大任务数量
        nh_.param<int>("max_task_count", max_task_count_, 3);
        // 规划策略参数
        nh_.param<std::string>("planning_strategy", planning_strategy_, "nearest_first");

        // 循环频率参数
        nh_.param<double>("loop_rate", loop_rate_, 10.0);

        // 获取话题名称参数
        std::string detection_topic, task_result_topic, info_topic, command_topic;
        // 订阅话题参数
        nh_.param<std::string>("detection_topic", detection_topic, "/object_detection/detected_objects");
        nh_.param<std::string>("task_result_topic", task_result_topic, "/motion_control/task_result");
        nh_.param<std::string>("info_topic", info_topic, "/ctrl/info");
        // 发布话题参数
        nh_.param<std::string>("command_topic", command_topic, "/task_planning/task_command");

        // 初始化订阅者
        detection_sub_ = nh_.subscribe(detection_topic, 5, &TaskPlanningNode::detectionCallback, this);
        task_result_sub_ = nh_.subscribe(task_result_topic, 5, &TaskPlanningNode::taskResultCallback, this);
        info_sub_ = nh_.subscribe(info_topic, 5, &TaskPlanningNode::infoCallback, this);

        // 初始化发布者
        command_pub_ = nh_.advertise<task_planning::TaskCommand>(command_topic, 1);

        rate_ = ros::Rate(loop_rate_);

        ROS_INFO("TaskPlanningNode initialized. Strategy: %s, Max tasks: %d, Rate: %.1f Hz",
                 planning_strategy_.c_str(), max_task_count_, loop_rate_);
    }

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
                case DONE:
                    handleDone();
                    break;
            }

            rate_.sleep();
        }
    }

private:
    enum State { IDLE, PLANNING, WAITING_RESULT, DONE, FIN };

    // ROS 节点句柄
    ros::NodeHandle nh_;

    // 订阅者和发布者
    ros::Subscriber detection_sub_;
    ros::Subscriber task_result_sub_;
    ros::Subscriber info_sub_;
    ros::Publisher command_pub_;

    // 循环频率控制器
    ros::Rate rate_;

    // 当前状态
    State state_ = IDLE; // 初始状态为 IDLE

    // 配置参数
    int max_task_count_;
    double loop_rate_;
    std::string planning_strategy_;

    // 任务执行状态
    int task_index_ = 0;
    int task_completed_count_ = 0;
    bool triggered_ = false;

    bool got_detection_ = false;
    object_detection::DetectionObjects::ConstPtr latest_detection_ = nullptr;

    bool got_result_ = false;
    task_planning::TaskResult::ConstPtr latest_result_ = nullptr;

    // 物体识别回调函数，接收检测结果并存储
    void detectionCallback(const object_detection::DetectionObjects::ConstPtr& msg) {
        latest_detection_ = msg;
        got_detection_ = true;
    }

    // 任务结果回调函数，接收执行结果并存储
    void taskResultCallback(const task_planning::TaskResult::ConstPtr& msg) {
        latest_result_ = msg;
        got_result_ = true;
    }

    // 机器人状态回调函数，接收状态信息并根据规划触发条件更新状态
    void infoCallback(const robot_core::Info::ConstPtr& msg) {
        if (msg->mode == robot_core::Info::TASK_SCHEDULE) {
            if (state_ == IDLE) {
                triggered_ = true;
                ROS_INFO("TASK_SCHEDULE trigger received, entering PLANNING state");
            }
        }
    }

    // 状态处理函数
    void handleIdle() {
        // 在IDLE状态下等待触发信号，触发后进入PLANNING状态
        if (triggered_) {
            state_ = PLANNING;
            ROS_INFO("State: IDLE -> PLANNING");
        }
    }

    void handlePlanning() {
        if (!got_detection_ || !latest_detection_) {
            // 如果没有收到检测数据，则保持在PLANNING状态，等待数据到来
            ROS_INFO_THROTTLE(5.0, "PLANNING: Waiting for detection data...");
            return;
        }

        // 如果没有检测到任何物体，则直接进入 FIN 状态
        if (latest_detection_->objects.empty()) {
            ROS_WARN("PLANNING: No objects detected. Going to FIN.");
            state_ = FIN;
            return;
        }

        auto object = selectTargetObject(latest_detection_);

        // 根据规划策略选择下一个目标物体，并发布 TaskCommand 消息
        const auto& obj = object;

        task_planning::TaskCommand cmd;
        cmd.target_class_id = obj.class_id;
        cmd.target_pose = obj.pose;
        cmd.task_index = task_index_;

        command_pub_.publish(cmd);
        ROS_INFO("Published TaskCommand: class_id=%d, index=%d",
                 cmd.target_class_id, cmd.task_index);

        got_detection_ = false; // 重置检测标志，等待下一轮检测数据
        state_ = WAITING_RESULT;
    }

    void handleWaitingResult() {
        // 如果没有收到任务执行结果，则保持在 WAITING_RESULT 状态，等待结果到来
        if (!got_result_ || !latest_result_) {
            return;
        }

        const auto& result = *latest_result_;

        if (result.status == task_planning::TaskResult::SUCCESS) {
            ROS_INFO("Task %d succeeded: %s", task_index_, result.message.c_str());
            task_index_++;
            task_completed_count_++;
            if (task_completed_count_ == max_task_count_) {
                ROS_INFO("Reached max task count (%d). Going to DONE.", max_task_count_);
                state_ = DONE;
            } else {
                state_ = PLANNING;
            }
        } else {
            ROS_ERROR("Task %d failed: %s. Going to DONE.", task_index_, result.message.c_str());
            task_index_++;
            state_ = DONE;
        }

        got_result_ = false; // 重置结果标志，等待下一轮结果数据
    }

    void handleDone() {
        ROS_INFO("Task planning cycle completed. %d tasks processed. Returning to IDLE.", task_completed_count_);
        state_ = IDLE;
        triggered_ = false;
        task_index_ = 0;
        task_completed_count_ = 0;
        got_detection_ = false;
        got_result_ = false;
        latest_detection_ = nullptr;
        latest_result_ = nullptr;
    }

    double calculateDistance(const geometry_msgs::Pose& pose) {
        double dx = pose.position.x;
        double dy = pose.position.y;
        double dz = pose.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    DetectionObject selectTargetObject(const object_detection::DetectionObjects::ConstPtr& detection) {
        if (planning_strategy_ == "nearest_first") {
            // 最近优先策略：选择距离最近的物体作为目标
            double min_distance = 10.0; // 设置一个较大的初始距离阈值
            DetectionObject nearest_obj;
            for (const auto& obj : detection->objects) {
                double distance = calculateDistance(obj.pose);
                if (distance < min_distance) {
                    min_distance = distance;
                    nearest_obj = obj;
                }
            }
            return nearest_obj;
        } else {
            // 其他策略（如 custom_logic）可以在这里实现
            // 目前默认返回第一个检测到的物体
            return detection->objects.front();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_planning_node");
    TaskPlanningNode node;
    node.run();
    return 0;
}
