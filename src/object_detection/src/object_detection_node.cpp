#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <detector/BoundingBoxes.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <object_detection/DetectionObject.h>
#include <object_detection/DetectionObjects.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

/**
 * @class ObjectDetectionNode
 * @brief 物体检测节点
 *
 * 同步订阅边界框、深度图和相机内参，基于深度图生成点云，
 * 计算每个检测物体的三维位姿，并发布检测结果。
 */
class ObjectDetectionNode {
public:
    /**
     * @brief 构造函数：加载参数、初始化 TF 监听器、同步订阅者和发布者
     */
    ObjectDetectionNode()
        // ROS 节点句柄
        : global_nh_()  ///< 全局节点句柄，用于订阅外部话题
        , private_nh_("~")  ///< 私有节点句柄，用于读取参数和发布话题
        // TF 监听器
        , tf_listener_(ros::Duration(private_nh_.param("tf_cache_time", 10.0)))  ///< TF 变换监听器，缓存时间 10 秒
        // 消息过滤器订阅者
        , bbox_sub_(global_nh_, private_nh_.param("bbox_topic", std::string("/detector/bounding_boxes")), 5)  ///< 边界框订阅者
        , depth_sub_(global_nh_, private_nh_.param("depth_topic", std::string("/kinect2/qhd/image_depth_rect")), 5)  ///< 深度图订阅者
        , camera_info_sub_(global_nh_, private_nh_.param("camera_info_topic", std::string("/kinect2/qhd/camera_info")), 5)  ///< 相机内参订阅者
        // 同步器
        , sync_(SyncPolicy(private_nh_.param("sync_queue_size", 10)), bbox_sub_, depth_sub_, camera_info_sub_)  ///< 边界框/深度图/相机内参同步器
        // 发布者
        , detection_object_pub_(private_nh_.advertise<object_detection::DetectionObjects>(
              private_nh_.param("object_detection_topic", std::string("/object_detection/detected_objects")), 1))  ///< 检测结果发布者
        // 深度处理参数
        , min_depth_(private_nh_.param("min_depth", 0.1))  ///< 有效最小深度（米）
        , max_depth_(private_nh_.param("max_depth", 6.0))  ///< 有效最大深度（米）
        , depth_tolerance_(private_nh_.param("depth_tolerance", 0.05))  ///< 前景过滤深度容差（米）
        // 类别修正参数
        , confidence_threshold_(private_nh_.param("confidence_threshold", 0.8))  ///< 置信度阈值，低于此值使用点云形状修正类别
        , large_object_diagonal_threshold_(private_nh_.param("large_object_diagonal_threshold", 0.18))  ///< 大物体对角线阈值（米）
        , min_points_for_shape_(private_nh_.param("min_points_for_shape", 30))  ///< 用于形状分析的最少点数
        // 其他参数
        , pose_frame_(private_nh_.param("pose_frame", std::string("base_link")))  ///< 目标输出坐标系
        , publish_point_cloud_(private_nh_.param("publish_point_cloud", false)) {  ///< 是否在检测结果中附带点云
        // 设置同步器最大时间间隔
        sync_.setMaxIntervalDuration(ros::Duration(private_nh_.param("sync_slop", 0.08)));
        // 注册同步回调函数
        sync_.registerCallback(boost::bind(&ObjectDetectionNode::syncCallback, this, _1, _2, _3));

        ROS_INFO("object_detection start | pose: %s", pose_frame_.c_str());
    }

private:
    /**
     * @brief 同步回调函数
     * @param bbox_msg 边界框消息
     * @param depth_msg 深度图消息
     * @param camera_info_msg 相机内参消息
     *
     * 对输入消息进行时间同步，处理每个边界框，生成点云并计算物体位姿，
     * 最终发布检测结果。
     */
    void syncCallback(
        const detector::BoundingBoxesConstPtr &bbox_msg,
        const sensor_msgs::ImageConstPtr &depth_msg,
        const sensor_msgs::CameraInfoConstPtr &camera_info_msg) {

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
        } catch (const cv_bridge::Exception &error) {
            ROS_ERROR("Failed to convert depth image: %s", error.what());
            return;
        }

        cv::Mat depth_image = normalizeDepthImage(cv_ptr->image, depth_msg->encoding);

        object_detection::DetectionObjects detection_objects_msg;
        detection_objects_msg.header.stamp = camera_info_msg->header.stamp;
        detection_objects_msg.header.frame_id = camera_info_msg->header.frame_id;

        // 获取相机内参
        const double fx = camera_info_msg->K[0];
        const double fy = camera_info_msg->K[4];
        const double cx = camera_info_msg->K[2];
        const double cy = camera_info_msg->K[5];

        if (fx == 0.0 || fy == 0.0) {
            ROS_ERROR_THROTTLE(5.0, "Invalid camera intrinsics: fx/fy cannot be zero");
            return;
        }

        for (std::size_t bbox_index = 0; bbox_index < bbox_msg->bounding_boxes.size(); ++bbox_index) {
            const auto &bounding_box = bbox_msg->bounding_boxes[bbox_index];

            int x_min = bounding_box.x_min;
            int y_min = bounding_box.y_min;
            int x_max = bounding_box.x_max;
            int y_max = bounding_box.y_max;

            object_detection::DetectionObject detection_object;
            detection_object.class_id = static_cast<int32_t>(bounding_box.class_id);

            // bbox 对应区域的点云生成
            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            const cv::Rect roi_rect(x_min, y_min, x_max - x_min + 1, y_max - y_min + 1);
            const cv::Mat depth_roi = depth_image(roi_rect);

            // 创建范围掩码，筛选出深度值在 min_depth_ 和 max_depth_ 范围内的像素
            cv::Mat range_mask;
            cv::inRange(depth_roi, static_cast<float>(min_depth_), static_cast<float>(max_depth_), range_mask);

            // 创建前景掩码：只保留与 bbox 内最小深度相差 depth_tolerance_ 以内的像素，
            // 用于剔除背景或远离前景目标的异常深度点
            cv::Mat foreground_mask;
            double min_depth_in_roi = 0.0;
            double max_depth_in_roi = 0.0;
            cv::Point min_loc;
            cv::Point max_loc;
            const cv::Mat roi_valid_mask = range_mask;
            cv::minMaxLoc(depth_roi, &min_depth_in_roi, &max_depth_in_roi, &min_loc, &max_loc, roi_valid_mask);
            if (roi_valid_mask.empty() || cv::countNonZero(roi_valid_mask) == 0) {
                // ROI 内没有有效深度，跳过该边界框
                continue;
            }
            cv::inRange(depth_roi, static_cast<float>(min_depth_in_roi), static_cast<float>(min_depth_in_roi + depth_tolerance_), foreground_mask);

            // 将范围掩码与前景掩码进行按位与运算，得到最终的有效前景像素掩码
            cv::Mat valid_mask;
            cv::bitwise_and(range_mask, foreground_mask, valid_mask);

            // 获取有效像素的坐标
            std::vector<cv::Point> valid_pixels;
            cv::findNonZero(valid_mask, valid_pixels);
            // 预分配点云内存，避免频繁扩容
            point_cloud.reserve(valid_pixels.size());
            // 预计算 x 和 y 方向的基准值，避免在循环中重复计算，提高效率
            cv::Mat x_base(1, depth_roi.cols, CV_32FC1);
            cv::Mat y_base(depth_roi.rows, 1, CV_32FC1);
            // x_base 和 y_base 的计算基于深度图像坐标系下的像素坐标与相机内参的关系，转换为相机坐标系下的单位向量
            for (int col = 0; col < depth_roi.cols; ++col) {
                x_base.at<float>(0, col) = (static_cast<float>(x_min + col) - static_cast<float>(cx)) / static_cast<float>(fx);
            }
            for (int row = 0; row < depth_roi.rows; ++row) {
                y_base.at<float>(row, 0) = (static_cast<float>(y_min + row) - static_cast<float>(cy)) / static_cast<float>(fy);
            }
            // 遍历有效像素，计算其在相机坐标系下的三维坐标，并添加到点云中
            for (const cv::Point &pixel : valid_pixels) {
                const float d = depth_roi.at<float>(pixel.y, pixel.x);
                const float px = x_base.at<float>(0, pixel.x) * d;
                const float py = y_base.at<float>(pixel.y, 0) * d;

                pcl::PointXYZ point;
                point.x = px;
                point.y = py;
                point.z = d;
                point_cloud.push_back(point);
            }

            if (!point_cloud.empty()) {
                // 如果检测框的置信度低于阈值，则使用点云形状进行类别修正
                const bool low_confidence = static_cast<double>(bounding_box.confidence) < confidence_threshold_;
                if (low_confidence) {
                    const int refined_class_id = refineClassByPointCloudShape(
                        point_cloud,
                        static_cast<int>(bounding_box.class_id),
                        large_object_diagonal_threshold_,
                        min_points_for_shape_);
                    detection_object.class_id = static_cast<int32_t>(refined_class_id);
                    ROS_WARN_THROTTLE(
                        5.0,
                        "Low-confidence detection (conf=%.3f) refined by point cloud shape: class_id=%d",
                        static_cast<double>(bounding_box.confidence),
                        detection_object.class_id);
                }

                // 点云质心
                Eigen::Vector4f centroid = computeObjectCentroid(point_cloud, static_cast<int>(detection_object.class_id));

                // 构造相机坐标系下的位姿
                geometry_msgs::PoseStamped pose_cam;
                geometry_msgs::PoseStamped pose_obj;

                pose_cam.header.stamp = detection_objects_msg.header.stamp;
                pose_cam.header.frame_id = detection_objects_msg.header.frame_id;
                pose_cam.pose.position.x = centroid[0];
                pose_cam.pose.position.y = centroid[1];
                pose_cam.pose.position.z = centroid[2];
                pose_cam.pose.orientation.x = 0.0;
                pose_cam.pose.orientation.y = 0.0;
                pose_cam.pose.orientation.z = 0.0;
                pose_cam.pose.orientation.w = 1.0;

                // 尝试将位姿从相机坐标系转换到目标坐标系，如果转换失败则使用相机坐标系下的位姿
                try {
                    tf_listener_.transformPose(pose_frame_, pose_cam, pose_obj);
                    detection_object.pose = pose_obj;
                } catch (const tf::TransformException &ex) {
                    ROS_WARN_THROTTLE(5.0, "TF transform to %s failed: %s", pose_frame_.c_str(), ex.what());
                    detection_object.pose = pose_cam;
                }

                if (publish_point_cloud_) {
                    // 将点云消息直接附加到检测对象消息中，供后续处理使用
                    // 注意：这种做法会增加消息的大小，可能会影响通信效率，实际使用时需要权衡利弊
                    // 点云消息
                    sensor_msgs::PointCloud2 cloud_msg;
                    // 保持点云消息与外层 detection_object 消息的时间戳和坐标系一致
                    cloud_msg.header.stamp = detection_objects_msg.header.stamp;
                    cloud_msg.header.frame_id = detection_objects_msg.header.frame_id;

                    pcl::toROSMsg(point_cloud, cloud_msg);
                    detection_object.cloud = cloud_msg;
                }
            }

            detection_objects_msg.objects.push_back(detection_object);
        }
        detection_object_pub_.publish(detection_objects_msg);
    }

    /**
     * @brief 深度图像归一化处理
     * @param depth_image 原始深度图像
     * @param encoding 图像编码
     * @return 以米为单位的 32 位浮点深度图，无效值为 NaN
     */
    cv::Mat normalizeDepthImage(const cv::Mat &depth_image, const std::string &encoding) const {
        cv::Mat normalized;
        // 如果深度图像的编码是 16 位无符号整数或单通道 16 位图像，则将其转换为以米为单位的 32 位浮点数
        if (encoding == "16UC1" || encoding == "mono16" || depth_image.type() == CV_16UC1) {
            depth_image.convertTo(normalized, CV_32FC1, 0.001);
        } else if (depth_image.type() == CV_32FC1) {
            normalized = depth_image.clone();
        } else {
            depth_image.convertTo(normalized, CV_32FC1);
        }

        // 向量化筛选有效深度：> 0（NaN 在比较中会被排除），其余统一置为 NaN

        // 创建掩码 positive_mask，标记出所有深度值大于零的位置
        cv::Mat positive_mask;
        cv::compare(normalized, 0.0f, positive_mask, cv::CMP_GT);
        cv::Mat valid_mask = positive_mask.clone();

        // 通过对 valid_mask 进行按位取反，得到 invalid_mask，标记出所有无效深度值的位置
        cv::Mat invalid_mask;
        cv::bitwise_not(valid_mask, invalid_mask);
        // 将 normalized 中所有无效深度值的位置（即 invalid_mask 中为非零的位置）设置为 NaN，确保后续处理时这些位置不会被误用
        normalized.setTo(cv::Scalar(std::numeric_limits<float>::quiet_NaN()), invalid_mask);

        return normalized;
    }

    /**
     * @brief 根据点云形状修正物体类别
     * @param point_cloud 物体点云
     * @param original_class_id 原始类别 ID
     * @param large_object_diagonal_threshold 大物体对角线阈值（米）
     * @param min_points_for_shape 形状分析所需最少点数
     * @return 修正后的类别 ID
     *
     * 通过点云包围盒对角线长度判断物体大小，进而修正类别。
     */
    int refineClassByPointCloudShape(
        const pcl::PointCloud<pcl::PointXYZ> &point_cloud,
        int original_class_id,
        double large_object_diagonal_threshold,
        std::size_t min_points_for_shape) {
        if (point_cloud.size() < min_points_for_shape) {
            return original_class_id;
        }

        pcl::PointXYZ min_point;
        pcl::PointXYZ max_point;
        pcl::getMinMax3D(point_cloud, min_point, max_point);

        const double dx = static_cast<double>(max_point.x - min_point.x);
        const double dy = static_cast<double>(max_point.y - min_point.y);
        const double dz = static_cast<double>(max_point.z - min_point.z);

        // 通过计算点云包围盒的对角线长度来判断物体的大小，进而进行类别修正
        const double diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
        const bool is_large = diagonal >= large_object_diagonal_threshold;

        int color_group_base = -1;
        if (original_class_id == 0 || original_class_id == 1) {
            color_group_base = 0;
        } else if (original_class_id == 2 || original_class_id == 3) {
            color_group_base = 2;
        }

        if (color_group_base < 0) {
            return original_class_id;
        }

        return color_group_base + (is_large ? 0 : 1);
    }

    /**
     * @brief 计算物体点云的质心
     * @param cloud 物体点云
     * @param class_id 物体类别 ID
     * @return 质心坐标（齐次坐标形式）
     *
     * 对立方体类物体，在质心基础上沿 z 轴偏移半个点云深度。
     */
    Eigen::Vector4f computeObjectCentroid(
        const pcl::PointCloud<pcl::PointXYZ>& cloud,
        int class_id) {

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(cloud, centroid);

        // 根据物体类型决定是否需要深度偏移
        switch (class_id) {
            case 0:
            case 1:
            case 2:
            case 3:
                // 立方体物体
                {
                    pcl::PointXYZ min_pt, max_pt;
                    pcl::getMinMax3D(cloud, min_pt, max_pt);
                    double dz = max_pt.z - min_pt.z;
                    centroid[2] += dz / 2.0;  // 偏移半个深度
                }
                break;
            default:
                // 默认不偏移
                break;
        }

        return centroid;
    }

    /**
     * @brief 同步策略类型
     *
     * 使用 ApproximateTime 策略同步边界框、深度图和相机内参消息。
     */
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        detector::BoundingBoxes,
        sensor_msgs::Image,
        sensor_msgs::CameraInfo>;

    // ROS 节点句柄
    ros::NodeHandle global_nh_;
    ros::NodeHandle private_nh_;

    // TF 监听器
    tf::TransformListener tf_listener_;

    // 消息过滤器订阅者
    message_filters::Subscriber<detector::BoundingBoxes> bbox_sub_;        ///< 边界框订阅者
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;            ///< 深度图订阅者
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub_; ///< 相机内参订阅者

    // 同步器
    message_filters::Synchronizer<SyncPolicy> sync_;  ///< 消息同步器

    // 发布者
    ros::Publisher detection_object_pub_;  ///< 检测结果发布者

    // 深度处理参数
    const double min_depth_;  ///< 有效最小深度（米）
    const double max_depth_;  ///< 有效最大深度（米）
    const double depth_tolerance_;  ///< 前景过滤深度容差（米）

    // 类别修正参数
    const double confidence_threshold_;           ///< 置信度阈值
    const double large_object_diagonal_threshold_; ///< 大物体对角线阈值（米）
    const int min_points_for_shape_;              ///< 形状分析所需最少点数

    // 其他参数
    const std::string pose_frame_;        ///< 目标输出坐标系
    const bool publish_point_cloud_;      ///< 是否在检测结果中附带点云
};

/**
 * @brief 主函数                    
 * @param argc 参数个数
 * @param argv 参数列表
 * @return 程序退出码
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "object_detection");
    ObjectDetectionNode node;
    ros::spin();
    return 0;
}
