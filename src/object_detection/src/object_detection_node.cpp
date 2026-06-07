#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <sstream>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <detector/BoundingBoxes.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
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

namespace {
    template <typename T>
    // 获取参数，优先级：私有 > 默认值
    bool getParamWithFallback(
        const ros::NodeHandle &private_nh,
        const std::string &private_name,
        T &value,
        const T &default_value) {
        if (private_nh.getParam(private_name, value)) {
            return true;
        }
        value = default_value;
        return false;
    }

    // 获取字符串参数，优先级：私有 > 默认值
    bool getStringParamWithFallback(
        const ros::NodeHandle &private_nh,
        const std::string &private_name,
        std::string &value,
        const std::string &default_value) {
        if (private_nh.getParam(private_name, value)) {
            return true;
        }
        value = default_value;
        return false;
    }

    // 解析坐标系，优先级：depth_camera_info > depth_image > bbox > 默认值
    std::string resolveFrameId(
        const detector::BoundingBoxes &bbox_msg,
        const sensor_msgs::Image &depth_msg,
        const sensor_msgs::CameraInfo &depth_camera_info_msg) {
        if (!depth_camera_info_msg.header.frame_id.empty()) {
            return depth_camera_info_msg.header.frame_id;
        }
        if (!depth_msg.header.frame_id.empty()) {
            return depth_msg.header.frame_id;
        }
        if (!bbox_msg.header.frame_id.empty()) {
            return bbox_msg.header.frame_id;
        }
        return "kinect2_rgb_optical_frame";
    }

    // 解析时间戳，优先级：depth_camera_info > depth_image > bbox > 当前时间
    ros::Time resolveTimestamp(
        const detector::BoundingBoxes &bbox_msg,
        const sensor_msgs::Image &depth_msg,
        const sensor_msgs::CameraInfo &depth_camera_info_msg) {
        if (!depth_camera_info_msg.header.stamp.isZero()) {
            return depth_camera_info_msg.header.stamp;
        }
        if (!depth_msg.header.stamp.isZero()) {
            return depth_msg.header.stamp;
        }
        if (!bbox_msg.header.stamp.isZero()) {
            return bbox_msg.header.stamp;
        }
        return ros::Time::now();
    }
}  // namespace

class ObjectDetectionNode {
public:
    ObjectDetectionNode()
        : private_nh_("~") // 初始化私有节点句柄
        , global_nh_() // 初始化全局节点句柄
        // 初始化消息过滤器订阅者和同步器
        , bbox_sub_(global_nh_, "", 1)
        , depth_sub_(global_nh_, "", 1)
        , depth_camera_info_sub_(global_nh_, "", 1)
        , sync_(SyncPolicy(10), bbox_sub_, depth_sub_, depth_camera_info_sub_)
        // 初始化 TF 监听器，设置缓存时间为 10 秒
        , tf_listener_(ros::Duration(10.0)) {
        // 获取字符串参数
        getStringParamWithFallback(
            private_nh_, "bbox_topic", bbox_topic_, "/detector/bounding_boxes");
        getStringParamWithFallback(
            private_nh_, "depth_topic", depth_topic_, "/kinect2/sd/image_depth_rect");
        getStringParamWithFallback(
            private_nh_, "depth_camera_info_topic", depth_camera_info_topic_, "/kinect2/sd/depth_camera_info");
        getStringParamWithFallback(
            private_nh_, "detection_object_topic", detection_object_topic_, "/object_detection/detected_objects");
        getStringParamWithFallback(
            private_nh_, "pose_frame", pose_frame_, "base_link");

        // 获取参数
        getParamWithFallback(private_nh_, "depth_window_size", depth_window_size_, 5);
        getParamWithFallback(private_nh_, "sync_queue_size", sync_queue_size_, 10);
        getParamWithFallback(private_nh_, "sync_slop", sync_slop_, 0.08);
        getParamWithFallback(private_nh_, "min_depth", min_depth_, 0.1);
        getParamWithFallback(private_nh_, "max_depth", max_depth_, 6.0);

        getParamWithFallback(private_nh_, "publish_point_cloud", publish_point_cloud_, false);

        // 确保深度采样窗口大小为正奇数
        if (depth_window_size_ < 1) {
            depth_window_size_ = 1;
        }
        if ((depth_window_size_ % 2) == 0) {
            ++depth_window_size_;
        }

        // 初始化发布者和订阅者
        detection_object_pub_ = global_nh_.advertise<object_detection::DetectionObjects>(detection_object_topic_, 1);

        bbox_sub_.subscribe(global_nh_, bbox_topic_, 5);
        depth_sub_.subscribe(global_nh_, depth_topic_, 5);
        depth_camera_info_sub_.subscribe(global_nh_, depth_camera_info_topic_, 5);

        // 设置同步器
        sync_.setMaxIntervalDuration(ros::Duration(sync_slop_));
        sync_.registerCallback(boost::bind(&ObjectDetectionNode::syncCallback, this, _1, _2, _3));

        ROS_INFO (
            "object_detection start | bbox=%s depth=%s depth_camera_info=%s detection_objects=%s pose_frame=%s",
            bbox_topic_.c_str(),
            depth_topic_.c_str(),
            depth_camera_info_topic_.c_str(),
            detection_object_topic_.c_str(),
            pose_frame_.c_str() );
    }

private:
    tf::TransformListener tf_listener_;
    std::string pose_frame_;

private:
    // 定义同步策略
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        detector::BoundingBoxes,
        sensor_msgs::Image,
        sensor_msgs::CameraInfo>;

    // 同步回调函数
    void syncCallback (
        const detector::BoundingBoxesConstPtr &bbox_msg,
        const sensor_msgs::ImageConstPtr &depth_msg,
        const sensor_msgs::CameraInfoConstPtr &depth_camera_info_msg) {

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
        } catch (const cv_bridge::Exception &error) {
            ROS_ERROR("Failed to convert depth image: %s", error.what());
            return;
        }

        cv::Mat depth_image = normalizeDepthImage(cv_ptr->image, depth_msg->encoding);

        object_detection::DetectionObjects detection_objects_msg;
        detection_objects_msg.header.stamp = resolveTimestamp(*bbox_msg, *depth_msg, *depth_camera_info_msg);
        detection_objects_msg.header.frame_id = resolveFrameId(*bbox_msg, *depth_msg, *depth_camera_info_msg);

        // 获取相机内参
        const double fx = depth_camera_info_msg->K[0];
        const double fy = depth_camera_info_msg->K[4];
        const double cx = depth_camera_info_msg->K[2];
        const double cy = depth_camera_info_msg->K[5];

        if (fx == 0.0 || fy == 0.0) {
            ROS_ERROR_THROTTLE(5.0, "Invalid camera intrinsics: fx/fy cannot be zero");
            return;
        }

        // 计算源图像坐标到深度图像坐标的缩放比例
        const double source_width = static_cast<double>(bbox_msg->width);
        const double source_height = static_cast<double>(bbox_msg->height);
        const double scale_x = static_cast<double>(depth_image.cols) / source_width;
        const double scale_y = static_cast<double>(depth_image.rows) / source_height;
        
        for (std::size_t bbox_index = 0; bbox_index < bbox_msg->bounding_boxes.size(); ++bbox_index) {
            const auto &bounding_box = bbox_msg->bounding_boxes[bbox_index];

            int x_min = 0;
            int y_min = 0;
            int x_max = 0;
            int y_max = 0;
            mapBoundingBoxToDepth(bounding_box, scale_x, scale_y, depth_image.cols, depth_image.rows, x_min, y_min, x_max, y_max);

            // 获取 bbox 中心点的像素坐标
            const int u = static_cast<int>(std::lround((x_min + x_max) / 2.0));
            const int v = static_cast<int>(std::lround((y_min + y_max) / 2.0));

            double depth_val = 0.0;
            sampleDepth(depth_image, u, v, depth_val);
            // 如果没有有效的深度值，则跳过该检测对象
            if (!std::isfinite(depth_val)) {
                ROS_WARN_THROTTLE(5.0, "No valid depth at bbox center (u=%d, v=%d)", u, v);
                continue;
            }

            // 将像素坐标和深度值转换为相机坐标系下的三维点
            const double x = (static_cast<double>(u) - cx) * depth_val / fx;
            const double y = (static_cast<double>(v) - cy) * depth_val / fy;

            object_detection::DetectionObject detection_object;
            detection_object.class_id = static_cast<int32_t>(bounding_box.class_id);

            // 构造相机坐标系下的位姿
            geometry_msgs::PoseStamped pose_cam;
            geometry_msgs::PoseStamped pose_obj;

            pose_cam.header.stamp = detection_objects_msg.header.stamp;
            pose_cam.header.frame_id = detection_objects_msg.header.frame_id;
            pose_cam.pose.position.x = x;
            pose_cam.pose.position.y = y;
            pose_cam.pose.position.z = depth_val;
            pose_cam.pose.orientation.x = 0.0;
            pose_cam.pose.orientation.y = 0.0;
            pose_cam.pose.orientation.z = 0.0;
            pose_cam.pose.orientation.w = 1.0;

            // bbox 对应区域的点云生成
            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            const cv::Rect roi_rect(x_min, y_min, x_max - x_min + 1, y_max - y_min + 1);
            const cv::Mat depth_roi = depth_image(roi_rect);

            // 创建掩码，筛选出深度值为有限数的像素
            cv::Mat finite_mask;
            cv::compare(depth_roi, depth_roi, finite_mask, cv::CMP_EQ);
            // 创建掩码，筛选出深度值在 min_depth_ 和 max_depth_ 范围内的像素
            cv::Mat range_mask;
            cv::inRange(depth_roi, static_cast<float>(min_depth_), static_cast<float>(max_depth_), range_mask);
            // 将两个掩码进行按位与运算，得到最终的有效像素掩码
            cv::Mat valid_mask;
            cv::bitwise_and(finite_mask, range_mask, valid_mask);

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
                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(point_cloud, cloud_msg);

                // 保持点云消息与外层 detection_object 消息的时间戳和坐标系一致
                cloud_msg.header.stamp = detection_objects_msg.header.stamp;
                cloud_msg.header.frame_id = detection_objects_msg.header.frame_id;

                // 使用点云，通过 PCA 计算物体的主方向，并将其转换为四元数表示的姿态
                pcl::PCA<pcl::PointXYZ> pca;
                pca.setInputCloud(point_cloud.makeShared());
                Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
                // 确保右手坐标系，如果特征向量矩阵的行列式为负数，则说明它是一个左手坐标系，需要将其中一列取反来修正
                if (eigen_vectors.determinant() < 0.0f) {
                    eigen_vectors.col(2) *= -1.0f;
                }
                // 将特征向量矩阵转换为四元数表示的旋转，假设特征向量矩阵的列分别对应于相机坐标系下的 x、y、z 轴方向
                Eigen::Quaternionf quaternion(eigen_vectors);
                // 由于 PCA 计算得到的特征向量可能没有单位长度，因此需要对四元数进行归一化，确保其表示的是一个有效的旋转
                quaternion.normalize();

                // 构造相机坐标系下的位姿
                pose_cam.pose.orientation.x = quaternion.x();
                pose_cam.pose.orientation.y = quaternion.y();
                pose_cam.pose.orientation.z = quaternion.z();
                pose_cam.pose.orientation.w = quaternion.w();

                if (publish_point_cloud_) {
                    // 将点云消息直接附加到检测对象消息中，供后续处理使用
                    // 注意：这种做法会增加消息的大小，可能会影响通信效率，实际使用时需要权衡利弊
                    detection_object.cloud = cloud_msg;
                }
            }

            // 尝试将位姿从相机坐标系转换到目标坐标系，如果转换失败则使用相机坐标系下的位姿
            try {
                tf_listener_.transformPose(pose_frame_, pose_cam, pose_obj);
                detection_object.pose = pose_obj.pose;
            } catch (const tf::TransformException &ex) {
                ROS_WARN_THROTTLE(5.0, "TF transform to %s failed: %s", pose_frame_.c_str(), ex.what());
                detection_object.pose = pose_cam.pose;
            }

            detection_objects_msg.objects.push_back(detection_object);
        }
        detection_object_pub_.publish(detection_objects_msg);
    }

    // 将 bbox 从源图坐标映射到深度图坐标
    void mapBoundingBoxToDepth(
        const detector::BoundingBox &bounding_box,
        double scale_x,
        double scale_y,
        int depth_width,
        int depth_height,
        int &x_min,
        int &y_min,
        int &x_max,
        int &y_max) const {
        
        // 将源图坐标映射到深度图坐标
        x_min = static_cast<int>(std::lround(bounding_box.x_min * scale_x));
        x_max = static_cast<int>(std::lround(bounding_box.x_max * scale_x));
        y_min = static_cast<int>(std::lround(bounding_box.y_min * scale_y));
        y_max = static_cast<int>(std::lround(bounding_box.y_max * scale_y));

        // 确保映射后的 bbox 坐标在深度图像范围内
        x_min = std::max(0, std::min(x_min, depth_width - 1));
        x_max = std::max(0, std::min(x_max, depth_width - 1));
        y_min = std::max(0, std::min(y_min, depth_height - 1));
        y_max = std::max(0, std::min(y_max, depth_height - 1));
    }

    // 深度图像归一化处理，转换为以米为单位的浮点数，并将无效值设置为 NaN
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

        // 向量化筛选有效深度：finite 且 > 0，其余统一置为 NaN
        // 创建掩码 finite_mask，标记出所有深度值为有限数的位置
        cv::Mat finite_mask;
        cv::compare(normalized, normalized, finite_mask, cv::CMP_EQ);
        // 创建掩码 positive_mask，标记出所有深度值大于零的位置
        cv::Mat positive_mask;
        cv::compare(normalized, 0.0f, positive_mask, cv::CMP_GT);
        // 将 finite_mask 和 positive_mask 进行按位与运算，得到一个新的掩码 valid_mask，标记出所有既是有限数又大于零的位置
        cv::Mat valid_mask;
        cv::bitwise_and(finite_mask, positive_mask, valid_mask);
        // 通过对 valid_mask 进行按位取反，得到 invalid_mask，标记出所有无效深度值的位置
        cv::Mat invalid_mask;
        cv::bitwise_not(valid_mask, invalid_mask);
        // 将 normalized 中所有无效深度值的位置（即 invalid_mask 中为非零的位置）设置为 NaN，确保后续处理时这些位置不会被误用
        normalized.setTo(cv::Scalar(std::numeric_limits<float>::quiet_NaN()), invalid_mask);

        return normalized;
    }

    // 从深度图像中采样，获取像素 (u, v) 附近窗口内的有效深度值，并返回中位数作为该点的深度
    void sampleDepth(const cv::Mat &depth_image, int u, int v, double &out_depth) const {
        const int half_window = depth_window_size_ / 2;
        const int x0 = std::max(0, u - half_window);
        const int x1 = std::min(depth_image.cols, u + half_window + 1);
        const int y0 = std::max(0, v - half_window);
        const int y1 = std::min(depth_image.rows, v + half_window + 1);

        // 复用采样缓存，避免每次采样时的动态分配开销
        depth_samples_buffer_.clear();
        depth_samples_buffer_.reserve(static_cast<std::size_t>((x1 - x0) * (y1 - y0)));

        for (int row = y0; row < y1; ++row) {
            const float *row_ptr = depth_image.ptr<float>(row);
            for (int col = x0; col < x1; ++col) {
                const float depth = row_ptr[col];
                if (std::isfinite(depth) && depth >= min_depth_ && depth <= max_depth_) {
                    depth_samples_buffer_.push_back(depth);
                }
            }
        }

        std::vector<float> &valid = depth_samples_buffer_;

        // 如果没有有效的深度值，则返回 NaN
        if (valid.empty()) {
            out_depth = std::numeric_limits<double>::quiet_NaN();
            return;
        }

        const std::size_t middle = valid.size() / 2;
        std::nth_element(valid.begin(), valid.begin() + middle, valid.end());
        if ((valid.size() % 2) == 0) {
            const float upper = valid[middle];
            std::nth_element(valid.begin(), valid.begin() + middle - 1, valid.begin() + middle);
            const float lower = valid[middle - 1];
            out_depth = (static_cast<double>(lower) + static_cast<double>(upper)) / 2.0;
        } else {
            out_depth = static_cast<double>(valid[middle]);
        }
    }

    // ROS 节点句柄
    ros::NodeHandle global_nh_;
    ros::NodeHandle private_nh_;

    // 参数
    std::string bbox_topic_;
    std::string depth_topic_;
    std::string depth_camera_info_topic_;
    std::string detection_object_topic_;

    int depth_window_size_ = 5;
    int sync_queue_size_ = 10;
    double sync_slop_ = 0.08;
    double min_depth_ = 0.1;
    double max_depth_ = 6.0;

    // 复用采样缓存，减少每次采样的动态分配开销
    mutable std::vector<float> depth_samples_buffer_;

    ros::Publisher detection_object_pub_;

    // 消息过滤器订阅者和同步器
    message_filters::Subscriber<detector::BoundingBoxes> bbox_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> depth_camera_info_sub_;

    message_filters::Synchronizer<SyncPolicy> sync_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_detection");
    ObjectDetectionNode node;
    ros::spin();
    return 0;
}
