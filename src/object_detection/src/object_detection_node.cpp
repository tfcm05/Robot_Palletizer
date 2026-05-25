#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <optional>
#include <string>
#include <sstream>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <detector/BoundingBoxes.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <object_detection/DetectionObject.h>
#include <object_detection/DetectionObjects.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

namespace {
    template <typename T>
    // 获取参数，优先级：私有 > 全局 > 默认值
    bool getParamWithFallback (
        const ros::NodeHandle &private_nh,
        const ros::NodeHandle &global_nh,
        const std::string &private_name,
        const std::string &global_name,
        T &value,
        const T &default_value ) {
        if (private_nh.getParam(private_name, value)) {
            return true;
        }
        if (global_nh.getParam(global_name, value)) {
            return true;
        }
        value = default_value;
        return false;
    }

    // 获取字符串参数，优先级：私有 > 全局（多个） > 默认值
    bool getStringParamWithFallback (
        const ros::NodeHandle &private_nh,
        const ros::NodeHandle &global_nh,
        const std::string &private_name,
        const std::initializer_list<std::string> &global_names,
        std::string &value,
        const std::string &default_value ) {
        if (private_nh.getParam(private_name, value)) {
            return true;
        }
        for (const auto &name : global_names) {
            if (global_nh.getParam(name, value)) {
                return true;
            }
        }
        value = default_value;
        return false;
    }

    // 解析坐标系，优先级：depth_camera_info > depth_image > bbox > 默认值
    std::string resolveFrameId (
        const detector::BoundingBoxes &bbox_msg,
        const sensor_msgs::Image &depth_msg,
        const sensor_msgs::CameraInfo &depth_camera_info_msg ) {
        if (!depth_camera_info_msg.header.frame_id.empty()) {
            return depth_camera_info_msg.header.frame_id;
        }
        if (!depth_msg.header.frame_id.empty()) {
            return depth_msg.header.frame_id;
        }
        if (!bbox_msg.header.frame_id.empty()) {
            return bbox_msg.header.frame_id;
        }
        return "kinect2_hd_optical_frame";
    }

    // 解析时间戳，优先级：depth_camera_info > depth_image > bbox > 当前时间
    ros::Time resolveTimestamp (
        const detector::BoundingBoxes &bbox_msg,
        const sensor_msgs::Image &depth_msg,
        const sensor_msgs::CameraInfo &depth_camera_info_msg ) {
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

    // 生成检测对象的唯一 key，仅使用时间戳、类别 ID 和像素坐标等信息
    std::string makeKey(const ros::Time &stamp, int32_t class_id, int u, int v) {
        std::ostringstream key;
        key << "stamp_" << stamp.toNSec()
            << "_class_" << class_id
            << "_center_" << u << "_" << v;
        return key.str();
    }
    
}  // namespace

class ObjectDetectionNode {
public:
    ObjectDetectionNode()
        : private_nh_("~")
        , bbox_sub_(global_nh_, "", 1)
        , depth_sub_(global_nh_, "", 1)
        , depth_camera_info_sub_(global_nh_, "", 1)
        , sync_(SyncPolicy(10), bbox_sub_, depth_sub_, depth_camera_info_sub_)
        , tf_listener_(ros::Duration(10.0)) {
        // 获取字符串参数
        getStringParamWithFallback(
            private_nh_, global_nh_, "bbox_topic", {"bbox_topic"}, bbox_topic_, "/detector/bounding_boxes");
        getStringParamWithFallback(
            private_nh_, global_nh_, "depth_topic", {"depth_topic"}, depth_topic_, "/kinect2/hd/image_depth_rect");
        getStringParamWithFallback(
            private_nh_, global_nh_, "depth_camera_info_topic", {"depth_camera_info_topic"}, depth_camera_info_topic_, "/kinect2/hd/depth_camera_info");
        getStringParamWithFallback(
            private_nh_, global_nh_, "detection_object_topic", {"detection_object_topic"}, detection_object_topic_, "/object_detection/detection_objects");
        getStringParamWithFallback(
            private_nh_, global_nh_, "pose_frame", {"pose_frame"}, pose_frame_, "world");

        // 获取参数
        getParamWithFallback(private_nh_, global_nh_, "depth_window_size", "depth_window_size", depth_window_size_, 5);
        getParamWithFallback(private_nh_, global_nh_, "sync_queue_size", "sync_queue_size", sync_queue_size_, 10);
        getParamWithFallback(private_nh_, global_nh_, "sync_slop", "sync_slop", sync_slop_, 0.08);
        getParamWithFallback(private_nh_, global_nh_, "min_depth", "min_depth", min_depth_, 0.1);
        getParamWithFallback(private_nh_, global_nh_, "max_depth", "max_depth", max_depth_, 6.0);

        // 确保深度采样窗口大小为正奇数
        if (depth_window_size_ < 1) {
            depth_window_size_ = 1;
        }
        if ((depth_window_size_ % 2) == 0) {
            ++depth_window_size_;
        }

        // 初始化发布者和订阅者
        detection_object_pub_ = global_nh_.advertise<object_detection::DetectionObjects>(detection_object_topic_, 1);

        bbox_sub_.subscribe(global_nh_, bbox_topic_, 10);
        depth_sub_.subscribe(global_nh_, depth_topic_, 10);
        depth_camera_info_sub_.subscribe(global_nh_, depth_camera_info_topic_, 10);

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
        const sensor_msgs::CameraInfoConstPtr &depth_camera_info_msg ) {

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

        const double fx = depth_camera_info_msg->K[0];
        const double fy = depth_camera_info_msg->K[4];
        const double cx = depth_camera_info_msg->K[2];
        const double cy = depth_camera_info_msg->K[5];

        if (fx == 0.0 || fy == 0.0) {
            ROS_ERROR_THROTTLE(5.0, "Invalid camera intrinsics: fx/fy cannot be zero");
            return;
        }

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
            if (!mapBoundingBoxToDepth(
                    bounding_box,
                    scale_x,
                    scale_y,
                    depth_image.cols,
                    depth_image.rows,
                    x_min,
                    y_min,
                    x_max,
                    y_max)) {
                continue;
            }

            const int u = static_cast<int>(std::lround((x_min + x_max) / 2.0));
            const int v = static_cast<int>(std::lround((y_min + y_max) / 2.0));

            double depth_val = 0.0;
            if (!sampleDepth(depth_image, u, v, depth_val)) {
                ROS_DEBUG(
                    "Skip bbox class=%d because depth is unavailable at pixel (%d, %d)",
                    static_cast<int>(bounding_box.class_id),
                    u,
                    v);
                continue;
            }

            const double x = (static_cast<double>(u) - cx) * depth_val / fx;
            const double y = (static_cast<double>(v) - cy) * depth_val / fy;

            object_detection::DetectionObject detection_object;
            detection_object.class_id = static_cast<int32_t>(bounding_box.class_id);

            // bbox 对应区域的点云生成
            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            for (int yy = y_min; yy <= y_max; ++yy) {
                if (yy < 0 || yy >= depth_image.rows) {
                    continue;
                }
                const float *row_ptr = depth_image.ptr<float>(yy);
                for (int xx = x_min; xx <= x_max; ++xx) {
                    if (xx < 0 || xx >= depth_image.cols) {
                        continue;
                    }
                    const float d = row_ptr[xx];
                    if (!std::isfinite(d) || d < min_depth_ || d > max_depth_) {
                        continue;
                    }

                    const double px = (static_cast<double>(xx) - cx) * d / fx;
                    const double py = (static_cast<double>(yy) - cy) * d / fy;

                    pcl::PointXYZ point;
                    point.x = static_cast<float>(px);
                    point.y = static_cast<float>(py);
                    point.z = static_cast<float>(d);
                    point_cloud.push_back(point);
                }
            }
            if (!point_cloud.empty()) {
                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(point_cloud, cloud_msg);

                // 保持点云消息与外层 detection_object 消息的时间戳和坐标系一致
                cloud_msg.header.stamp = detection_objects_msg.header.stamp;
                cloud_msg.header.frame_id = detection_objects_msg.header.frame_id;

                // 填充 pose（世界坐标），使用 TF 将相机 / 深度坐标系下的点变换到目标 pose_frame_
                geometry_msgs::PointStamped pt_cam;
                geometry_msgs::PointStamped pt_world;

                // 保持坐标消息与外层 detection_object 消息的时间戳和坐标系一致，方便 TF 查找变换关系
                pt_cam.header.stamp = detection_objects_msg.header.stamp;
                pt_cam.header.frame_id = detection_objects_msg.header.frame_id;

                pt_cam.point.x = x;
                pt_cam.point.y = y;
                pt_cam.point.z = depth_val;
                try {
                    tf_listener_.transformPoint(pose_frame_, pt_cam, pt_world);
                    detection_object.pose.position.x = pt_world.point.x;
                    detection_object.pose.position.y = pt_world.point.y;
                    detection_object.pose.position.z = pt_world.point.z;
                    detection_object.pose.orientation.x = 0.0;
                    detection_object.pose.orientation.y = 0.0;
                    detection_object.pose.orientation.z = 0.0;
                    detection_object.pose.orientation.w = 1.0;
                } catch (const tf::TransformException &ex) {
                    ROS_WARN_THROTTLE(5.0, "TF transform to %s failed: %s", pose_frame_.c_str(), ex.what());
                    detection_object.pose.position.x = x;
                    detection_object.pose.position.y = y;
                    detection_object.pose.position.z = depth_val;
                    detection_object.pose.orientation.x = 0.0;
                    detection_object.pose.orientation.y = 0.0;
                    detection_object.pose.orientation.z = 0.0;
                    detection_object.pose.orientation.w = 1.0;
                }

                detection_object.cloud = cloud_msg;
                detection_objects_msg.objects.push_back(detection_object);
            }
        }
        detection_object_pub_.publish(detection_objects_msg);
    }

    // 将 bbox 从源图坐标映射到深度图坐标
    bool mapBoundingBoxToDepth(
        const detector::BoundingBox &bounding_box,
        double scale_x,
        double scale_y,
        int depth_width,
        int depth_height,
        int &x_min,
        int &y_min,
        int &x_max,
        int &y_max ) const {
        if (depth_width <= 0 || depth_height <= 0) {
            return false;
        }

        const int src_x_min = static_cast<int>(std::lround(std::min(bounding_box.x_min, bounding_box.x_max)));
        const int src_x_max = static_cast<int>(std::lround(std::max(bounding_box.x_min, bounding_box.x_max)));
        const int src_y_min = static_cast<int>(std::lround(std::min(bounding_box.y_min, bounding_box.y_max)));
        const int src_y_max = static_cast<int>(std::lround(std::max(bounding_box.y_min, bounding_box.y_max)));

        if (src_x_max < src_x_min || src_y_max < src_y_min) {
            return false;
        }

        x_min = static_cast<int>(std::lround(src_x_min * scale_x));
        x_max = static_cast<int>(std::lround(src_x_max * scale_x));
        y_min = static_cast<int>(std::lround(src_y_min * scale_y));
        y_max = static_cast<int>(std::lround(src_y_max * scale_y));

        x_min = std::max(0, std::min(x_min, depth_width - 1));
        x_max = std::max(0, std::min(x_max, depth_width - 1));
        y_min = std::max(0, std::min(y_min, depth_height - 1));
        y_max = std::max(0, std::min(y_max, depth_height - 1));

        if (x_max < x_min || y_max < y_min) {
            return false;
        }

        return true;
    }

    // 深度图像归一化处理，转换为以米为单位的浮点数，并将无效值设置为 NaN
    cv::Mat normalizeDepthImage(const cv::Mat &depth_image, const std::string &encoding) const {
        cv::Mat normalized;
        if (encoding == "16UC1" || encoding == "mono16" || depth_image.type() == CV_16UC1) {
            depth_image.convertTo(normalized, CV_32FC1, 0.001);
        } else if (depth_image.type() == CV_32FC1) {
            normalized = depth_image.clone();
        } else {
            depth_image.convertTo(normalized, CV_32FC1);
        }

        for (int row = 0; row < normalized.rows; ++row) {
            float *row_ptr = normalized.ptr<float>(row);
            for (int col = 0; col < normalized.cols; ++col) {
                if (!std::isfinite(row_ptr[col]) || row_ptr[col] <= 0.0f) {
                    row_ptr[col] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }

        return normalized;
    }

    // 从深度图像中采样，获取像素 (u, v) 附近窗口内的有效深度值，并返回中位数作为该点的深度
    bool sampleDepth(const cv::Mat &depth_image, int u, int v, double &out_depth) const {
        if (depth_image.empty() || depth_image.cols <= 0 || depth_image.rows <= 0) {
            return false;
        }

        const int half_window = depth_window_size_ / 2;
        const int x0 = std::max(0, u - half_window);
        const int x1 = std::min(depth_image.cols, u + half_window + 1);
        const int y0 = std::max(0, v - half_window);
        const int y1 = std::min(depth_image.rows, v + half_window + 1);

        if (x0 >= x1 || y0 >= y1) {
            return false;
        }

        std::vector<float> valid;
        valid.reserve(static_cast<std::size_t>((x1 - x0) * (y1 - y0)));

        for (int row = y0; row < y1; ++row) {
            const float *row_ptr = depth_image.ptr<float>(row);
            for (int col = x0; col < x1; ++col) {
                const float depth = row_ptr[col];
                if (std::isfinite(depth) && depth >= min_depth_ && depth <= max_depth_) {
                    valid.push_back(depth);
                }
            }
        }

        if (valid.empty()) {
            return false;
        }

        const std::size_t middle = valid.size() / 2;
        std::nth_element(valid.begin(), valid.begin() + middle, valid.end());
        if ((valid.size() % 2) == 0) {
            const auto max_left = *std::max_element(valid.begin(), valid.begin() + middle);
            out_depth = (static_cast<double>(max_left) + static_cast<double>(valid[middle])) / 2.0;
        } else {
            out_depth = static_cast<double>(valid[middle]);
        }

        return true;
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
