#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <detector/BoundingBoxes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
        // TF2 缓冲区与监听器
        , tf_buffer_(ros::Duration(private_nh_.param("tf_cache_time", 10.0)))  ///< TF2 变换缓冲区，缓存时间 10 秒
        , tf_listener_(tf_buffer_)  ///< TF2 变换监听器
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
        , max_depth_(private_nh_.param("max_depth", 5.0))  ///< 有效最大深度（米）
        , depth_tolerance_(private_nh_.param("depth_tolerance", 0.08))  ///< 深度一致性容差（米）
        , center_sample_ratio_(private_nh_.param("center_sample_ratio", 0.2))  ///< 中心采样窗口比例
        // 尺寸推断参数
        , size_confidence_threshold_(private_nh_.param("size_confidence_threshold", 0.6))  ///< 尺寸推断的置信度阈值，低于此值不信任 class_id 改用点云 extent 推断
        // 输出参数
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

            // 将边界框裁剪到深度图范围内，防止 ROI 越界导致崩溃
            int x_min = std::max(0, std::min(static_cast<int>(bounding_box.x_min), depth_image.cols - 1));
            int y_min = std::max(0, std::min(static_cast<int>(bounding_box.y_min), depth_image.rows - 1));
            int x_max = std::max(0, std::min(static_cast<int>(bounding_box.x_max), depth_image.cols - 1));
            int y_max = std::max(0, std::min(static_cast<int>(bounding_box.y_max), depth_image.rows - 1));
            if (x_max <= x_min || y_max <= y_min) {
                continue;
            }

            object_detection::DetectionObject detection_object;
            detection_object.class_id = static_cast<int32_t>(bounding_box.class_id);
            detection_object.track_id = static_cast<int32_t>(bounding_box.track_id);

            // bbox 对应区域的点云生成
            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            const cv::Rect roi_rect(x_min, y_min, x_max - x_min + 1, y_max - y_min + 1);
            const cv::Mat depth_roi = depth_image(roi_rect);

            // 创建范围掩码，筛选出深度值在 min_depth_ 和 max_depth_ 范围内的像素
            cv::Mat range_mask;
            cv::inRange(depth_roi, static_cast<float>(min_depth_), static_cast<float>(max_depth_), range_mask);

            // 创建深度一致性掩码：基于 ROI 中心区域的中值深度，过滤与目标深度不一致的点
            const int cw_x_min = static_cast<int>(roi_rect.width  * (1.0 - center_sample_ratio_) / 2.0);
            const int cw_x_max = static_cast<int>(roi_rect.width  * (1.0 + center_sample_ratio_) / 2.0);
            const int cw_y_min = static_cast<int>(roi_rect.height * (1.0 - center_sample_ratio_) / 2.0);
            const int cw_y_max = static_cast<int>(roi_rect.height * (1.0 + center_sample_ratio_) / 2.0);
            double center_depth;
            sampleDepth(depth_roi, cw_x_min, cw_x_max, cw_y_min, cw_y_max, center_depth);
            if (std::isnan(center_depth)) {
                continue;
            }

            const float lo = static_cast<float>(center_depth - depth_tolerance_);
            const float hi = static_cast<float>(center_depth + depth_tolerance_);
            cv::Mat foreground_mask;
            cv::inRange(depth_roi, lo, hi, foreground_mask);

            // 将范围掩码与深度一致性掩码进行按位与运算，得到最终的有效像素掩码
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
                // PCA 分析：
                Eigen::Matrix3f pca_eigenvectors;
                bool pca_valid = false;
                {
                    pcl::PCA<pcl::PointXYZ> pca;
                    pca.setInputCloud(point_cloud.makeShared());
                    // getEigenVectors() 返回 3×3 矩阵，三列按特征值从大到小排列，
                    pca_eigenvectors = pca.getEigenVectors();
                    pca_valid = true;
                }

                // 低置信度时用点云 extent 修正 class_id（纠正尺寸位，保留颜色位）
                int corrected_class_id = static_cast<int>(detection_object.class_id);
                if (pca_valid && bounding_box.confidence < static_cast<float>(size_confidence_threshold_)) {
                    corrected_class_id = correctClassId(corrected_class_id, point_cloud, pca_eigenvectors);
                    detection_object.class_id = static_cast<int32_t>(corrected_class_id);
                }

                // 点云质心
                Eigen::Vector4f centroid = computeObjectCentroid(
                    point_cloud,
                    corrected_class_id,
                    pca_eigenvectors,
                    pca_valid);

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
                if (pca_valid) {
                    // 将 PCA 特征向量矩阵转为有效旋转矩阵：
                    // PCA 得到的三个特征向量两两正交，但可能为左手系（行列式 < 0），
                    // 而四元数/旋转矩阵要求右手系，因此翻转第三列使其行列式为正。
                    if (pca_eigenvectors.determinant() < 0) {
                        pca_eigenvectors.col(2) = -pca_eigenvectors.col(2);
                    }
                    // 构造四元数：旋转矩阵的列即为物体三个主轴在相机坐标系下的方向。
                    Eigen::Quaternionf q(pca_eigenvectors);
                    q.normalize();
                    pose_cam.pose.orientation.x = q.x();
                    pose_cam.pose.orientation.y = q.y();
                    pose_cam.pose.orientation.z = q.z();
                    pose_cam.pose.orientation.w = q.w();
                }

                // 尝试将位姿从相机坐标系转换到目标坐标系，如果转换失败则使用相机坐标系下的位姿
                try {
                    tf_buffer_.transform(pose_cam, pose_obj, pose_frame_);
                    detection_object.pose = pose_obj;
                } catch (const tf2::TransformException &ex) {
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
     * @brief 对深度图中指定矩形区域采样，返回有效深度的中值
     * @param depth_image 深度图像（CV_32FC1，NaN 表示无效）
     * @param x_min 采样区域左边界（列索引，含）
     * @param x_max 采样区域右边界（列索引，不含）
     * @param y_min 采样区域上边界（行索引，含）
     * @param y_max 采样区域下边界（行索引，不含）
     * @param out_depth 输出：中值深度（米），无效时返回 NaN
     */
    void sampleDepth(
        const cv::Mat &depth_image,
        int x_min, int x_max, int y_min, int y_max,
        double &out_depth) {

        // 使用 thread_local 局部缓存：每个线程独立持有自己的副本，
        thread_local std::vector<float> depth_samples_buffer;

        // 清空采样缓存，并预分配容量避免循环中频繁扩容
        depth_samples_buffer.clear();
        depth_samples_buffer.reserve(static_cast<std::size_t>((x_max - x_min) * (y_max - y_min)));

        // 遍历采样区域，收集所有有限且在有效深度范围内的像素值
        for (int row = y_min; row < y_max; ++row) {
            const float *row_ptr = depth_image.ptr<float>(row);
            for (int col = x_min; col < x_max; ++col) {
                const float depth = row_ptr[col];
                if (std::isfinite(depth) && depth >= min_depth_ && depth <= max_depth_) {
                    depth_samples_buffer.push_back(depth);
                }
            }
        }

        std::vector<float> &valid = depth_samples_buffer;

        // 无有效采样值时返回 NaN
        if (valid.empty()) {
            out_depth = std::numeric_limits<double>::quiet_NaN();
            return;
        }

        // 用 nth_element 部分排序取中值，避免对整个序列完全排序
        const std::size_t middle = valid.size() / 2;
        std::nth_element(valid.begin(), valid.begin() + middle, valid.end());
        if ((valid.size() % 2) == 0) {
            // 偶数个样本：取中间两个的平均作为中值
            const float upper = valid[middle];
            std::nth_element(valid.begin(), valid.begin() + middle - 1, valid.begin() + middle);
            const float lower = valid[middle - 1];
            out_depth = (static_cast<double>(lower) + static_cast<double>(upper)) / 2.0;
        } else {
            // 奇数个样本：直接取中间元素
            out_depth = static_cast<double>(valid[middle]);
        }
    }

    /**
     * @brief 修正检测器输出的 class_id
     * @param class_id 检测器给出的类别 ID
     * @param cloud 物体点云
     * @param eigenvectors PCA 主方向（3×3 旋转矩阵，各列为特征向量）
     * @return 修正后的 class_id
     *
     * 将点云投影到三个 PCA 主方向上计算可见范围，取两个较大的面内 extent 推断真实尺寸
     * 调用方根据置信度决定是否调用本函数。
     */
    int correctClassId(
        int class_id,
        const pcl::PointCloud<pcl::PointXYZ>& cloud,
        const Eigen::Matrix3f& eigenvectors) {

        // 以点云质心作为投影原点
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(cloud, centroid);
        Eigen::Vector3f mean3(centroid[0], centroid[1], centroid[2]);

        // 计算点云在三个主方向上的可见范围
        std::array<float, 3> extents{};
        for (int comp = 0; comp < 3; ++comp) {
            Eigen::Vector3f axis = eigenvectors.col(comp);
            float min_proj = std::numeric_limits<float>::max();
            float max_proj = -std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < cloud.size(); ++i) {
                Eigen::Vector3f p(cloud[i].x - mean3[0],
                                  cloud[i].y - mean3[1],
                                  cloud[i].z - mean3[2]);
                float proj = p.dot(axis);
                if (proj < min_proj) min_proj = proj;
                if (proj > max_proj) max_proj = proj;
            }
            extents[comp] = max_proj - min_proj;
        }

        // 取两个较大的面内 extent 推断真实尺寸
        std::array<float, 3> sorted_extent = extents;
        std::sort(sorted_extent.begin(), sorted_extent.end());
        const float face_extent = std::max(sorted_extent[1], sorted_extent[2]);
        const bool extent_is_large = (face_extent > 0.12f);
        const bool class_is_large = ((class_id % 2) == 0);

        // extent 与 class_id 一致，无需修正
        if (extent_is_large == class_is_large) {
            return class_id;
        }

        return class_id ^ 1;
    }

    /**
     * @brief 计算物体点云的质心
     * @param cloud 物体点云
     * @param class_id 物体类别 ID（已修正，用于推断边长）
     * @param eigenvectors PCA 主方向（3×3 旋转矩阵，各列为特征向量）
     * @param pca_valid PCA 是否有效
     * @return 质心坐标
     *
     * 使用 PCA 主方向与点云可见范围对质心进行校正，估计立方体几何中心：
     * 1) 由已修正的 class_id 推断边长 L
     * 2) 将点投影到每个主方向上，得到该方向的可见范围 [min_proj, max_proj] 和 extent
     * 3) 若某方向 extent 接近真实边长（> 0.7×L），说明该方向能看到完整尺寸，用中点作为中心
     * 4) 若 extent 明显小于真实边长，说明只看到该方向的一个面，用 L/2 从可见表面
     *    向远离相机的一侧偏移，以估计立方体几何中心
     */
    Eigen::Vector4f computeObjectCentroid(
        const pcl::PointCloud<pcl::PointXYZ>& cloud,
        int class_id,
        const Eigen::Matrix3f& eigenvectors,
        bool pca_valid) {

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(cloud, centroid);

        if (!pca_valid) {
            return centroid;
        }

        // 由已修正的 class_id 推断边长 L
        double L = (class_id == 1 || class_id == 3) ? 0.10 : 0.15;
        float Lf = static_cast<float>(L);

        // 以原始质心作为 PCA 投影的原点
        Eigen::Vector3f mean3(centroid[0], centroid[1], centroid[2]);

        for (int comp = 0; comp < 3; ++comp) {
            // 当前主方向（单位向量）
            Eigen::Vector3f axis = eigenvectors.col(comp);
            float min_proj = std::numeric_limits<float>::max();
            float max_proj = -std::numeric_limits<float>::max();

            // 将所有点投影到当前主方向上，记录最大/最小投影
            for (std::size_t i = 0; i < cloud.size(); ++i) {
                Eigen::Vector3f p(cloud[i].x - mean3[0],
                                  cloud[i].y - mean3[1],
                                  cloud[i].z - mean3[2]);
                float proj = p.dot(axis);
                if (proj < min_proj) min_proj = proj;
                if (proj > max_proj) max_proj = proj;
            }

            const float extent = max_proj - min_proj;
            if (extent > Lf * 0.7f) {
                // 该方向能看到完整尺寸 → 直接用中点作为该维度的中心
                const float mid_proj = (min_proj + max_proj) / 2.0f;
                centroid[0] += mid_proj * axis[0];
                centroid[1] += mid_proj * axis[1];
                centroid[2] += mid_proj * axis[2];
            } else {
                // 该方向只看到单个面，无法从点云得到完整尺寸
                // 因此根据已知边长 L 将质心从可见表面向物体内部偏移 L/2
                // 偏移方向应远离相机：取与相机→物体方向夹角为锐角的方向
                Eigen::Vector3f dir_to_obj(mean3[0], mean3[1], mean3[2]);
                const float norm = dir_to_obj.norm();
                if (norm > 1e-6f) {
                    dir_to_obj /= norm;
                    if (axis.dot(dir_to_obj) < 0.0f) {
                        axis = -axis;
                    }
                } else if (axis[2] < 0.0f) {
                    // 若质心在原点附近（异常），则默认让主轴 z 分量为正，指向远离相机方向
                    axis = -axis;
                }
                centroid[0] += (Lf / 2.0f) * axis[0];
                centroid[1] += (Lf / 2.0f) * axis[1];
                centroid[2] += (Lf / 2.0f) * axis[2];
            }
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

    // TF2 缓冲区与监听器
    tf2_ros::Buffer tf_buffer_;                 ///< TF2 变换缓冲区
    tf2_ros::TransformListener tf_listener_;    ///< TF2 变换监听器

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
    const double depth_tolerance_;  ///< 深度一致性容差（米）
    const double center_sample_ratio_;  ///< 中心采样窗口比例
    
    // 尺寸推断参数
    const double size_confidence_threshold_;  ///< 尺寸推断的置信度阈值，低于此值不信任 class_id

    // 输出参数
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
