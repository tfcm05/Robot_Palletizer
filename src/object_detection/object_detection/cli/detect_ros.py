import cv2
import numpy as np
import rospy
from typing import Optional, Tuple
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from object_detection.msg import DetectionPoint, DetectionPointArray

from object_detection.config import DetectionConfig
from object_detection.detector import Detector

class ROSDetectionNode:
    def __init__(self):
        rospy.init_node("object_detection_node", anonymous=True)

        # 默认使用 YOLO 模型
        model = rospy.get_param("~model", "yolo")

        # 默认相机话题
        camera_topic = rospy.get_param("~camera_topic", "/kinect2/qhd/image_color_rect")
        depth_topic = rospy.get_param("~depth_topic", "/kinect2/qhd/image_depth_rect")
        camera_info_topic = rospy.get_param("~camera_info_topic", "/kinect2/qhd/camera_info")
        
        # 默认检测结果话题
        detections_topic = rospy.get_param("~detections_topic", "/object_detection/detections")

        # 是否显示检测结果的可视化窗口
        self._display = rospy.get_param("~display", True)
        # 深度值的最大有效范围，单位为米
        self._max_depth = rospy.get_param("~max_depth", 10.0)

        rospy.loginfo(f"Loading model {model}...")
        self._detector = Detector(model=model, config=DetectionConfig())
        self._detector.load_model()
        rospy.loginfo(f"Model loaded")

        self._bridge = CvBridge()
        self._detections_pub = rospy.Publisher(detections_topic, DetectionPointArray, queue_size=5)
        
        # 深度图和相机内参
        self._depth_image: Optional[np.ndarray] = None
        self._fx: Optional[float] = None
        self._fy: Optional[float] = None
        self._cx: Optional[float] = None
        self._cy: Optional[float] = None

        # 订阅 RGB 图像、深度图和相机内参
        self._rgb_sub = rospy.Subscriber(camera_topic, Image, self._callback, queue_size=1)
        self._depth_sub = rospy.Subscriber(depth_topic, Image, self._depth_callback, queue_size=1)
        self._info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self._camera_info_callback, queue_size=1)

    # 获取相机内参
    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._fx = msg.K[0]
        self._fy = msg.K[4]
        self._cx = msg.K[2]
        self._cy = msg.K[5]

    # 获取深度图
    def _depth_callback(self, msg: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(f"Depth bridge error: {e}")
            return

        if depth.dtype == np.uint16:
            self._depth_image = depth.astype(np.float32) / 1000.0
        else:
            self._depth_image = depth.astype(np.float32)

    # 将像素坐标转换为相机坐标系中的 3D 坐标
    def _pixel_to_3d(self, u: int, v: int) -> Optional[Tuple[float, float, float]]:
        if self._depth_image is None:
            return None
        if self._fx is None or self._fy is None or self._cx is None or self._cy is None:
            return None

        fx = self._fx
        fy = self._fy
        cx = self._cx
        cy = self._cy

        # 检查像素坐标是否在图像范围内
        h, w = self._depth_image.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return None

        # 获取深度值
        z = float(self._depth_image[v, u])
        if not np.isfinite(z) or z <= 0.0 or z > float(self._max_depth):
            return None

        # 将像素坐标转换为相机坐标系中的 3D 坐标
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return x, y, z

    # 处理 RGB 图像，进行检测并发布结果
    def _callback(self, msg: Image) -> None:
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Bridge error: {e}")
            return

        detections = self._detector(cv_image)
        annotated = self._detector.draw(cv_image, detections)

        detections_msg = DetectionPointArray()
        detections_msg.header = msg.header
        for det in detections:
            x1, y1, x2, y2 = det.bbox
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            xyz = self._pixel_to_3d(center_x, center_y)
            if xyz is None:
                continue

            x, y, z = xyz
            detections_msg.detections.append(
                DetectionPoint(
                    id=int(det.class_id),
                    x=x,
                    y=y,
                    z=z,
                )
            )

        # 输出检测结果日志
        if len(detections) > 0:
            names = [f"{d.class_name}({d.confidence:.2f})" for d in detections]
            rospy.logdebug(f"Detected: {', '.join(names)}")

        # 发布检测结果
        self._detections_pub.publish(detections_msg)

        # 可视化检测结果
        if self._display and annotated is not None:
            cv2.imshow("Object Detection", annotated)
            cv2.waitKey(1)


def main():
    ROSDetectionNode()
    rospy.spin()

if __name__ == "__main__":
    main()
