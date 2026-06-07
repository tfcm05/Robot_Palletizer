from typing import List

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from detector.msg import BoundingBox, BoundingBoxes

import os
import sys
scripts_dir = os.path.dirname(os.path.abspath(__file__))
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from detectors.base_detector import BaseDetector, Detection
from detectors.yolo_detector import YoloDetector

class DetectorNode:
    def __init__(self):
        model_name = rospy.get_param("~model_name", "yolo")

        self.model_config = {
            "version": rospy.get_param("~model_version", "11"),
            "scale": rospy.get_param("~model_scale", "n"),
            "weight": rospy.get_param("~model_weight", "pt"),

            "conf_threshold": rospy.get_param("~confidence_threshold", rospy.get_param("~conf_threshold", 0.25)),
            "iou_threshold": rospy.get_param("~iou_threshold", 0.45),
            "device": rospy.get_param("~device", "cpu"),
            "classes": rospy.get_param("~classes", [0, 1, 2, 3]),
        }

        self._image_topic = rospy.get_param("~image_topic", "/kinect2/hd/image_color_rect")
        self._annotated_image_topic = rospy.get_param("~annotated_image_topic", "/detector/annotated_images")
        self._bounding_box_topic = rospy.get_param("~bounding_box_topic", "/detector/bounding_boxes")
        self._show_window = rospy.get_param("~show_window", True)

        self._min_interval = rospy.get_param("~min_interval", 0.033)
        self._last_process_time = rospy.Time(0)

        self._bridge = CvBridge()

        self._engine: BaseDetector = self._create_engine(model_name)
        self._load_model()

        self._image_sub = rospy.Subscriber(
            self._image_topic,
            Image,
            self._image_callback,
            queue_size=10,
        )

        self._annotated_image_pub = rospy.Publisher(
            self._annotated_image_topic, 
            Image,
            queue_size=1
        )
        self._bounding_box_pub = rospy.Publisher(
            self._bounding_box_topic, 
            BoundingBoxes, 
            queue_size=1
        )


    def _create_engine(self, model_name: str) -> BaseDetector:
        if model_name == "yolo":
            return YoloDetector(self.model_config)
        raise ValueError(f"Unknown model: {model_name}")
    
    def _load_model(self) -> None:
        self._engine.load_model()

    def _detect(self, image: np.ndarray) -> List[Detection]:
        return self._engine.detect(image)

    def _publish(self, image_msg: Image, detections: List[Detection]) -> None:
        bounding_boxes = BoundingBoxes()
        bounding_boxes.header = Header(stamp=image_msg.header.stamp, frame_id=image_msg.header.frame_id)

        for detection in detections:
            x_min, y_min, x_max, y_max = detection.bbox
            bounding_box = BoundingBox()
            bounding_box.x_min = x_min
            bounding_box.y_min = y_min
            bounding_box.x_max = x_max
            bounding_box.y_max = y_max
            bounding_box.class_id = detection.class_id
            bounding_boxes.bounding_boxes.append(bounding_box)

        bounding_boxes.width = image_msg.width
        bounding_boxes.height = image_msg.height
        self._bounding_box_pub.publish(bounding_boxes)

    def _draw(self, image: np.ndarray, detections: List[Detection]) -> np.ndarray:
        annotated_image = image.copy()
        for detection in detections:
            x_min, y_min, x_max, y_max = detection.bbox
            cv2.rectangle(annotated_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            label = f"{detection.class_id} {detection.confidence:.2f}"
            label_origin = (x_min, max(0, y_min - 8))
            cv2.putText(
                annotated_image,
                label,
                label_origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        return annotated_image

    def _image_callback(self, image_msg: Image) -> None:
        # 限制处理频率，避免过高的帧率导致系统过载
        now = rospy.Time.now()
        if now - self._last_process_time < self._min_interval:
            return
        self._last_process_time = now

        try:
            cv_image = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logerr("Failed to convert image message: %s", error)
            return

        detections = self._detect(cv_image)
        self._publish(image_msg, detections)

        annotated_image = self._draw(cv_image, detections)
        cv2.imwrite("/tmp/detector_annotated_debug.jpg", annotated_image)

        if self._show_window and annotated_image is not None:
            cv2.imshow("detector", annotated_image)
            cv2.waitKey(1)
    
def main():
    rospy.init_node("detector_node", anonymous=False)
    detector_node = DetectorNode()
    rospy.spin()

if __name__ == "__main__":
    main()