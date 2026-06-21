import math
import os
import sys
from typing import List

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import rospy

from sensor_msgs.msg import Image
from detector.msg import BoundingBox, BoundingBoxes

scripts_dir = os.path.dirname(os.path.abspath(__file__))
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from detectors.base_detector import BaseDetector, Detection
from detectors.yolo_detector import YoloDetector


class DetectorNode:
    """
    ROS 检测节点，负责订阅图像话题、运行模型推理、时序聚合 2D 检测结果并发布。
    主要流程：图像回调 -> 模型检测 -> 跨帧聚合 -> 定时发布稳定快照。
    """

    def __init__(self):
        # ================================================================
        #  参数读取
        # ================================================================

        # ---- 模型参数 ----
        model = rospy.get_param("~model", "yolo11n.pt")

        self.model_config = {
            "model": model,
            "conf_threshold": rospy.get_param("~confidence_threshold", 0.25),
            "iou_threshold": rospy.get_param("~iou_threshold", 0.45),
            "device": rospy.get_param("~device", "cpu"),
            "classes": rospy.get_param("~classes", [0, 1, 2, 3]),
            "use_track": rospy.get_param("~use_track", False),
            "tracker_type": rospy.get_param("~tracker_type", "bytetrack"),
            "track_persist": rospy.get_param("~track_persist", True),
        }

        # ---- 话题参数 ----
        self._image_topic = rospy.get_param("~image_topic", "/kinect2/qhd/image_color_rect")
        self._bounding_box_topic = rospy.get_param("~bounding_box_topic", "/detector/bounding_boxes")
        
        self._show_window = rospy.get_param("~show_window", True)

        # ---- 聚合参数 ----
        self._agg_config = {
            "bbox_alpha": float(rospy.get_param("~bbox_alpha", 0.4)),
            "max_missed_frames": int(rospy.get_param("~max_missed_frames", 5)),
            "match_distance": float(rospy.get_param("~match_distance", 50.0)),
            "publish_rate": float(rospy.get_param("~publish_rate", 10.0)),
        }

        # ================================================================
        #  引擎初始化
        # ================================================================

        self._engine: BaseDetector = self._create_engine(model)
        self._load_model()

        # ================================================================
        #  内部状态
        # ================================================================

        self._bridge = CvBridge()
        self._aggregated = {}
        self._last_header = None

        # ================================================================
        #  IO 建立
        # ================================================================

        self._image_sub = rospy.Subscriber(
            self._image_topic, Image, self._image_callback, queue_size=5,
        )
        self._bounding_box_pub = rospy.Publisher(
            self._bounding_box_topic, BoundingBoxes, queue_size=1,
        )
        self._global_timer = rospy.Timer(
            rospy.Duration(1.0 / self._agg_config["publish_rate"]),
            self._publish_timer,
        )

    # ================================================================
    #  引擎管理
    # ================================================================

    def _create_engine(self, model: str) -> BaseDetector:
        """根据模型文件名前缀选择并创建对应的检测引擎实例。"""
        if model.startswith("yolo"):
            return YoloDetector(self.model_config)
        raise ValueError(f"Unsupported model: {model}")

    def _load_model(self) -> None:
        """加载模型权重到引擎中。"""
        self._engine.load_model()

    def _detect(self, image: np.ndarray) -> List[Detection]:
        """对单帧图像执行检测，返回 Detection 列表。"""
        return self._engine.detect(image)

    # ================================================================
    #  可视化：在图像上绘制检测框与标签
    # ================================================================

    def _draw(self, image: np.ndarray, detections: List[Detection]) -> np.ndarray:
        """
        在图像上绘制所有检测结果：绿色矩形框 + 标签文字。
        标签格式: id={track_id} cls={class_id} {confidence}
        """
        annotated_image = image.copy()
        for detection in detections:
            x_min, y_min, x_max, y_max = detection.bbox
            cv2.rectangle(annotated_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            label = (
                f"id={detection.track_id} "
                f"cls={detection.class_id} "
                f"{detection.confidence:.2f}"
            )
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

    # ================================================================
    #  图像回调：推理 + 聚合 + 可视化
    # ================================================================

    def _image_callback(self, image_msg: Image) -> None:
        """
        图像订阅回调：
        1. 记录消息头
        2. ROS 图像转 OpenCV 格式
        3. 执行模型检测
        4. 更新时序聚合
        5. 绘制标注图并保存/显示
        """
        self._last_header = image_msg.header

        try:
            cv_image = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logerr("Failed to convert image message: %s", error)
            return

        detections = self._detect(cv_image)
        self._update_aggregation(detections)

        annotated_image = self._draw(cv_image, detections)
        cv2.imwrite("/tmp/detector_annotated_debug.jpg", annotated_image)

        if self._show_window and annotated_image is not None:
            cv2.imshow("detector", annotated_image)
            cv2.waitKey(1)

    # ================================================================
    #  时序聚合：将逐帧检测融为稳定的全局物体快照
    # ================================================================

    def _update_aggregation(self, detections: List[Detection]) -> None:
        """
        用当前帧的检测结果更新聚合物体池：
        - 有 track_id 的物体直接更新
        - 无 track_id（-1）的物体按类 + 中心距离近邻匹配
        - 未在当前帧出现的已有物体增加 missed_frames，超阈值后标记丢失
        """
        seen_ids = set()
        for det in detections:
            tid = det.track_id
            if tid != -1:
                # 有跟踪 ID，直接更新
                seen_ids.add(tid)
                self._update_tracked(tid, det)
            else:
                # 无跟踪 ID，尝试近邻匹配
                matched_tid = self._match_untracked(det)
                if matched_tid is not None:
                    seen_ids.add(matched_tid)
                    self._update_tracked(matched_tid, det)

        # 未在当前帧出现的物体，增加漏检计数
        for tid, info in self._aggregated.items():
            if tid not in seen_ids:
                info["missed_frames"] += 1
                if info["missed_frames"] > self._agg_config["max_missed_frames"]:
                    info["is_lost"] = True

    def _update_tracked(self, tid: int, det: Detection) -> None:
        """
        更新（或创建）track_id 对应的物体状态：
        - 新物体：创建记录并初始化
        - 已有物体：EMA 平滑 bbox、更新类别/置信度、重置漏检计数
        """
        info = self._aggregated.get(tid)
        if info is None:
            # 首次出现，创建新记录
            self._aggregated[tid] = {
                "class_id": det.class_id,
                "bbox": list(det.bbox),
                "confidence": det.confidence,
                "age": 1,
                "missed_frames": 0,
                "is_lost": False,
            }
        else:
            # 已有物体，平滑更新
            info["bbox"] = self._ema_bbox(info["bbox"], det.bbox)
            info["class_id"] = det.class_id
            info["confidence"] = det.confidence
            info["age"] += 1
            info["missed_frames"] = 0
            info["is_lost"] = False

    def _ema_bbox(self, old: List[int], new: List[int]) -> List[int]:
        """
        对边界框四坐标分别做指数移动平均（EMA）平滑。
        alpha 越大，新观测值的权重越高，平滑效果越弱。
        """
        a = self._agg_config["bbox_alpha"]
        return [
            int(a * new[0] + (1.0 - a) * old[0]),
            int(a * new[1] + (1.0 - a) * old[1]),
            int(a * new[2] + (1.0 - a) * old[2]),
            int(a * new[3] + (1.0 - a) * old[3]),
        ]

    def _match_untracked(self, det: Detection) -> int:
        """
        将无 track_id（-1）的检测结果与现有聚合物体按近邻匹配。
        匹配条件：同类物体 && 中心点距离小于阈值。
        返回匹配到的 track_id，若无匹配则返回 None。
        """
        best_tid = None
        best_dist = self._agg_config["match_distance"]
        cx, cy = self._bbox_center(det.bbox)
        for tid, info in self._aggregated.items():
            if info["is_lost"] or info["class_id"] != det.class_id:
                continue
            ox, oy = self._bbox_center(info["bbox"])
            d = math.sqrt((cx - ox) ** 2 + (cy - oy) ** 2)
            if d < best_dist:
                best_dist = d
                best_tid = tid
        return best_tid

    @staticmethod
    def _bbox_center(bbox) -> tuple:
        """计算边界框的中心点坐标 (xc, yc)。"""
        return ((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0)

    # ================================================================
    #  定时发布：按固定频率发布聚合后的稳定物体快照
    # ================================================================

    def _publish_timer(self, event) -> None:
        """
        定时器回调：将聚合池中存活的物体打包为 BoundingBoxes 消息，
        剔除标记为丢失的物体后发布。
        """
        if self._last_header is None:
            return

        # 清理已丢失的物体
        lost = [tid for tid, info in self._aggregated.items() if info["is_lost"]]
        for tid in lost:
            del self._aggregated[tid]

        msg = BoundingBoxes()
        msg.header = self._last_header
        for tid, info in self._aggregated.items():
            bbox = BoundingBox()
            bbox.x_min, bbox.y_min, bbox.x_max, bbox.y_max = info["bbox"]
            bbox.class_id = info["class_id"]
            bbox.track_id = tid
            bbox.confidence = info["confidence"]
            msg.bounding_boxes.append(bbox)
        self._bounding_box_pub.publish(msg)


def main():
    """ROS 节点入口：初始化节点，实例化 DetectorNode，进入事件循环。"""
    rospy.init_node("detector_node", anonymous=False)
    detector_node = DetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
