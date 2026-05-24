#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import rospy
import tf
from geometry_msgs.msg import Pose, Point, PointStamped
from std_msgs.msg import String
from object_detection.msg import DetectionPointArray, DetectionPoint


class BlockInfo:
    def __init__(self, block_id, class_id, class_name, block_type, position):
        self.block_id = block_id
        self.class_id = class_id
        self.class_name = class_name
        self.block_type = block_type
        self.position = position
        self.status = "pending"

    def to_dict(self):
        return {
            "block_id": self.block_id,
            "class_id": self.class_id,
            "class_name": self.class_name,
            "block_type": self.block_type,
            "position": {
                "x": round(self.position[0], 3),
                "y": round(self.position[1], 3),
                "z": round(self.position[2], 3),
            },
            "status": self.status,
        }


class TaskPlanningNode:
    def __init__(self):
        rospy.init_node("task_planning_node", anonymous=True)

        self._large_ids = rospy.get_param("~block_classes/large", [0, 2])
        self._small_ids = rospy.get_param("~block_classes/small", [1, 3])
        self._strategy = rospy.get_param("~planning_strategy", "small_first")
        self._proximity = rospy.get_param("~proximity_threshold", 0.3)
        self._scan_frames = rospy.get_param("~scan_frames", 5)
        self._camera_frame = rospy.get_param("~camera_frame", "kinect2_rgb_optical_frame")
        self._publish_frame = rospy.get_param("~publish_frame", "base_footprint")

        self._blocks = {}
        self._scan_buffer = []
        self._current_target_id = None
        self._tf_listener = tf.TransformListener()

        self._target_pub = rospy.Publisher("/task_planning/next_target", Pose, queue_size=10)
        self._info_pub = rospy.Publisher("/task_planning/target_info", String, queue_size=10)
        self._list_pub = rospy.Publisher("/task_planning/block_list", String, queue_size=10)

        rospy.Subscriber("/object_detection/detections", DetectionPointArray, self._detection_cb, queue_size=10)
        rospy.Subscriber("/wpb_home/grab_result", String, self._grab_result_cb, queue_size=10)
        rospy.Subscriber("/wpb_home/place_result", String, self._place_result_cb, queue_size=10)

        rospy.loginfo("任务规划节点已启动 | 策略: %s | 扫描帧数: %d", self._strategy, self._scan_frames)

    def _class_to_type(self, class_id):
        if class_id in self._large_ids:
            return "large"
        if class_id in self._small_ids:
            return "small"
        return None

    def _class_to_name(self, class_id):
        mapping = {}
        for cid in self._large_ids:
            mapping[cid] = "large"
        for cid in self._small_ids:
            mapping[cid] = "small"
        name_map = {0: "red_large", 1: "red_small", 2: "green_large", 3: "green_small"}
        return name_map.get(class_id, mapping.get(class_id, str(class_id)))

    def _make_block_id(self, class_id, x, y):
        return "{}_{:.2f}_{:.2f}".format(class_id, x, y)

    def _distance(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2 + (pos1[2] - pos2[2]) ** 2)

    def _transform_point(self, point, source_frame):
        try:
            ps = PointStamped()
            ps.header.frame_id = source_frame
            ps.header.stamp = rospy.Time(0)
            ps.point.x = point[0]
            ps.point.y = point[1]
            ps.point.z = point[2]
            transformed = self._tf_listener.transformPoint(self._publish_frame, ps)
            return (transformed.point.x, transformed.point.y, transformed.point.z)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF 坐标转换失败: %s", e)
            return None

    def _detection_cb(self, msg):
        self._scan_buffer.append(msg)
        if len(self._scan_buffer) >= self._scan_frames:
            self._process_detections()
            self._scan_buffer = []
            self._plan_next()

    def _process_detections(self):
        for msg in self._scan_buffer:
            source_frame = msg.header.frame_id if msg.header.frame_id else self._camera_frame
            for det in msg.detections:
                block_type = self._class_to_type(det.id)
                if block_type is None:
                    continue
                cam_pos = (det.x, det.y, det.z)
                world_pos = self._transform_point(cam_pos, source_frame)
                if world_pos is None:
                    continue
                block_id = self._make_block_id(det.id, world_pos[0], world_pos[1])
                if block_id in self._blocks:
                    existing = self._blocks[block_id]
                    if existing.status == "pending":
                        avg_x = (existing.position[0] + world_pos[0]) / 2
                        avg_y = (existing.position[1] + world_pos[1]) / 2
                        avg_z = (existing.position[2] + world_pos[2]) / 2
                        existing.position = (avg_x, avg_y, avg_z)
                else:
                    matched = False
                    for bid, blk in self._blocks.items():
                        if blk.status != "pending":
                            continue
                        if blk.class_id == det.id and self._distance(blk.position, world_pos) < self._proximity:
                            avg_x = (blk.position[0] + world_pos[0]) / 2
                            avg_y = (blk.position[1] + world_pos[1]) / 2
                            avg_z = (blk.position[2] + world_pos[2]) / 2
                            blk.position = (avg_x, avg_y, avg_z)
                            matched = True
                            break
                    if not matched:
                        self._blocks[block_id] = BlockInfo(
                            block_id=block_id,
                            class_id=det.id,
                            class_name=self._class_to_name(det.id),
                            block_type=block_type,
                            position=world_pos,
                        )
        self._publish_block_list()

    def _plan_next(self):
        if self._current_target_id is not None:
            return
        pending = [b for b in self._blocks.values() if b.status == "pending"]
        if not pending:
            rospy.loginfo("没有待搬运的方块")
            return
        target = self._select_target(pending)
        if target is None:
            return
        target.status = "assigned"
        self._current_target_id = target.block_id
        pose_msg = Pose()
        pose_msg.position.x = target.position[0]
        pose_msg.position.y = target.position[1]
        pose_msg.position.z = target.position[2]
        pose_msg.orientation.w = 1.0
        self._target_pub.publish(pose_msg)
        info = {
            "block_id": target.block_id,
            "class_name": target.class_name,
            "block_type": target.block_type,
            "strategy": self._strategy,
            "position": {
                "x": round(target.position[0], 3),
                "y": round(target.position[1], 3),
                "z": round(target.position[2], 3),
            },
            "pending_count": len(pending) - 1,
            "done_count": sum(1 for b in self._blocks.values() if b.status == "done"),
        }
        self._info_pub.publish(json.dumps(info, ensure_ascii=False))
        rospy.logwarn(
            "规划目标: [%s] %s | 位置 (%.2f, %.2f, %.2f) | 策略: %s",
            target.block_type, target.class_name,
            target.position[0], target.position[1], target.position[2],
            self._strategy,
        )

    def _select_target(self, pending_blocks):
        if self._strategy == "nearest":
            robot_pos = self._get_robot_position()
            if robot_pos is not None:
                pending_blocks.sort(key=lambda b: self._distance(b.position, robot_pos))
            return pending_blocks[0] if pending_blocks else None
        small = [b for b in pending_blocks if b.block_type == "small"]
        large = [b for b in pending_blocks if b.block_type == "large"]
        robot_pos = self._get_robot_position()
        key_func = (lambda b: self._distance(b.position, robot_pos)) if robot_pos is not None else (lambda b: 0)
        small.sort(key=key_func)
        large.sort(key=key_func)
        if self._strategy == "small_first":
            ordered = small + large
        elif self._strategy == "large_first":
            ordered = large + small
        else:
            ordered = pending_blocks
        return ordered[0] if ordered else None

    def _get_robot_position(self):
        return (0.0, 0.0, 0.0)

    def _grab_result_cb(self, msg):
        if msg.data == "done" and self._current_target_id is not None:
            block = self._blocks.get(self._current_target_id)
            if block is not None and block.status == "assigned":
                block.status = "grabbed"
                rospy.loginfo("方块 [%s] 抓取完成，状态 → grabbed", block.block_id)
                self._publish_block_list()

    def _place_result_cb(self, msg):
        if msg.data == "done" and self._current_target_id is not None:
            block = self._blocks.get(self._current_target_id)
            if block is not None and block.status == "grabbed":
                block.status = "done"
                self._current_target_id = None
                rospy.loginfo("方块 [%s] 放置完成，状态 → done", block.block_id)
                self._publish_block_list()
                self._plan_next()

    def _publish_block_list(self):
        data = [b.to_dict() for b in self._blocks.values()]
        self._list_pub.publish(json.dumps(data, ensure_ascii=False))


if __name__ == "__main__":
    TaskPlanningNode()
    rospy.spin()
