#!/usr/bin/env python3
import rospy
from task_planning.msg import GraspCommand, GraspResult


class MockGraspExecutor:
    def __init__(self):
        rospy.Subscriber('/task_planning/grasp_command', GraspCommand, self.cb)
        self.pub = rospy.Publisher('/motion_control/grasp_result', GraspResult, queue_size=10)
        self.duration = rospy.get_param('~grasp_duration', 2.0)

    def cb(self, msg):
        rospy.loginfo('GraspCommand received: class_id=%d, track_id=%d, index=%d',
                      msg.target_class_id, msg.target_track_id, msg.grasp_command_index)
        rospy.sleep(self.duration)
        res = GraspResult()
        res.result = GraspResult.SUCCESS
        res.grasp_command_index = msg.grasp_command_index
        self.pub.publish(res)
        rospy.loginfo('GraspResult SUCCESS published for index %d', msg.grasp_command_index)


if __name__ == '__main__':
    rospy.init_node('mock_grasp_executor')
    MockGraspExecutor()
    rospy.spin()
