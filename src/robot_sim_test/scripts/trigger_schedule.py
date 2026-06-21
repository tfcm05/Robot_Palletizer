#!/usr/bin/env python3
import rospy
from robot_core.msg import Info

if __name__ == '__main__':
    rospy.init_node('trigger_schedule')
    delay = rospy.get_param('~delay', 8.0)
    rospy.loginfo('Will trigger TASK_SCHEDULE in %.1f seconds', delay)
    rospy.sleep(delay)
    pub = rospy.Publisher('/ctrl/info', Info, queue_size=1, latch=True)
    rospy.sleep(0.5)
    msg = Info()
    msg.mode = Info.TASK_SCHEDULE
    pub.publish(msg)
    rospy.loginfo('TASK_SCHEDULE triggered')
