#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped


class DummyArucoSubscriber:
    def __init__(self):
        rospy.init_node("dummy_aruco_subscriber")
        rospy.Subscriber("/aruco_tracker/pose", PoseStamped, self.on_tf)
        rospy.spin()

    def on_tf(self, msg):
        pass


if __name__ == "__main__":
    das = DummyArucoSubscriber()
    rospy.spin()
