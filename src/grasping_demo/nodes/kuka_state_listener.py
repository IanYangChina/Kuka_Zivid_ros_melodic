#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped


class StateListener:
    def __init__(self):
        rospy.init_node('listener_node', anonymous=True)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
        self.current_pose_msg = PoseStamped()
        self.current_xyz = np.array([0.0, 0.0, 0.0])
        self.current_header_seq = 0

    def current_pose_callback(self, msg):
        self.current_pose_msg = msg
        self.current_xyz = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.current_header_seq = msg.header.seq
        print("New pose")
        print("capture_pose_.pose.position.x = " + str(msg.pose.position.x))
        print("capture_pose_.pose.position.y = " + str(msg.pose.position.y))
        print("capture_pose_.pose.position.z = " + str(msg.pose.position.z))
        print("capture_pose_.pose.orientation.w = " + str(msg.pose.orientation.w))
        print("capture_pose_.pose.orientation.x = " + str(msg.pose.orientation.x))
        print("capture_pose_.pose.orientation.y = " + str(msg.pose.orientation.y))
        print("capture_pose_.pose.orientation.z = " + str(msg.pose.orientation.z))


if __name__ == "__main__":
    listener_node = StateListener()
    rospy.spin()