#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
np.set_printoptions(precision=3)


def from_ps_to_np(pose_msg):
    euler = Rotation.from_quat([
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w
    ]).as_euler('zxy', degrees=True)

    return np.array([
        pose_msg.pose.position.x,
        pose_msg.pose.position.y,
        pose_msg.pose.position.z,
    ]), euler, np.array([
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w
    ])


class Controller:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback=self.current_pose)
        self.pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=2)

    def current_pose(self, data):
        position, angle, quat = from_ps_to_np(data)
        # print(Rotation.from_euler('zxy', angle, degrees=True).as_quat())
        # print(-quat)
        # quat = Rotation.from_euler('zxy', [-57.3, 1.59, -180.24], degrees=True).as_quat()
        # quat *= -1
        rospy.loginfo('Pose received')
        # data.pose.position.x = -0.5
        # data.pose.position.y = 0.0
        # data.pose.orientation.x = quat[0]
        # data.pose.orientation.y = quat[1]
        # data.pose.orientation.z = quat[2]
        # data.pose.orientation.w = quat[3]
        # self.pub.publish(data)

        # self.publish_pose([0.0, -0.5, -0.5], Rotation.from_euler('zxy', [-57.3, 1.59, -178.24], degrees=True).as_quat(), data)
        # print(data)

    def publish_pose(self, position, quat, data):
        quat = (-quat).tolist()
        pose_msg = PoseStamped()
        pose_msg.header.seq = data.header.seq
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'iiwa_link_0'

        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        rospy.sleep(1)
        self.pub.publish(pose_msg)
        rospy.loginfo('publishing')


if __name__ == '__main__':
    controller = Controller()
    # controller.publish_pose([-0.5, 0.0, -0.5], Rotation.from_euler('zxy', [-57.3, 1.59, -178.24], degrees=True).as_quat())
    rospy.spin()