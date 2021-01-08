#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


class Controller:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
        rospy.Subscriber('/iiwa/pcd_result/TargetGraspPose', PoseStamped, callback=self.target_pose_callback)
        self.pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=2)
        self.current_pose_msg = None

    def current_pose_callback(self, data):
        self.current_pose_msg = data
        rospy.loginfo('Pose received')

    def target_pose_callback(self, data):
        if self.current_pose_msg is None:
            return
        data.header.seq = self.current_pose_msg.header.seq
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = 'iiwa_link_0'
        rospy.sleep(1)
        self.pub.publish(data)
        rospy.loginfo('publishing')


# if __name__ == '__main__':
#     controller = Controller()
#     rospy.spin()
