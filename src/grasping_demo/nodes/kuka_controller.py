#!/usr/bin/env python2

import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg


class Controller:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
        rospy.Subscriber('TargetGraspPose', PoseStamped, callback=self.target_pose_callback)
        self.pub_move_cmd = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)
        self.pub_gripper_cmd = rospy.Publisher('Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, queue_size=1)
        self.pub_target_reached = rospy.Publisher('TargetPoseReached', Bool, queue_size=1)
        self.current_xyz = np.array([0.0, 0.0, 0.0])
        self.current_header_seq = 0

    def current_pose_callback(self, data):
        self.current_xyz = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z
        ])
        self.current_header_seq = data.header.seq
        rospy.loginfo('Pose received')

    def target_pose_callback(self, data):
        # Bool ros msg to inform the prompt program
        finished_moving = Bool()
        finished_moving.data = False
        # record target xyz for distance tracking
        target_xyz = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z
        ])
        rospy.loginfo('Received a target pose, publishing to the kuka robot...')
        # move the robot towards the object
        finished_moving_to_object = False
        while not finished_moving_to_object:
            data.header.seq = self.current_header_seq
            data.header.stamp = rospy.Time.now()
            data.header.frame_id = 'iiwa_link_0'
            rospy.sleep(1)
            self.pub_move_cmd.publish(data)
            d = np.sqrt(np.sum(np.square(self.current_xyz - target_xyz)))
            if d < 0.01:
                # todo: tune the threshold
                finished_moving_to_object = True

        ans = raw_input("[ USER INPUT] Would you like to grasp and lift the object? [y/n]")
        if ans == 'y':
            # close the gripper
            # todo: closing the griper fingers here
            gripper_msg = inputMsg.Robotiq2FGripper_robot_input()

            # lift the object up for 0.1 meters
            rospy.loginfo('Lifting the object...')
            data.pose.position.z += 0.1
            target_xyz[-1] += 0.1
            finished_lifting = False
            while not finished_lifting:
                data.header.seq = self.current_header_seq
                data.header.stamp = rospy.Time.now()
                data.header.frame_id = 'iiwa_link_0'
                rospy.sleep(1)
                self.pub_move_cmd.publish(data)
                d = np.sqrt(np.sum(np.square(self.current_xyz - target_xyz)))
                if d < 0.01:
                    # todo: tune the threshold
                    finished_lifting = True

            # put down the object
            rospy.loginfo('Placing back the object...')
            data.pose.position.z -= 0.1
            target_xyz[-1] -= 0.1
            while not finished_moving.data:
                data.header.seq = self.current_header_seq
                data.header.stamp = rospy.Time.now()
                data.header.frame_id = 'iiwa_link_0'
                rospy.sleep(1)
                self.pub_move_cmd.publish(data)
                d = np.sqrt(np.sum(np.square(self.current_xyz - target_xyz)))
                if d < 0.01:
                    # todo: tune the threshold
                    finished_moving.data = True
                self.pub_target_reached.publish(finished_moving)

            # release the gripper fingers
            # todo: opening the fingers here
            gripper_msg = inputMsg.Robotiq2FGripper_robot_input()
        else:
            finished_moving.data = True
            self.pub_target_reached.publish(finished_moving)


if __name__ == '__main__':
    controller = Controller()
    rospy.spin()
