#!/usr/bin/env python2

import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg


waiting_pose = PoseStamped()
waiting_pose.pose.position.x = -0.0
waiting_pose.pose.position.y = -0.55
waiting_pose.pose.position.z = 0.6
# in euler: [-90, 180, 0]
waiting_pose.pose.orientation.w = 0.0000
waiting_pose.pose.orientation.x = -0.7071067811
waiting_pose.pose.orientation.y = 0.7071067811
waiting_pose.pose.orientation.z = 0.0000

pre_grasping_pose = PoseStamped()
pre_grasping_pose.pose.position.x = -0.55
pre_grasping_pose.pose.position.y = -0.0
pre_grasping_pose.pose.position.z = 0.6
# in euler: [-90, 180, 0]
pre_grasping_pose.pose.orientation.w = 0.0000
pre_grasping_pose.pose.orientation.x = 0.0000
pre_grasping_pose.pose.orientation.y = 1.0000
pre_grasping_pose.pose.orientation.z = 0.0000

DISTANCE_THRESHOLD = 0.001

gripper_activation = outputMsg.Robotiq2FGripper_robot_output()
gripper_activation.rACT = 1
gripper_activation.rGTO = 1
gripper_activation.rSP = 250
gripper_activation.rFR = 150

gripper_close = outputMsg.Robotiq2FGripper_robot_output()
gripper_close.rACT = 1
gripper_close.rGTO = 1
gripper_close.rPR = 255
gripper_close.rSP = 200
gripper_close.rFR = 150

gripper_open = outputMsg.Robotiq2FGripper_robot_output()
gripper_open.rACT = 1
gripper_open.rPR = 0
gripper_open.rGTO = 1
gripper_open.rSP = 200
gripper_open.rFR = 150

gripper_reset = outputMsg.Robotiq2FGripper_robot_output()
gripper_reset.rACT = 0
gripper_reset.rGTO = 0
gripper_reset.rSP = 0
gripper_reset.rFR = 0


class Controller:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
        rospy.Subscriber('TargetGraspPose', PoseStamped, callback=self.target_pose_callback)
        rospy.Subscriber('Robotiq2FGripperRobotIutput', inputMsg.Robotiq2FGripper_robot_input, callback=self.gripper_msg)
        self.pub_move_cmd = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=2)
        self.pub_gripper_cmd = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=2)
        self.pub_attempt_finished = rospy.Publisher('AttemptFinished', Bool, queue_size=2)
        self.current_pose_msg = PoseStamped()
        self.current_xyz = np.array([0.0, 0.0, 0.0])
        self.current_header_seq = 0
        self.init_robot()

    def init_robot(self):
        rospy.loginfo("Initializing robot...")
        self.publish_pose(waiting_pose)
        self.publish_grip_cmd(gripper_reset)
        self.publish_grip_cmd(gripper_activation)

    def gripper_msg(self, data):
        pass

    def current_pose_callback(self, data):
        self.current_pose_msg = data
        self.current_xyz = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z
        ])
        self.current_header_seq = data.header.seq

    def target_pose_callback(self, data):
        # Bool ros msg to inform the prompt program
        attempt_finished = Bool()
        attempt_finished.data = False
        rospy.loginfo('Received a target pose, publishing to the kuka robot...')
        # move the robot towards the object
        self.publish_pose(pre_grasping_pose)
        self.publish_pose(data)
        # close the gripper
        self.publish_grip_cmd(gripper_close)

        ans = raw_input("[USER INPUT] Would you like to grasp and lift the object? [y/n]")
        if ans == 'y':

            # lift the object up for 0.1 meters
            rospy.loginfo('Lifting the object...')
            self.publish_pose(pre_grasping_pose)

            # put down the object
            rospy.loginfo('Placing back the object...')
            self.publish_pose(data)

        # release the gripper fingers
        self.publish_grip_cmd(gripper_open)

        rospy.loginfo("Move gripper to waiting pose...")
        self.publish_pose(pre_grasping_pose)
        self.publish_pose(waiting_pose)
        attempt_finished.data = True
        rospy.sleep(0.5)
        self.pub_attempt_finished.publish(attempt_finished)

    def publish_pose(self, data):
        # record target xyz for distance tracking
        target_xyz = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z
        ])
        data.header.seq = self.current_header_seq
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = 'iiwa_link_0'
        rospy.sleep(0.5)
        self.pub_move_cmd.publish(data)
        done = False
        while not done:
            d = np.sqrt(np.sum(np.square(self.current_xyz - target_xyz)))
            if d < DISTANCE_THRESHOLD:
                rospy.loginfo("Movement finished")
                done = True

    def publish_grip_cmd(self, data):
        self.pub_gripper_cmd.publish(data)
        rospy.sleep(1)


if __name__ == '__main__':
    controller = Controller()
    rospy.spin()
