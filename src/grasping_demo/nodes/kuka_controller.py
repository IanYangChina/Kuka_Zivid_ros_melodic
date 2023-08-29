#!/usr/bin/env python3

import rospy
import numpy as np
import open3d as o3d
import quaternion
from copy import deepcopy as dcp
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, PoseArray
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotInput as inputMsg
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput as outputMsg
# from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
# from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg

waiting_pose_calibration = PoseStamped()
waiting_pose_calibration.pose.position.x = -0.52
waiting_pose_calibration.pose.position.y = -0.00
waiting_pose_calibration.pose.position.z = 0.55
waiting_pose_calibration.pose.orientation.w = -0.0594
waiting_pose_calibration.pose.orientation.x = -0.664
waiting_pose_calibration.pose.orientation.y = 0.745
waiting_pose_calibration.pose.orientation.z = 0.0054

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

part_placing_pose = PoseStamped()
part_placing_pose.pose.position.x = -0.59
part_placing_pose.pose.position.y = -0.30
part_placing_pose.pose.position.z = 0.325
part_placing_pose.pose.orientation.w = -0.0123448805919
part_placing_pose.pose.orientation.x = -0.691465195872
part_placing_pose.pose.orientation.y = 0.722219705582
part_placing_pose.pose.orientation.z = 0.0110529923381

sprayer_placing_pose = PoseStamped()
sprayer_placing_pose.pose.position.x = -0.63
sprayer_placing_pose.pose.position.y = 0.2775
sprayer_placing_pose.pose.position.z = 0.29
sprayer_placing_pose.pose.orientation.w = -0.00509371623762
sprayer_placing_pose.pose.orientation.x = 0.704621165094
sprayer_placing_pose.pose.orientation.y = 0.709524989128
sprayer_placing_pose.pose.orientation.z = 0.00757522078764

grasping_pose_msg = PoseStamped()

DISTANCE_THRESHOLD = 0.001

gripper_activation = outputMsg()
gripper_activation.rACT = 1
gripper_activation.rGTO = 1
gripper_activation.rMOD = 1
gripper_activation.rSPA = 250
gripper_activation.rFRA = 150

gripper_close = outputMsg()
gripper_close.rACT = 1
gripper_close.rGTO = 1
gripper_close.rMOD = 1
gripper_close.rPRA = 255
gripper_close.rSPA = 200
gripper_close.rFRA = 150

gripper_open = outputMsg()
gripper_open.rACT = 1
gripper_open.rGTO = 1
gripper_open.rMOD = 1
gripper_open.rPRA = 0
gripper_open.rSPA = 200
gripper_open.rFRA = 150

gripper_reset = outputMsg()
gripper_reset.rACT = 0
gripper_reset.rGTO = 0
gripper_reset.rMOD = 0
gripper_reset.rPRA = 0
gripper_reset.rSPA = 0
gripper_reset.rFRA = 0


class Controller:
    def __init__(self, translation_speed=0.05, rotation_speed=0.1):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
        rospy.Subscriber('TargetGraspPose', PoseStamped, callback=self.target_pose_callback)
        rospy.Subscriber('TargetGraspPoses', PoseArray, callback=self.target_poses_callback)
        rospy.Subscriber('Robotiq3FGripperRobotIutput', inputMsg, callback=self.gripper_msg, queue_size=2)
        rospy.Subscriber('/keyboard', String, callback=self.keyboard_callback)
        self.pub_move_cmd = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=2)
        self.pub_gripper_cmd = rospy.Publisher('Robotiq3FGripperRobotOutput', outputMsg, queue_size=2)
        self.pub_attempt_finished = rospy.Publisher('AttemptFinished', Bool, queue_size=2)
        self.current_pose_msg = PoseStamped()
        self.current_xyz = np.array([0.0, 0.0, 0.0])
        self.current_header_seq = 0
        self.translation_speed = translation_speed
        self.rotation_speed = rotation_speed

        self.init_robot()

    def init_robot(self):
        rospy.loginfo("Initializing robot...")
        self.publish_pose(waiting_pose_calibration)
        self.publish_grip_cmd(gripper_reset)
        self.publish_grip_cmd(gripper_activation)

    def keyboard_callback(self, data):
        key_pressed = data.data

        target_pose = self.current_pose_msg
        if key_pressed == '1':
            target_pose.pose.position.x += self.translation_speed
        elif key_pressed == '2':
            target_pose.pose.position.x -= self.translation_speed
        elif key_pressed == '3':
            target_pose.pose.position.y += self.translation_speed
        elif key_pressed == '4':
            target_pose.pose.position.y -= self.translation_speed
        elif key_pressed == '5':
            target_pose.pose.position.z += self.translation_speed
        elif key_pressed == '6':
            target_pose.pose.position.z -= self.translation_speed
        else:
            quat = np.array([target_pose.pose.orientation.w,
                             target_pose.pose.orientation.x,
                             target_pose.pose.orientation.y,
                             target_pose.pose.orientation.z])
            rotation = o3d.geometry.get_rotation_matrix_from_quaternion(np.array(quat).reshape((4, 1)))
            if key_pressed == '7':
                rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz(np.array([0.0, 0.0, self.rotation_speed]))
            elif key_pressed == '8':
                rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz(np.array([0.0, 0.0, -self.rotation_speed]))
            elif key_pressed == '9':
                rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz(np.array([0.0, self.rotation_speed, 0.0]))
            elif key_pressed == '0':
                rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz(np.array([0.0, -self.rotation_speed, 0.0]))
            elif key_pressed == '-':
                rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz(np.array([self.rotation_speed, 0.0, 0.0]))
            elif key_pressed == '=':
                rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz(np.array([-self.rotation_speed, 0.0, 0.0]))
            else:
                rotation_delta = np.eye(3)
            rotation_new = np.dot(rotation_delta, rotation)
            quat_new = quaternion.as_float_array(quaternion.from_rotation_matrix(rotation_new))  # w, x, y, z
            target_pose.pose.orientation.w = quat_new[0]
            target_pose.pose.orientation.x = quat_new[1]
            target_pose.pose.orientation.y = quat_new[2]
            target_pose.pose.orientation.z = quat_new[3]

        self.publish_pose(target_pose)

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

    def target_poses_callback(self, data):
        attempt_finished = Bool()
        attempt_finished.data = False
        rospy.loginfo('Received two target poses')
        ans = raw_input("[USER INPUT] Execute part grasping? [y/n]")
        if ans == 'y':
            self.publish_pose(pre_grasping_pose)
            pose = dcp(grasping_pose_msg)
            pose.pose = data.poses[0]
            self.publish_pose(pose)

            self.publish_grip_cmd(gripper_close)
            ans = raw_input("[USER INPUT] Would you like to place the object? [y/n]")
            if ans == 'y':
                # lift the object up for 0.1 meters
                rospy.loginfo('Lifting the object...')
                self.publish_pose(pre_grasping_pose)
                # put down the object
                rospy.loginfo('Placing back the object...')
                self.publish_pose(part_placing_pose)
            # release the gripper fingers
            self.publish_grip_cmd(gripper_open)
            rospy.loginfo("Move gripper to waiting pose...")
            self.publish_pose(pre_grasping_pose)
            # self.publish_pose(waiting_pose)

        ans = raw_input("[USER INPUT] Execute sprayer grasping? [y/n]")
        if ans == 'y':
            self.publish_pose(pre_grasping_pose)
            pose = dcp(grasping_pose_msg)
            pose.pose = data.poses[1]
            self.publish_pose(pose)

            self.publish_grip_cmd(gripper_close)
            ans = raw_input("[USER INPUT] Would you like to place the object? [y/n]")
            if ans == 'y':
                # lift the object up for 0.1 meters
                rospy.loginfo('Lifting the object...')
                self.publish_pose(pre_grasping_pose)
                # put down the object
                rospy.loginfo('Placing back the object...')
                self.publish_pose(sprayer_placing_pose)
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
