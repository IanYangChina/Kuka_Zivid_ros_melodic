#!/usr/bin/env python2

import os
import sys
from copy import deepcopy as dcp
import rospy, rosnode
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from utils.kuka_poses import *
import moveit_commander
import open3d as o3d
import matplotlib.pyplot as plt
from grasping_demo.srv import Reset, ResetResponse
script_path = os.path.dirname(os.path.realpath(__file__))

DISTANCE_THRESHOLD = 0.001


def qmul(q, r):
    terms = np.outer(r, q)
    w = terms[0, 0] - terms[1, 1] - terms[2, 2] - terms[3, 3]
    x = terms[0, 1] + terms[1, 0] - terms[2, 3] + terms[3, 2]
    y = terms[0, 2] + terms[1, 3] + terms[2, 0] - terms[3, 1]
    z = terms[0, 3] - terms[1, 2] + terms[2, 1] + terms[3, 0]
    out = np.array([w, x, y, z])
    return out / np.sqrt(out.dot(out))


def construct_homogeneous_transform_matrix(translation, orientation):
    translation = np.array(translation).reshape((3, 1))  # xyz
    if len(orientation) == 4:
        rotation = o3d.geometry.get_rotation_matrix_from_quaternion(np.array(orientation).reshape((4, 1)))  # wxyz
    else:
        assert len(orientation) == 3, 'orientation should be a quaternion or 3 axis angles'
        rotation = np.radians(np.array(orientation).astype("float")).reshape((3, 1))  # CBA in radians
        rotation = o3d.geometry.get_rotation_matrix_from_zyx(rotation)
    transformation = np.append(rotation, translation, axis=1)
    transformation = np.append(transformation, np.array([[0, 0, 0, 1]]), axis=0)
    return transformation


class Controller:
    def __init__(self, robot_name='iiwa'):
        rospy.init_node('controller_node', anonymous=True)
        self.current_pose_msg = PoseStamped()
        self.current_xyz = np.array([0.0, 0.0, 0.0])
        self.current_header_seq = 0
        rospy.Subscriber('/'+robot_name+'/state/CartesianPose', PoseStamped, callback=self.current_pose_callback, queue_size=1)
        self.pub_move_cmd = rospy.Publisher('/'+robot_name+'/command/CartesianPose', PoseStamped, queue_size=1)
        ROSSMartServo_on = False
        while not ROSSMartServo_on:
            rospy.loginfo('Please start the ROSSMartServo application on the Sunrise Cabinet')
            if '/'+robot_name+'/iiwa_subscriber' in rosnode.get_node_names():
                ROSSMartServo_on = True
            else:
                rospy.sleep(1)
        self.init_robot()
        rospy.loginfo('init success!')
        rospy.sleep(1)

        self.reset_service = rospy.Service('reset', Reset, self.reset)

        self.transform_cam_to_ee_fine_tuned = np.load(os.path.join(script_path, '..', '..', 'easy_handeye_calibration',
                                                          'result_realsense', 'transform_cam_to_ee_fine_tuned.npy'))
        self.transform_ee_to_cam = np.linalg.inv(self.transform_cam_to_ee_fine_tuned)
        self.fixed_aruco_to_base = None
        self.fixed_base_to_ee = None
        self.transform_base_to_cam = None
        self.fixed_aruco_to_fixed_ee = None

        self.current_aruco_result_msg = TransformStamped()
        self.current_aruco_to_base = None

        rospy.Subscriber('/aruco_tracker/transform', TransformStamped, callback=self.aruco_result_callback, queue_size=1)

        self.moveit_commander = moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_robot = moveit_commander.RobotCommander(robot_description="robot_description")
        self.moveit_scene = moveit_commander.PlanningSceneInterface()
        self.moveit_group = moveit_commander.MoveGroupCommander("manipulator", robot_description="robot_description")

        self.delta_position = 0.005  # meter per waypoint
        self.delta_angle = 5  # angle per waypoint

    def init_robot(self):
        rospy.loginfo("Initializing robot...")
        self.publish_pose(waiting_pose_vs)
        rospy.sleep(2)

    def plan_and_show(self, waypoints, show=False, save_tr=False, tr_name='tr'):
        (plan, fraction) = self.moveit_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   self.delta_position,      # eef_step
                                   0.0)         # jump_threshold

        # moveit sometimes uses the same time value for the last two trajectory points, causing failure execution
        if plan.joint_trajectory.points[-2].time_from_start.nsecs == plan.joint_trajectory.points[-1].time_from_start.nsecs:
            plan.joint_trajectory.points[-1].time_from_start.nsecs += 1000

        if show:
            self.plot_eef_v(plan, save_tr, tr_name)
        return plan

    def reset(self, req):
        self.publish_pose(waiting_pose_vs)
        return ResetResponse()

    def current_pose_callback(self, data):
        self.current_pose_msg = data
        self.current_xyz = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z
        ])
        self.current_header_seq = data.header.seq

    def aruco_result_callback(self, data):
        if self.fixed_base_to_ee is None:
            self.fixed_base_to_ee = construct_homogeneous_transform_matrix(
                translation=[
                    self.current_pose_msg.pose.position.x,
                    self.current_pose_msg.pose.position.y,
                    self.current_pose_msg.pose.position.z
                ],
                orientation=[
                    self.current_pose_msg.pose.orientation.w,
                    self.current_pose_msg.pose.orientation.x,
                    self.current_pose_msg.pose.orientation.y,
                    self.current_pose_msg.pose.orientation.z
                ]
            )
        self.current_aruco_result_msg = data
        cam_to_aruco = construct_homogeneous_transform_matrix(
            translation=[
                data.transform.translation.x,
                data.transform.translation.y,
                data.transform.translation.z
            ],
            orientation=[
                data.transform.rotation.w,
                data.transform.rotation.x,
                data.transform.rotation.y,
                data.transform.rotation.z
            ]
        )
        cur_base_to_ee = construct_homogeneous_transform_matrix(
                translation=[
                    self.current_pose_msg.pose.position.x,
                    self.current_pose_msg.pose.position.y,
                    self.current_pose_msg.pose.position.z
                ],
                orientation=[
                    self.current_pose_msg.pose.orientation.w,
                    self.current_pose_msg.pose.orientation.x,
                    self.current_pose_msg.pose.orientation.y,
                    self.current_pose_msg.pose.orientation.z
                ]
            )
        self.transform_base_to_cam = np.matmul(cur_base_to_ee, self.transform_ee_to_cam)

        if self.fixed_aruco_to_base is None:
            base_to_fixed_aruco = np.matmul(self.transform_base_to_cam, cam_to_aruco)
            self.fixed_aruco_to_base = np.linalg.inv(base_to_fixed_aruco)
        else:
            if self.fixed_aruco_to_fixed_ee is None:
                self.fixed_aruco_to_fixed_ee = np.matmul(self.fixed_aruco_to_base, self.fixed_base_to_ee)

            new_base_to_aruco = np.matmul(self.transform_base_to_cam, cam_to_aruco)
            print(new_base_to_aruco[:-1, -1])
            new_base_to_ee = np.matmul(new_base_to_aruco, self.fixed_aruco_to_fixed_ee)
            new_ee_pose_msg = dcp(self.current_pose_msg)
            new_ee_pose_msg.pose.position.x = new_base_to_ee[0, -1]
            new_ee_pose_msg.pose.position.y = new_base_to_ee[1, -1]
            new_ee_pose_msg.pose.position.z = new_base_to_ee[2, -1]
            self.publish_pose(new_ee_pose_msg)
            # (plan, fraction) = self.moveit_group.compute_cartesian_path(
            #                            [self.current_pose_msg.pose, new_ee_pose_msg.pose],   # waypoints to follow
            #                            self.delta_position,      # eef_step
            #                            0.0)         # jump_threshold
            # moveit sometimes uses the same time value for the last two trajectory points, causing failure execution
            # if plan.joint_trajectory.points[-2].time_from_start.nsecs == plan.joint_trajectory.points[-1].time_from_start.nsecs:
            #     plan.joint_trajectory.points[-1].time_from_start.nsecs += 10000

            # self.moveit_group.execute(plan, wait=True)
            rospy.sleep(2)

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


if __name__ == '__main__':
    controller = Controller()
    rospy.spin()
