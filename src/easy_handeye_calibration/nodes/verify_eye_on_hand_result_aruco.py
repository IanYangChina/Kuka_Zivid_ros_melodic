#!/usr/bin/env python3

import os
import time

import rospy
import rosnode
import roslaunch
import numpy as np
import open3d as o3d
from geometry_msgs.msg import PoseStamped


def construct_homogeneous_transform_matrix(translation, orientation):
    translation = np.array(translation).reshape((3, 1))  # xyz
    if len(orientation) == 4:
        rotation = o3d.geometry.get_rotation_matrix_from_quaternion(np.array(orientation).reshape((4, 1)))  # wxyz
    else:
        assert len(orientation) == 3, 'orientation should be a quaternion or 3 axis angles'
        rotation = np.radians(np.array(orientation).astype("float")).reshape((3, 1))  # ABC in radians
        rotation = o3d.geometry.get_rotation_matrix_from_zyx(rotation)
    transformation = np.append(rotation, translation, axis=1)
    transformation = np.append(transformation, np.array([[0, 0, 0, 1]]), axis=0)
    return transformation.copy()


script_path = os.path.dirname(os.path.realpath(__file__))
# Extrinsic calibration result
raw_transform_ee_to_cam = np.load(os.path.join(script_path, '..', 'results_eye_on_hand', 'transform_ee_to_cam.npy'))
raw_transform_cam_to_ee = np.load(os.path.join(script_path, '..', 'results_eye_on_hand', 'transform_cam_to_ee.npy'))


class ArucoPoseLisetener:
    def __init__(self):
        rospy.init_node("aruco_listener_node", anonymous=True)
        rospy.loginfo("Starting aruco_listener_node")
        self.aruco_pose_in_world_frame = None
        self.aruco_pose_subscriber = rospy.Subscriber('/aruco_tracker/pose', PoseStamped, self.aruco_pose_callback)
        self.kuka_pose_in_world_frame = None
        self.kuka_pose_subscription = rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, self.kuka_pose_callback)

    def kuka_pose_callback(self, msg):
        self.kuka_pose_in_world_frame = construct_homogeneous_transform_matrix(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]
        )

    def aruco_pose_callback(self, msg):
        rospy.loginfo('Aruco pose received, convert to world frame...')
        aruco_pose_in_cam_frame = construct_homogeneous_transform_matrix(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]
        )
        aruco_pose_in_ee_frame = np.dot(raw_transform_cam_to_ee, aruco_pose_in_cam_frame)
        self.aruco_pose_in_world_frame = np.dot(self.kuka_pose_in_world_frame, aruco_pose_in_ee_frame)
        rospy.loginfo("Current marker pose in world frame: \n{}".format(aruco_pose_listener.aruco_pose_in_world_frame))


if __name__ == '__main__':
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    now = time.time()
    ROSSMartServo_on = False
    while not ROSSMartServo_on:
        rospy.loginfo('Please start the ROSSMartServo application on the Sunrise Cabinet')
        rospy.loginfo('Program terminate in {} seconds if ROSSMartServo is not started'.format(30 - (time.time() - now)))
        rospy.sleep(1)
        if '/iiwa/iiwa_subscriber' in rosnode.get_node_names():
            ROSSMartServo_on = True
        else:
            if time.time() - now > 30:
                rospy.logerr('Please start the ROSSMartServo application on the Sunrise Cabinet first')
                exit()

    rospy.loginfo('Starting camera and aruco tracker...')
    launch_camera = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(script_path, '..', 'launch', 'tracking.launch')])
    launch_camera.start()
    rospy.sleep(5)

    rospy.loginfo('Starting grasping controller...')
    launch_controller = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(script_path, '..', '..', 'grasping_demo', 'launch', 'grasping_controller.launch')])
    launch_controller.start()
    rospy.sleep(3)

    rospy.loginfo('Run the keyboard controller from grasping_demo package to move the robot around')

    aruco_pose_listener = ArucoPoseLisetener()

    close_smart_servo = False
    while not close_smart_servo:
        if not ('/iiwa/iiwa_subscriber' in rosnode.get_node_names()):
            close_smart_servo = True
        else:
            rospy.loginfo('Please shutdown the SmartServo application on Sunrise Cabinet when you finished verification.')
            rospy.sleep(1)

    launch_camera.shutdown()
    launch_controller.shutdown()
