#!/usr/bin/env python2

import os
import yaml
import rospy
import rosnode
import roslaunch
import numpy as np
import open3d as o3d
from copy import deepcopy as dcp


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

workspace_bounding_box_array = np.load(
    os.path.join(script_path, '..', 'src', 'workspace_bounding_box_array_in_base.npy'))
workspace_bounding_box_array = o3d.utility.Vector3dVector(workspace_bounding_box_array.astype('float'))
workspace_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points=workspace_bounding_box_array)
workspace_bounding_box.color = (0, 1, 0)

if __name__ == '__main__':
    captured = raw_input("[USER INPUT] Have you captured point clouds? [y/n]")
    if captured == 'n':
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        ROSSMartServo_on = False
        while not ROSSMartServo_on:
            rospy.loginfo('Please make sure you have started the ROSSMartServo application on the Sunrise Cabinet')
            ans = raw_input(
                '[USER INPUT] Have you started the ROSSMartServo? [y/n] + Enter')
            if ans == 'y':
                if '/iiwa/iiwa_subscriber' in rosnode.get_node_names():
                    ROSSMartServo_on = True
                else:
                    rospy.loginfo(
                        'IIWA topics not detected, check network connection if you have started the SmartServo')
            else:
                rospy.loginfo('Exiting the program...')
                exit()

        print('[INFO] Starting kuka controller...')
        launch_camera = roslaunch.parent.ROSLaunchParent(uuid,
                                                         [os.path.join(script_path, '..', 'launch', 'camera.launch')])
        launch_camera.start()
        rospy.sleep(3)

        launch_controller = roslaunch.parent.ROSLaunchParent(uuid, [
            os.path.join(script_path, '..', 'launch', 'sample_two_pcd.launch')])
        launch_controller.start()

        close_smart_servo = False
        while not close_smart_servo:
            if not ('/iiwa/iiwa_subscriber' in rosnode.get_node_names()):
                close_smart_servo = True
            else:
                # print('[INFO]Please **now** shutdown the SmartServo application on Sunrise Cabinet when you finished capturing the point clouds')
                rospy.sleep(1)

        launch_camera.shutdown()
        launch_controller.shutdown()

    # Extrinsic calibration result
    raw_transform_ee_to_cam = np.array(
        [[0.02583816, -0.99958781, 0.012514, 0.14214873],
         [0.98471024, 0.02760688, 0.17199882, -0.07069426],
         [-0.1722734, 0.00787853, 0.98501767, 0.07813663],
         [0., 0., 0., 1.]]
    )

    raw_transform_cam_to_ee = np.linalg.inv(raw_transform_ee_to_cam)

    # homogeneous transformation matrix from Robot frame to gripper frame
    # capture_pose_1 = PoseStamped()
    # capture_pose_1.pose.position.x = -0.47544
    # capture_pose_1.pose.position.y = -0.16066
    # capture_pose_1.pose.position.z = 0.70063
    # capture_pose_1.pose.orientation.w = -0.30597
    # capture_pose_1.pose.orientation.x = -0.34994
    # capture_pose_1.pose.orientation.y = 0.85375
    # capture_pose_1.pose.orientation.z = 0.23455
    transform_base_to_ee_1 = construct_homogeneous_transform_matrix(
        translation=np.array([-0.47544, -0.16066, 0.70063]),
        orientation=np.array([-0.30597, -0.34994, 0.85375, 0.23455])
    )
    transform_ee_1_to_base = np.linalg.inv(transform_base_to_ee_1.copy())

    # capture_pose_2 = PoseStamped()
    # capture_pose_2.pose.position.x = -0.4056036
    # capture_pose_2.pose.position.y = -0.02022997
    # capture_pose_2.pose.position.z = 0.8205197
    # capture_pose_2.pose.orientation.w = -0.38359
    # capture_pose_2.pose.orientation.x = -0.078744
    # capture_pose_2.pose.orientation.y = 0.91777
    # capture_pose_2.pose.orientation.z = -0.06585
    transform_base_to_ee_2 = construct_homogeneous_transform_matrix(
        translation=np.array([-0.4056036, -0.02022997, 0.8205197]),
        orientation=np.array([-0.38359, -0.078744, 0.91777, -0.06585])
    )
    transform_ee_2_to_base = np.linalg.inv(transform_base_to_ee_2.copy())

    # capture_pose_3 = PoseStamped()
    # capture_pose_3.pose.position.x = -0.39813
    # capture_pose_3.pose.position.y = 0.21598803
    # capture_pose_3.pose.position.z = 0.800553
    # capture_pose_3.pose.orientation.w = -0.33227530
    # capture_pose_3.pose.orientation.x = 0.4818123
    # capture_pose_3.pose.orientation.y = 0.692790448
    # capture_pose_3.pose.orientation.z = -0.42129724
    transform_base_to_ee_3 = construct_homogeneous_transform_matrix(
        translation=np.array([-0.39813, 0.21598803, 0.800553]),
        orientation=np.array([-0.33227530, 0.4818123, 0.692790448, -0.42129724])
    )
    transform_ee_3_to_base = np.linalg.inv(transform_base_to_ee_3.copy())

    # EE frames in robot frame
    ee_1_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    ee_1_frame = ee_1_frame.transform(transform_base_to_ee_1.copy())
    ee_2_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    ee_2_frame = ee_2_frame.transform(transform_base_to_ee_2.copy())
    ee_3_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    ee_3_frame = ee_3_frame.transform(transform_base_to_ee_3.copy())

    # The x, y, z axis will be rendered as red, green, and blue arrows respectively.
    # Robot frame, origin of the world
    robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    # pcd
    pcd_reference_file_1 = os.path.join(script_path, '..', 'results_eye_on_hand', 'pcd_reference_0.ply')
    original_pcd_1_in_cam_frame = o3d.io.read_point_cloud(pcd_reference_file_1)
    pcd_reference_file_2 = os.path.join(script_path, '..', 'results_eye_on_hand', 'pcd_reference_1.ply')
    original_pcd_2_in_cam_frame = o3d.io.read_point_cloud(pcd_reference_file_2)
    pcd_reference_file_3 = os.path.join(script_path, '..', 'results_eye_on_hand', 'pcd_reference_2.ply')
    original_pcd_3_in_cam_frame = o3d.io.read_point_cloud(pcd_reference_file_3)
    # pcd to world
    raw_transform_cam_to_base_1 = np.matmul(raw_transform_cam_to_ee.copy(), transform_ee_1_to_base.copy())
    raw_transform_base_to_cam_1 = np.matmul(transform_base_to_ee_1.copy(), raw_transform_ee_to_cam)
    pcd_1_in_base_frame = dcp(original_pcd_1_in_cam_frame).transform(
        raw_transform_base_to_cam_1.copy()).paint_uniform_color([0.1, 0.6, 0.6])

    raw_transform_cam_to_base_2 = np.matmul(raw_transform_cam_to_ee.copy(), transform_ee_2_to_base.copy())
    raw_transform_base_to_cam_2 = np.matmul(transform_base_to_ee_2.copy(), raw_transform_ee_to_cam.copy())
    pcd_2_in_base_frame = dcp(original_pcd_2_in_cam_frame).transform(
        raw_transform_base_to_cam_2.copy()).paint_uniform_color([0.6, 0.1, 0.6])

    raw_transform_cam_to_base_3 = np.matmul(raw_transform_cam_to_ee.copy(), transform_ee_3_to_base.copy())
    raw_transform_base_to_cam_3 = np.matmul(transform_base_to_ee_3.copy(), raw_transform_ee_to_cam.copy())
    pcd_3_in_base_frame = dcp(original_pcd_3_in_cam_frame).transform(
        raw_transform_base_to_cam_3.copy()).paint_uniform_color([0.6, 0.6, 0.1])

    # cam to world
    raw_cam_frame_1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    cam_frame_in_base_1 = raw_cam_frame_1.transform(raw_transform_base_to_cam_1.copy())
    raw_cam_frame_2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    cam_frame_in_base_2 = raw_cam_frame_2.transform(raw_transform_base_to_cam_2.copy())
    raw_cam_frame_3 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    cam_frame_in_base_3 = raw_cam_frame_3.transform(raw_transform_base_to_cam_3.copy())
    print("You can view the visualization of the raw extrinsics applied to the reference point cloud.\n"
          "Feel free to use your mouse the rotate and zoom in/out the pcd.\n"
          "If the easy_handeye calibration result is decent, you can expect the robot base and grasp pose frames to"
          "be at the roughly correctly positions, if not exactly so.")
    while True:
        ans = raw_input("View the raw calibration result? [y/n]: ")
        if ans == 'n':
            break
        elif ans == 'y':
            o3d.visualization.draw_geometries([
                robot_frame,
                ee_1_frame,
                cam_frame_in_base_1,
                pcd_1_in_base_frame,
                ee_2_frame,
                cam_frame_in_base_2,
                pcd_2_in_base_frame,
                ee_3_frame,
                cam_frame_in_base_3,
                pcd_3_in_base_frame
            ])
            break
        else:
            print("Invalid input: ", ans)
