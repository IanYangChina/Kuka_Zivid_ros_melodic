#!/usr/bin/env python3

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
    captured = input("[USER INPUT] Have you captured point clouds? [y/n]")
    if captured == 'n':
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        ROSSMartServo_on = False
        while not ROSSMartServo_on:
            rospy.loginfo('Please make sure you have started the ROSSMartServo application on the Sunrise Cabinet')
            ans = input(
                '[USER INPUT] Have you started the ROSSMartServo? [y/n] + Enter')
            if ans == 'y':
                if '/iiwa/iiwa_subscriber' in rosnode.get_node_names():
                    ROSSMartServo_on = True
                else:
                    rospy.loginfo('IIWA topics not detected, check network connection if you have started the SmartServo')
            else:
                rospy.loginfo('Exiting the program...')
                exit()

        print('[INFO] Starting kuka controller...')
        launch_camera = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(script_path, '..', 'launch', 'camera.launch')])
        launch_camera.start()
        rospy.sleep(3)

        launch_controller = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(script_path, '..', 'launch', 'sample_two_pcd.launch')])
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
        [[-0.98015414, -0.0509059, -0.19158928, 0.08890082],
         [0.04942863, -0.99869962, 0.01248517, 0.12680363],
         [-0.19197571, 0.0027674, 0.98139578, 0.0579286],
         [0., 0., 0., 1.]]
    )
    # while True:
    #     start_from_scratch = input("[USER INPUT] Are you starting from zero? [y/n]")
    #     if start_from_scratch == 'n':
    #         raw_transform_ee_to_cam = np.load(
    #             os.path.join(script_path, '..', 'results_eye_on_hand', 'Extrinsic_zivid.npy'))
    #         break
    #     elif start_from_scratch == 'y':
    #         raw_transform_ee_to_cam = construct_homogeneous_transform_matrix(
    #             translation=np.array([0., 0., 0.]),
    #             orientation=np.array([0., 0., 0.])
    #         )
    #         break
    #     else:
    #         print("Invalid input: ", start_from_scratch)
    # inverse
    raw_transform_cam_to_ee = np.linalg.inv(raw_transform_ee_to_cam)

    # homogeneous transformation matrix from Robot frame to gripper frame
    # capture_pose_1 = PoseStamped()
    # capture_pose_1.pose.position.x = -0.35817
    # capture_pose_1.pose.position.y = 0.19131
    # capture_pose_1.pose.position.z = 0.55848
    # capture_pose_1.pose.orientation.w = 0.13814
    # capture_pose_1.pose.orientation.x = -0.48401
    # capture_pose_1.pose.orientation.y = 0.78845
    # capture_pose_1.pose.orientation.z = -0.35354
    transform_base_to_ee_1 = construct_homogeneous_transform_matrix(
        translation=np.array([-0.35817, 0.19131, 0.55848]),
        orientation=np.array([0.13814, -0.48401, 0.78845, -0.35354])
    )
    transform_ee_1_to_base = np.linalg.inv(transform_base_to_ee_1.copy())

    # capture_pose_2 = PoseStamped()
    # capture_pose_2.pose.position.x = -0.34841
    # capture_pose_2.pose.position.y = -0.16340
    # capture_pose_2.pose.position.z = 0.52647
    # capture_pose_2.pose.orientation.w = -0.29588
    # capture_pose_2.pose.orientation.x = 0.78232
    # capture_pose_2.pose.orientation.y = -0.49780
    # capture_pose_2.pose.orientation.z = 0.22937
    transform_base_to_ee_2 = construct_homogeneous_transform_matrix(
        translation=np.array([-0.34841, -0.16340, 0.52647]),
        orientation=np.array([-0.29588, 0.78232, -0.49780, 0.22937])
    )
    transform_ee_2_to_base = np.linalg.inv(transform_base_to_ee_2.copy())

    # EE frames in robot frame
    ee_1_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    ee_1_frame = ee_1_frame.transform(transform_base_to_ee_1.copy())
    ee_2_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    ee_2_frame = ee_2_frame.transform(transform_base_to_ee_2.copy())

    # The x, y, z axis will be rendered as red, green, and blue arrows respectively.
    # Robot frame, origin of the world
    robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    # pcd
    pcd_reference_file_1 = os.path.join(script_path, '..', 'results_eye_on_hand', 'pcd_reference_0.ply')
    original_pcd_1_in_cam_frame = o3d.io.read_point_cloud(pcd_reference_file_1)
    pcd_reference_file_2 = os.path.join(script_path, '..', 'results_eye_on_hand', 'pcd_reference_1.ply')
    original_pcd_2_in_cam_frame = o3d.io.read_point_cloud(pcd_reference_file_2)
    # pcd to world
    raw_transform_cam_to_base_1 = np.matmul(raw_transform_cam_to_ee.copy(), transform_ee_1_to_base.copy())
    raw_transform_base_to_cam_1 = np.matmul(transform_base_to_ee_1.copy(), raw_transform_ee_to_cam)
    pcd_1_in_base_frame = dcp(original_pcd_1_in_cam_frame).transform(
        raw_transform_base_to_cam_1.copy()).paint_uniform_color([0.6, 0.6, 0.6])

    raw_transform_cam_to_base_2 = np.matmul(raw_transform_cam_to_ee.copy(), transform_ee_2_to_base.copy())
    raw_transform_base_to_cam_2 = np.matmul(transform_base_to_ee_2.copy(), raw_transform_ee_to_cam.copy())
    pcd_2_in_base_frame = dcp(original_pcd_2_in_cam_frame).transform(
        raw_transform_base_to_cam_2.copy()).paint_uniform_color([0.6, 0.6, 0.1])

    # cam to world
    raw_cam_frame_1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    cam_frame_in_base_1 = raw_cam_frame_1.transform(raw_transform_base_to_cam_1.copy())
    raw_cam_frame_2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    cam_frame_in_base_2 = raw_cam_frame_2.transform(raw_transform_base_to_cam_2.copy())
    print("You can view the visualization of the raw extrinsics applied to the reference point cloud.\n"
          "Feel free to use your mouse the rotate and zoom in/out the pcd.\n"
          "If the easy_handeye calibration result is decent, you can expect the robot base and grasp pose frames to"
          "be at the roughly correctly positions, if not exactly so.")
    while True:
        ans = input("View the raw calibration result? [y/n]: ")
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
                pcd_2_in_base_frame
            ])
            break
        else:
            print("Invalid input: ", ans)
