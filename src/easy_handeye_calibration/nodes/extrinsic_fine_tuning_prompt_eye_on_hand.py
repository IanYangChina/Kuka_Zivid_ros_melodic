#!/usr/bin/env python2

import os
import yaml
# import rospy
# import rosnode
# import roslaunch
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
        rotation = o3d.geometry.get_rotation_matrix_from_xyz(rotation)
    transformation = np.append(rotation, translation, axis=1)
    transformation = np.append(transformation, np.array([[0, 0, 0, 1]]), axis=0)
    return transformation.copy()


script_path = os.path.dirname(os.path.realpath(__file__))

workspace_bounding_box_array = np.load(os.path.join(script_path, '..', 'src', 'workspace_bounding_box_array_in_base.npy'))
workspace_bounding_box_array = o3d.utility.Vector3dVector(workspace_bounding_box_array.astype('float'))
workspace_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points=workspace_bounding_box_array)
workspace_bounding_box.color = (0, 1, 0)

if __name__ == '__main__':
    captured = input("[USER INPUT] Have you captured point clouds? [y/n]")
    # if captured == 'n':
    #     uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #     roslaunch.configure_logging(uuid)
    #
    #     ROSSMartServo_on = False
    #     while not ROSSMartServo_on:
    #         rospy.loginfo('Please make sure you have started the ROSSMartServo application on the Sunrise Cabinet')
    #         ans = input(
    #             '[USER INPUT] Have you started the ROSSMartServo? [y/n] + Enter')
    #         if ans == 'y':
    #             if '/iiwa/iiwa_subscriber' in rosnode.get_node_names():
    #                 ROSSMartServo_on = True
    #             else:
    #                 rospy.loginfo('IIWA topics not detected, check network connection if you have started the SmartServo')
    #         else:
    #             rospy.loginfo('Exiting the program...')
    #             exit()
    #
    #     print('[INFO] Starting kuka controller...')
    #     launch_camera = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(script_path, '..', 'launch', 'camera.launch')])
    #     launch_camera.start()
    #     rospy.sleep(3)
    #
    #     launch_controller = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(script_path, '..', 'launch', 'controller.launch')])
    #     launch_controller.start()
    #
    #     close_smart_servo = False
    #     while not close_smart_servo:
    #         if not ('/iiwa/iiwa_subscriber' in rosnode.get_node_names()):
    #             close_smart_servo = True
    #         else:
    #             print('[INFO]Please **now** shutdown the SmartServo application on Sunrise Cabinet when you finished capturing the point clouds')
    #             rospy.sleep(2)
    #
    #     launch_camera.shutdown()
    #     launch_controller.shutdown()

    while True:
        start_from_scratch = input("[USER INPUT] Are you starting from zero? [y/n]")
        if start_from_scratch == 'n':
            raw_transform_ee_to_cam = np.load(os.path.join(script_path, '..', 'results_eye_on_hand', 'Extrinsic_zivid.npy'))
            break
        elif start_from_scratch == 'y':
            raw_transform_ee_to_cam = construct_homogeneous_transform_matrix(
                translation=np.array([0., 0., 0.]),
                orientation=np.array([0., 0., 0.])
            )
            break
        else:
            print("Invalid input: ", start_from_scratch)
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
    raw_transform_cam_to_base_1 = np.matmul(raw_transform_cam_to_ee, transform_ee_1_to_base.copy())
    raw_transform_base_to_cam_1 = np.matmul(transform_base_to_ee_1.copy(), raw_transform_ee_to_cam)
    pcd_1_in_base_frame = dcp(original_pcd_1_in_cam_frame).transform(raw_transform_cam_to_base_1.copy()).paint_uniform_color([0.6, 0.6, 0.6])

    raw_transform_cam_to_base_2 = np.matmul(raw_transform_cam_to_ee, transform_ee_2_to_base.copy())
    raw_transform_base_to_cam_2 = np.matmul(transform_base_to_ee_2.copy(), raw_transform_ee_to_cam)
    pcd_2_in_base_frame = dcp(original_pcd_2_in_cam_frame).transform(raw_transform_cam_to_base_2.copy()).paint_uniform_color([0.6, 0.6, 0.1])

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

    while True:
        ans = input("Would you like to start the fine-tuning process? [y/n]: ")
        if ans == 'n':
            print("Exiting the program...")
            exit()
        elif ans == 'y':
            break
        else:
            print("Invalid input: ", ans)

    # Get a identity transformation matrix as a starting point for extrinsics fine-tuning
    translation_extra = np.array([0., 0., 0.])  # XYZ
    orientation_extra = np.array([0., 0., 0.])  # CBA

    done_fine_tuning = False
    while not done_fine_tuning:
        print(
            "Based on the previous visualization, input a translation and an orientation to tune the pose of the point cloud within the world frame:")
        x = float(input("X (in meters): "))
        y = float(input("Y (in meters): "))
        z = float(input("Z (in meters): "))
        a = float(input("A (rotate about X, in degree): "))
        b = float(input("B (rotate about Y, in degree): "))
        c = float(input("C (rotate about Z, in degree): "))
        translation_extra += np.array([x, y, z])
        orientation_extra += np.array([a, b, c])
        # fine-tune in the ee frame
        print("new ee-to-cam transform: ", translation_extra, orientation_extra)
        transformation_extra = construct_homogeneous_transform_matrix(translation=translation_extra,
                                                                      orientation=orientation_extra)

        new_transform_cam_to_base_1 = np.matmul(transformation_extra.copy(), transform_ee_1_to_base.copy())
        new_transform_base_to_cam_1 = np.linalg.inv(new_transform_cam_to_base_1.copy())
        fine_tuned_pcd_1_in_base_frame = dcp(original_pcd_1_in_cam_frame).transform(new_transform_cam_to_base_1.copy())

        new_transform_cam_to_base_2 = np.matmul(transformation_extra.copy(), transform_ee_2_to_base.copy())
        new_transform_base_to_cam_2 = np.linalg.inv(new_transform_cam_to_base_2.copy())
        fine_tuned_pcd_2_in_base_frame = dcp(original_pcd_2_in_cam_frame).transform(new_transform_cam_to_base_2.copy())

        # Camera frame, transformed into Robot frame using loaded extrinsics
        # cam to world
        new_cam_frame_1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        new_cam_frame_in_base_1 = raw_cam_frame_1.transform(new_transform_base_to_cam_1.copy())
        new_cam_frame_2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        new_cam_frame_in_base_2 = raw_cam_frame_2.transform(new_transform_base_to_cam_2.copy())

        transform_cam_to_ee_1 = np.matmul(new_transform_cam_to_base_1.copy(), transform_base_to_ee_1)
        transform_cam_to_ee_2 = np.matmul(new_transform_cam_to_base_2.copy(), transform_base_to_ee_2)
        print("Matching residual: ", (transform_cam_to_ee_1 - transform_cam_to_ee_2).mean())

        print("You will now be given a visualization of the raw extrinsics applied to the reference point cloud.")
        done = False
        while not done:
            ans = input("View the fine-tuned result? [y/n]: ")
            if ans == 'n':
                done = True
            elif ans == 'y':
                o3d.visualization.draw_geometries([
                    robot_frame,
                    ee_1_frame,
                    new_cam_frame_in_base_1,
                    fine_tuned_pcd_1_in_base_frame,
                    ee_2_frame,
                    new_cam_frame_in_base_2,
                    fine_tuned_pcd_2_in_base_frame
                ])
                done = True
            else:
                print("Invalid input: ", ans)

        done_one = False
        while not done_one:
            ans = input("Is the result satisfactory? [y/n]: ")
            if ans == 'n':
                done = False
                while not done:
                    ans = input(
                        "Reset [r] the point cloud, or Carry [c] on based on the current transformation, or Exit [e]?\n"
                        "[r/c/e]: ")
                    if ans == 'r':
                        translation_extra = np.array([0.0, 0.0, 0.0])  # XYZ
                        orientation_extra = np.array([0.0, 0.0, 0.0])  # CBA
                        done = True
                    elif ans == 'c':
                        done = True
                    elif ans == 'e':
                        print("Exiting the program...")
                        exit()
                    else:
                        print("Invalid input: ", ans)
                done_one = True
            elif ans == 'y':
                print("Finishing fine-tuning...")
                done_one = True
                done_fine_tuning = True
            else:
                print("Invalid input: ", ans)

    np.save(os.path.join(script_path, '..', 'results_eye_on_hand', 'transform_cam_to_ee_fine_tuned'), new_raw_transform_cam_to_ee)
    print("Result file \'transform_cam_to_ee_fine_tuned.npy\' has been saved in ../results_/")

    exit()
