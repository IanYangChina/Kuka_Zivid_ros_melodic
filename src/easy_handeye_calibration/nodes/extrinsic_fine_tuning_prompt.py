#!/usr/bin/env python2

import os
import yaml
import numpy as np
import open3d as o3d
import quaternion as quat
from copy import deepcopy as dcp


def construct_homogeneous_transform_matrix(translation, orientation):
    translation = np.array(translation).reshape((3, 1))   # xyz
    if len(orientation) == 4:
        rotation = o3d.geometry.get_rotation_matrix_from_quaternion(np.array(orientation).reshape((4, 1)))  # wxyz
    else:
        assert len(orientation) == 3, 'orientation should be a quaternion or 3 axis angles'
        rotation = np.radians(np.array(orientation).astype("float")).reshape((3, 1))  # CBA in radians
        rotation = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation)
    transformation = np.append(rotation, translation, axis=1)
    transformation = np.append(transformation, np.array([[0, 0, 0, 1]]), axis=0)
    return transformation


script_path = os.path.dirname(os.path.realpath(__file__))

workspace_bounding_box_array = np.load(os.path.join(script_path, '..', 'src', 'workspace_bounding_box_array_in_base.npy'))
workspace_bounding_box_array = o3d.utility.Vector3dVector(workspace_bounding_box_array.astype('float'))
workspace_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points=workspace_bounding_box_array)
workspace_bounding_box.color = (0, 1, 0)

if __name__ == '__main__':

    while True:
        ans = raw_input("This program will walk you through a fine-tuning process of the extrinsics found by easy_handeye calibration.\n"
                        "You must complete the following 2 steps before starting the fine-tuning process:\n"
                        "- 1. Finish extrinsic calibration by running \'rosrun easy_handeye_valibration calibration_cmd_prompt.py.\'\n"
                        "- 2. Capture a point cloud of a reference object and obtain a ground truth grasping pose in the robot base frame.\n"
                        "For detailed instruction of doing the above mentioned 2 steps, "
                        "see: https://github.com/IanYangChina/Zivid_project/wiki/Camera-calibration-via-iiwa_stack-and-easy_handeye\n"
                        "Have you done these 2 steps? [y/n]: ")
        if ans == 'n':
            print("Exiting the program...")
            exit()
        elif ans == 'y':
            break
        else:
            print("Invalid input: ", ans)

    calibration_result_file = os.path.join(script_path, '..', 'results', 'Extrinsics.yaml')
    if not os.path.exists(calibration_result_file):
        print("Calibration result file not found")
        raise OSError("File not found: {}\n"
                      "Please make sure you have finished extrinsic calibration and copy the file to .../easy_handeye_calibration/results/\n"
                      "The calibration_cmd_prompt.py node will copy the file and rename it to \'Extrinsics.yaml\' for you."
                      "see: https://github.com/IanYangChina/Zivid_project/wiki/Camera-calibration-via-iiwa_stack-and-easy_handeye".format(calibration_result_file))

    with open(calibration_result_file) as extrinsics_file:
        extrinsics = yaml.load(extrinsics_file, Loader=yaml.FullLoader)

    raw_transformation_base_to_cam = construct_homogeneous_transform_matrix(
        translation=[extrinsics['transformation']['x'],
                     extrinsics['transformation']['y'],
                     extrinsics['transformation']['z']],
        orientation=[extrinsics['transformation']['qw'],
                     extrinsics['transformation']['qx'],
                     extrinsics['transformation']['qy'],
                     extrinsics['transformation']['qz']]
    )
    # inverse
    raw_transformation_cam_to_base = np.linalg.inv(raw_transformation_base_to_cam)

    pcd_reference_file = os.path.join(script_path, '..', 'src', 'pcd_reference.ply')
    if not os.path.exists(pcd_reference_file):
        print("Reference point cloud file not found")
        raise OSError("File not found: {}\n"
                      "Please make sure you have capture a reference point cloud and copy the file to .../easy_handeye_calibration/src/\n"
                      "The sample_pcd.py node will do that for you."
                      "see: https://github.com/IanYangChina/Zivid_project/wiki/Camera-calibration-via-iiwa_stack-and-easy_handeye".format(calibration_result_file))
    original_pcd_reference_in_cam_frame = o3d.io.read_point_cloud(pcd_reference_file)

    # The x, y, z axis will be rendered as red, green, and blue arrows respectively.
    # Robot frame, origin of the world
    robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)

    # Camera frame, transformed into Robot frame using raw extrinsics
    raw_cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    raw_cam_frame.transform(raw_transformation_base_to_cam)

    print("Please input the ground truth grasping pose relative to the robot base,\n"
          "     you can obtain it by manually moving the iiwa robot and check the SmartPad info tag:")
    while True:
        ans = raw_input("Do you like to continue? [y/n]: ")
        if ans == 'n':
            print("Exiting the program...")
            exit()
        elif ans == 'y':
            break
        else:
            print("Invalid input: ", ans)
    x = raw_input("X (in meters): ")
    y = raw_input("Y (in meters): ")
    z = raw_input("Z (in meters): ")
    a = raw_input("A (in degree): ")
    b = raw_input("B (in degree): ")
    c = raw_input("C (in degree): ")

    # homogeneous transformation matrix from Robot frame to gripper frame
    # example ref grasping pose: [-0.66257, -0.07707, 0.30143], [-179.00, -1.21, 11.36]
    transformation_base_to_ref_grasp = construct_homogeneous_transform_matrix([x, y, z], [c, b, a])
    # Gripper pose in robot frame
    grip_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    grip_frame.transform(transformation_base_to_ref_grasp)

    print("You will now be given a visualization of the raw extrinsics applied to the reference point cloud.\n"
          "Feel free to use your mouse the rotate and zoom in/out the pcd.\n"
          "If the easy_handeye calibration result is decent, you can expect the robot base and grasp pose frames to"
          "be at the roughly correctly positions, if not exactly so.")
    while True:
        ans = raw_input("View the raw calibration result? [y/n]: ")
        if ans == 'n':
            print("Exiting the program...")
            exit()
        elif ans == 'y':
            break
        else:
            print("Invalid input: ", ans)
    pcd_reference_in_base_frame = dcp(original_pcd_reference_in_cam_frame)
    pcd_reference_in_base_frame.transform(raw_transformation_cam_to_base)
    o3d.visualization.draw_geometries([robot_frame, grip_frame, workspace_bounding_box, pcd_reference_in_base_frame])

    while True:
        ans = raw_input("Would like to start the fine-tuning process? [y/n]: ")
        if ans == 'n':
            print("Exiting the program...")
            exit()
        elif ans == 'y':
            break
        else:
            print("Invalid input: ", ans)

    # Get a identity transformation matrix as a starting point for extrinsics fine-tuning
    translation_extra = np.array([0.0, 0.0, 0.0])  # XYZ
    orientation_extra = np.array([0.01, 0.01, 0.01])  # CBA

    while True:
        ans = raw_input("Would you like to load a saved transformation matrix and start from it? [y/n]: ")
        if ans == 'y':
            if not os.path.exists(os.path.join(script_path, '..', 'results', 'transformation_cam_to_base_fine_tuned.npy')):
                print("Please save the matrix as ../results/transformation_cam_to_base_fine_tuned.npy and restart.")
                exit()
            loaded_transformation_cam_to_base = np.load(os.path.join(script_path, '..', 'results', 'transformation_cam_to_base_fine_tuned.npy'))

            pcd_reference_in_base_frame = dcp(original_pcd_reference_in_cam_frame)
            pcd_reference_in_base_frame.transform(loaded_transformation_cam_to_base)
            print("You will now be given a visualization of the raw extrinsics applied to the reference point cloud.")
            ans = raw_input("View the loaded transformation result? [y/n]: ")
            if ans != 'y':
                print("Exiting the program...")
                exit()
            o3d.visualization.draw_geometries([robot_frame, grip_frame, workspace_bounding_box, pcd_reference_in_base_frame])
            raw_transformation_cam_to_base = loaded_transformation_cam_to_base.copy()
            break
        elif ans == 'n':
            break
        else:
            print("Invalid input: ", ans)

    done_fine_tuning = False
    while not done_fine_tuning:
        print("Based on the previous visualization, input a translation and an orientation to tune the pose of the point cloud within the world frame:")
        x = float(raw_input("X (in meters): "))
        y = float(raw_input("Y (in meters): "))
        z = float(raw_input("Z (in meters): "))
        a = float(raw_input("A (rotate about X, in degree): "))
        b = float(raw_input("B (rotate about Y, in degree): "))
        c = float(raw_input("C (rotate about Z, in degree): "))
        translation_extra += np.array([x, y, z])
        orientation_extra += np.array([c, b, a])
        transformation_extra_within_base = construct_homogeneous_transform_matrix(translation=translation_extra,
                                                                                  orientation=orientation_extra)
        fine_tuned_pcd_reference_in_base_frame = dcp(pcd_reference_in_base_frame)
        fine_tuned_pcd_reference_in_base_frame.transform(transformation_extra_within_base)
        print("You will now be given a visualization of the raw extrinsics applied to the reference point cloud.")
        done = False
        while not done:
            ans = raw_input("View the fine-tuned result? [y/n]: ")
            if ans == 'n':
                done = True
            elif ans == 'y':
                o3d.visualization.draw_geometries([robot_frame, grip_frame, workspace_bounding_box, fine_tuned_pcd_reference_in_base_frame])
                done = True
            else:
                print("Invalid input: ", ans)

        print("Current translation and orientation: ", translation_extra, orientation_extra)

        done_one = False
        while not done_one:
            ans = raw_input("Is the result satisfactory? [y/n]: ")
            if ans == 'n':
                done = False
                while not done:
                    ans = raw_input("Reset [r] the point cloud, or Carry [c] on based on the current transformation, or Exit [e]?\n"
                                    "[r/c/e]: ")
                    if ans == 'r':
                        translation_extra = np.array([0.0, 0.0, 0.0])  # XYZ
                        orientation_extra = np.array([0.01, 0.01, 0.01])  # CBA
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
                done_crop = False
                while not done_crop:
                    ans = raw_input("Crop and view the point cloud using the green bounding box? [y/n]: ")
                    if ans == 'y':
                        cropped = fine_tuned_pcd_reference_in_base_frame.crop(workspace_bounding_box)
                        o3d.visualization.draw_geometries([robot_frame, grip_frame, workspace_bounding_box, cropped])
                        o3d.io.write_point_cloud(os.path.join(script_path, '..', 'results', 'cropped_pcd_reference_in_world_frame.ply'), cropped)
                        print("Result file \'cropped_pcd_reference_in_world_frame.ply\' has been saved in ../results/")
                        done_crop = True
                    elif ans == 'n':
                        done_crop = True
                    else:
                        print("Invalid input: ", ans)
                print("Finishing fine-tuning...")
                done_one = True
                done_fine_tuning = True
            else:
                print("Invalid input: ", ans)

    transformation_cam_to_base_fine_tuned = np.matmul(transformation_extra_within_base, raw_transformation_cam_to_base)
    np.save(os.path.join(script_path, '..', 'results', 'transformation_cam_to_base_fine_tuned'), transformation_cam_to_base_fine_tuned)
    print("Result file \'transformation_cam_to_base_fine_tuned.npy\' has been saved in ../results/")
    transformation_base_to_cam_fine_tuned = np.linalg.inv(transformation_cam_to_base_fine_tuned)
    np.save(os.path.join(script_path, '..', 'results', 'transformation_base_to_cam_fine_tuned'), transformation_base_to_cam_fine_tuned)
    print("Result file \'transformation_base_to_cam_fine_tuned.npy\' has been saved in ../results/")

    o3d.io.write_point_cloud(os.path.join(script_path, '..', 'results', 'pcd_reference_in_world_frame.ply'), fine_tuned_pcd_reference_in_base_frame)
    print("Result file \'pcd_reference_in_world_frame.ply\' has been saved in ../results/")
