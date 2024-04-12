#!/usr/bin/env python2

import os
import yaml
import rospy
import rosnode
from geometry_msgs.msg import PoseStamped
import numpy as np
import open3d as o3d
from copy import deepcopy as dcp


class KukaStateReader(object):
    def __init__(self):
        rospy.init_node('reader_node', anonymous=True)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
        self.current_pose_msg = PoseStamped()
        self.current_xyz = np.array([0.0, 0.0, 0.0])
        self.current_wxyz = np.array([1.0, 0.0, 0.0, 0.0])

    def current_pose_callback(self, data):
        self.current_pose_msg = data
        self.current_xyz = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z
        ])
        self.current_wxyz = np.array([
            data.pose.orientation.w,
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z
        ])


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


script_path = os.path.dirname(os.path.realpath(__file__))

workspace_bounding_box_array = np.load(
    os.path.join(script_path, '..', 'src', 'workspace_bounding_box_array_in_base.npy'))
workspace_bounding_box_array = o3d.utility.Vector3dVector(workspace_bounding_box_array.astype('float'))
workspace_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points=workspace_bounding_box_array)
workspace_bounding_box.color = (0, 1, 0)

if __name__ == '__main__':

    while True:
        ans = raw_input(
            "This program will walk you through a fine-tuning process of the extrinsics found by easy_handeye calibration.\n"
            "You must complete the following 2 steps before starting the fine-tuning process:\n"
            "- 1. Finish extrinsic calibration by running \'rosrun easy_handeye_calibration calibration_cmd_prompt.py.\'\n"
            "- 2. Capture a point cloud of a reference object by running \'rosrun easy_handeye_calibration sample_pcd.py.\n"
            "- 3. move the robot arm to a grasping pose w.r.t. the object pcd."
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

    calibration_result_file = os.path.join(script_path, '..', 'results', 'Extrinsics_0.yaml')
    if not os.path.exists(calibration_result_file):
        print("Calibration result file not found")
        raise OSError("File not found: {}\n"
                      "Please make sure you have finished extrinsic calibration and copy the file to .../easy_handeye_calibration/results/\n"
                      "The calibration_cmd_prompt.py node will copy the file and rename it to \'Extrinsics.yaml\' for you."
                      "see: https://github.com/IanYangChina/Zivid_project/wiki/Camera-calibration-via-iiwa_stack-and-easy_handeye".format(
            calibration_result_file))
    calibration_result_file_ = os.path.join(script_path, '..', 'results', 'Extrinsics_1.yaml')
    if os.path.exists(calibration_result_file_):
        print("There seems to exist multiple extrinsic parameter files:\n"
              "{}\n{}\n...".format(calibration_result_file, calibration_result_file_))
        while True:
            ans = raw_input("Input the id of the one you would like to use: [int] ")
            calibration_result_file = os.path.join(script_path, '..', 'results', 'Extrinsics_' + ans + '.yaml')
            if not os.path.exists(calibration_result_file):
                print("Calibration result file not found, please input the correct id as a single integer")
            else:
                break

    with open(calibration_result_file) as extrinsics_file:
        extrinsics = yaml.load(extrinsics_file)

    raw_transform_base_to_cam = construct_homogeneous_transform_matrix(
        translation=[extrinsics['transformation']['x'],
                     extrinsics['transformation']['y'],
                     extrinsics['transformation']['z']],
        orientation=[extrinsics['transformation']['qw'],
                     extrinsics['transformation']['qx'],
                     extrinsics['transformation']['qy'],
                     extrinsics['transformation']['qz']]
    )
    # inverse
    raw_transform_cam_to_base = np.linalg.inv(raw_transform_base_to_cam)

    pcd_reference_file = os.path.join(script_path, '..', 'src', 'pcd_reference.ply')
    if not os.path.exists(pcd_reference_file):
        print("Reference point cloud file not found")
        raise OSError("File not found: {}\n"
                      "Please make sure you have capture a reference point cloud and copy the file to .../easy_handeye_calibration/src/\n"
                      "The sample_pcd.py node will do that for you."
                      "see: https://github.com/IanYangChina/Zivid_project/wiki/Camera-calibration-via-iiwa_stack-and-easy_handeye".format(
            calibration_result_file))
    original_pcd_reference_in_cam_frame = o3d.io.read_point_cloud(pcd_reference_file)

    # The x, y, z axis will be rendered as red, green, and blue arrows respectively.
    # Robot frame, origin of the world
    robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)

    # Camera frame, transformed into Robot frame using raw extrinsics
    raw_cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    cam_frame_in_base = raw_cam_frame.transform(raw_transform_cam_to_base)

    ROSSMartServo_on = False
    while not ROSSMartServo_on:
        rospy.loginfo('Please make sure you have started the ROSSMartServo application on the Sunrise Cabinet')
        ans = raw_input(
            '[USER INPUT] Type [y] and press [enter] if you have started the ROSSMartServo, otherwise exit the program: ')
        if ans == 'y':
            if '/iiwa/iiwa_subscriber' in rosnode.get_node_names():
                ROSSMartServo_on = True
            else:
                rospy.loginfo('IIWA topics not detected, check network connection if you have started the SmartServo')
        else:
            rospy.loginfo('Exiting the program...')
            exit()

    kuka_state_reader = KukaStateReader()
    while True:
        print("The current kuka Cartesian state is xyz: {}, wxyz{}".format(kuka_state_reader.current_xyz,
                                                                           kuka_state_reader.current_wxyz))
        ans = raw_input(
            'Check the Kuka control pad. Is the current grasping pose roughly match the real robot state? [y/n]'
        )
        if ans == 'y':
            break
        else:
            print("Read the pose again.")

    # homogeneous transformation matrix from Robot frame to gripper frame
    transform_base_to_reference_grasp = construct_homogeneous_transform_matrix(kuka_state_reader.current_xyz,
                                                                               kuka_state_reader.current_wxyz)
    np.save(os.path.join(script_path, '..', 'results', 'transform_base_to_reference_grasp'),
            transform_base_to_reference_grasp)
    print("Result file \'transform_base_to_reference_grasp.npy\' has been saved in ../results/")
    # Gripper pose in robot frame
    grip_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    grip_frame = grip_frame.transform(transform_base_to_reference_grasp)

    pcd_reference_in_base_frame = dcp(original_pcd_reference_in_cam_frame)
    pcd_reference_in_base_frame = pcd_reference_in_base_frame.transform(raw_transform_cam_to_base)
    print("You can view the visualization of the raw extrinsics applied to the reference point cloud.\n"
          "Feel free to use your mouse the rotate and zoom in/out the pcd.\n"
          "If the easy_handeye calibration result is decent, you can expect the robot base and grasp pose frames to"
          "be at the roughly correctly positions, if not exactly so.")
    while True:
        ans = raw_input("View the raw calibration result? [y/n]: ")
        if ans == 'n':
            break
        elif ans == 'y':
            o3d.visualization.draw_geometries(
                [robot_frame, grip_frame, cam_frame_in_base, workspace_bounding_box, pcd_reference_in_base_frame])
            break
        else:
            print("Invalid input: ", ans)

    while True:
        ans = raw_input("Would you like to start the fine-tuning process? [y/n]: ")
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
            if not os.path.exists(os.path.join(script_path, '..', 'results', 'transform_cam_to_base_fine_tuned.npy')):
                print("Please save the matrix as ../results/transform_cam_to_base_fine_tuned.npy and restart.")
                exit()
            loaded_transform_cam_to_base = np.load(
                os.path.join(script_path, '..', 'results', 'transform_cam_to_base_fine_tuned.npy'))

            pcd_reference_in_base_frame = dcp(original_pcd_reference_in_cam_frame)
            pcd_reference_in_base_frame.transform(loaded_transform_cam_to_base)
            # Camera frame, transformed into Robot frame using loaded extrinsics
            raw_cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
            cam_frame_in_base = raw_cam_frame.transform(loaded_transform_cam_to_base)

            raw_transform_cam_to_base = loaded_transform_cam_to_base.copy()
            print("You can now view the visualization of the raw extrinsics applied to the reference point cloud.")
            ans = raw_input("View the loaded transformation result? [y/n]: ")
            if ans == 'y':
                o3d.visualization.draw_geometries(
                    [robot_frame, grip_frame, cam_frame_in_base, workspace_bounding_box, pcd_reference_in_base_frame])
            break
        elif ans == 'n':
            break
        else:
            print("Invalid input: ", ans)

    done_fine_tuning = False
    while not done_fine_tuning:
        print(
            "Based on the previous visualization, input a translation and an orientation to tune the pose of the point cloud within the world frame:")
        x = float(raw_input("X (in meters): "))
        y = float(raw_input("Y (in meters): "))
        z = float(raw_input("Z (in meters): "))
        a = float(raw_input("A (rotate about X, in degree): "))
        b = float(raw_input("B (rotate about Y, in degree): "))
        c = float(raw_input("C (rotate about Z, in degree): "))
        translation_extra += np.array([x, y, z])
        orientation_extra += np.array([c, b, a])
        transform_extra_within_base = construct_homogeneous_transform_matrix(translation=translation_extra,
                                                                             orientation=orientation_extra)
        fine_tuned_pcd_reference_in_base_frame = dcp(pcd_reference_in_base_frame)
        fine_tuned_pcd_reference_in_base_frame.transform(transform_extra_within_base)

        transform_cam_to_base_fine_tuned = np.matmul(transform_extra_within_base, raw_transform_cam_to_base)
        # Camera frame, transformed into Robot frame using loaded extrinsics
        raw_cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        cam_frame_in_base = raw_cam_frame.transform(raw_transform_cam_to_base)

        print("You will now be given a visualization of the raw extrinsics applied to the reference point cloud.")
        done = False
        while not done:
            ans = raw_input("View the fine-tuned result? [y/n]: ")
            if ans == 'n':
                done = True
            elif ans == 'y':
                o3d.visualization.draw_geometries(
                    [robot_frame, grip_frame, cam_frame_in_base, workspace_bounding_box,
                     fine_tuned_pcd_reference_in_base_frame])
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
                    ans = raw_input(
                        "Reset [r] the point cloud, or Carry [c] on based on the current transformation, or Exit [e]?\n"
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
                cropped = fine_tuned_pcd_reference_in_base_frame.crop(workspace_bounding_box)
                done_crop = False
                while not done_crop:
                    ans = raw_input("Crop and view the point cloud using the green bounding box? [y/n]: ")
                    if ans == 'y':
                        o3d.visualization.draw_geometries(
                            [robot_frame, grip_frame, cam_frame_in_base, workspace_bounding_box, cropped])
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

    transform_cam_to_base_fine_tuned = np.matmul(transform_extra_within_base, raw_transform_cam_to_base)
    np.save(os.path.join(script_path, '..', 'results', 'transform_cam_to_base_fine_tuned'),
            transform_cam_to_base_fine_tuned)
    print("Result file \'transform_cam_to_base_fine_tuned.npy\' has been saved in ../results/")
    transform_base_to_cam_fine_tuned = np.linalg.inv(transform_cam_to_base_fine_tuned)
    np.save(os.path.join(script_path, '..', 'results', 'transform_base_to_cam_fine_tuned'),
            transform_base_to_cam_fine_tuned)
    print("Result file \'transform_base_to_cam_fine_tuned.npy\' has been saved in ../results/")

    o3d.io.write_point_cloud(os.path.join(script_path, '..', 'results', 'pcd_reference_in_world_frame.ply'),
                             fine_tuned_pcd_reference_in_base_frame)
    print("Result file \'pcd_reference_in_world_frame.ply\' has been saved in ../results/")
    o3d.io.write_point_cloud(os.path.join(script_path, '..', 'results', 'cropped_pcd_reference_in_world_frame.ply'),
                             cropped)
    print("Result file \'cropped_pcd_reference_in_world_frame.ply\' has been saved in ../results/")

    close_smart_servo = False
    while not close_smart_servo:
        if not ('/iiwa/iiwa_subscriber' in rosnode.get_node_names()):
            close_smart_servo = True
        else:
            print('[INFO]Please **now** shutdown the SmartServo application on Sunrise Cabinet')
            rospy.sleep(2)
    print('[INFO] Exiting the program...')
    exit()
