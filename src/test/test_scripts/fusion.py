import os
import open3d as o3d
import numpy as np
import copy


cwd = os.getcwd()
# load hand-calibrated transformation matrices
transform_base_to_cam_hand_calibrated = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_base_to_cam_hand_calibrated.npy'))
transform_base_to_reference_grasp = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_base_to_reference_grasp.npy'))
transform_cam_to_base_hand_calibrated = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_cam_to_base_hand_calibrated.npy'))
# create a robot frame and transform into cam frame
robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
# Robot frame, transformed into gripper frame
grip_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
grip_frame.transform(transform_base_to_reference_grasp)

merged_1 = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'reference_grasp', 'part_reference_merged.ply'))
merged_1.transform(transform_cam_to_base_hand_calibrated)
merged_1.paint_uniform_color([1, 0, 0])

merged_2 = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'reference_grasp', 'part_reference_merged_2.ply'))
merged_2.transform(transform_cam_to_base_hand_calibrated)
merged_2.paint_uniform_color([0, 1, 1])

o3d.visualization.draw_geometries([robot_frame, grip_frame, merged_1, merged_2])