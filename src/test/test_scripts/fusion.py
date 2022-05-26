import os
import open3d as o3d
import numpy as np
import copy


cwd = os.getcwd()
# load hand-calibrated transformation matrices
transform_base_to_cam_hand_calibrated = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_base_to_cam_fine_tuned.npy'))
transform_base_to_reference_grasp = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_base_to_reference_grasp.npy'))
transform_cam_to_base_hand_calibrated = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_cam_to_base_fine_tuned.npy'))
# create a robot frame and transform into cam frame
robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
# Robot frame, transformed into gripper frame
grip_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
grip_frame.transform(transform_base_to_reference_grasp)

source = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'sprayer_ref_grasp', 'pcd_reference_merged.ply'))
# source.transform(transform_cam_to_base_hand_calibrated)
# source.paint_uniform_color([1, 0, 0])

to_merge = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'sprayer', 'pcd_reference_1_crop_registered.ply'))
# merged_2.transform(transform_cam_to_base_hand_calibrated)
# to_merge.paint_uniform_color([0, 1, 1])

o3d.visualization.draw_geometries([robot_frame, grip_frame, source, to_merge])

merged = source + to_merge
o3d.io.write_point_cloud(os.path.join(cwd, '..', 'objects', 'sprayer_ref_grasp', 'part_reference_merged.ply'), merged)

# pcd = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'reference_grasp', 'part_reference_merged.ply'))
# o3d.visualization.draw_geometries([robot_frame, grip_frame, pcd])
