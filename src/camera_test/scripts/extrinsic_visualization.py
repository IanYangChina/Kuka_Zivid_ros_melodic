import os
import numpy as np
import open3d as o3d  # 0.12.0
import json

cwd = os.getcwd()

workspace_bounding_box_array = np.load(os.path.join(cwd, 'transformation_matrices', 'workspace_bounding_box_array_in_base.npy'))
workspace_bounding_box_array = o3d.utility.Vector3dVector(workspace_bounding_box_array.astype('float64'))
workspace_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points=workspace_bounding_box_array)
workspace_bounding_box.color = (0, 1, 0)

"""Create transformation matrices (already saved)"""
# quat_ = np.array([-0.0006727, -0.00906054, -0.9966595, 0.08083814])
# quat_norm = quat_ / np.linalg.norm(quat_)
# -0.000672717656, -0.009060777, -0.996685658, 0.0808402617

# homogeneous transformation matrix from Robot frame to Camera frame
# data from extrinsic calibration
translation_cam = np.array([-0.4650, 0.107, 1.102322]).reshape((3, 1))   # xyz
rotation_cam = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([-0.000672717656, -0.009060777, -0.996685658, 0.0808402617]).reshape((4, 1)))  # wxyz
transformation_cam = np.append(rotation_cam, translation_cam, axis=1)
transformation_cam = np.append(transformation_cam, np.array([[0, 0, 0, 1]]), axis=0)
# homogeneous transformation matrix from Camera frame to Robot frame (inverse)
transformation_cam_inv = np.linalg.inv(transformation_cam)

# extra homogeneous transformation matrix from Robot frame to Camera frame
# hand calibration
translation = np.array([-0.12, 0.06, 0.0]).reshape((3, 1))
rotation = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([1.0, 0.0, 0.0, 0.0]).reshape((4, 1)))  # wxyz
transformation_extra = np.append(rotation, translation, axis=1)
transformation_extra = np.append(transformation_extra, np.array([[0, 0, 0, 1]]), axis=0)

# homogeneous transformation matrix from Robot frame to gripper frame
translate_grip = np.array([-0.66257, -0.07707, 0.30143]).reshape((3, 1))   # xyz
rotate_grip = np.radians(np.array([-179.00, -1.21, 11.36])).reshape((3, 1))  # CBA in radians
rotate_grip = o3d.geometry.get_rotation_matrix_from_axis_angle(rotate_grip)
transform_grip = np.append(rotate_grip, translate_grip, axis=1)
transform_grip = np.append(transform_grip, np.array([[0, 0, 0, 1]]), axis=0)

"""Visualize transformation"""
# The x, y, z axis will be rendered as red, green, and blue arrows respectively.
# Robot frame, origin of the world
robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
# Camera frame, transformed into Robot frame
cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
cam_frame.transform(transformation_cam_inv)
cam_frame.transform(transformation_extra)
# Robot frame, transformed into gripper frame
grip_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
grip_frame.transform(transform_grip)

# print("Load point cloud")
# pcd = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'reference_grasp', 'part_reference_crop.ply'))
pcd = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'reference_grasp', 'part_reference.ply'))
pcd = pcd.transform(transformation_cam_inv)
pcd = pcd.transform(transformation_extra)
o3d.visualization.draw_geometries([robot_frame, cam_frame, grip_frame, pcd, workspace_bounding_box])

# voxel_down_pcd.estimate_normals(
#     o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))

# print("Crop visualization")
# cropped = pcd.crop(part_bounding_box)
# bounding_box = cropped.get_axis_aligned_bounding_box()
# bounding_box.color = (0, 0, 1)
# o3d.visualization.draw_geometries([mesh, mesh_t, mesh_grip, cropped, part_bounding_box, workspace_bounding_box, bounding_box])

# o3d.io.write_point_cloud(os.path.join(cwd, '..', 'objects', 'reference_grasp', 'part_reference_crop.ply'), cropped)
