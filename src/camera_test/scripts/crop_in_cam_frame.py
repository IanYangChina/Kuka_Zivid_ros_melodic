import os
import numpy as np
import open3d as o3d  # 0.12.0
import json

cwd = os.getcwd()

# load hand-calibrated transformation matrices
transform_base_to_cam_hand_calibrated = np.load(os.path.join(cwd, 'transform_base_to_cam_hand_calibrated.npy'))
transform_base_to_reference_grasp = np.load(os.path.join(cwd, 'transform_base_to_reference_grasp.npy'))

# load a bounding box and transform it into cam frame
workspace_bounding_box_array = np.load(os.path.join(cwd, 'workspace_bounding_box_array_in_base.npy'))
workspace_bounding_box_array = np.transpose(workspace_bounding_box_array)
ones = np.ones(workspace_bounding_box_array.shape[1]).reshape((1, workspace_bounding_box_array.shape[1]))
workspace_bounding_box_array = np.append(workspace_bounding_box_array, ones, axis=0)
workspace_bounding_box_array = np.matmul(transform_base_to_cam_hand_calibrated, workspace_bounding_box_array)
workspace_bounding_box_array = np.transpose(workspace_bounding_box_array)[:, :-1]  # discard the one values
# create bounding box
workspace_bounding_box_array = o3d.utility.Vector3dVector(workspace_bounding_box_array.astype('float64'))
workspace_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points=workspace_bounding_box_array)
workspace_bounding_box.color = (0, 1, 0)

# The x, y, z axis will be rendered as red, green, and blue arrows respectively.
# Camera frame
cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
# Robot frame, transformed into cam frame
robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
robot_frame.transform(transform_base_to_cam_hand_calibrated)
# Reference grasp frame, transformed into world frame, and than into cam frame
reference_grasp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
reference_grasp_frame.transform(transform_base_to_reference_grasp)
reference_grasp_frame.transform(transform_base_to_cam_hand_calibrated)

# for i in ['0', '1', '2', '3', '4']:
#     pcd = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'part_xyz_'+i+'.ply'))
#     cropped = pcd.crop(workspace_bounding_box)
#     o3d.visualization.draw_geometries([robot_frame, cam_frame, cropped, workspace_bounding_box])
#     o3d.io.write_point_cloud(os.path.join(cwd, '..', 'objects', 'part_xyz_'+i+'_crop.ply'), cropped)
#
#     pcd = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'part_xyz_'+i+'_crop.ply'))
#     o3d.visualization.draw_geometries([robot_frame, cam_frame, pcd, workspace_bounding_box])

pcd = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'reference_grasp', 'part_reference.ply'))
o3d.visualization.draw_geometries([robot_frame, cam_frame, reference_grasp_frame, pcd, workspace_bounding_box])
cropped = pcd.crop(workspace_bounding_box)
o3d.visualization.draw_geometries([robot_frame, cam_frame, reference_grasp_frame, cropped, workspace_bounding_box])
o3d.io.write_point_cloud(os.path.join(cwd, '..', 'objects', 'reference_grasp', 'part_reference_crop.ply'), cropped)