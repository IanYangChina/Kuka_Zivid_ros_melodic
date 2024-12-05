#!/usr/bin/env python3
import os
import yaml
import numpy as np
import open3d as o3d
from copy import deepcopy as dcp

script_path = os.path.dirname(os.path.realpath(__file__))

DISTANCE_THRESHOLD = 0.001


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
    return transformation.copy()


calibration_result_file = os.path.join(script_path, '..', 'result_realsense', 'Extrinsics.yaml')
with open(calibration_result_file, 'r') as extrinsics_file:
    extrinsics = yaml.load(extrinsics_file, Loader=yaml.FullLoader)

raw_transform_ee_to_cam = np.asarray(extrinsics['matrix'])

transform_base_to_ee = construct_homogeneous_transform_matrix(
    translation=[-0.5504689, -0.09321245, 0.43060415],
    orientation=[-0.13915032, 0.0159282636, 0.98966670, 0.0307138836]
)

transform_base_to_cam = np.matmul(transform_base_to_ee, raw_transform_ee_to_cam)
transform_cam_to_base = np.linalg.inv(transform_base_to_cam)

pcd_path = os.path.join(script_path, '..', 'result_realsense', 'pcd_reference.ply')
pcd = o3d.io.read_point_cloud(pcd_path)

workspace_bounding_box_array = np.array([
 [-0.0,  -0.2,  -0.1],
 [-0.8,  -0.2,  -0.1],
 [-0.8,  0.2,  -0.1],
 [-0.0,  0.2,  -0.1],
 [-0.0,  -0.2,   0.1],
 [-0.8,  -0.2,   0.1],
 [-0.8,  0.2,   0.1],
 [-0.0,  0.2,   0.1]])
workspace_bounding_box_array = o3d.utility.Vector3dVector(workspace_bounding_box_array.astype('float'))
bbx = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
    points=workspace_bounding_box_array)
bbx.color = (0, 1, 0)

table_surface = o3d.geometry.TriangleMesh.create_box(width=0.5, height=0.5, depth=0.001)
table_surface.translate([-1.0, -0.25, -0.00646])
robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)

translation_extra = np.array([-0.83, 0.133, 0.106])  # XYZ
orientation_extra = np.array([-26, -17, -1])  # CBA
transform_extra_within_base = construct_homogeneous_transform_matrix(translation_extra, orientation_extra)
transform_base_to_cam_correct = np.matmul(transform_extra_within_base, transform_base_to_cam)
pcd_world = dcp(pcd).transform(transform_base_to_cam_correct)
# pcd_world = pcd_world.transform(transform_extra_within_base)
pcd_world.crop(bbx)

obj_centre = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
obj_centre.translate([-0.716, -0.0796, -0.00646])

o3d.visualization.draw_geometries([pcd_world, bbx, table_surface, robot_frame, obj_centre])

transform_base_to_cam_correct = np.matmul(transform_extra_within_base, transform_base_to_cam)
transform_cam_to_base_fine_tuned = np.linalg.inv(transform_base_to_cam_correct)
transform_cam_to_ee_fine_tuned = np.matmul(transform_cam_to_base_fine_tuned, transform_base_to_ee)
np.save(os.path.join(script_path, '..', 'result_realsense', 'transform_cam_to_ee_fine_tuned.npy'), transform_cam_to_ee_fine_tuned)
