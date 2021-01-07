# examples/Python/Advanced/fast_global_registration.py

import os
import open3d as o3d
import numpy as np
import copy


def preprocess_point_cloud(pcd, voxel_size, radius_normal, radius_feature):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=100))
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        input=pcd_down,
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=200))
    return pcd_down, pcd_fpfh


def execute_fast_global_registration(source_down, target_down, source_fpfh, target_fpfh, distance_threshold):
    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        source=source_down,
        target=target_down,
        source_feature=source_fpfh,
        target_feature=target_fpfh,
        option=o3d.pipelines.registration.FastGlobalRegistrationOption(maximum_correspondence_distance=distance_threshold)
    )
    return result


def refine_registration(source, target, previous_transformation, distance_threshold):
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, previous_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result


cwd = os.getcwd()
# load hand-calibrated transformation matrices
transform_base_to_cam_hand_calibrated = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_base_to_cam_hand_calibrated.npy'))
transform_base_to_reference_grasp = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_base_to_reference_grasp.npy'))
# create a robot frame and transform into cam frame
robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
robot_frame.transform(transform_base_to_cam_hand_calibrated)

# registration parameters unit: meter
voxel_size = 0.01
radius_normal = 0.01
radius_feature = 0.2
distance_threshold = 0.5

# load, preprocess target point cloud (one with a reference grasp)
target = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'reference_grasp', 'part_reference_merged.ply'))
target.paint_uniform_color([0, 0, 1])
target_down, target_fpfh = preprocess_point_cloud(target, voxel_size=voxel_size, radius_normal=radius_normal, radius_feature=radius_feature)
target_down.paint_uniform_color([0, 0, 1])

print(":: Load two point clouds and disturb initial pose.")
source_original = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'pcl_part', 'part_xyz_4_crop.ply'))
source_original.paint_uniform_color([1, 0, 0])
o3d.visualization.draw_geometries([robot_frame, target, source_original])

source = copy.deepcopy(source_original)
# transform the source pcd to an arbitrary pose far away from the target
source.transform(transform_base_to_cam_hand_calibrated)
source.paint_uniform_color([1, 0, 0])

# down-sample and compute fpfh features
source_down, source_fpfh = preprocess_point_cloud(source, voxel_size=voxel_size, radius_normal=radius_normal, radius_feature=radius_feature)
# fast global registration
result_fast = execute_fast_global_registration(source_down, target_down,
                                               source_fpfh, target_fpfh,
                                               distance_threshold=distance_threshold)
print(result_fast)
# ICP refinement
result_refine = refine_registration(source_down, target_down, result_fast.transformation, distance_threshold=0.005)
print(result_refine)

result_transform = np.matmul(result_refine.transformation, transform_base_to_cam_hand_calibrated)
source_after_icp_refinement = copy.deepcopy(source_original)
source_after_icp_refinement.transform(result_transform)
source_after_icp_refinement.paint_uniform_color([1, 0, 1])

# Robot frame, transformed into gripper frame
grip_frame_original = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
grip_frame_original.transform(transform_base_to_reference_grasp)
grip_frame_original.transform(transform_base_to_cam_hand_calibrated)

grip_frame = copy.deepcopy(grip_frame_original)
result_transform_inv = np.linalg.inv(result_transform)
grip_frame.transform(result_transform_inv)
o3d.visualization.draw_geometries([robot_frame, grip_frame, grip_frame_original, source_original, target, source_after_icp_refinement])