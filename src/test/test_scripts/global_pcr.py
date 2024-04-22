import os
import open3d as o3d
import numpy as np
import copy


def preprocess_point_cloud(pcd, voxel_size, radius_normal, radius_feature):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    cl, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd_down = pcd_down.select_down_sample(ind)
    pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=100))
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        input=pcd_down,
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=200))
    return pcd_down, pcd_fpfh


def execute_fast_global_registration(source_down, target_down, source_fpfh, target_fpfh, distance_threshold):
    result = o3d.registration.registration_fast_based_on_feature_matching(
        source=source_down,
        target=target_down,
        source_feature=source_fpfh,
        target_feature=target_fpfh,
        option=o3d.registration.FastGlobalRegistrationOption(maximum_correspondence_distance=distance_threshold)
    )
    return result


def refine_registration(source, target, previous_transformation, distance_threshold):
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, previous_transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    return result


cwd = os.getcwd()
object_name = 'part'
# load hand-calibrated transformation matrices
transform_base_to_cam_hand_calibrated = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_base_to_cam_fine_tuned.npy'))
transform_base_to_reference_grasp = np.load(os.path.join(cwd, 'transformation_matrices', f'transform_base_to_reference_grasp_{object_name}.npy'))
# create a robot frame and transform into cam frame
robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
cam_frame.transform(transform_base_to_cam_hand_calibrated)
# Robot frame, transformed into gripper frame
grip_frame_original = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
grip_frame_original.transform(transform_base_to_reference_grasp)

# registration parameters unit: meter
voxel_size = 0.007
radius_normal = 0.01
radius_feature = 0.015
distance_threshold = 0.005

# load, preprocess target point cloud (one with a reference grasp)
target = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', f'{object_name}_ref_grasp', 'pcd_reference_merged.ply'))
target.paint_uniform_color([0, 0, 1])
target_down, target_fpfh = preprocess_point_cloud(target, voxel_size=voxel_size, radius_normal=radius_normal, radius_feature=radius_feature)
target_down.paint_uniform_color([0, 0, 1])

print(":: Load two point clouds and disturb initial pose.")
index_source_pcd = '2'
# source_original = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'reference_grasp', 'cropped_pcd_reference_in_world_frame.ply'))
source_original = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', f'{object_name}', 'pcd_reference_'+index_source_pcd+'_crop.ply'))
source_original.paint_uniform_color([1, 0, 0])
# o3d.visualization.draw_geometries([robot_frame, grip_frame_original, cam_frame, target, source_original])

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
result_refine = refine_registration(source_down, target_down, result_fast.transformation, distance_threshold=0.01)
print(result_refine)

result_transform = np.matmul(result_refine.transformation, transform_base_to_cam_hand_calibrated)
source_after_icp_refinement = copy.deepcopy(source_original)
source_after_icp_refinement.transform(result_transform)
source_after_icp_refinement.paint_uniform_color([1, 0, 1])

grip_frame = copy.deepcopy(grip_frame_original)
result_transform_inv = np.linalg.inv(result_transform)
grip_frame.transform(result_transform_inv)

o3d.visualization.draw_geometries([robot_frame, grip_frame, cam_frame, grip_frame_original,
                                   # source_original,
                                   target, source_after_icp_refinement])

o3d.io.write_point_cloud(os.path.join(cwd, '..', 'objects', f'{object_name}', 'pcd_reference_'+index_source_pcd+'_crop_registered.ply'), source_after_icp_refinement)
