import os
import json
import numpy as np
import quaternion as quat
import open3d as o3d
from copy import deepcopy as dcp
from geometry_msgs.msg import PoseStamped, PoseArray

script_dir = os.path.dirname(os.path.realpath(__file__))
# load hand-calibrated transformation matrices
transform_base_to_cam_hand_calibrated = np.load(
    os.path.join(script_dir, '../transformation_matrices', 'transform_base_to_cam_fine_tuned.npy'))
transform_cam_to_base_hand_calibrated = np.load(
    os.path.join(script_dir, '../transformation_matrices', 'transform_cam_to_base_fine_tuned.npy'))
transform_base_to_part_reference_grasp = np.load(
    os.path.join(script_dir, '../transformation_matrices', 'transform_base_to_part_reference_grasp.npy'))
transform_base_to_brash_reference_grasp = np.load(
    os.path.join(script_dir, '../transformation_matrices', 'transform_base_to_brash_reference_grasp.npy'))
identity_transform = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
])

# load and create a bounding box
workspace_bounding_box_array = np.load(
    os.path.join(script_dir, '../transformation_matrices', 'workspace_bounding_box_array_in_base.npy'))
workspace_bounding_box_array[4][-1] += 0.005
workspace_bounding_box_array[5][-1] += 0.005
workspace_bounding_box_array[6][-1] += 0.005
workspace_bounding_box_array[7][-1] += 0.005

workspace_bounding_box_array = o3d.utility.Vector3dVector(workspace_bounding_box_array.astype('float64'))
workspace_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points=workspace_bounding_box_array)
workspace_bounding_box.color = (0, 1, 0)

# The x, y, z axis will be rendered as red, green, and blue arrows respectively.
# Robot frame
robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
# Camera frame, transformed into robot frame
cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
cam_frame.transform(transform_cam_to_base_hand_calibrated)

# default registration parameters unit: meter
config_file = os.path.join(script_dir, 'registration_configs.json')

ICP_REFINE_DISTANCE_THRESHOLD = 0.002
GLOBAL_REGISTRATION_MAX_ITER = 100
GLOBAL_REGISTRATION_RMSE_THRESHOLD = 0.005


def preprocess_point_cloud(pcd, voxel_size, radius_normal, radius_feature):
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=4.0)
    pcd_inliners = pcd.select_down_sample(ind)
    pcd_down = pcd_inliners.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=100))
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        input=pcd_down,
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=200))
    return pcd_down, pcd_fpfh


def execute_fast_global_registration(source_down, target_down, source_fpfh, target_fpfh,
                                     distance_threshold):
    print("[INFO] Fast global registration with fpfh features")
    best_result = o3d.registration.registration_fast_based_on_feature_matching(
        source=source_down,
        target=target_down,
        source_feature=source_fpfh,
        target_feature=target_fpfh,
        option=o3d.registration.FastGlobalRegistrationOption(maximum_correspondence_distance=distance_threshold)
    )
    n = 0
    while best_result.inlier_rmse > GLOBAL_REGISTRATION_RMSE_THRESHOLD and n < GLOBAL_REGISTRATION_MAX_ITER:
        n += 1
        result = o3d.registration.registration_fast_based_on_feature_matching(
            source=source_down,
            target=target_down,
            source_feature=source_fpfh,
            target_feature=target_fpfh,
            option=o3d.registration.FastGlobalRegistrationOption(maximum_correspondence_distance=distance_threshold)
        )
        if best_result.inlier_rmse > result.inlier_rmse:
            best_result = dcp(result)
        else:
            continue
    return best_result


def refine_registration(source, target, previous_transformation,
                        distance_threshold=ICP_REFINE_DISTANCE_THRESHOLD):
    print("[INFO] Point-to-plane ICP registration")
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, previous_transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    return result


# load, preprocess target point cloud (one with a reference grasp)
part_target = o3d.io.read_point_cloud(
    os.path.join(script_dir, '../reference_grasp', 'cropped_part_pcd_in_world_frame.ply'))
brash_target = o3d.io.read_point_cloud(
    os.path.join(script_dir, '../reference_grasp', 'cropped_brash_pcd_in_world_frame.ply'))

part_target.paint_uniform_color([0.6, 0.6, 0.6])
brash_target.paint_uniform_color([0.3, 0.6, 0.3])

# use an identity transformation as the initial transform that moves the source pcd away from the target
init_transformation_for_global_registration = identity_transform.copy()

# PoseStamped msg
pose_msg = PoseStamped()
poses_msg = PoseArray()


def get_target_grasp_pose(source_pcd):
    with open(config_file, 'r') as f:
        config = json.load(f)

    # compute fpfh
    part_target_down, part_target_fpfh = preprocess_point_cloud(part_target,
                                                                config['part']['VOXEL_SIZE'],
                                                                config['part']['RADIUS_NORMAL'],
                                                                config['part']['RADIUS_FEATURE'])
    brash_target_down, brash_target_fpfh = preprocess_point_cloud(brash_target,
                                                                  config['brash']['VOXEL_SIZE'],
                                                                  config['brash']['RADIUS_NORMAL'],
                                                                  config['brash']['RADIUS_FEATURE'])

    # copy the source pcd and transform into robot frame
    source_pcd_original = dcp(source_pcd)
    source_pcd_original.transform(transform_cam_to_base_hand_calibrated)
    # o3d.visualization.draw_geometries([robot_frame, cam_frame, target, workspace_bounding_box, source_pcd_original])

    # copy one to be processed, and crop
    source_pcd_to_process = dcp(source_pcd_original)
    source_pcd_to_process = source_pcd_to_process.crop(workspace_bounding_box)

    # pre-processing for registration
    # transform the source pcd to an arbitrary pose far away from the target
    source_pcd_to_process.transform(init_transformation_for_global_registration)
    # compute fpfh
    source_down_for_part, source_fpfh_for_part = preprocess_point_cloud(source_pcd_to_process,
                                                                        config['part']['VOXEL_SIZE'],
                                                                        config['part']['RADIUS_NORMAL'],
                                                                        config['part']['RADIUS_FEATURE'])
    source_down_for_brash, source_fpfh_for_brash = preprocess_point_cloud(source_pcd_to_process,
                                                                          config['brash']['VOXEL_SIZE'],
                                                                          config['brash']['RADIUS_NORMAL'],
                                                                          config['brash']['RADIUS_FEATURE'])

    # global registration
    result_part_fast = execute_fast_global_registration(source_down_for_part, part_target_down,
                                                        source_fpfh_for_part, part_target_fpfh,
                                                        config['part']['GLOBAL_DISTANCE_THRESHOLD'])
    result_brash_fast = execute_fast_global_registration(source_down_for_brash, brash_target_down,
                                                         source_fpfh_for_brash, brash_target_fpfh,
                                                         config['brash']['GLOBAL_DISTANCE_THRESHOLD'])
    # icp refinement
    result_part_refine = refine_registration(source_down_for_part, part_target_down, result_part_fast.transformation)
    result_brash_refine = refine_registration(source_down_for_brash, brash_target_down, result_brash_fast.transformation)
    # final transformation should includes the initial transformation
    transform_source_to_part_target = np.matmul(result_part_refine.transformation,
                                                init_transformation_for_global_registration)
    # transform_source_to_sprayer_target = np.matmul(result_sprayer_refine.transformation, init_transformation_for_global_registration)
    transform_source_to_brash_target = np.matmul(result_brash_refine.transformation,
                                                 init_transformation_for_global_registration)
    # transform_source_to_cam_mount_target = np.matmul(result_cam_mount_refine.transformation, init_transformation_for_global_registration)

    # visualizing result
    part_grasp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    brash_grasp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    transform_part_target_to_source = np.linalg.inv(transform_source_to_part_target)
    transform_brash_target_to_source = np.linalg.inv(transform_source_to_brash_target)
    transform_part_target_grasp = np.matmul(transform_part_target_to_source, transform_base_to_part_reference_grasp)
    transform_brash_target_grasp = np.matmul(transform_brash_target_to_source, transform_base_to_brash_reference_grasp)
    part_grasp_frame.transform(transform_part_target_grasp)
    brash_grasp_frame.transform(transform_brash_target_grasp)
    print('[INFO] Visualizing the found grasping pose')
    o3d.visualization.draw_geometries([robot_frame,
                                       part_grasp_frame,
                                       brash_grasp_frame,
                                       source_pcd_original, brash_target, part_target],
                                      window_name='Grasping pose proposal', width=1200, height=960)

    part_pose_msg = get_pose_msg(transform_part_target_grasp)
    # sprayer_pose_msg = get_pose_msg(transform_sprayer_target_grasp)
    brash_pose_msg = get_pose_msg(transform_brash_target_grasp)
    # cam_mount_pose_msg = get_pose_msg(transform_cam_mount_target_grasp)
    poses_msg_to_go = dcp(poses_msg)
    poses_msg_to_go.poses = [brash_pose_msg.pose, part_pose_msg.pose]

    return poses_msg_to_go


def get_pose_msg(transformation_mat):
    # convert transformation matrix to coordinate and quaternion
    rotation_quat = quat.from_rotation_matrix(transformation_mat[:-1, :-1])  # this is already normalized
    rotation_quat = quat.as_float_array(rotation_quat).tolist()  # wxyz
    translate_matrix = transformation_mat[:-1, -1].tolist()  # xyz
    # construct and return a PoseStamped msg
    pose = dcp(pose_msg)
    pose.pose.position.x = translate_matrix[0]
    pose.pose.position.y = translate_matrix[1]
    pose.pose.position.z = translate_matrix[2]
    pose.pose.orientation.w = rotation_quat[0]
    pose.pose.orientation.x = rotation_quat[1]
    pose.pose.orientation.y = rotation_quat[2]
    pose.pose.orientation.z = rotation_quat[3]
    return pose
