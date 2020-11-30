# examples/Python/Advanced/fast_global_registration.py

import open3d as o3d
import numpy as np
import copy

import time


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # source_temp.paint_uniform_color([1, 0.706, 0])
    # target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def preprocess_point_cloud(pcd, voxel_size, radius_normal, radius_feature):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    # radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    # radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, distance_threshold):
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result


if __name__ == "__main__":

    print(":: Load two point clouds and disturb initial pose.")
    source = o3d.io.read_point_cloud("/home/xintong/Documents/PyProjects/ICP_test/src/Zivid_project/point_clouds/01_ds_cropped.ply")
    target = o3d.io.read_point_cloud("/home/xintong/Documents/PyProjects/ICP_test/src/Zivid_project/point_clouds/04_ds_cropped.ply")
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size=1.0, radius_normal=2.3, radius_feature=10.0)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size=1.0, radius_normal=2.3, radius_feature=10.0)
    o3d.visualization.draw_geometries([source_down, target_down])

    start = time.time()
    result_fast = execute_fast_global_registration(source_down, target_down,
                                                   source_fpfh, target_fpfh,
                                                   distance_threshold=2.8)
    print("Fast global registration took %.3f sec.\n" % (time.time() - start))
    print(result_fast)
    draw_registration_result(source_down, target_down,
                             result_fast.transformation)
