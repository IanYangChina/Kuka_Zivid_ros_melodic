import os
import numpy as np
import open3d as o3d

cwd = os.getcwd()

polygon = np.array([
    # [-0.7, 0.2, 2.5],
    # [-0.3, 0.2, 2.5],
    # [-0.7, -0.3, 2.5],
    # [-0.3, -0.3, 2.5]
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 0.02],
    [0.0, 0.0, 0.04],
    [0.0, 0.0, 0.06],
    [0.0, 0.0, 0.08],
    [0.0, 0.0, 0.10],
    [0.0, 0.0, 0.12],
    [0.0, 0.0, 0.14],
    [0.02, 0.0, 0.0],
    [0.04, 0.0, 0.0],
    [0.06, 0.0, 0.0],
    [0.08, 0.0, 0.0],
    [0.10, 0.0, 0.0],
    [0.0, 0.02, 0.0],
    [0.0, 0.04, 0.0],
    [0.0, 0.06, 0.0],
    [0.0, 0.08, 0.0],
    [0.0, 0.10, 0.0],
    [0.0, 0.12, 0.0]
])
polygon = o3d.geometry.PointCloud(
        points=o3d.utility.Vector3dVector(polygon.tolist())
    )
polygon.paint_uniform_color([1, 0, 0])

# homogeneous transformation matrix from Robot frame to Camera frame
translation_matrix = np.array([-0.4650, 0.107589, 1.102322]).reshape((3, 1))   # xyz
rotation_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([0.0, 0.0, -0.99, 0.08]).reshape((4, 1)))  # xyzw
transformation_matrix = np.append(rotation_matrix, translation_matrix, axis=1)
transformation_matrix = np.append(transformation_matrix, np.array([[0, 0, 0, 1]]), axis=0)

# homogeneous transformation matrix from Camera frame to Robot frame
transformation_matrix_ = np.linalg.inv(transformation_matrix)


def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_down_sample(ind)
    outlier_cloud = cloud.select_down_sample(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


if __name__ == "__main__":
    # Robot frame
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    # Camera frame, transformed into Robot frame
    mesh_t = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2).transform(transformation_matrix_)

    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud(cwd + "/../objects/part_xyz_4.ply")
    # print(np.asarray(pcd.points))
    # o3d.visualization.draw_geometries([pcd])

    print("Downsample the point cloud with a voxel of 0.02")
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.005)
    voxel_down_pcd = voxel_down_pcd.transform(transformation_matrix_)
    voxel_down_pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))

    # voxel_down_pcd = o3d.geometry.voxel_down_sample(pcd, voxel_size=2.0)
    # o3d.visualization.draw_geometries([voxel_down_pcd])
    # print(voxel_down_pcd)

    # voxel_down_pcd.translate(translation=translation_matrix)
    # voxel_down_pcd.rotate(R=rotation_matrix)
    # print(np.asarray(voxel_down_pcd.points))

    print("Polygon visualisation")
    o3d.visualization.draw_geometries([mesh, mesh_t, voxel_down_pcd])

    # print("Load a polygon volume and use it to crop the original point cloud")
    # vol = o3d.visualization.read_selection_polygon_volume(cwd + "/json/crop.json")
    # cropped = vol.crop_point_cloud(voxel_down_pcd)
    # o3d.visualization.draw_geometries([cropped])
    # print(cropped)

    # o3d.io.write_point_cloud(cwd + "/../point_clouds/0"+str(file_index)+"_ds_cropped.ply", cropped)
