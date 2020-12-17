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
    [0.0, 0.0, 0.1],
    [0.0, 0.0, 0.15],
    [0.0, 0.0, 0.2],
    [0.0, 0.0, 0.25],
    [0.0, 0.0, 0.3],
    [0.0, 0.0, 0.35],
    [0.1, 0.0, 0.0],
    [0.15, 0.0, 0.0],
    [0.2, 0.0, 0.0],
    [0.25, 0.0, 0.0],
    [0.3, 0.0, 0.0],
    [0.0, 0.1, 0.0],
    [0.0, 0.15, 0.0],
    [0.0, 0.2, 0.0],
    [0.0, 0.25, 0.0],
    [0.0, 0.3, 0.0],
    [0.0, 0.35, 0.0]
])
polygon = o3d.geometry.PointCloud(
        points=o3d.utility.Vector3dVector(polygon.tolist())
    )
polygon.paint_uniform_color([1, 0, 0])

translation_matrix = -np.array([-0.526, 0.162, 1.090]).reshape((3, 1))   # xyz
rotation_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([0.003, -0.995, 0.086, 0.035]).reshape((4, 1)))  # wxyz
# rotation_matrix = np.transpose(rotation_matrix)


def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_down_sample(ind)
    outlier_cloud = cloud.select_down_sample(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


if __name__ == "__main__":
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud(cwd + "/../objects/cube_xyz_4.ply")
    # print(np.asarray(pcd.points))
    # o3d.visualization.draw_geometries([pcd])

    print("Downsample the point cloud with a voxel of 0.02")
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.005)
    # voxel_down_pcd = o3d.geometry.voxel_down_sample(pcd, voxel_size=2.0)
    # o3d.visualization.draw_geometries([voxel_down_pcd])
    # print(voxel_down_pcd)

    # voxel_down_pcd.translate(translation=translation_matrix)
    # voxel_down_pcd.rotate(R=rotation_matrix)
    # print(np.asarray(voxel_down_pcd.points))

    print("Polygon visualisation")
    o3d.visualization.draw_geometries([voxel_down_pcd, polygon])

    # print("Load a polygon volume and use it to crop the original point cloud")
    # vol = o3d.visualization.read_selection_polygon_volume(cwd + "/json/crop.json")
    # cropped = vol.crop_point_cloud(voxel_down_pcd)
    # o3d.visualization.draw_geometries([cropped])
    # print(cropped)

    # o3d.io.write_point_cloud(cwd + "/../point_clouds/0"+str(file_index)+"_ds_cropped.ply", cropped)
