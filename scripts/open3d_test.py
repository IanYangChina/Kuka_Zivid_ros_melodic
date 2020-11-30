import os
import numpy as np
import open3d as o3d

cwd = os.getcwd()

polygon = np.array([
    [205.65166596967674, 0.0, 1028.413119262879],
    [155.0499772842004, 0.0, 1032.445198671467],
    [73.80532362865074, 0.0, 1016.6043954808599],
    [-94.00185124238419, 0.0, 981.7712312769554],
    [-133.10475579901095, 0.0, 978.0846610041165],
    [-168.24728942877005, 0.0, 988.0394238478543],
    [-205.57395947768327, 0.0, 986.1151610371774],
    [-210.19713595939356, 0.0, 959.772503784535],
    [-137.62206991340804, 0.0, 523.6029527855811],
    [-113.033142895634, 0.0, 519.0092223484955],
    [-96.60768465970932, 0.0, 471.14114987476125],
    [-76.05827064518553, 0.0, 459.29965086909544],
    [232.38639101070794, 0.0, 508.9847459227732],
    [263.0084743329453, 0.0, 541.4301847238818],
    [249.05362833028835, 0.0, 577.9195024254442],
    [281.45218506907827, 0.0, 598.505839095664],
    [288.24514936174, 0.0, 616.7740108960115],
    [229.60189378189693, 0.0, 991.5133885513412],
    [220.03432699868677, 0.0, 1022.8672599622978],
    [210.5703290171582, 0.0, 1027.3389494824646]
])
polygon *= 1.0
polygon = np.transpose(polygon)
polygon[0] -= 100
polygon[-1] -= 100
polygon = np.transpose(polygon)
# pcd = o3d.io.read_point_cloud('/home/xintong/Documents/PyProjects/ICP_test/src/Zivid_project/point_clouds/05.ply')
# print(np.asarray(pcd.points))
# o3d.visualization.draw_geometries([pcd])

file_index = 5


def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_down_sample(ind)
    outlier_cloud = cloud.select_down_sample(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


if __name__ == "__main__":
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud(cwd + "/../point_clouds/0" + str(file_index) + "_ds.ply")
    print(pcd)
    # o3d.visualization.draw_geometries([pcd])

    print("Downsample the point cloud with a voxel of 0.02")
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=2.0)
    # o3d.visualization.draw_geometries([voxel_down_pcd])
    print(voxel_down_pcd)

    # print("Polygon visualisation")
    # print(polygon.tolist())
    # polygon = o3d.geometry.PointCloud(
    #     points=o3d.utility.Vector3dVector(polygon.tolist())
    # )
    # polygon.paint_uniform_color([1, 0, 0])
    # o3d.visualization.draw_geometries([voxel_down_pcd, polygon])

    print("Load a polygon volume and use it to crop the original point cloud")
    vol = o3d.visualization.read_selection_polygon_volume(cwd + "/json/crop.json")
    cropped = vol.crop_point_cloud(voxel_down_pcd)
    # o3d.visualization.draw_geometries([cropped])
    # print(cropped)

    o3d.io.write_point_cloud(cwd + "/../point_clouds/0"+str(file_index)+"_ds_cropped.ply", cropped)
