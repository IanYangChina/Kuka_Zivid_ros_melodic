import os
import numpy as np
import open3d as o3d  # 0.9.0

cwd = os.getcwd()

# load hand-calibrated transformation matrices
transform_base_to_cam_fine_tuned = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_base_to_cam_fine_tuned.npy'))
transform_cam_to_base_fine_tuned = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_cam_to_base_fine_tuned.npy'))
transform_base_to_reference_grasp = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_base_to_reference_grasp_sprayer.npy'))

# load a bounding box and transform it into cam frame
workspace_bounding_box_array = np.load(os.path.join(cwd, 'transformation_matrices', 'workspace_bounding_box_array_in_base.npy'))
workspace_bounding_box_array[4][-1] += 0.005
workspace_bounding_box_array[5][-1] += 0.005
workspace_bounding_box_array[6][-1] += 0.005
workspace_bounding_box_array[7][-1] += 0.005
# workspace_bounding_box_array[2][1] = -0.2
# workspace_bounding_box_array[3][1] = -0.2
# workspace_bounding_box_array[6][1] = -0.2
# workspace_bounding_box_array[7][1] = -0.2
# workspace_bounding_box_array = np.transpose(workspace_bounding_box_array)
# ones = np.ones(workspace_bounding_box_array.shape[1]).reshape((1, workspace_bounding_box_array.shape[1]))
# workspace_bounding_box_array = np.append(workspace_bounding_box_array, ones, axis=0)
# workspace_bounding_box_array = np.matmul(transform_base_to_cam_fine_tuned, workspace_bounding_box_array)
# workspace_bounding_box_array = np.transpose(workspace_bounding_box_array)[:, :-1]  # discard the one values
# create bounding box
workspace_bounding_box_array = o3d.utility.Vector3dVector(workspace_bounding_box_array.astype('float64'))
workspace_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points=workspace_bounding_box_array)
workspace_bounding_box.color = (0, 1, 0)

# The x, y, z axis will be rendered as red, green, and blue arrows respectively.
# Camera frame
cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
cam_frame.transform(transform_cam_to_base_fine_tuned)
# Robot frame, transformed into cam frame
robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
# robot_frame.transform(transform_base_to_cam_fine_tuned)
# Reference grasp frame, transformed into world frame, and than into cam frame
reference_grasp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
reference_grasp_frame.transform(transform_base_to_reference_grasp)
# reference_grasp_frame.transform(transform_base_to_cam_fine_tuned)

pcd = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'sprayer', 'pcd_reference_2.ply'))
pcd.transform(transform_cam_to_base_fine_tuned)
o3d.visualization.draw_geometries([robot_frame, cam_frame, reference_grasp_frame, pcd, workspace_bounding_box])
# exit()

# for i in ['1', '2', '3', '4', '5', '6']:
#     pcd = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'pcl_part', 'pcd_reference_'+i+'.ply'))
#     pcd.transform(transform_cam_to_base_fine_tuned)
#     o3d.visualization.draw_geometries([robot_frame, cam_frame, reference_grasp_frame, pcd, workspace_bounding_box])
#     cropped = pcd.crop(workspace_bounding_box)
#     o3d.visualization.draw_geometries([robot_frame, cam_frame, reference_grasp_frame, cropped, workspace_bounding_box])
#     o3d.io.write_point_cloud(os.path.join(cwd, '..', 'objects', 'pcl_part', 'pcd_reference_'+i+'_crop.ply'), cropped)

# pcd = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'reference_grasp', 'part_reference_merged_inlier.ply'))
# o3d.visualization.draw_geometries([robot_frame, cam_frame, reference_grasp_frame, pcd, workspace_bounding_box])
# new_pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=0.4)
cropped = pcd.crop(workspace_bounding_box)
# new_pcd, ind = cropped.remove_radius_outlier(nb_points=100, radius=0.05)
# new_pcd, ind = cropped.remove_statistical_outlier(nb_neighbors=100, std_ratio=0.4)
bounding_box = cropped.get_axis_aligned_bounding_box()
o3d.visualization.draw_geometries([robot_frame, cam_frame, reference_grasp_frame, cropped, workspace_bounding_box, bounding_box])
o3d.io.write_point_cloud(os.path.join(cwd, '..', 'objects', 'sprayer', 'pcd_reference_2_crop.ply'), cropped)
