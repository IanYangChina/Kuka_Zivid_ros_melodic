import os
import open3d as o3d
import numpy as np

cwd = os.getcwd()

# load hand-calibrated transformation matrices
transform_base_to_cam_fine_tuned = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_base_to_cam_fine_tuned.npy'))
transform_cam_to_base_fine_tuned = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_cam_to_base_fine_tuned.npy'))
transform_base_to_reference_grasp = np.load(os.path.join(cwd, 'transformation_matrices', 'transform_base_to_reference_grasp_brash.npy'))
workspace_bounding_box_array = np.load(os.path.join(cwd, 'transformation_matrices', 'workspace_bounding_box_array_in_base.npy'))
workspace_bounding_box_array = o3d.utility.Vector3dVector(workspace_bounding_box_array.astype('float64'))
workspace_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points=workspace_bounding_box_array)
workspace_bounding_box.color = (0, 1, 0)
cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
cam_frame.transform(transform_cam_to_base_fine_tuned)
robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
reference_grasp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
reference_grasp_frame.transform(transform_base_to_reference_grasp)

target = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'brash_ref_grasp', 'pcd_reference_merged.ply'))
pcd_0 = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'brash_ref_grasp', f'pcd_reference.ply'))
pcd_1 = o3d.io.read_point_cloud(os.path.join(cwd, '..', 'objects', 'brash', f'pcd_reference_1_crop_registered.ply'))

fused = target + pcd_1
fused = fused.voxel_down_sample(voxel_size=0.0005)

o3d.visualization.draw_geometries([robot_frame, fused, workspace_bounding_box,cam_frame])
o3d.io.write_point_cloud(os.path.join(cwd, '..', 'objects', 'brash_ref_grasp', 'pcd_reference_merged.ply'), fused)
