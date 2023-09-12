import os
import numpy as np
import open3d as o3d  # >= 0.12.0


script_path = os.path.dirname(os.path.abspath(__file__))
pcd_path = os.path.join(script_path, '..', 'objects', 'pcd_to_mesh', 'pcd_0.ply')
bounding_box_array = np.load(os.path.join(script_path, 'transformation_matrices', 'reconstruction_bounding_box_array_in_base.npy'))

bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points=o3d.utility.Vector3dVector(bounding_box_array))
world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
pcd = o3d.io.read_point_cloud(pcd_path).crop(bounding_box)
print(pcd)
pcd = pcd.voxel_down_sample(voxel_size=0.002)
print(pcd)
cl, ind = pcd.remove_radius_outlier(nb_points=12, radius=0.003)
outliner = pcd.select_by_index(ind, invert=True).paint_uniform_color([1, 0, 0])
pcd = pcd.select_by_index(ind)
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
pcd.orient_normals_consistent_tangent_plane(200)
o3d.io.write_point_cloud(os.path.join(script_path, '..', 'objects', 'pcd_to_mesh', 'pcd_0_processed.ply'), pcd)
o3d.visualization.draw_geometries([world_frame, pcd, bounding_box, outliner], width=800, height=800)

# alpha = 0.02  # smaller alpha gives more detailed mesh
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)

radii = [0.005, 0.01, 0.02, 0.04, 0.08, 0.1, 0.2, 0.25]
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh).fill_holes()
mesh_ = o3d.geometry.TriangleMesh()
mesh_.vertices = mesh.vertex
mesh_.triangles = mesh.triangle

# mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=3)
print(mesh)
o3d.visualization.draw_geometries([mesh_],
                                  width=800, height=800,
                                  mesh_show_back_face=True,
                                  mesh_show_wireframe=True)

o3d.io.write_triangle_mesh(os.path.join(script_path, '..', 'objects', 'pcd_to_mesh', 'mesh_0.obj'), mesh)
