import os
import numpy as np
import open3d as o3d  # >= 0.12.0
import pymeshfix
import pyvista as pv

script_path = os.path.dirname(os.path.abspath(__file__))
pcd_path = os.path.join(script_path, '..', 'objects', 'pcd_to_mesh', 'pcd_0.ply')
bounding_box_array = np.load(os.path.join(script_path, 'transformation_matrices', 'reconstruction_bounding_box_array_in_base.npy'))

bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points=o3d.utility.Vector3dVector(bounding_box_array))
world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
pcd = o3d.io.read_point_cloud(pcd_path).crop(bounding_box)

_, ind = pcd.remove_statistical_outlier(nb_neighbors=25, std_ratio=1.5)
outliner = pcd.select_by_index(ind, invert=True).paint_uniform_color([1, 0, 0])
pcd = pcd.select_by_index(ind).paint_uniform_color([0, 0.5, 0.5])

_, ind = pcd.remove_radius_outlier(nb_points=15, radius=0.004)
outliner = outliner + pcd.select_by_index(ind, invert=True).paint_uniform_color([0, 1, 0])
pcd = pcd.select_by_index(ind).paint_uniform_color([0, 0.5, 0.5])

pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.006, max_nn=30))
pcd.orient_normals_consistent_tangent_plane(50)
o3d.io.write_point_cloud(os.path.join(script_path, '..', 'objects', 'pcd_to_mesh', 'pcd_0_processed.ply'), pcd)
o3d.visualization.draw_geometries([world_frame, pcd, bounding_box, outliner], width=800, height=800, point_show_normal=False)

radii = [0.003, 0.006]
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))

o3d.visualization.draw_geometries([pcd, world_frame, mesh, bounding_box],
                                  width=800, height=800,
                                  mesh_show_back_face=True,
                                  mesh_show_wireframe=True)

mesh_path = os.path.join(script_path, '..', 'objects', 'pcd_to_mesh', 'mesh_0.obj')
o3d.io.write_triangle_mesh(mesh_path, mesh)

# todo: looking for better mesh repairing tools

mesh = pv.read(mesh_path)
mf = pymeshfix.MeshFix(mesh)
mf.repair()
repaired = mf.mesh
repaired_mesh_path = os.path.join(script_path, '..', 'objects', 'pcd_to_mesh', 'mesh_0_repaired.ply')
repaired.save(repaired_mesh_path)

repaired_mesh = o3d.io.read_triangle_mesh(repaired_mesh_path)
o3d.visualization.draw_geometries([pcd, world_frame, repaired_mesh, bounding_box],
                                  width=800, height=800,
                                  mesh_show_back_face=True,
                                  mesh_show_wireframe=True)
repaired_mesh_path = os.path.join(script_path, '..', 'objects', 'pcd_to_mesh', 'mesh_0_repaired.obj')
o3d.io.write_triangle_mesh(repaired_mesh_path, repaired_mesh)
