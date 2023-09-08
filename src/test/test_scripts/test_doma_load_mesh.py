import os
import numpy as np
import open3d as o3d
from doma.envs import ClayEnv

script_path = os.path.dirname(os.path.realpath(__file__))
mesh_file_path = os.path.join(script_path, '..', 'objects', 'pcd_to_mesh', 'mesh_0.obj')
env = ClayEnv(ptcl_density=2e7, horizon=500,
              mesh_file=mesh_file_path, material_id=3, voxelise_res=500, initial_pos=(0.25, 0.25, 0.15))
env.reset()
# done = False
# while not done:
#     env.render(mode='human')
# env.mpm_env.apply_agent_action_p(np.array((1., 1., 1.)))
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
real_pcd = o3d.io.read_point_cloud(os.path.join(script_path, '..', 'objects', 'pcd_to_mesh', 'pcd_0_down.ply')).paint_uniform_color([0.1, 0.3, 0.3])
xyz_real = np.asarray(real_pcd.points)

x, x_ = env.render(mode='point_cloud')
x -= np.array([0.25, 0.25, 0.15])
x += env.original_center
obj_vec = o3d.utility.Vector3dVector(x)
obj_pcd = o3d.geometry.PointCloud(obj_vec).paint_uniform_color([0.6, 0.6, 0.6])
xyz_sim = np.asarray(obj_pcd.points)

d = real_pcd.compute_point_cloud_distance(obj_pcd)
print(np.asarray(d).mean())

o3d.visualization.draw_geometries([frame, obj_pcd, real_pcd], width=800, height=600)

