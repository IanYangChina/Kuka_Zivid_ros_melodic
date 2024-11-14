import os
import numpy as np
import open3d as o3d
import taichi as ti


DTYPE_NP = np.float32
DTYPE_TI = ti.f32


def show_hm(data_path, data_ind):
    ti.reset()
    ti.init(arch=ti.opengl,
            default_fp=DTYPE_TI, default_ip=ti.i32,
            fast_math=False, random_seed=1)

    height_map_res = 32
    height_map_size = 0.11  # meter
    height_map_xy_offset = (0.25 * 1000, 0.25 * 1000)
    height_map_pixel_size = height_map_size * 1000 / height_map_res
    height_map_pcd_target = ti.field(dtype=DTYPE_TI, shape=(height_map_res, height_map_res), needs_grad=True)
    down_sample_voxel_size = 0.001

    @ti.func
    def from_xy_to_uv(x, y):
        u = (x - height_map_xy_offset[0]) / height_map_pixel_size + height_map_res / 2
        v = (y - height_map_xy_offset[1]) / height_map_pixel_size + height_map_res / 2
        return ti.floor(u, ti.i32), ti.floor(v, ti.i32)

    target_pcd_path = os.path.join(data_path, f'pcd_{data_ind}.ply')
    obj_start_centre_real = np.load(os.path.join(data_path, f'mesh_{data_ind}0_repaired_centre.npy')).astype(DTYPE_NP)
    obj_start_centre_top_normalised = np.load(
        os.path.join(data_path, f'mesh_{data_ind}0_repaired_normalised_centre_top.npy')).astype(DTYPE_NP)
    obj_start_initial_pos = np.array([0.25, 0.25, obj_start_centre_top_normalised[-1] + 0.01], dtype=DTYPE_NP)
    pcd_offset = - obj_start_centre_real + obj_start_initial_pos

    target_pcd = o3d.io.read_point_cloud(target_pcd_path).voxel_down_sample(voxel_size=down_sample_voxel_size)
    target_pcd_points_np = np.asarray(target_pcd.points, dtype=DTYPE_NP) + pcd_offset
    target_pcd_points_np *= 1000  # convert to mm
    n_target_pcd_points = target_pcd_points_np.shape[0]
    print(f'===>  {n_target_pcd_points:7d} target points loaded.')
    target_pcd_points = ti.Vector.field(3, dtype=DTYPE_TI, shape=n_target_pcd_points)
    target_pcd_points.from_numpy(target_pcd_points_np)
    height_map_pcd_target.fill(0)

    @ti.kernel
    def compute_height_map_pcd():
        for i in range(n_target_pcd_points):
            u, v = from_xy_to_uv(target_pcd_points[i][0], target_pcd_points[i][1])
            ti.atomic_max(height_map_pcd_target[u, v], target_pcd_points[i][2])

    compute_height_map_pcd()
    height_map_pcd = height_map_pcd_target.to_numpy()
    np.save(os.path.join(data_path, f'real_hm_{data_ind-1}.npy'),
            height_map_pcd)
    ti.reset()
    return height_map_pcd


# script_path = os.path.dirname(os.path.realpath(__file__))
# hm = show_hm(data_path=os.path.join(script_path, '..', '..', '..',
#                                     'test', 'deformable_objects', 'pcd_to_mesh'),
#         data_ind=0)
# import matplotlib.pyplot as plt
#
# plt.imshow(hm, cmap='YlOrBr', vmin=0, vmax=60)
# plt.xticks([])
# plt.yticks([])
# plt.savefig(os.path.join(script_path, '..', '..', '..',
#                          'test', 'deformable_objects', 'pcd_to_mesh',
#                          f'target_hm{0}.pdf'), bbox_inches='tight', pad_inches=0, dpi=300)
