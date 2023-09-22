import os
import open3d as o3d
import numpy as np


def construct_homogeneous_transform_matrix(translation, orientation):
    translation = np.array(translation).reshape((3, 1))  # xyz
    if len(orientation) == 4:
        rotation = o3d.geometry.get_rotation_matrix_from_quaternion(np.array(orientation).reshape((4, 1)))  # wxyz
    else:
        assert len(orientation) == 3, 'orientation should be a quaternion or 3 axis angles'
        rotation = np.radians(np.array(orientation).astype("float")).reshape((3, 1))  # ABC in radians
        rotation = o3d.geometry.get_rotation_matrix_from_zyx(rotation)
    transformation = np.append(rotation, translation, axis=1)
    transformation = np.append(transformation, np.array([[0, 0, 0, 1]]), axis=0)
    return transformation.copy()


script_path = os.path.dirname(os.path.realpath(__file__))
transform_ee_to_cam = np.load(os.path.join(script_path, '..', '..', 'easy_handeye_calibration',
                                                        'results_eye_on_hand',
                                                        'transform_ee_to_cam.npy'))
transform_cam_to_ee = np.linalg.inv(transform_ee_to_cam)


workspace_bounding_box_array = np.load(
            os.path.join(script_path, '..', '..', 'test', 'test_scripts',
                         'transformation_matrices', 'reconstruction_bounding_box_array_in_base.npy'))
workspace_bounding_box_array = o3d.utility.Vector3dVector(workspace_bounding_box_array.astype('float'))
workspace_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(
    points=workspace_bounding_box_array)
workspace_bounding_box.color = (0, 1, 0)
world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

p1 = [-0.04734622258766853, -0.4919436287843519, 0.43001648103150053]
p1_q = [0.012216610382942398, 0.7293590307235718, -0.5428170215254676, -0.4162160513421742]
base_to_ee_1 = construct_homogeneous_transform_matrix(p1, p1_q)
ee_1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0]).transform(base_to_ee_1.copy())
base_to_cam_1 = base_to_ee_1 @ transform_ee_to_cam
cam_1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0]).transform(base_to_cam_1.copy())
path_1 = os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_0.ply')
pcd_1 = o3d.io.read_point_cloud(path_1).voxel_down_sample(voxel_size=0.002)
o3d.io.write_point_cloud(path_1, pcd_1)
pcd_1 = pcd_1.transform(base_to_cam_1.copy()).crop(workspace_bounding_box).paint_uniform_color([1, 0, 0])

p2 = [-0.33955532155200313, -0.4919240673409368, 0.44322820096677445]
p2_q = [-0.09543869118016003, -0.34123433590221613, 0.8551217913627625, 0.3784405799265335]
base_to_ee_2 = construct_homogeneous_transform_matrix(p2, p2_q)
ee_2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0]).transform(base_to_ee_2.copy())
base_to_cam_2 = np.matmul(base_to_ee_2, transform_ee_to_cam)
cam_2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0]).transform(base_to_cam_2.copy())
path_2 = os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_1.ply')
pcd_2 = o3d.io.read_point_cloud(path_2).voxel_down_sample(voxel_size=0.002)
o3d.io.write_point_cloud(path_2, pcd_2)
pcd_2 = pcd_2.transform(base_to_cam_2.copy()).crop(workspace_bounding_box).paint_uniform_color([0, 1, 0])

p3 = [-0.6118599785082319, -0.2651555369743888, 0.4384940804041449]
p3_q = [0.06499232341504475, -0.15117182322076156, 0.9701617360115051, 0.17807115250318148]
base_to_ee_3 = construct_homogeneous_transform_matrix(p3, p3_q)
ee_3 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0]).transform(base_to_ee_3.copy())
base_to_cam_3 = np.matmul(base_to_ee_3, transform_ee_to_cam)
cam_3 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0]).transform(base_to_cam_3.copy())
path_3 = os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_2.ply')
pcd_3 = o3d.io.read_point_cloud(path_3).voxel_down_sample(voxel_size=0.002)
o3d.io.write_point_cloud(path_3, pcd_3)
pcd_3 = pcd_3.transform(base_to_cam_3.copy()).crop(workspace_bounding_box).paint_uniform_color([0, 0, 1])

p4 = [-0.6238490498687387, 0.06191429843213932, 0.6677410608636462]
p4_q = [0.019115162028591103, 0.013566951526794428, 0.9967168569564819, -0.07749866150190958]
base_to_ee_4 = construct_homogeneous_transform_matrix(p4, p4_q)
ee_4 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0]).transform(base_to_ee_4.copy())
base_to_cam_4 = np.matmul(base_to_ee_4, transform_ee_to_cam)
cam_4 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0]).transform(base_to_cam_4.copy())
path_4 = os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_3.ply')
pcd_4 = o3d.io.read_point_cloud(path_4).voxel_down_sample(voxel_size=0.002)
o3d.io.write_point_cloud(path_4, pcd_4)
pcd_4 = pcd_4.transform(base_to_cam_4.copy()).crop(workspace_bounding_box).paint_uniform_color([1, 1, 0])

p5 = [-0.6180526689359269, 0.3675919243464007, 0.30419067214329537]
p5_q = [0.04657256733135922, -0.053342850612119526, 0.8997739553451538, -0.43057209076882225]
base_to_ee_5 = construct_homogeneous_transform_matrix(p5, p5_q)
ee_5 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0]).transform(base_to_ee_5.copy())
base_to_cam_5 = np.matmul(base_to_ee_5, transform_ee_to_cam)
cam_5 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0]).transform(base_to_cam_5.copy())
path_5 = os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_4.ply')
pcd_5 = o3d.io.read_point_cloud(path_5).voxel_down_sample(voxel_size=0.002)
o3d.io.write_point_cloud(path_5, pcd_5)
pcd_5 = pcd_5.transform(base_to_cam_5.copy()).crop(workspace_bounding_box).paint_uniform_color([1, 0, 1])

p6 = [-0.2977190125241298, 0.4572420263707471, 0.2795479430701567]
p6_q = [-0.14926434756769433, 0.25636200667857995, 0.819904088973999, -0.4896488070420422]
base_to_ee_6 = construct_homogeneous_transform_matrix(p6, p6_q)
ee_6 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0]).transform(base_to_ee_6.copy())
base_to_cam_6 = np.matmul(base_to_ee_6, transform_ee_to_cam)
cam_6 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0]).transform(base_to_cam_6.copy())
path_6 = os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_5.ply')
pcd_6 = o3d.io.read_point_cloud(path_6).voxel_down_sample(voxel_size=0.002)
o3d.io.write_point_cloud(path_6, pcd_6)
pcd_6 = pcd_6.transform(base_to_cam_6.copy()).crop(workspace_bounding_box).paint_uniform_color([0, 1, 1])

p7 = [-0.10247700473305638, 0.4572632022505956, 0.31452236331668904]
p7_q = [0.10310314660490129, 0.8379407525062561, 0.2704636279176586, -0.4626818098029889]
base_to_ee_7 = construct_homogeneous_transform_matrix(p7, p7_q)
ee_7 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0]).transform(base_to_ee_7.copy())
base_to_cam_7 = np.matmul(base_to_ee_7, transform_ee_to_cam)
cam_7 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0]).transform(base_to_cam_7.copy())
path_7 = os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_6.ply')
pcd_7 = o3d.io.read_point_cloud(path_7).voxel_down_sample(voxel_size=0.002)
o3d.io.write_point_cloud(path_7, pcd_7)
pcd_7 = pcd_7.transform(base_to_cam_7.copy()).crop(workspace_bounding_box).paint_uniform_color([0.5, 0.5, 0.5])

p8 = [-0.31217047814232096, 0.29355731896807297, 0.5762597186817668]
p8_q = [0.09861816149797507, 0.9411432147026062, 0.27879016182263155, -0.16370694892710425]
base_to_ee_8 = construct_homogeneous_transform_matrix(p8, p8_q)
ee_8 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0]).transform(base_to_ee_8.copy())
base_to_cam_8 = np.matmul(base_to_ee_8, transform_ee_to_cam)
cam_8 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0]).transform(base_to_cam_8.copy())
path_8 = os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_7.ply')
pcd_8 = o3d.io.read_point_cloud(path_8).voxel_down_sample(voxel_size=0.002)
o3d.io.write_point_cloud(path_8, pcd_8)
pcd_8 = pcd_8.transform(base_to_cam_8.copy()).crop(workspace_bounding_box).paint_uniform_color([0.5, 1, 0.5])

fused = pcd_1 + pcd_2 + pcd_3 + pcd_4 + pcd_5 + pcd_6 + pcd_7 + pcd_8
fused = fused.voxel_down_sample(voxel_size=0.0015)

o3d.visualization.draw_geometries([
    world_frame, fused, workspace_bounding_box,
    ee_1,
    cam_1,
    # ee_2,
    # cam_2,
    ee_3,
    cam_3,
    # ee_4,
    # cam_4,
    ee_5,
    cam_5,
    # ee_6,
    # cam_6,
    # ee_7,
    # cam_7,
    ee_8,
    cam_8,
])
