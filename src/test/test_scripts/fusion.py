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

p1 = [0.205588227, -0.4726177, 0.2929954]
p1_q = [-0.32243624, -0.41420078, 0.748203, 0.40578757]
base_to_ee_1 = construct_homogeneous_transform_matrix(p1, p1_q)
base_to_cam_1 = base_to_ee_1 @ transform_ee_to_cam
pcd_1 = o3d.io.read_point_cloud(os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_0.ply')).transform(base_to_cam_1.copy()).crop(workspace_bounding_box)

p2 = [-0.1893279, -0.613106227, 0.3408583854]
p2_q = [-0.24049455, -0.08196321, 0.85442548, 0.453212345]
base_to_ee_2 = construct_homogeneous_transform_matrix(p2, p2_q)
base_to_cam_2 = np.matmul(base_to_ee_2, transform_ee_to_cam)
pcd_2 = o3d.io.read_point_cloud(os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_1.ply')).transform(base_to_cam_2.copy()).crop(workspace_bounding_box)

p3 = [-0.60128682, -0.22389851, 0.56897717]
p3_q = [0.0928390974, 0.1008101, 0.98776471, 0.0744250]
base_to_ee_3 = construct_homogeneous_transform_matrix(p3, p3_q)
base_to_cam_3 = np.matmul(base_to_ee_3, transform_ee_to_cam)
pcd_3 = o3d.io.read_point_cloud(os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_2.ply')).transform(base_to_cam_3.copy()).crop(workspace_bounding_box)

p4 = [-0.615176691, 0.01267901, 0.62168631]
p4_q = [0.08020656915, 0.0922129480, 0.99192285537, -0.03395198]
base_to_ee_4 = construct_homogeneous_transform_matrix(p4, p4_q)
base_to_cam_4 = np.matmul(base_to_ee_4, transform_ee_to_cam)
pcd_4 = o3d.io.read_point_cloud(os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_3.ply')).transform(base_to_cam_4.copy()).crop(workspace_bounding_box)

p5 = [-0.6412664, 0.2062521441, 0.32637888]
p5_q = [0.322494, 0.421453563, 0.828587, -0.17837222]
base_to_ee_5 = construct_homogeneous_transform_matrix(p5, p5_q)
base_to_cam_5 = np.matmul(base_to_ee_5, transform_ee_to_cam)
pcd_5 = o3d.io.read_point_cloud(os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_4.ply')).transform(base_to_cam_5.copy()).crop(workspace_bounding_box)

p6 = [-0.282081, 0.589129066, 0.533521235]
p6_q = [-0.1838169, 0.14804636, 0.89802867, -0.3712654]
base_to_ee_6 = construct_homogeneous_transform_matrix(p6, p6_q)
base_to_cam_6 = np.matmul(base_to_ee_6, transform_ee_to_cam)
pcd_6 = o3d.io.read_point_cloud(os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_5.ply')).transform(base_to_cam_6.copy()).crop(workspace_bounding_box)

p7 = [0.0341350440, 0.3919146, 0.3725248]
p7_q = [-0.092143585, 0.841706156, 0.31827776, -0.42630925]
base_to_ee_7 = construct_homogeneous_transform_matrix(p7, p7_q)
base_to_cam_7 = np.matmul(base_to_ee_7, transform_ee_to_cam)
pcd_7 = o3d.io.read_point_cloud(os.path.join(script_path, '..', 'objects', 'pcd_to_fuse_testing', 'pcd_6.ply')).transform(base_to_cam_7.copy()).crop(workspace_bounding_box)


fused = pcd_1 + pcd_2 + pcd_3 + pcd_4 + pcd_5 + pcd_6 + pcd_7
fused = fused.voxel_down_sample(voxel_size=0.0015)

o3d.visualization.draw_geometries([world_frame, fused])
