#!/usr/bin/env python3
import os
import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseArray
import rosnode, roslaunch
from grasping_demo.srv import Sample, SampleResponse
from grasping_demo.srv import TrajectoryThree, TrajectoryThreeResponse
from grasping_demo.srv import TrajectoryFour, TrajectoryFourResponse
from zivid_camera.srv import *
from sensor_msgs.msg import PointCloud2
import ros_numpy
import open3d as o3d
from utils.kuka_poses import *
from copy import deepcopy as dcp
from pymeshfix import _meshfix
import pyvista as pv
import trimesh

script_path = os.path.dirname(os.path.realpath(__file__))

DISTANCE_THRESHOLD = 0.001


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


class KukaPcdSampler:
    def __init__(self, robot_name='iiwa'):
        rospy.init_node('kuka_pcd_sampler_node', anonymous=True)
        rospy.Subscriber(f'/{robot_name}/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
        self.pub_move_cmd = rospy.Publisher(f'/{robot_name}/command/CartesianPose', PoseStamped, queue_size=2)
        self.pub_attempt_finished = rospy.Publisher('AttemptFinished', Bool, queue_size=2)
        self.current_pose_msg = PoseStamped()
        self.current_xyz = np.array([0.0, 0.0, 0.0])
        self.current_quat = np.array([0.0, 0.0, 0.0, 0.0])
        self.current_header_seq = 0

        rospy.loginfo("Checking if camera is launched...")
        camera_launched = False
        while not camera_launched:
            if '/zivid_camera/zivid_camera' in rosnode.get_node_names():
                camera_launched = True
            else:
                rospy.sleep(1)
        rospy.Subscriber("/zivid_camera/points/xyzrgba", PointCloud2, self.on_points, queue_size=2)
        ca_suggest_settings_service = "/zivid_camera/capture_assistant/suggest_settings"
        rospy.wait_for_service(ca_suggest_settings_service)
        self.capture_assistant_service = rospy.ServiceProxy(
            ca_suggest_settings_service, CaptureAssistantSuggestSettings
        )
        self.capture_service = rospy.ServiceProxy("/zivid_camera/capture", Capture)
        self.capture_assistant_suggest_settings()
        self.num_pcd_samples = 0

        self.transform_ee_to_cam = np.load(os.path.join(script_path, '..', '..', 'easy_handeye_calibration',
                                                        'results_eye_on_hand',
                                                        'transform_ee_to_cam.npy'))
        self.transform_base_to_ee = None

        self.transform_base_to_ee_1 = None
        self.transform_base_to_ee_2 = None
        self.transform_base_to_ee_3 = None
        self.transform_base_to_ee_4 = None
        self.transform_base_to_ee_5 = None
        self.transform_base_to_ee_6 = None
        self.transform_ee_1_to_ees = []
        self.pcd_nudges = [
            (0.0, 0.003, 0.001),
            (0.0, 0.0, -0.001),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, -0.002, 0.001),
            (-0.005, 0.004, 0.0)
        ]

        self.workspace_bounding_box = None
        self.workspace_bounding_box_array = np.load(
            os.path.join(script_path, '..', '..', 'test', 'test_scripts',
                         'transformation_matrices', 'reconstruction_bounding_box_array_in_base.npy'))
        self.make_bounding_box()

        self.original_pcd_list = []
        self.pcd_list = []
        pcd_saving_path = str(rospy.get_param('/iiwa/kuka_pcd_sampler/pcd_saving_path'))
        if pcd_saving_path == 'none':
            self.pcd_saving_path = os.path.join(script_path, '..', '..', 'test',
                                                'deformable_objects', 'pcd_to_mesh')
        else:
            self.pcd_saving_path = pcd_saving_path + '/pcd_to_mesh'
        self.save_original_pcd = rospy.get_param('/iiwa/kuka_pcd_sampler/save_original_pcd')
        os.makedirs(self.pcd_saving_path, exist_ok=True)
        rospy.loginfo("Fused pcd will be saved in " + str(self.pcd_saving_path))
        if self.save_original_pcd:
            self.original_pcd_saving_path = self.pcd_saving_path.replace('/pcd_to_mesh', '/pcd_to_fuse_testing')
            os.makedirs(self.original_pcd_saving_path, exist_ok=True)
            rospy.loginfo("Original pcd will be saved in " + str(self.original_pcd_saving_path))

        self.check_rosapp_on()
        self.init_robot()

        self.tr1_service = rospy.ServiceProxy('trajectory_3', TrajectoryThree)
        self.tr2_service = rospy.ServiceProxy('trajectory_4', TrajectoryFour)

        self.sample_service = rospy.Service('kuka_pcd_sample_service', Sample, self.sample)
        rospy.loginfo("Sampling service ready to be called...")

    def check_rosapp_on(self):
        ROSSMartServo_on = False
        while not ROSSMartServo_on:
            rospy.loginfo('Please start the ROSSMartServo application on the Sunrise Cabinet')
            if '/iiwa/iiwa_subscriber' in rosnode.get_node_names():
                ROSSMartServo_on = True
            else:
                rospy.sleep(1)

    def init_robot(self):
        rospy.loginfo("Initializing robot...")
        self.publish_pose(waiting_pose)
        rospy.sleep(0.1)

    def make_bounding_box(self):
        self.workspace_bounding_box_array = np.load(
            os.path.join(script_path, '..', '..', 'test', 'test_scripts',
                         'transformation_matrices', 'reconstruction_bounding_box_array_in_base.npy'))
        workspace_bounding_box_array = o3d.utility.Vector3dVector(self.workspace_bounding_box_array.astype('float'))
        self.workspace_bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
            points=workspace_bounding_box_array)
        self.workspace_bounding_box.color = (0, 1, 0)

    def sample(self, req):
        rospy.loginfo('Taking PCD samples for reconstruction...')
        self.pcd_list = []
        self.num_pcd_samples = 0
        self.make_bounding_box()
        self.publish_pose(waiting_pose)
        rospy.sleep(0.2)

        self.publish_pose(capture_pose_1)
        rospy.sleep(0.5)
        self.transform_base_to_ee = construct_homogeneous_transform_matrix(
            translation=self.current_xyz, orientation=self.current_quat)
        self.transform_base_to_ee_1 = self.transform_base_to_ee.copy()
        self.capture()
        while not self.num_pcd_samples == 1:
            rospy.sleep(0.2)

        self.publish_pose(capture_pose_2)
        rospy.sleep(0.5)
        self.transform_base_to_ee = construct_homogeneous_transform_matrix(
            translation=self.current_xyz, orientation=self.current_quat)
        self.transform_base_to_ee_2 = self.transform_base_to_ee.copy()
        self.transform_ee_1_to_ees.append(np.matmul(np.linalg.inv(self.transform_base_to_ee_1),
                                                    self.transform_base_to_ee_2))
        self.capture()
        while not self.num_pcd_samples == 2:
            rospy.sleep(0.2)

        self.publish_pose(capture_pose_3)
        rospy.sleep(0.5)
        self.transform_base_to_ee = construct_homogeneous_transform_matrix(
            translation=self.current_xyz, orientation=self.current_quat)
        self.transform_base_to_ee_3 = self.transform_base_to_ee.copy()
        self.transform_ee_1_to_ees.append(np.matmul(np.linalg.inv(self.transform_base_to_ee_1),
                                                self.transform_base_to_ee_3))
        self.capture()
        while not self.num_pcd_samples == 3:
            rospy.sleep(0.2)

        self.publish_pose(capture_pose_4)
        rospy.sleep(0.5)
        self.transform_base_to_ee = construct_homogeneous_transform_matrix(
            translation=self.current_xyz, orientation=self.current_quat)
        self.transform_base_to_ee_4 = self.transform_base_to_ee.copy()
        self.transform_ee_1_to_ees.append(np.matmul(np.linalg.inv(self.transform_base_to_ee_1),
                                                self.transform_base_to_ee_4))
        self.capture()
        while not self.num_pcd_samples == 4:
            rospy.sleep(0.2)

        self.publish_pose(capture_pose_5)
        rospy.sleep(0.5)
        self.transform_base_to_ee = construct_homogeneous_transform_matrix(
            translation=self.current_xyz, orientation=self.current_quat)
        self.transform_base_to_ee_5 = self.transform_base_to_ee.copy()
        self.transform_ee_1_to_ees.append(np.matmul(np.linalg.inv(self.transform_base_to_ee_1),
                                                self.transform_base_to_ee_5))
        self.capture()
        while not self.num_pcd_samples == 5:
            rospy.sleep(0.2)

        self.publish_pose(capture_pose_6)
        rospy.sleep(0.5)
        self.transform_base_to_ee = construct_homogeneous_transform_matrix(
            translation=self.current_xyz, orientation=self.current_quat)
        self.transform_base_to_ee_6 = self.transform_base_to_ee.copy()
        self.transform_ee_1_to_ees.append(np.matmul(np.linalg.inv(self.transform_base_to_ee_1),
                                                self.transform_base_to_ee_6))
        self.capture()
        while not self.num_pcd_samples == 6:
            rospy.sleep(0.2)

        self.publish_pose(waiting_pose)
        rospy.sleep(0.1)

        if self.save_original_pcd:
            i = 0
            for original_pcd in self.original_pcd_list:
                path_to_save_pcd = os.path.join(self.original_pcd_saving_path, 'pcd_' + str(i) + '.ply')
                o3d.io.write_point_cloud(path_to_save_pcd, original_pcd)
                i += 1

        fused_pcd = self.pcd_list[0] + self.pcd_list[1] + self.pcd_list[2] + \
                    self.pcd_list[3] + self.pcd_list[4] + self.pcd_list[5]
        fused_pcd = fused_pcd.voxel_down_sample(voxel_size=0.0001)
        fused_pcd.translate([-0.004, 0, 0])
        o3d.visualization.draw_geometries([fused_pcd, self.workspace_bounding_box],
                                          width=800, height=800)
        path_to_save_pcd = os.path.join(self.pcd_saving_path, 'pcd_0.ply')
        i = 0
        while True:
            if os.path.exists(path_to_save_pcd):
                i += 1
                path_to_save_pcd = os.path.join(self.pcd_saving_path, 'pcd_' + str(i) + '.ply')
            else:
                break
        o3d.io.write_point_cloud(path_to_save_pcd, fused_pcd)
        rospy.loginfo("Point cloud \'pcd_" + str(i) + ".ply\' has been saved in " + path_to_save_pcd)
        rospy.loginfo("Generating mesh")
        self.pcd_to_mesh(i)

        self.find_best_action_mpm(i, target_ind=1)
        return SampleResponse()

    def find_best_action_mpm(self, pcd_id ,target_ind):
        from utils.simulation import find_best_action, simulate
        from utils.generate_hm import show_hm
        show_hm(self.pcd_saving_path, pcd_id)

        best_action, _, _, _ = find_best_action(pcd_id, target_ind)
        eef_target_xyz = np.load(os.path.join(self.pcd_saving_path,
                                              'pcd_' + str(pcd_id) + '_end_effector_target.npy'))
        eef_target_xyz += np.array([best_action[1] * 0.02, 0, 0])
        sim = input("Simulate the best action? [Y/N")
        if sim in ['Y', 'y']:
            simulate(best_action, pcd_id, target_ind)
        print("Make sure you start ROSSmartServo and restart moveit node")
        input("Press enter to continue...")
        self.move_to_target(eef_target_xyz)
        if best_action[0] == 1:
            self.tr1_service()
        else:
            self.tr2_service()

    def capture_assistant_suggest_settings(self):
        max_capture_time = rospy.Duration.from_sec(1)
        rospy.loginfo(
            "Calling capture assistant service with max capture time = %.2f sec",
            max_capture_time.to_sec(),
        )
        self.capture_assistant_service(
            max_capture_time=max_capture_time)

    def capture(self):
        rospy.loginfo("Calling capture service")
        self.capture_assistant_suggest_settings()
        self.capture_service()

    def on_points(self, data):
        rospy.loginfo("PointCloud received")
        cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        points = ros_numpy.point_cloud2.get_xyz_points(cloud_array, remove_nans=True)
        pcd = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(points)).voxel_down_sample(voxel_size=0.0001)
        self.original_pcd_list.append(dcp(pcd))
        if self.num_pcd_samples > 0:
            transform_ee_1_to_cam_n = np.matmul(self.transform_ee_1_to_ees[self.num_pcd_samples - 1],
                                                self.transform_ee_to_cam)
            transform_base_to_cam_n = np.matmul(self.transform_base_to_ee_1,
                                                transform_ee_1_to_cam_n)
        else:
            transform_base_to_cam_n = np.matmul(self.transform_base_to_ee_1,
                                                self.transform_ee_to_cam)
        pcd_in_world_frame = pcd.transform(transform_base_to_cam_n.copy())
        pcd_in_world_frame = pcd_in_world_frame.crop(self.workspace_bounding_box)
        pcd_in_world_frame.translate(np.asarray(self.pcd_nudges[self.num_pcd_samples]))
        pcd_in_world_frame.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.001, max_nn=30))
        pcd_in_world_frame.orient_normals_towards_camera_location(camera_location=transform_base_to_cam_n.copy()[:-1, -1])
        # o3d.visualization.draw_geometries([pcd_in_world_frame, self.workspace_bounding_box])
        self.pcd_list.append(pcd_in_world_frame.paint_uniform_color([1.0,
                                                                     1.0/(self.num_pcd_samples+1),
                                                                     1.0/(self.num_pcd_samples+1)]))
        self.num_pcd_samples += 1

    def pcd_to_mesh(self, pcd_id):
        pcd = o3d.io.read_point_cloud(os.path.join(self.pcd_saving_path, 'pcd_'+str(pcd_id)+'.ply'))
        original_pcd = dcp(pcd)
        centre = np.asarray(pcd.points).mean(0)
        pcd = pcd.voxel_down_sample(voxel_size=0.002)
        _, ind = pcd.remove_radius_outlier(nb_points=8, radius=0.003)
        outliner = pcd.select_by_index(ind, invert=True).paint_uniform_color([1, 0, 0])
        pcd = pcd.select_by_index(ind).paint_uniform_color([0, 0.5, 0.5])

        new_points = np.asarray(pcd.points).copy()
        new_points[:, -1] = np.min(self.workspace_bounding_box_array[:, -1])
        addition_pcd_vec = o3d.utility.Vector3dVector(new_points)
        addition_pcd = o3d.geometry.PointCloud(addition_pcd_vec)
        addition_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.001, max_nn=30))
        addition_pcd.orient_normals_towards_camera_location(camera_location=[centre[0], centre[1], -0.1])
        pcd = pcd + addition_pcd

        radii = [0.004]
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
        o3d.visualization.draw_geometries([mesh, pcd, outliner, self.workspace_bounding_box],
                                          width=800, height=800,
                                          mesh_show_back_face=True,
                                          mesh_show_wireframe=True)

        mesh_path = os.path.join(self.pcd_saving_path, 'mesh.ply')
        o3d.io.write_triangle_mesh(mesh_path, mesh)
        mesh = pv.read(mesh_path)
        os.remove(mesh_path)
        mesh.save(mesh_path)

        mesh_to_fix = _meshfix.PyTMesh()
        mesh_to_fix.load_file(mesh_path)
        os.remove(mesh_path)
        mesh_to_fix.join_closest_components()
        mesh_to_fix.fill_small_boundaries()
        repaired_mesh_path = os.path.join(self.pcd_saving_path,
                                          'mesh_' + str(pcd_id) + '0_repaired.ply')
        mesh_to_fix.save_file(repaired_mesh_path)

        points, faces = mesh_to_fix.return_arrays()
        mesh_centre = (points.max(0) + points.min(0)) / 2
        np.save(os.path.join(self.pcd_saving_path, 'mesh_' + str(pcd_id) + '0_repaired_centre.npy'), mesh_centre)
        mesh_top_centre = mesh_centre.copy()
        mesh_top_centre[-1] = points.max(0)[-1]
        end_effector_target_xyz = mesh_top_centre.copy()
        end_effector_target_xyz[0] += 0.011  # real robot eef link offset = 0.008 m
        end_effector_target_xyz[-1] += 0.094  # real robot eef link offset = 0.0935 m
        print(f"Real end effector frame location: {end_effector_target_xyz}")
        np.save(os.path.join(self.pcd_saving_path,
                             'pcd_' + str(pcd_id) + '_end_effector_target.npy'), end_effector_target_xyz)

        repaired_mesh_0_to_normalised = trimesh.load(repaired_mesh_path, force='mesh', skip_texture=True)
        repaired_mesh_0_to_normalised.vertices -= mesh_centre
        normalised_mesh_path = os.path.join(self.pcd_saving_path,
                                            'mesh_' + str(pcd_id) + '0_repaired_normalised.obj')
        repaired_mesh_0_to_normalised.export(normalised_mesh_path, file_type='obj')
        normalised_mesh_top_centre = mesh_top_centre.copy() - mesh_centre
        np.save((os.path.join(self.pcd_saving_path,
                              'mesh_' + str(pcd_id)) + '0_repaired_normalised_centre_top.npy'),
                normalised_mesh_top_centre)

    def move_to_target(self, xyz):
        p = dcp(waiting_pose)
        p.pose.position.x = xyz[0]
        p.pose.position.y = xyz[1]
        p.pose.position.z = xyz[2] + 0.01
        self.publish_pose(p)

        p.pose.position.z = xyz[2]
        self.publish_pose(p)

    def current_pose_callback(self, data):
        self.current_pose_msg = data
        self.current_xyz = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z
        ])
        self.current_quat = np.array([
            data.pose.orientation.w,
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z
        ])
        self.current_header_seq = data.header.seq

    def publish_pose(self, data):
        # record target xyz for distance tracking
        target_xyz = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z
        ])
        data.header.seq = self.current_header_seq
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = 'iiwa_link_0'
        rospy.sleep(0.5)
        self.pub_move_cmd.publish(data)
        done = False
        while not done:
            d = np.sqrt(np.sum(np.square(self.current_xyz - target_xyz)))
            if d < DISTANCE_THRESHOLD:
                rospy.loginfo("Movement finished")
                done = True


if __name__ == '__main__':
    sampler = KukaPcdSampler()
    rospy.spin()
