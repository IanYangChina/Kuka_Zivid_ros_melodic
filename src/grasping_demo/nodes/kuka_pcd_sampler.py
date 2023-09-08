#!/usr/bin/env python3

import os
import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseArray
import rosnode, roslaunch
from grasping_demo.srv import Sample, SampleResponse
from zivid_camera.srv import *
from sensor_msgs.msg import PointCloud2
import ros_numpy
import open3d as o3d

script_path = os.path.dirname(os.path.realpath(__file__))

waiting_pose = PoseStamped()
waiting_pose.pose.position.x = -0.5
waiting_pose.pose.position.y = 0.0
waiting_pose.pose.position.z = 0.55
# in euler: [-90, 180, 0]
waiting_pose.pose.orientation.w = -0.0594
waiting_pose.pose.orientation.x = -0.664
waiting_pose.pose.orientation.y = 0.745
waiting_pose.pose.orientation.z = 0.0054

capture_pose_1 = PoseStamped()
capture_pose_1.pose.position.x = -0.52358
capture_pose_1.pose.position.y = -0.01965
capture_pose_1.pose.position.z = 0.60888
capture_pose_1.pose.orientation.w = 0.16641
capture_pose_1.pose.orientation.x = -0.67152
capture_pose_1.pose.orientation.y = 0.69363
capture_pose_1.pose.orientation.z = -0.20056

capture_pose_2 = PoseStamped()
capture_pose_2.pose.position.x = -0.44896
capture_pose_2.pose.position.y = -0.32470
capture_pose_2.pose.position.z = 0.56047
capture_pose_2.pose.orientation.w = -0.29034
capture_pose_2.pose.orientation.x = 0.76764
capture_pose_2.pose.orientation.y = -0.57110
capture_pose_2.pose.orientation.z = 0.01619

capture_pose_3 = PoseStamped()
capture_pose_3.pose.position.x = -0.0334
capture_pose_3.pose.position.y = -0.41866
capture_pose_3.pose.position.z = 0.55560
capture_pose_3.pose.orientation.w = -0.09314
capture_pose_3.pose.orientation.x = 0.80210
capture_pose_3.pose.orientation.y = -0.51920
capture_pose_3.pose.orientation.z = -0.27996

capture_pose_4 = PoseStamped()
capture_pose_4.pose.position.x = -0.52880
capture_pose_4.pose.position.y = 0.14304
capture_pose_4.pose.position.z = 0.58465
capture_pose_4.pose.orientation.w = 0.07235
capture_pose_4.pose.orientation.x = -0.45595
capture_pose_4.pose.orientation.y = 0.84126
capture_pose_4.pose.orientation.z = -0.28131

capture_pose_5 = PoseStamped()
capture_pose_5.pose.position.x = -0.38582
capture_pose_5.pose.position.y = 0.32856
capture_pose_5.pose.position.z = 0.54296
capture_pose_5.pose.orientation.w = -0.06659
capture_pose_5.pose.orientation.x = -0.47187
capture_pose_5.pose.orientation.y = 0.81250
capture_pose_5.pose.orientation.z = -0.33574

capture_pose_6 = PoseStamped()
capture_pose_6.pose.position.x = -0.05036
capture_pose_6.pose.position.y = 0.51409
capture_pose_6.pose.position.z = 0.55562
capture_pose_6.pose.orientation.w = -0.17963
capture_pose_6.pose.orientation.x = 0.41851
capture_pose_6.pose.orientation.y = 0.79119
capture_pose_6.pose.orientation.z = -0.40829

DISTANCE_THRESHOLD = 0.001

workspace_bounding_box_array = np.load(
    os.path.join(script_path, '..', '..', 'test', 'test_scripts',
                 'transformation_matrices', 'reconstruction_bounding_box_array_in_base.npy'))
workspace_bounding_box_array = o3d.utility.Vector3dVector(workspace_bounding_box_array.astype('float'))
workspace_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points=workspace_bounding_box_array)
workspace_bounding_box.color = (0, 1, 0)


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


# todo: test velocity configuration on RosSmartServo
# https://github.com/IFL-CAMP/iiwa_stack/wiki/configuresmartservo
# https://github.com/IFL-CAMP/iiwa_stack/issues/78
class KukaPcdSampler:
    def __init__(self):
        rospy.init_node('kuka_pcd_sampler_node', anonymous=True)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
        self.pub_move_cmd = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=2)
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

        ROSSMartServo_on = False
        while not ROSSMartServo_on:
            rospy.loginfo('Please start the ROSSMartServo application on the Sunrise Cabinet')
            if '/iiwa/iiwa_subscriber' in rosnode.get_node_names():
                ROSSMartServo_on = True
            else:
                rospy.sleep(1)
        self.init_robot()

        self.pcd_list = []
        self.sample_service = rospy.Service('kuka_pcd_sample_service', Sample, self.sample)
        rospy.loginfo("Sampling service ready to be called...")
        self.pcd_saving_path = rospy.get_param('/kuka_pcd_sampler/kuka_pcd_sampler/pcd_saving_path')
        if self.pcd_saving_path == 'none':
            self.pcd_saving_path = os.path.join(script_path, '..', '..', 'test', 'objects', 'pcd_to_mesh')
        os.makedirs(self.pcd_saving_path, exist_ok=True)
        rospy.loginfo("Sampled pcd will be saved in "+str(self.pcd_saving_path))

    def init_robot(self):
        rospy.loginfo("Initializing robot...")
        self.publish_pose(waiting_pose)
        rospy.sleep(0.1)

    def sample(self, request):
        rospy.loginfo('Taking PCD samples for reconstruction...')
        self.pcd_list = []
        self.num_pcd_samples = 0
        self.publish_pose(waiting_pose)
        rospy.sleep(0.1)

        self.publish_pose(capture_pose_1)
        rospy.sleep(2)
        self.transform_base_to_ee = construct_homogeneous_transform_matrix(
            translation=self.current_xyz, orientation=self.current_quat)
        self.capture()
        while not self.num_pcd_samples == 1:
            rospy.sleep(0.1)

        self.publish_pose(capture_pose_2)
        rospy.sleep(2)
        self.transform_base_to_ee = construct_homogeneous_transform_matrix(
            translation=self.current_xyz, orientation=self.current_quat)
        self.capture()
        while not self.num_pcd_samples == 2:
            rospy.sleep(0.1)

        self.publish_pose(capture_pose_3)
        rospy.sleep(2)
        self.transform_base_to_ee = construct_homogeneous_transform_matrix(
            translation=self.current_xyz, orientation=self.current_quat)
        self.capture()
        while not self.num_pcd_samples == 3:
            rospy.sleep(0.1)

        self.publish_pose(waiting_pose)
        rospy.sleep(0.1)

        self.publish_pose(capture_pose_4)
        rospy.sleep(2)
        self.transform_base_to_ee = construct_homogeneous_transform_matrix(
            translation=self.current_xyz, orientation=self.current_quat)
        self.capture()
        while not self.num_pcd_samples == 4:
            rospy.sleep(0.1)

        self.publish_pose(capture_pose_5)
        rospy.sleep(2)
        self.transform_base_to_ee = construct_homogeneous_transform_matrix(
            translation=self.current_xyz, orientation=self.current_quat)
        self.capture()
        while not self.num_pcd_samples == 5:
            rospy.sleep(0.1)

        self.publish_pose(capture_pose_6)
        rospy.sleep(2)
        self.transform_base_to_ee = construct_homogeneous_transform_matrix(
            translation=self.current_xyz, orientation=self.current_quat)
        self.capture()
        while not self.num_pcd_samples == 6:
            rospy.sleep(0.1)

        self.publish_pose(waiting_pose)
        rospy.sleep(0.1)

        fused_pcd = self.pcd_list[0] + self.pcd_list[1] + self.pcd_list[2] + \
                    self.pcd_list[3] + self.pcd_list[4] + self.pcd_list[5]
        world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        o3d.visualization.draw_geometries([world_frame, fused_pcd, workspace_bounding_box])
        path_to_save_pcd = os.path.join(self.pcd_saving_path, 'pcd_0.ply')
        i = 0
        while True:
            if os.path.exists(path_to_save_pcd):
                i += 1
                path_to_save_pcd = os.path.join(self.pcd_saving_path, 'pcd_'+str(i)+'.ply')
            else:
                break
        o3d.io.write_point_cloud(path_to_save_pcd, fused_pcd)
        rospy.loginfo("Point cloud \'pcd_"+str(i)+".ply\' has been saved in "+path_to_save_pcd)
        return SampleResponse()

    def capture_assistant_suggest_settings(self):
        max_capture_time = rospy.Duration.from_sec(1.20)
        rospy.loginfo(
            "Calling capture assistant service with max capture time = %.2f sec",
            max_capture_time.to_sec(),
        )
        self.capture_assistant_service(
            max_capture_time=max_capture_time,
            ambient_light_frequency=CaptureAssistantSuggestSettingsRequest.AMBIENT_LIGHT_FREQUENCY_NONE,
        )

    def capture(self):
        rospy.loginfo("Calling capture service")
        self.capture_service()

    def on_points(self, data):
        rospy.loginfo("PointCloud received")
        cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        points = ros_numpy.point_cloud2.get_xyz_points(cloud_array, remove_nans=True)
        pcd = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(points))

        transform_base_to_cam = np.matmul(self.transform_base_to_ee.copy(), self.transform_ee_to_cam.copy())
        pcd_in_world_frame = pcd.transform(transform_base_to_cam.copy()).crop(workspace_bounding_box)
        self.pcd_list.append(pcd_in_world_frame)
        self.num_pcd_samples += 1

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
