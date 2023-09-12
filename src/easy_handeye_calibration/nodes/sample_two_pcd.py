#!/usr/bin/env python2

import os
import rospy
import numpy as np
import ros_numpy
import open3d as o3d
from zivid_camera.srv import *
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseArray


waiting_pose = PoseStamped()
waiting_pose.pose.position.x = -0.47344
waiting_pose.pose.position.y = -0.02111
waiting_pose.pose.position.z = 0.60625
waiting_pose.pose.orientation.w = -0.01690
waiting_pose.pose.orientation.x = 0.05149
waiting_pose.pose.orientation.y = 0.99749
waiting_pose.pose.orientation.z = -0.04539

capture_pose_1 = PoseStamped()
capture_pose_1.pose.position.x = -0.47544
capture_pose_1.pose.position.y = -0.16066
capture_pose_1.pose.position.z = 0.70063
capture_pose_1.pose.orientation.w = -0.30597
capture_pose_1.pose.orientation.x = -0.34994
capture_pose_1.pose.orientation.y = 0.85375
capture_pose_1.pose.orientation.z = 0.23455

capture_pose_2 = PoseStamped()
capture_pose_2.pose.position.x = -0.4056036
capture_pose_2.pose.position.y = -0.02022997
capture_pose_2.pose.position.z = 0.8205197
capture_pose_2.pose.orientation.w = -0.38359
capture_pose_2.pose.orientation.x = -0.078744
capture_pose_2.pose.orientation.y = 0.91777
capture_pose_2.pose.orientation.z = -0.06585

capture_pose_3 = PoseStamped()
capture_pose_3.pose.position.x = -0.39813
capture_pose_3.pose.position.y = 0.21598803
capture_pose_3.pose.position.z = 0.800553
capture_pose_3.pose.orientation.w = -0.33227530
capture_pose_3.pose.orientation.x = 0.4818123
capture_pose_3.pose.orientation.y = 0.692790448
capture_pose_3.pose.orientation.z = -0.42129724

DISTANCE_THRESHOLD = 0.001
script_path = os.path.dirname(os.path.realpath(__file__))


class TwoPCDSampler:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
        self.pub_move_cmd = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=2)
        self.pub_attempt_finished = rospy.Publisher('AttemptFinished', Bool, queue_size=2)
        self.current_pose_msg = PoseStamped()
        self.current_xyz = np.array([0.0, 0.0, 0.0])
        self.current_quat = np.array([0.0, 0.0, 0.0, 0.0])
        self.current_header_seq = 0

        rospy.Subscriber("/zivid_camera/points/xyzrgba", PointCloud2, self.on_points, queue_size=2)
        ca_suggest_settings_service = "/zivid_camera/capture_assistant/suggest_settings"
        rospy.wait_for_service(ca_suggest_settings_service)
        self.capture_assistant_service = rospy.ServiceProxy(
            ca_suggest_settings_service, CaptureAssistantSuggestSettings
        )
        self.capture_service = rospy.ServiceProxy("/zivid_camera/capture", Capture)
        self.capture_assistant_suggest_settings()

        self.move_robot()

    def move_robot(self):
        rospy.loginfo("Initializing robot...")
        self.publish_pose(waiting_pose)
        self.publish_pose(capture_pose_1)
        rospy.loginfo("Calling capture service")
        rospy.loginfo("Capture PCD 1 at pose (xyz wabc): \n{}, {}".format(self.current_xyz, self.current_quat))
        self.capture_service()
        rospy.sleep(3)

        self.publish_pose(capture_pose_2)
        rospy.loginfo("Calling capture service")
        rospy.loginfo("Capture PCD 2 at pose (xyz wabc): \n{}, {}".format(self.current_xyz, self.current_quat))
        self.capture_service()
        rospy.sleep(3)

        self.publish_pose(capture_pose_3)
        rospy.loginfo("Calling capture service")
        rospy.loginfo("Capture PCD 3 at pose (xyz wabc): \n{}, {}".format(self.current_xyz, self.current_quat))
        self.capture_service()
        rospy.sleep(3)

        self.publish_pose(waiting_pose)

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

    def on_points(self, data):
        rospy.loginfo("PointCloud received")
        cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        points = ros_numpy.point_cloud2.get_xyz_points(cloud_array, remove_nans=True)
        pcd_raw = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(points))
        raw_cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        o3d.visualization.draw_geometries([raw_cam_frame, pcd_raw])
        path_to_save_pcd = os.path.join(script_path, '..', 'results_eye_on_hand', 'pcd_reference_0.ply')
        i = 0
        while True:
            if os.path.exists(path_to_save_pcd):
                i += 1
                path_to_save_pcd = os.path.join(script_path, '..', 'results_eye_on_hand', 'pcd_reference_'+str(i)+'.ply')
            else:
                break
        o3d.io.write_point_cloud(path_to_save_pcd, pcd_raw)
        rospy.loginfo("Point cloud \'pcd_reference.ply\' has been saved in ../src/")


if __name__ == '__main__':
    sampler = TwoPCDSampler()
    rospy.spin()
