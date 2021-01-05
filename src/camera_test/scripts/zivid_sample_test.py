#!/usr/bin/env python

import cv2
import pcl
import rospy
import ros_numpy
import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
from cv_bridge import CvBridge
from zivid_camera.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2

K = np.array([
    [2747.785993, 0.0, 1008.464956],
    [0.0, 2737.721711, 598.176104],
    [0.0, 0.0, 1.0]
])

d = np.array([-0.25515395, 0.0, 0.0, 0.0, 0.0])

translation_matrix = np.array([-0.526, 0.162, 1.090]).reshape((3, 1))   # xyz
rotation_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([0.035, 0.003, -0.995, 0.086]).reshape((4, 1)))  # wxyz

polygon = np.array([
    [-0.7, 0.2, 0.0],
    [-0.3, 0.2, 0.0],
    [-0.7, -0.3, 0.0],
    [-0.3, -0.3, 0.0]
])
polygon = o3d.geometry.PointCloud(
        points=o3d.utility.Vector3dVector(polygon.tolist())
    )
polygon.paint_uniform_color([1, 0, 0])


class Sample:
    def __init__(self):
        self.bridge = CvBridge()

        rospy.init_node("sample_capture_assistant_py", anonymous=True)

        rospy.loginfo("Starting sample_capture_assistant.py")

        ca_suggest_settings_service = "/zivid_camera/capture_assistant/suggest_settings"

        rospy.wait_for_service(ca_suggest_settings_service, 30.0)

        self.capture_assistant_service = rospy.ServiceProxy(
            ca_suggest_settings_service, CaptureAssistantSuggestSettings
        )
        self.capture_service = rospy.ServiceProxy("/zivid_camera/capture", Capture)

        rospy.Subscriber("/zivid_camera/points/xyzrgba", PointCloud2, self.on_points)
        rospy.Subscriber("/zivid_camera/color/image_color", Image, self.on_image_color)

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
        pcd_raw = o3d.geometry.PointCloud(
            points=o3d.utility.Vector3dVector(points)
        )
        # pcd_raw.paint_uniform_color([1, 0, 0])
        # pcd = o3d.geometry.PointCloud(
        #     points=o3d.utility.Vector3dVector(points)
        # )
        # pcd.paint_uniform_color([0, 1, 0])
        # print(np.asarray(pcd.points))
        # pcd.translate(translation=translation_matrix)
        # pcd.rotate(R=rotation_matrix)
        # print(np.asarray(pcd.points))
        # o3d.visualization.draw_geometries([pcd_raw, pcd])
        o3d.io.write_point_cloud("/home/xintong/Documents/PyProjects/Zivid_project/src/camera_test/objects/part_reference.ply", pcd_raw)
        # print('')

    def on_image_color(self, data):
        # cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        # h, w = cv_image.shape[:2]
        #
        # newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(K, d, (w, h), 0)
        # mapx, mapy = cv2.initUndistortRectifyMap(K, d, None, newcameramatrix, (w, h), 5)
        # newimg = cv2.remap(cv_image, mapx, mapy, cv2.INTER_LINEAR)
        #
        # fig, (oldimg_ax, newimg_ax) = plt.subplots(1, 2)
        # oldimg_ax.imshow(cv_image)
        # oldimg_ax.set_title('Original image')
        # newimg_ax.imshow(newimg)
        # newimg_ax.set_title('Unwarped image')
        # plt.show()

        rospy.loginfo("2D color image received")


if __name__ == "__main__":
    s = Sample()
    s.capture_assistant_suggest_settings()
    s.capture()
    rospy.spin()
