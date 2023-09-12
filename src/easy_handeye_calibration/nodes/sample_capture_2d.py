#!/usr/bin/env python2

import os
import cv2
import yaml
import rospy
import numpy as np
import dynamic_reconfigure.client
from zivid_camera.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


script_path = os.path.dirname(os.path.realpath(__file__))
if not os.path.exists(os.path.join(script_path, '..', 'src', 'Intrinsics.yml')):
    raise OSError("\nIntrinsics file not found: {}\n"
                  "Please make sure you have finished intrinsic calibration and copy the .yaml file into .../easy_handeye_calibration/src/\n"
                  "Zivid camera has an official C++ API to obtain the intrinsics,"
                  "see https://github.com/zivid/zivid-cpp-samples/blob/master/source/Camera/InfoUtilOther/GetCameraIntrinsics/GetCameraIntrinsics.cpp"
                  .format(os.path.join(script_path, '..', 'src', 'intrinsics.yml')))

with open(os.path.join(script_path, '..', 'src', 'Intrinsics.yml')) as intrinsics_file:
    intrinsics = yaml.load(intrinsics_file)

intrinsics = intrinsics['CameraIntrinsics']
# zivid build-in camera matrix
K = np.array([
    [intrinsics['CameraMatrix']['FX'], 0.0, intrinsics['CameraMatrix']['CX']],
    [0.0, intrinsics['CameraMatrix']['FY'], intrinsics['CameraMatrix']['CY']],
    [0.0, 0.0, 1.0]
])

d = np.array([intrinsics['Distortion']['K1'],
              intrinsics['Distortion']['K2'],
              intrinsics['Distortion']['P1'],
              intrinsics['Distortion']['P2'],
              intrinsics['Distortion']['K3']])


class Sample:
    def __init__(self):
        self.bridge = CvBridge()

        rospy.init_node("sample_capture_2d_py", anonymous=True)
        rospy.loginfo("Starting sample_capture_2d.py")
        rospy.wait_for_service("/zivid_camera/capture_2d", 30.0)
        rospy.Subscriber("/zivid_camera/color/image_color", Image, self.on_image_color)

        self.image_pub = rospy.Publisher("/zivid_camera/color/undistorted_image_color", Image, queue_size=10)
        self.capture_2d_service = rospy.ServiceProxy(
            "/zivid_camera/capture_2d", Capture2D
        )

        rospy.loginfo("Configuring 2D settings")
        acquisition_0_client = dynamic_reconfigure.client.Client(
            "/zivid_camera/settings_2d/acquisition_0"
        )
        acquisition_0_config = {
            "enabled": True,
            "aperture": 2.83,
            "exposure_time": 10000,
            "brightness": 1.0,
        }
        acquisition_0_client.update_configuration(acquisition_0_config)

        self.delay = 0.2

    def capture(self):
        rospy.loginfo("Calling capture service")
        self.capture_2d_service()

    def on_image_color(self, data):
        rospy.loginfo("Color image received")
        cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        h, w = cv_image.shape[:2]

        newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(K, d, (w, h), 0)
        mapx, mapy = cv2.initUndistortRectifyMap(K, d, None, newcameramatrix, (w, h), 5)
        newimg = cv2.remap(cv_image, mapx, mapy, cv2.INTER_LINEAR)

        rospy.loginfo("Publish undistorted image")
        img_msg = self.bridge.cv2_to_imgmsg(newimg, "rgb8")
        img_msg.header = data.header
        self.image_pub.publish(img_msg)

        self.capture()
        rospy.sleep(self.delay)


if __name__ == "__main__":
    s = Sample()
    s.capture()
    rospy.spin()
