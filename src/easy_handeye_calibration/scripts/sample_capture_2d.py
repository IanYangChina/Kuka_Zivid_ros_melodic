#!/usr/bin/env python

import cv2
import rospy
import numpy as np
import dynamic_reconfigure.client
from zivid_camera.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# zivid build-in camera matrix
K = np.array([
    [2768.09082, 0.0, 949.384399],
    [0.0, 2767.74096, 591.804870],
    [0.0, 0.0, 1.0]
])

d = np.array([-0.2714506, 0.42517313, 0.00039262, -0.00094772, -0.5916040])


class Sample:
    def __init__(self):
        self.bridge = CvBridge()

        rospy.init_node("sample_capture_2d_py", anonymous=True)

        rospy.loginfo("Starting sample_capture_2d.py")

        rospy.wait_for_service("/zivid_camera/capture_2d", 30.0)

        rospy.Subscriber("/zivid_camera/color/image_color", Image, self.on_image_color)

        self.image_pub = rospy.Publisher("/zivid_camera/color/undistorted_image_color", Image)

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
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(newimg, "rgb8"))

        self.capture()


if __name__ == "__main__":
    s = Sample()
    s.capture()
    rospy.spin()
