#!/usr/bin/env python

import rospy
import rosnode
import numpy as np
import dynamic_reconfigure.client
from zivid_camera.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()


class Sample:
    def __init__(self):
        rospy.init_node("sample_capture_2d_py", anonymous=True)

        rospy.loginfo("Starting sample_capture_2d.py")

        rospy.wait_for_service("/zivid_camera/capture_2d", 30.0)

        rospy.Subscriber("/zivid_camera/color/image_color", Image, self.on_image_color)

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
        cv_images = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        cv_images = np.asarray(cv_images)
        print(cv_images.dtype)
        self.capture()


if __name__ == "__main__":
    s = Sample()
    s.capture()
    rospy.spin()
