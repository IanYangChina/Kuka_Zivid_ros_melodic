import rospy
from zivid_camera.srv import *
from std_msgs.msg import Bool


class CaptureHelper:
    def __init__(self):
        self.attempt_finished = False
        self.attempt_num_passed = 0
        rospy.init_node("capture_helper_node", anonymous=True)
        rospy.loginfo("Starting capture_helper_node")
        rospy.Subscriber('AttemptFinished', Bool, callback=self.target_pose_reached_callback)
        ca_suggest_settings_service = "/zivid_camera/capture_assistant/suggest_settings"
        rospy.wait_for_service(ca_suggest_settings_service)
        self.capture_assistant_service = rospy.ServiceProxy(
            ca_suggest_settings_service, CaptureAssistantSuggestSettings
        )
        self.capture_service = rospy.ServiceProxy("/zivid_camera/capture", Capture)
        self.capture_assistant_suggest_settings()

    def target_pose_reached_callback(self, data):
        rospy.loginfo("Received attempt status data")
        self.attempt_finished = data.data

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


if __name__ == '__main__':
    capture_helper = CaptureHelper()
    rospy.spin()
