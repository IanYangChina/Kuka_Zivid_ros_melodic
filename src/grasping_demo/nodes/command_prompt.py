#!/usr/bin/env python2

import os
import rospy
import rosnode
import roslaunch
import keyboard
from zivid_camera.srv import *
from std_msgs.msg import Bool


class CaptureHelper:
    def __init__(self):
        self.attempt_finished = False
        self.attempt_num_passed = 0
        rospy.Subscriber('TargetPoseReached', Bool, callback=self.target_pose_reached_callback)
        ca_suggest_settings_service = "/zivid_camera/capture_assistant/suggest_settings"
        rospy.wait_for_service(ca_suggest_settings_service)
        self.capture_assistant_service = rospy.ServiceProxy(
            ca_suggest_settings_service, CaptureAssistantSuggestSettings
        )
        self.capture_service = rospy.ServiceProxy("/zivid_camera/capture", Capture)
        self.capture_assistant_suggest_settings()

    def target_pose_reached_callback(self, data):
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
    print('Starting ros communication system..')
    script_path = os.path.dirname(os.path.realpath(__file__))
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(script_path, '..', 'launch', 'grasping_demo.launch')])
    launch.start()
    rospy.sleep(2)
    capture_helper = CaptureHelper()
    ROSSMartServo_on = False
    while not ROSSMartServo_on:
        rospy.loginfo('Please make sure you have the ROSSMartServo application on the Sunrise Cabinet\n'
                      'Check network connection if you have told the program SmartServo is started but still seeing this message\n')
        ans = raw_input(
            '[ USER INPUT] Type [y] and press [enter] if you have started the ROSSMartServo, otherwise exit the program: ')
        if ans == 'y':
            # todo: add codes to make sure ROSSMartServo has been started on the Sunrise Cabinet
            print(rosnode.get_node_names())
            ROSSMartServo_on = True
        else:
            rospy.loginfo('Exiting the program...')
            exit()

    rospy.loginfo('Ros communication system has been properly setup')
    while True:
        rospy.loginfo('Start a new grasping trail')
        raw_input('[ USER INPUT] Place the object within the workspace and press [enter]')
        rospy.loginfo('Capture and processing point cloud...')
        capture_helper.attempt_finished = False
        capture_helper.attempt_num_passed += 1
        capture_helper.capture()
        while not capture_helper.attempt_finished:
            pass
        ans = raw_input('[ USER INPUT] Would you like to continue with a new object pose? [y/n]')
        if ans == 'y':
            continue
        else:
            rospy.loginfo('Exiting the program...')
            exit()
