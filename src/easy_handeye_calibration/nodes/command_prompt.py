#!/usr/bin/env python2

import os
import rospy
import rosnode
import roslaunch
from zivid_camera.srv import *
from std_msgs.msg import Bool


if __name__ == '__main__':
    print('[INFO] Starting tracking system...')
    script_path = os.path.dirname(os.path.realpath(__file__))
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_tracking = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(script_path, '..', 'launch', 'tracking.launch')])
    launch_tracking.start()
    rospy.sleep(5)

    ROSSMartServo_on = False
    while not ROSSMartServo_on:
        rospy.loginfo('Please make sure you have started the ROSSMartServo application on the Sunrise Cabinet')
        ans = raw_input(
            '[USER INPUT] Type [y] and press [enter] if you have started the ROSSMartServo, otherwise exit the program: ')
        if ans == 'y':
            # todo: add codes to make sure ROSSMartServo has been started on the Sunrise Cabinet
            if '/iiwa/iiwa_subscriber' in rosnode.get_node_names():
                ROSSMartServo_on = True
            else:
                rospy.loginfo('IIWA topics not detected, check network connection if you have started the SmartServo')
        else:
            rospy.loginfo('Exiting the program...')
            exit()

    print('[INFO] Starting calibration program...')
    launch_calibration = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(script_path, '..', 'launch', 'my_calibrate.launch')])
    launch_calibration.start()

    while not rospy.is_shutdown():
        rospy.sleep(5)

    close_smart_servo = False
    while not close_smart_servo:
        if not ('/iiwa/iiwa_subscriber' in rosnode.get_node_names()):
            close_smart_servo = True
        else:
            rospy.loginfo("Please **now** shutdown the SmartServo application on Sunrise Cabinet")
            rospy.sleep(5)
    rospy.loginfo('Exiting the program...')
    exit()