#!/usr/bin/env python2

import os
import rospy
import rosnode
import rosgraph
import roslaunch
import shutil


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
            launch_tracking.shutdown()
            exit()

    print('[INFO] Starting calibration program...')
    launch_calibration = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(script_path, '..', 'launch', 'my_calibrate.launch')])
    launch_calibration.start()

    calibration_result_file = os.path.join(os.getenv('HOME'), '.ros/easy_handeye/my_eye_on_base.yaml')

    # todo: this loop condition is not appropriate, better using ros node info...
    while rosgraph.is_master_online():
        rospy.sleep(1)

    # check if the calibration result has been saved
    # if so, move the file to ../results/
    if os.path.exists(calibration_result_file):
        if not os.path.exists(os.path.join(script_path, '..', 'results')):
            os.mkdir(os.path.join(script_path, '..', 'results'))
        shutil.move(calibration_result_file, os.path.join(script_path, '..', 'results', 'Extrinsics.yaml'))

    launch_tracking.shutdown()
    launch_calibration.shutdown()
    exit()
