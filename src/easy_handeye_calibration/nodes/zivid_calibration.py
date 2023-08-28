#!/usr/bin/env python2

import os
import rospy
import rosnode
import rosgraph
import roslaunch
import shutil
from std_msgs.msg import String


class KeyboardNode:
    def __init__(self):
        rospy.init_node('keyboard_node', anonymous=True)
        self.key_publisher = rospy.Publisher('/keyboard', String, queue_size=1)

    def publish(self, key):
        self.key_publisher.publish(key)
        rospy.sleep(0.1)


if __name__ == "__main__":
    keyboard_node = KeyboardNode()
    done = False
    while not done:
        user_input = input('Input 0 to = for moving, t for capture, c for calibration, q for exit.')
        if user_input == 'q':
            break
        else:
            keyboard_node.publish(user_input)
