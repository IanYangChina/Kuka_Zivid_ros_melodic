#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from utils.kuka_poses import *
from pynput.keyboard import Key, Listener


class Sender:
    def __init__(self):
        rospy.init_node('sender_node', anonymous=True)
        self.key_publisher = rospy.Publisher('/PoseToSend', String, queue_size=1)
        self.running = True

    def publish(self, key):
        self.key_publisher.publish(key)
        rospy.sleep(0.1)


if __name__ == "__main__":
    sender_node = Sender()
    rospy.loginfo('Press 1 to 6 for moving, esc for exit.')

    def on_press(key):
        print(f"Key {key.char} pressed.")
        if key == Key.esc:
            return False

        if key.char in ['1', '2', '3', '4', '5', '6', '7']:
            sender_node.publish(key.char)
        else:
            rospy.loginfo('Unused key pressed.')

    with Listener(on_press=on_press) as listener:
        while True:
            if not listener.running:
                break
