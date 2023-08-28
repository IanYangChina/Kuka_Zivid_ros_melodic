#!/usr/bin/env python2

import os
import datetime
import rospy
import numpy as np
import ros_numpy
import quaternion
import open3d as o3d
import zivid
from zivid_camera.srv import *
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, PoseArray


waiting_pose = PoseStamped()
waiting_pose.pose.position.x = -0.52
waiting_pose.pose.position.y = -0.00
waiting_pose.pose.position.z = 0.55
waiting_pose.pose.orientation.w = -0.0594
waiting_pose.pose.orientation.x = -0.664
waiting_pose.pose.orientation.y = 0.745
waiting_pose.pose.orientation.z = 0.0054

DISTANCE_THRESHOLD = 0.001
script_path = os.path.dirname(os.path.realpath(__file__))


def construct_homogeneous_transform_matrix(translation, orientation):
    translation = np.array(translation).reshape((3, 1))  # xyz
    orientation = np.deg2rad(orientation)
    rotation_quat = np.array([  # wxyz
        np.cos(orientation[0] / 2) * np.cos(orientation[1] / 2) * np.cos(orientation[2] / 2) + np.sin(orientation[0] / 2) * np.sin(orientation[1] / 2) * np.sin(orientation[2] / 2),
        np.cos(orientation[0] / 2) * np.cos(orientation[1] / 2) * np.sin(orientation[2] / 2) - np.sin(orientation[0] / 2) * np.sin(orientation[1] / 2) * np.cos(orientation[2] / 2),
        np.cos(orientation[0] / 2) * np.sin(orientation[1] / 2) * np.cos(orientation[2] / 2) + np.sin(orientation[0] / 2) * np.cos(orientation[1] / 2) * np.sin(orientation[2] / 2),
        np.sin(orientation[0] / 2) * np.cos(orientation[1] / 2) * np.cos(orientation[2] / 2) - np.cos(orientation[0] / 2) * np.sin(orientation[1] / 2) * np.sin(orientation[2] / 2)
    ])
    rotation_quat_base = np.sqrt(np.sum(np.square(rotation_quat))) + 1e-6
    rotation_quat_norm = rotation_quat / rotation_quat_base
    rotation_quat_norm = rotation_quat_norm.reshape((4, 1))
    rotation = o3d.geometry.get_rotation_matrix_from_zyx(orientation)
    rotation_ = o3d.geometry.get_rotation_matrix_from_quaternion(rotation_quat_norm)  # wxyz
    # rotation_ = 2 * np.array([
    #     [0.5 - np.square(rotation_quat[2][0]) - np.square(rotation_quat[3][0]),  # 0.5 - y^2 - z^2
    #         rotation_quat[1][0] * rotation_quat[2][0] - rotation_quat[0][0] * rotation_quat[3][0],  # x*y - w*z
    #         rotation_quat[1][0] * rotation_quat[3][0] + rotation_quat[0][0] * rotation_quat[2][0]],  # x*z + w*y
    #     [rotation_quat[1][0] * rotation_quat[2][0] + rotation_quat[0][0] * rotation_quat[3][0],  # x*y + w*z
    #         0.5 - np.square(rotation_quat[1][0]) - np.square(rotation_quat[3][0]),  # 0.5 - x^2 - z^2
    #         rotation_quat[2][0] * rotation_quat[3][0] - rotation_quat[0][0] * rotation_quat[1][0]],  # y*z - w*x
    #     [rotation_quat[1][0] * rotation_quat[3][0] - rotation_quat[0][0] * rotation_quat[2][0],  # x*z - w*y
    #         rotation_quat[2][0] * rotation_quat[3][0] + rotation_quat[0][0] * rotation_quat[1][0],  # y*z + w*x
    #         0.5 - np.square(rotation_quat[1][0]) - np.square(rotation_quat[2][0])]]  # 0.5 - x^2 - y^2
    # )
    print(rotation)
    print(rotation_)
    transformation = np.append(rotation, translation, axis=1)
    transformation = np.append(transformation, np.array([[0, 0, 0, 1]]), axis=0)
    return transformation


class Controller:
    def __init__(self, translation_speed=0.05, rotation_speed=0.1):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
        rospy.Subscriber('/keyboard', String, callback=self.keyboard_callback)
        self.pub_move_cmd = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=2)
        self.pub_attempt_finished = rospy.Publisher('AttemptFinished', Bool, queue_size=2)
        self.current_pose_msg = PoseStamped()
        self.current_xyz = np.array([0.0, 0.0, 0.0])
        self.current_header_seq = 0
        self.translation_speed = translation_speed
        self.rotation_speed = rotation_speed

        self.app = zivid.Application()
        print("Connecting to camera")
        self.camera = self.app.connect_camera()
        self.current_pose_id = 0
        self.hand_eye_input = []
        self.calibration_type = 'eih'

        self.move_robot()

    def move_robot(self):
        rospy.loginfo("Initializing robot...")
        self.publish_pose(waiting_pose)

    def keyboard_callback(self, data):
        key_pressed = data.data
        if key_pressed == 'p':
            self.capture()
            return
        elif key_pressed == 'c':
            self.calibrate()
            return

        target_pose = self.current_pose_msg
        if key_pressed == '1':
            target_pose.pose.position.x += self.translation_speed
        elif key_pressed == '2':
            target_pose.pose.position.x -= self.translation_speed
        elif key_pressed == '3':
            target_pose.pose.position.y += self.translation_speed
        elif key_pressed == '4':
            target_pose.pose.position.y -= self.translation_speed
        elif key_pressed == '5':
            target_pose.pose.position.z += self.translation_speed
        elif key_pressed == '6':
            target_pose.pose.position.z -= self.translation_speed
        else:
            quat = np.array([target_pose.pose.orientation.w,
                             target_pose.pose.orientation.x,
                             target_pose.pose.orientation.y,
                             target_pose.pose.orientation.z])
            rotation = o3d.geometry.get_rotation_matrix_from_quaternion(np.array(quat).reshape((4, 1)))
            if key_pressed == '7':
                rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz(np.array([0.0, 0.0, self.rotation_speed]))
            elif key_pressed == '8':
                rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz(np.array([0.0, 0.0, -self.rotation_speed]))
            elif key_pressed == '9':
                rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz(np.array([0.0, self.rotation_speed, 0.0]))
            elif key_pressed == '0':
                rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz(np.array([0.0, -self.rotation_speed, 0.0]))
            elif key_pressed == '-':
                rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz(np.array([self.rotation_speed, 0.0, 0.0]))
            elif key_pressed == '=':
                rotation_delta = o3d.geometry.get_rotation_matrix_from_xyz(np.array([-self.rotation_speed, 0.0, 0.0]))
            else:
                rotation_delta = np.eye(3)
            rotation_new = np.dot(rotation_delta, rotation)
            quat_new = quaternion.as_float_array(quaternion.from_rotation_matrix(rotation_new))  # w, x, y, z
            target_pose.pose.orientation.w = quat_new[0]
            target_pose.pose.orientation.x = quat_new[1]
            target_pose.pose.orientation.y = quat_new[2]
            target_pose.pose.orientation.z = quat_new[3]

        self.publish_pose(target_pose)

    def current_pose_callback(self, data):
        self.current_pose_msg = data
        self.current_xyz = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z
        ])
        self.current_header_seq = data.header.seq

    def publish_pose(self, data):
        # record target xyz for distance tracking
        target_xyz = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z
        ])
        data.header.seq = self.current_header_seq
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = 'iiwa_link_0'
        rospy.sleep(0.5)
        self.pub_move_cmd.publish(data)
        done = False
        while not done:
            d = np.sqrt(np.sum(np.square(self.current_xyz - target_xyz)))
            if d < DISTANCE_THRESHOLD:
                rospy.loginfo("Movement finished")
                done = True

    def capture(self):
        robot_pose_msg = self.current_pose_msg
        robot_pose = construct_homogeneous_transform_matrix(
            np.array([robot_pose_msg.pose.position.x,
                      robot_pose_msg.pose.position.y,
                      robot_pose_msg.pose.position.z]),
            np.array([robot_pose_msg.pose.orientation.w,
                      robot_pose_msg.pose.orientation.x,
                      robot_pose_msg.pose.orientation.y,
                      robot_pose_msg.pose.orientation.z])
        )
        suggest_settings_parameters = zivid.capture_assistant.SuggestSettingsParameters(
            max_capture_time=datetime.timedelta(milliseconds=800),
            ambient_light_frequency=zivid.capture_assistant.SuggestSettingsParameters.AmbientLightFrequency.none,
        )
        settings = zivid.capture_assistant.suggest_settings(self.camera, suggest_settings_parameters)
        frame = self.camera.capture(settings)
        detection_result = zivid.calibration.detect_feature_points(frame.point_cloud())
        if detection_result.valid():
            print("Calibration board detected")
            self.hand_eye_input.append(zivid.calibration.HandEyeInput(robot_pose, detection_result))
            self.current_pose_id += 1
            print("Number of added poses:", self.current_pose_id)
        else:
            print(
                "Failed to detect calibration board, ensure that the entire board is in the view of the camera"
            )

    def calibrate(self):
        if self.calibration_type.lower() == "eth":
            print("Performing eye-to-hand calibration")
            hand_eye_output = zivid.calibration.calibrate_eye_to_hand(self.hand_eye_input)
        elif self.calibration_type.lower() == "eih":
            print("Performing eye-in-hand calibration")
            hand_eye_output = zivid.calibration.calibrate_eye_in_hand(self.hand_eye_input)
        else:
            print("Invalid calibration type")
            return
        transform = hand_eye_output.transform()
        transform_file_path = os.path.join(script_path, '..', 'results_eye_on_hand', 'Extrinsic_zivid.npy')
        np.save(os.path.join(transform_file_path), transform)


if __name__ == '__main__':
    controller = Controller()
    rospy.spin()
