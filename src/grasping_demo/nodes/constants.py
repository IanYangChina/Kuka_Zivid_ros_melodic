#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, PoseArray
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotInput as inputMsg
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput as outputMsg
from std_msgs.msg import Bool
import numpy as np
import rospy
import math

DISTANCE_THRESHOLD = 0.001

waiting_pose = PoseStamped()
waiting_pose.pose.position.x = 0.0
waiting_pose.pose.position.y = -0.55
waiting_pose.pose.position.z = 0.6

waiting_pose.pose.orientation.w = 0.0000
waiting_pose.pose.orientation.x = -0.7071067811
waiting_pose.pose.orientation.y = 0.7071067811
waiting_pose.pose.orientation.z = 0.0000

pre_grasping_pose = PoseStamped()
pre_grasping_pose.pose.position.x = -0.55
pre_grasping_pose.pose.position.y = 0.0
pre_grasping_pose.pose.position.z = 0.45

pre_grasping_pose.pose.orientation.w = 0.0000
pre_grasping_pose.pose.orientation.x = 0.0000
pre_grasping_pose.pose.orientation.y = 1.0000
pre_grasping_pose.pose.orientation.z = 0.0000




gripper_activation = outputMsg()
gripper_activation.rACT = 1
gripper_activation.rGTO = 1
gripper_activation.rMOD = 1
gripper_activation.rSPA = 250
gripper_activation.rFRA = 150

gripper_close = outputMsg()
gripper_close.rACT = 1
gripper_close.rGTO = 1
gripper_close.rMOD = 1
gripper_close.rPRA = 255
gripper_close.rSPA = 200
gripper_close.rFRA = 150

gripper_half = outputMsg()
gripper_half.rACT = 1
gripper_half.rGTO = 1
gripper_half.rMOD = 1
gripper_half.rPRA = 100
gripper_half.rSPA = 200
gripper_half.rFRA = 150

gripper_pre_1 = outputMsg()
gripper_pre_1.rACT = 1
gripper_pre_1.rGTO = 1
gripper_pre_1.rMOD = 1
gripper_pre_1.rPRA = 50
gripper_pre_1.rSPA = 200
gripper_pre_1.rFRA = 150

gripper_pre_2 = outputMsg()
gripper_pre_2.rACT = 1
gripper_pre_2.rGTO = 1
gripper_pre_2.rMOD = 1
gripper_pre_2.rPRA = 20
gripper_pre_2.rSPA = 200
gripper_pre_2.rFRA = 150

gripper_open = outputMsg()
gripper_open.rACT = 1
gripper_open.rGTO = 1
gripper_open.rMOD = 1
gripper_open.rPRA = 0
gripper_open.rSPA = 200
gripper_open.rFRA = 150

gripper_reset = outputMsg()
gripper_reset.rACT = 0
gripper_reset.rGTO = 0
gripper_reset.rMOD = 0
gripper_reset.rPRA = 0
gripper_reset.rSPA = 0
gripper_reset.rFRA = 0

def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def euler_from_quaternion(x, y, z, w):
        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z


class Controller:
    def __init__(self):
        # rospy.init_node('kuka_controller', anonymous=True)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
        self.pub_move_cmd = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=2)
        self.pub_gripper_cmd = rospy.Publisher('Robotiq3FGripperRobotOutput', outputMsg, queue_size=2)
        self.pub_attempt_finished = rospy.Publisher('AttemptFinished', Bool, queue_size=2)
        self.current_pose_msg = PoseStamped()
        self.current_xyz = np.array([0.0, 0.0, 0.0])
        self.current_header_seq = 0

        self.waiting_pose = PoseStamped()
        self.waiting_pose.pose.position.x = -0.0
        self.waiting_pose.pose.position.y = 0.5
        self.waiting_pose.pose.position.z = 0.5
        self.waiting_pose.pose.orientation.w = 0.0000
        self.waiting_pose.pose.orientation.x = 0.675
        self.waiting_pose.pose.orientation.y = 0.7373
        self.waiting_pose.pose.orientation.z = 0

    def get_angles(self,pos,wrist):
        ###wrist value in radians
        euler=[0,0,0]
        euler[0]=3.14159
        
        euler[2]=math.atan(pos[1]/pos[0])-1.5707+wrist
        euler[2]=-1.5707+wrist
        print(euler[2]*360/(2*3.1416))
        #euler=[0,0,0]
        #rospy.loginfo(euler,pos)
        return euler

    def get_message_pose(self,pos,qua):
        grasping_pose = PoseStamped()
        grasping_pose.pose.position.x = pos[0]
        grasping_pose.pose.position.y = pos[1]
        grasping_pose.pose.position.z = pos[2]
        grasping_pose.pose.orientation.w = qua[3]
        grasping_pose.pose.orientation.x = qua[0]
        grasping_pose.pose.orientation.y = qua[1]
        grasping_pose.pose.orientation.z = qua[2]
        return grasping_pose

    def init_robot(self):
        rospy.loginfo("Initializing kuka robot...!!")
        self.publish_grip_cmd(gripper_reset)
        self.publish_grip_cmd(gripper_activation)
        print('moving to waiting_pose ', self.waiting_pose )
        if hasattr(self, 'waiting_pose'):
            self.publish_pose(self.waiting_pose)
        else:
            raise Exception('expecting an init_pose, got None')

    def home(self):
        rospy.loginfo("Initializing kuka robot...")
        # self.publish_grip_cmd(gripper_pre_1)
        self.publish_pose(self.waiting_pose)

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
        while not done and not rospy.is_shutdown():
            d = np.sqrt(np.sum(np.square(self.current_xyz - target_xyz)))
            if d < DISTANCE_THRESHOLD:
                rospy.loginfo("Movement finished")
                done = True

    def publish_grip_cmd(self, data):
        self.pub_gripper_cmd.publish(data)
        rospy.sleep(1)

# kuka=Controller()
