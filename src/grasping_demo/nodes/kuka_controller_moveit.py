#!/usr/bin/env python2

import copy
import sys
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from utils.kuka_poses import *
import moveit_commander
from scipy.spatial.transform import Rotation

DISTANCE_THRESHOLD = 0.001


def qmul(q, r):
    terms = np.outer(r, q)
    w = terms[0, 0] - terms[1, 1] - terms[2, 2] - terms[3, 3]
    x = terms[0, 1] + terms[1, 0] - terms[2, 3] + terms[3, 2]
    y = terms[0, 2] + terms[1, 3] + terms[2, 0] - terms[3, 1]
    z = terms[0, 3] - terms[1, 2] + terms[2, 1] + terms[3, 0]
    out = np.array([w, x, y, z])
    return out / np.sqrt(out.dot(out))


class Controller:
    def __init__(self, translation_speed=0.05, rotation_speed=0.05*np.pi):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
        self.pub_move_cmd = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=2)

        self.current_pose_msg = PoseStamped()
        self.current_xyz = np.array([0.0, 0.0, 0.0])
        self.current_header_seq = 0

        self.moveit_commander = moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_robot = moveit_commander.RobotCommander(robot_description="robot_description")
        self.moveit_scene = moveit_commander.PlanningSceneInterface()
        self.moveit_group = moveit_commander.MoveGroupCommander("manipulator", robot_description="robot_description")
        self.moveit_group_eelink = self.moveit_group.get_end_effector_link()

        self.v = 1.0  # m/s, this is an approximation
        self.w = 180  # deg/s, this is an approximation
        self.dt = 0.0002  # matching the simulation dt

        self.init_robot()

    def init_robot(self):
        rospy.loginfo("Initializing robot...")
        self.publish_pose(waiting_pose)
        rospy.sleep(2)

        self.move_distance([0.1, 0.0, -0.2, 0, 0, 90])

    def move_distance(self, dp):
        waypoints = []
        p = self.moveit_group.get_current_pose().pose
        rospy.loginfo("Current pose: {}".format(p))
        tar_p = copy.deepcopy(p)
        tar_p.position.x += dp[0]
        tar_p.position.y += dp[1]
        tar_p.position.z += dp[2]
        delta_quat_xyzw = Rotation.from_euler('xyz', dp[3:], degrees=True).as_quat()
        delta_quat_wxyz = [delta_quat_xyzw[1], delta_quat_xyzw[2], delta_quat_xyzw[3], delta_quat_xyzw[0]]
        tar_quat = qmul([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z], delta_quat_wxyz)
        tar_p.orientation.w = tar_quat[0]
        tar_p.orientation.x = tar_quat[1]
        tar_p.orientation.y = tar_quat[2]
        tar_p.orientation.z = tar_quat[3]
        rospy.loginfo("Target pose: {}".format(tar_p))

        dx = tar_p.position.x - p.position.x
        vx = self.v if dx > 0 else -self.v
        abs_dx = np.abs(dx)
        dy = tar_p.position.y - p.position.y
        vy = self.v if dy > 0 else -self.v
        abs_dy = np.abs(dy)
        dz = tar_p.position.z - p.position.z
        vz = self.v if dz > 0 else -self.v
        abs_dz = np.abs(dz)

        current_euler = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_euler('xyz', degrees=True)
        rospy.loginfo("Current euler: {}".format(current_euler))
        da = dp[3]
        va = self.w if da > 0 else -self.w
        abs_da = np.abs(da)
        db = dp[4]
        vb = self.w if db > 0 else -self.w
        abs_db = np.abs(db)
        dc = dp[5]
        vc = self.w if dc > 0 else -self.w
        abs_dc = np.abs(dc)
        tar_euler = current_euler + np.array([da, db, dc])
        rospy.loginfo("Target euler: {}".format(tar_euler))

        n_t = max(int(np.max([abs_dx, abs_dy, abs_dz]) / (self.v * self.dt)),
                  int(np.max([abs_da, abs_db, abs_dc]) / (self.w * self.dt)))

        for _ in range(n_t):
            if np.abs(p.position.x - tar_p.position.x) <= 0.0002:
                vx = 0
            if np.abs(p.position.y - tar_p.position.y) <= 0.0002:
                vy = 0
            if np.abs(p.position.z - tar_p.position.z) <= 0.0002:
                vz = 0
            if np.abs(current_euler[0] - tar_euler[0]) <= 0.009:
                va = 0
            if np.abs(current_euler[1] - tar_euler[1]) <= 0.009:
                vb = 0
            if np.abs(current_euler[2] - tar_euler[2]) <= 0.009:
                vc = 0
            p.position.x += vx * self.dt
            p.position.y += vy * self.dt
            p.position.z += vz * self.dt
            current_euler[0] += va * self.dt
            current_euler[1] += vb * self.dt
            current_euler[2] += vc * self.dt
            quat_xyzw = Rotation.from_euler('xyz', current_euler, degrees=True).as_quat()
            p.orientation.w = quat_xyzw[3]
            p.orientation.x = quat_xyzw[0]
            p.orientation.y = quat_xyzw[1]
            p.orientation.z = quat_xyzw[2]
            waypoints.append(copy.deepcopy(p))

        (plan, fraction) = self.moveit_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.0001,        # eef_step
                                   0.0)         # jump_threshold
        rospy.loginfo("Executing plan with fraction: {}".format(fraction))
        self.moveit_group.execute(plan, wait=True)
        rospy.loginfo("Done executing plan")
        p = self.moveit_group.get_current_pose().pose
        rospy.loginfo("Current pose: {}".format(p))
        reached_euler = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_euler('xyz', degrees=True)
        rospy.loginfo("Current euler: {}".format(reached_euler))

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


if __name__ == '__main__':
    controller = Controller()
    rospy.spin()
