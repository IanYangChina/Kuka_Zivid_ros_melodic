#!/usr/bin/env python2

import copy
import sys
import rospy, rosnode
import numpy as np
from geometry_msgs.msg import PoseStamped
from utils.kuka_poses import *
import moveit_commander
from scipy.spatial.transform import Rotation
from grasping_demo.srv import TargetPose, TargetPoseResponse, Reset, ResetResponse, MoveDistance, MoveDistanceResponse
from grasping_demo.srv import TrajectoryOne, TrajectoryOneResponse

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

        self.delta_position = 0.0002  # meter per waypoint
        self.delta_angle = 0.036  # angle per waypoint

        ROSSMartServo_on = False
        while not ROSSMartServo_on:
            rospy.loginfo('Please start the ROSSMartServo application on the Sunrise Cabinet')
            if '/iiwa/iiwa_subscriber' in rosnode.get_node_names():
                ROSSMartServo_on = True
            else:
                rospy.sleep(1)
        self.init_robot()

        self.sample_service = rospy.Service('move_to_target', TargetPose, self.move_target)
        self.reset_service = rospy.Service('reset', Reset, self.reset)
        self.move_service = rospy.Service('move_distance', MoveDistance, self.move_distance)
        self.tr1_service = rospy.Service('trajectory_1', TrajectoryOne, self.trajectory_1)

    def init_robot(self):
        rospy.loginfo("Initializing robot...")
        self.publish_pose(waiting_pose)
        rospy.sleep(2)

        # self.move_distance([0.1, 0.0, -0.2, 0, 0, 90])

    def move_distance(self, req):
        rospy.loginfo("Received moving request, generating plan...")
        waypoints = []

        p = self.moveit_group.get_current_pose().pose
        tar_p = copy.deepcopy(p)
        tar_p.position.x += req.x
        tar_p.position.y += req.y
        tar_p.position.z += req.z
        delta_quat_xyzw = Rotation.from_euler('xyz', np.array([req.a, req.b, req.c]), degrees=True).as_quat()
        delta_quat_wxyz = [delta_quat_xyzw[1], delta_quat_xyzw[2], delta_quat_xyzw[3], delta_quat_xyzw[0]]
        tar_quat = qmul([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z], delta_quat_wxyz)
        tar_p.orientation.w = tar_quat[0]
        tar_p.orientation.x = tar_quat[1]
        tar_p.orientation.y = tar_quat[2]
        tar_p.orientation.z = tar_quat[3]

        dx = tar_p.position.x - p.position.x
        delta_x = self.delta_position if dx > 0 else -self.delta_position
        abs_dx = np.abs(dx)
        dy = tar_p.position.y - p.position.y
        delta_y = self.delta_position if dy > 0 else -self.delta_position
        abs_dy = np.abs(dy)
        dz = tar_p.position.z - p.position.z
        delta_z = self.delta_position if dz > 0 else -self.delta_position
        abs_dz = np.abs(dz)

        current_euler = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_euler('xyz', degrees=True)
        da = req.a
        delta_a = self.delta_angle if da > 0 else -self.delta_angle
        abs_da = np.abs(da)
        db = req.b
        delta_b = self.delta_angle if db > 0 else -self.delta_angle
        abs_db = np.abs(db)
        dc = req.c
        delta_c = self.delta_angle if dc > 0 else -self.delta_angle
        abs_dc = np.abs(dc)
        tar_euler = current_euler + np.array([da, db, dc])

        n_t = max(int(np.max([abs_dx, abs_dy, abs_dz]) / self.delta_position),
                  int(np.max([abs_da, abs_db, abs_dc]) / self.delta_angle))

        for _ in range(n_t):
            if np.abs(p.position.x - tar_p.position.x) <= 0.0002:
                delta_x = 0
            if np.abs(p.position.y - tar_p.position.y) <= 0.0002:
                delta_y = 0
            if np.abs(p.position.z - tar_p.position.z) <= 0.0002:
                delta_z = 0
            if np.abs(current_euler[0] - tar_euler[0]) <= 0.009:
                delta_a = 0
            if np.abs(current_euler[1] - tar_euler[1]) <= 0.009:
                delta_b = 0
            if np.abs(current_euler[2] - tar_euler[2]) <= 0.009:
                delta_c = 0
            p.position.x += delta_x
            p.position.y += delta_y
            p.position.z += delta_z
            current_euler[0] += delta_a
            current_euler[1] += delta_b
            current_euler[2] += delta_c
            quat_xyzw = Rotation.from_euler('xyz', current_euler.copy(), degrees=True).as_quat()
            p.orientation.x = quat_xyzw[0]
            p.orientation.y = quat_xyzw[1]
            p.orientation.z = quat_xyzw[2]
            p.orientation.w = quat_xyzw[3]
            waypoints.append(copy.deepcopy(p))

        (plan, fraction) = self.moveit_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.0001,      # eef_step
                                   0.0)         # jump_threshold
        # moveit sometimes uses the same time value for the last two trajectory points, causing failure execution
        if plan.joint_trajectory.points[-2].time_from_start.nsecs == plan.joint_trajectory.points[-1].time_from_start.nsecs:
            plan.joint_trajectory.points[-1].time_from_start.nsecs += 10000

        self.moveit_group.execute(plan, wait=True)
        dt = plan.joint_trajectory.points[-1].time_from_start.nsecs / 10e9
        rospy.loginfo("Time spent: "+str(dt)+" secs")

        return MoveDistanceResponse()

    def trajectory_1(self, req):
        rospy.loginfo("Executing trajectory 1...")
        # This trajectory takes approximately 0.03 seconds to complete
        # 0.015 seconds down, 0.03 seconds up
        down_d = 0.015
        up_d = 0.03
        waypoints = []
        p = self.moveit_group.get_current_pose().pose
        z_0 = p.position.z
        n_t = int(down_d / self.delta_position)
        for _ in range(n_t):
            p.position.z -= self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_0) >= down_d:
                break
        n_t = int(up_d / self.delta_position)
        for _ in range(n_t):
            p.position.z += self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_0) >= up_d:
                break
        (plan, fraction) = self.moveit_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.0001,      # eef_step
                                   0.0)         # jump_threshold
        # moveit sometimes uses the same time value for the last two trajectory points, causing failure execution
        if plan.joint_trajectory.points[-2].time_from_start.nsecs == plan.joint_trajectory.points[-1].time_from_start.nsecs:
            plan.joint_trajectory.points[-1].time_from_start.nsecs += 10000

        self.moveit_group.execute(plan, wait=True)
        dt = plan.joint_trajectory.points[-1].time_from_start.nsecs / 10e9
        rospy.loginfo("Time spent: "+str(dt)+" secs")

        return TrajectoryOneResponse()

    def reset(self, req):
        self.publish_pose(waiting_pose)
        return ResetResponse()

    def move_target(self, req):
        p = copy.deepcopy(waiting_pose)
        p.pose.position.x = req.x
        p.pose.position.y = req.y
        p.pose.position.z = req.z
        self.publish_pose(p)
        return TargetPoseResponse()

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
