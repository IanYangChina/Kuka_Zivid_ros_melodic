#!/usr/bin/env python2

import os
import copy
import sys
import math
import rospy, rosnode
import numpy as np
from geometry_msgs.msg import PoseStamped
from utils.kuka_poses import *
import moveit_commander
from scipy.spatial.transform import Rotation
from grasping_demo.srv import TargetPose, TargetPoseResponse, Reset, ResetResponse, MoveDistance, MoveDistanceResponse
from grasping_demo.srv import TrajectoryOne, TrajectoryOneResponse, TrajectoryTwo, TrajectoryTwoResponse, TrajectoryThree, TrajectoryThreeResponse
from grasping_demo.srv import TrajectoryFour, TrajectoryFourResponse
from grasping_demo.srv import TrajectoryRecValid, TrajectoryRoundValid, TrajectoryCyldrValid, TrajectoryRecValidResponse, TrajectoryRoundValidResponse, TrajectoryCyldrValidResponse
from grasping_demo.srv import TrajectorySpoon, TrajectorySpoonResponse
import matplotlib.pyplot as plt

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
    def __init__(self, translation_speed=0.05, rotation_speed=0.05*np.pi, robot_name='iiwa'):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('/'+robot_name+'/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
        self.pub_move_cmd = rospy.Publisher('/'+robot_name+'/command/CartesianPose', PoseStamped, queue_size=2)

        self.current_pose_msg = PoseStamped()
        self.current_xyz = np.array([0.0, 0.0, 0.0])
        self.current_header_seq = 0

        self.moveit_commander = moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_robot = moveit_commander.RobotCommander(robot_description="robot_description")
        self.moveit_scene = moveit_commander.PlanningSceneInterface()
        self.moveit_group = moveit_commander.MoveGroupCommander("manipulator", robot_description="robot_description")

        self.delta_position = 0.005  # meter per waypoint
        self.delta_angle = 5  # angle per waypoint

        ROSSMartServo_on = False
        while not ROSSMartServo_on:
            rospy.loginfo('Please start the ROSSMartServo application on the Sunrise Cabinet')
            if '/'+robot_name+'/iiwa_subscriber' in rosnode.get_node_names():
                ROSSMartServo_on = True
            else:
                rospy.sleep(1)
        self.init_robot()
        rospy.loginfo('init success!')

        self.sample_service = rospy.Service('move_to_target', TargetPose, self.move_target)
        self.reset_service = rospy.Service('reset', Reset, self.reset)
        self.move_service = rospy.Service('move_distance', MoveDistance, self.move_distance)
        self.tr1_service = rospy.Service('trajectory_1', TrajectoryOne, self.trajectory_1)
        self.tr2_service = rospy.Service('trajectory_2', TrajectoryTwo, self.trajectory_2)
        self.tr3_service = rospy.Service('trajectory_3', TrajectoryThree, self.trajectory_3)
        self.tr4_service = rospy.Service('trajectory_4', TrajectoryFour, self.trajectory_4)

        self.tr_rec_valid_service = rospy.Service('trajectory_rec_valid', TrajectoryRecValid, self.trajectory_rec_valid)
        self.tr_round_valid_service = rospy.Service('trajectory_round_valid', TrajectoryRoundValid, self.trajectory_round_valid)
        self.tr_cyldr_valid_service = rospy.Service('trajectory_cyldr_valid', TrajectoryCyldrValid, self.trajectory_cyldr_valid)

        self.tr_spoon_service = rospy.Service('trajectory_spoon', TrajectorySpoon, self.trajectory_spoon)

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
                                   self.delta_position,      # eef_step
                                   0.0)         # jump_threshold
        # moveit sometimes uses the same time value for the last two trajectory points, causing failure execution
        if plan.joint_trajectory.points[-2].time_from_start.nsecs == plan.joint_trajectory.points[-1].time_from_start.nsecs:
            plan.joint_trajectory.points[-1].time_from_start.nsecs += 10000

        self.moveit_group.execute(plan, wait=True)
        dt = plan.joint_trajectory.points[-1].time_from_start.secs + plan.joint_trajectory.points[-1].time_from_start.nsecs / 1e9
        rospy.loginfo("Time spent: "+str(dt)+" secs")

        return MoveDistanceResponse()

    def plan_and_show(self, waypoints, show=False, save_tr=False, tr_name='tr'):
        (plan, fraction) = self.moveit_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   self.delta_position,      # eef_step
                                   0.0)         # jump_threshold

        # moveit sometimes uses the same time value for the last two trajectory points, causing failure execution
        if plan.joint_trajectory.points[-2].time_from_start.nsecs == plan.joint_trajectory.points[-1].time_from_start.nsecs:
            plan.joint_trajectory.points[-1].time_from_start.nsecs += 1000

        if show:
            self.plot_eef_v(plan, save_tr, tr_name)
        return plan

    def trajectory_1(self, req):
        rospy.loginfo("Executing trajectory 1...")
        # 0.015 m down, 0.03 m up
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

        plan = self.plan_and_show(waypoints, False, False, 'tr1')

        self.moveit_group.execute(plan, wait=True)
        dt = plan.joint_trajectory.points[-1].time_from_start.secs + plan.joint_trajectory.points[-1].time_from_start.nsecs / 1e9
        rospy.loginfo("Time spent: "+str(dt)+" secs")

        return TrajectoryOneResponse()

    def trajectory_2(self, req):
        rospy.loginfo("Executing trajectory 2...")
        # 0.02 m down, 0.03 m up
        down_d = 0.02
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

        plan = self.plan_and_show(waypoints, False, False, 'tr2')

        self.moveit_group.execute(plan, wait=True)
        dt = plan.joint_trajectory.points[-1].time_from_start.secs + plan.joint_trajectory.points[-1].time_from_start.nsecs / 1e9
        rospy.loginfo("Time spent: "+str(dt)+" secs")

        return TrajectoryTwoResponse()

    def trajectory_3(self, req):
        rospy.loginfo("Executing trajectory 3...")
        # 0.02 m down, 0.03 m right, 0.03 m up
        down_d = 0.02
        right_d = 0.03
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
        x_1 = p.position.x
        n_t = int(right_d / self.delta_position)
        for _ in range(n_t):
            p.position.x -= self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.x - x_1) >= right_d:
                break
        z_1 = p.position.z
        n_t = int(up_d / self.delta_position)
        for _ in range(n_t):
            p.position.z += self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_1) >= up_d:
                break

        plan = self.plan_and_show(waypoints, False, False, 'tr3')

        self.moveit_group.execute(plan, wait=True)
        dt = plan.joint_trajectory.points[-1].time_from_start.secs + plan.joint_trajectory.points[-1].time_from_start.nsecs / 1e9
        rospy.loginfo("Time spent: "+str(dt)+" secs")

        return TrajectoryThreeResponse()

    def trajectory_4(self, req):
        rospy.loginfo("Executing trajectory 4...")
        # 0.02 m down, 0.03 m left, 0.03 m up
        down_d = 0.02
        left_d = 0.03
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
        x_1 = p.position.x
        n_t = int(left_d / self.delta_position)
        for _ in range(n_t):
            p.position.x += self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.x - x_1) >= left_d:
                break
        z_1 = p.position.z
        n_t = int(up_d / self.delta_position)
        for _ in range(n_t):
            p.position.z += self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_1) >= up_d:
                break

        plan = self.plan_and_show(waypoints, False, False, 'tr4')

        self.moveit_group.execute(plan, wait=True)
        dt = plan.joint_trajectory.points[-1].time_from_start.secs + plan.joint_trajectory.points[-1].time_from_start.nsecs / 1e9
        rospy.loginfo("Time spent: "+str(dt)+" secs")

        return TrajectoryFourResponse()

    def trajectory_round_valid(self, req):
        rospy.loginfo("Executing validation trajectory for round eef...")
        # 0.02 m down, 0.03 m left, 0.03 m up
        waypoints = []
        p = self.moveit_group.get_current_pose().pose
        forth_d = 0.025
        y_0 = p.position.y
        n_t = int(forth_d / self.delta_position)
        for _ in range(n_t):
            p.position.y += self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.y - y_0) >= forth_d:
                break

        down_d = 0.025
        z_0 = p.position.z
        n_t = int(down_d / self.delta_position)
        for _ in range(n_t):
            p.position.z -= self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_0) >= down_d:
                break

        up_d = 0.025
        z_1 = p.position.z
        n_t = int(up_d / self.delta_position)
        for _ in range(n_t):
            p.position.z += self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_1) >= up_d:
                break

        back_d = 0.025
        y_1 = p.position.y
        n_t = int(back_d / self.delta_position)
        for _ in range(n_t):
            p.position.y -= self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.y - y_1) >= back_d:
                break

        down_d_2 = 0.025
        z_2 = p.position.z
        n_t = int(down_d_2 / self.delta_position)
        for _ in range(n_t):
            p.position.z -= self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_2) >= down_d_2:
                break

        up_d_2 = 0.025
        z_3 = p.position.z
        n_t = int(up_d_2 / self.delta_position)
        for _ in range(n_t):
            p.position.z += self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_3) >= up_d_2:
                break

        back_d = 0.025
        y_2 = p.position.y
        n_t = int(back_d / self.delta_position)
        for _ in range(n_t):
            p.position.y -= self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.y - y_2) >= back_d:
                break

        down_d_3 = 0.025
        z_4 = p.position.z
        n_t = int(down_d_3 / self.delta_position)
        for _ in range(n_t):
            p.position.z -= self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_4) >= down_d_3:
                break

        up_d_3 = 0.025
        z_5 = p.position.z
        n_t = int(up_d_3 / self.delta_position)
        for _ in range(n_t):
            p.position.z += self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_5) >= up_d_3:
                break

        plan = self.plan_and_show(waypoints, False, False, 'tr_round_valid')

        self.moveit_group.execute(plan, wait=True)
        dt = plan.joint_trajectory.points[-1].time_from_start.secs + plan.joint_trajectory.points[-1].time_from_start.nsecs / 1e9
        rospy.loginfo("Time spent: "+str(dt)+" secs")

        return TrajectoryRoundValidResponse()

    def trajectory_rec_valid(self, req):
        rospy.loginfo("Executing validation trajectory for rectangle eef...")
        # 0.025 m down, 0.025 m up, 0.025 m left, 0.025 m down, 0.025 m up, 0.045 m right, 0.025 m down, 0.025 m up
        waypoints = []
        p = self.moveit_group.get_current_pose().pose
        down_d = 0.025
        z_0 = p.position.z
        n_t = int(down_d / self.delta_position)
        for _ in range(n_t):
            p.position.z -= self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_0) >= down_d:
                break

        current_euler = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_euler('xyz', degrees=True)
        c_0 = current_euler[2]
        n_t = int(180 / self.delta_angle)
        for _ in range(n_t):
            current_euler[2] -= self.delta_angle
            quat_xyzw = Rotation.from_euler('xyz', current_euler.copy(), degrees=True).as_quat()
            p.orientation.x = quat_xyzw[0]
            p.orientation.y = quat_xyzw[1]
            p.orientation.z = quat_xyzw[2]
            p.orientation.w = quat_xyzw[3]
            waypoints.append(copy.deepcopy(p))
            if np.abs(current_euler[2] - c_0) >= 180:
                break

        up_d = 0.025
        z_0 = p.position.z
        n_t = int(up_d / self.delta_position)
        for _ in range(n_t):
            p.position.z += self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_0) >= up_d:
                break

        plan = self.plan_and_show(waypoints, True, True, 'tr_rec_valid')

        self.moveit_group.execute(plan, wait=True)
        dt = plan.joint_trajectory.points[-1].time_from_start.secs + plan.joint_trajectory.points[-1].time_from_start.nsecs / 1e9
        rospy.loginfo("Time spent: "+str(dt)+" secs")

        return TrajectoryRecValidResponse()

    def trajectory_cyldr_valid(self, req):
        rospy.loginfo("Executing validation trajectory for cylinder eef...")
        # 0.025 m down, 0.025 m left, 0.025 m up, 0.025 m right, 0.025 m down, 0.025 m right, 0.025 m up
        waypoints = []
        p = self.moveit_group.get_current_pose().pose
        down_d = 0.025
        z_0 = p.position.z
        n_t = int(down_d / self.delta_position)
        for _ in range(n_t):
            p.position.z -= self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_0) >= down_d:
                break

        left_d = 0.025
        x_0 = p.position.x
        n_t = int(left_d / self.delta_position)
        for _ in range(n_t):
            p.position.x += self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.x - x_0) >= left_d:
                break

        up_d = 0.025
        z_1 = p.position.z
        n_t = int(up_d / self.delta_position)
        for _ in range(n_t):
            p.position.z += self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_1) >= up_d:
                break

        right_d = 0.025
        x_1 = p.position.x
        n_t = int(right_d / self.delta_position)
        for _ in range(n_t):
            p.position.x -= self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.x - x_1) >= right_d:
                break

        down_d_2 = 0.025
        z_2 = p.position.z
        n_t = int(down_d_2 / self.delta_position)
        for _ in range(n_t):
            p.position.z -= self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_2) >= down_d_2:
                break

        right_d_2 = 0.025
        x_2 = p.position.x
        n_t = int(right_d_2 / self.delta_position)
        for _ in range(n_t):
            p.position.x -= self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.x - x_2) >= right_d_2:
                break

        up_d_2 = 0.025
        z_3 = p.position.z
        n_t = int(up_d_2 / self.delta_position)
        for _ in range(n_t):
            p.position.z += self.delta_position
            waypoints.append(copy.deepcopy(p))
            if np.abs(p.position.z - z_3) >= up_d_2:
                break

        plan = self.plan_and_show(waypoints, False, False, 'tr_cyldr_valid')

        self.moveit_group.execute(plan, wait=True)
        dt = plan.joint_trajectory.points[-1].time_from_start.secs + plan.joint_trajectory.points[-1].time_from_start.nsecs / 1e9
        rospy.loginfo("Time spent: "+str(dt)+" secs")

        return TrajectoryCyldrValidResponse()

    def trajectory_spoon(self, rep):
        self.publish_pose(minglun_demo_init_pose)
        rospy.sleep(2)
        p_in = copy.deepcopy(minglun_demo_init_pose)
        p_in.pose.position.z -= 0.09
        self.publish_pose(p_in)
        rospy.sleep(2)
        for n in range(3):
            n_str = str(n+1)
            delta_p = np.load(os.path.join(os.path.dirname(__file__), '..', 'src', 'minglun_demo_trajectories', 'delta_p_'+n_str+'.npy'))
            delta_rot = np.load(os.path.join(os.path.dirname(__file__), '..', 'src', 'minglun_demo_trajectories', 'delta_rot_'+n_str+'.npy'))
            p = self.moveit_group.get_current_pose().pose
            current_euler = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_euler('xyz', degrees=True)
            waypoints = []
            for i in range(delta_p.shape[0]):
                p.position.x += delta_p[i, 0]
                p.position.y += delta_p[i, 1]
                p.position.z += delta_p[i, 2]
                current_euler[1] -= delta_rot[i, 0]
                new_quat_xyzw = Rotation.from_euler('xyz', current_euler.copy(), degrees=True).as_quat()
                p.orientation.x = new_quat_xyzw[0]
                p.orientation.y = new_quat_xyzw[1]
                p.orientation.z = new_quat_xyzw[2]
                p.orientation.w = new_quat_xyzw[3]

                waypoints.append(copy.deepcopy(p))

            plan = self.plan_and_show(waypoints, False, False, 'tr_spoon')
            self.moveit_group.execute(plan, wait=True)
            dt = plan.joint_trajectory.points[-1].time_from_start.secs + plan.joint_trajectory.points[-1].time_from_start.nsecs / 1e9
            rospy.loginfo("Time spent: "+str(dt)+" secs")
            rospy.sleep(2)

        return TrajectorySpoonResponse()

    def reset(self, req):
        self.publish_pose(waiting_pose)
        return ResetResponse()

    def move_target(self, req):
        p = copy.deepcopy(waiting_pose)
        p.pose.position.x = req.x
        p.pose.position.y = req.y
        p.pose.position.z = req.z + 0.01
        self.publish_pose(p)

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

    def plot_eef_v(self, plan, save=False, tr_name='tr'):
        cartesian_positions = []
        cartesian_velocities = []
        time_frames = []
        time_difference = [0.0]
        for i in range(len(plan.joint_trajectory.points)-1):
            position = plan.joint_trajectory.points[i].positions
            velocity = plan.joint_trajectory.points[i].velocities
            jacobian = self.moveit_group.get_jacobian_matrix(list(position))
            cartesian_position = np.dot(jacobian, np.array(position))
            cartesian_positions.append(cartesian_position)
            cartesian_velocity = np.dot(jacobian, np.array(velocity))
            cartesian_velocities.append(cartesian_velocity)
            time_frames.append(plan.joint_trajectory.points[i].time_from_start.secs + plan.joint_trajectory.points[i].time_from_start.nsecs / 1e9)
            if i > 0:
                time_difference.append(time_frames[-1] - time_frames[-2])
            # print('Timestamp: {}'.format(time_frames[-1]))
            # print('Time difference: {}'.format(time_difference[-1]))
            # print('Cartesian velocity: {}'.format(cartesian_velocity))

        if save:
            script_dir = os.path.dirname(__file__)
            data_dir = os.path.join(script_dir, '..', '..', 'test', 'cartesian_velocities')
            n = 0
            while os.path.exists(os.path.join(data_dir, tr_name+'_eef_v_'+str(n)+'.npy')):
                n += 1
            np.save(os.path.join(data_dir, tr_name+'_eef_v_'+str(n)+'.npy'), np.array(cartesian_velocities))
            np.save(os.path.join(data_dir, tr_name+'_timestamps_'+str(n)+'.npy'), np.array(time_frames))

        legends = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        plt.plot(cartesian_velocities)
        plt.legend(legends)
        plt.xlabel('Horizon')
        plt.ylabel('Velocity')
        plt.title('End-effector velocity')
        plt.show()

        plt.plot(cartesian_positions)
        plt.legend(legends)
        plt.xlabel('Horizon')
        plt.ylabel('Pose')
        plt.title('End-effector pose')
        plt.show()

        plt.plot(time_frames)
        plt.xlabel('Horizon')
        plt.ylabel('Timestamps')
        plt.title('Trajectory timestamps')
        plt.show()

        plt.plot(time_difference)
        plt.xlabel('Horizon')
        plt.ylabel('Time difference')
        plt.title('Trajectory waypoint time difference')
        plt.show()


if __name__ == '__main__':
    controller = Controller()
    rospy.spin()
