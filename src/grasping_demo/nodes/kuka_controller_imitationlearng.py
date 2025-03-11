#!/usr/bin/env python2

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from moveit_commander.conversions import pose_to_list
from constants import Controller, gripper_activation, gripper_reset, gripper_open, gripper_close

import time
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose, PoseArray
import numpy as np

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)


    robot = moveit_commander.RobotCommander()
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name, ns ="/iiwa")
    # move_group.set_planner_id("chomp")
    move_group.set_planner_id("LIN")

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print( "============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("============ Printing robot state")
    print (robot.get_current_state())

    ## END_SUB_TUTORIAL

    # Misc variables
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.target_listener= rospy.Subscriber('/planning_target', Pose, self.go_to_pose_goal)
    self.waypoint_listener= rospy.Subscriber('/waypoint', PoseArray, self.plan_excute_cartesian_path)
    rospy.set_param('/waypoint_done', False)
    rospy.set_param('/waypoint_plan', True)

    #self.controller = Controller()
    #self.controller.init_robot()
    self.add_box()

    print('spining')

  def set_current_targe(self, msg):
    self.current_target = msg

  def gripper_open(self):
    self.controller.publish_grip_cmd(gripper_open)

  def gripper_close(self):
    self.controller.publish_grip_cmd(gripper_close)

  def gripper_reset(self):
    self.controller.publish_grip_cmd(gripper_reset)

  def gripper_activation(self):
    self.controller.publish_grip_cmd(gripper_activation)

  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0 # slightly above the end effector
    box_pose.pose.position.y = 0.35 # slightly above the end effector
    box_pose.pose.position.z = 1 # slightly above the end effector
    box_name = "box1"
    self.scene.add_box(box_name, box_pose, size=(3, 0.1, 2))
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.75
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0.8
    box_name2 = "box2"
    self.scene.add_box(box_name2, box_pose, size=(0.1, 2, 0.1))
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = -0.01
    box_name3 = "box3"
    self.scene.add_box(box_name3, box_pose, size=(2, 2, 0.01))
    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    # self.box_name=box_name
    return



  def plan_excute_cartesian_path(self,data, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    rospy.loginfo("Received PoseArray with %d poses", len(data.poses))
    waypoints = []
    # current_pose = self.move_group.get_current_pose().pose
    # waypoints.append(current_pose)
    for i, pose in enumerate(data.poses):
        rospy.loginfo("Pose %d: Position (x: %.2f, y: %.2f, z: %.2f) Orientation (x: %.2f, y: %.2f, z: %.2f, w: %.2f)",
                      i,
                      pose.position.x, pose.position.y, pose.position.z,
                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        waypoints.append(pose)

    move_group = self.move_group
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    if fraction < 0.9:
        rospy.logwarn("No Cartesian path found")
        rospy.set_param('/waypoint_plan', False)

        return
    else:
        rospy.loginfo("Plan found with %f success", fraction)
        self.display_trajectory(plan)
        try:
            success = move_group.execute(plan, wait=True)

            if success:
                rospy.loginfo("Executed plan")
                rospy.set_param('/waypoint_done', True)
                rospy.set_param('/waypoint_plan', True)
                time.sleep(0.5)
            else:
                rospy.set_param('/waypoint_plan', False)


        except Exception as e:
            rospy.logerr(e)
            # rospy.set_param('/waypoint_done', True)
            rospy.set_param('/waypoint_plan', False)
            return









  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    print(scene.get_attached_objects(["box"]))

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Received
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def go_to_pose_goal(self, msg):
    print('goal',msg)
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    # pose_goal = geometry_msgs.msg.Pose()
    # current_pose = self.move_group.get_current_pose().pose
    # print('current:',current_pose)
    #
    # pose_goal = current_pose
    #
    # pose_goal.position.z = 1.2



    move_group.set_pose_target(msg)
    move_group.set_planning_time(10)  # Set planning time to 10 seconds
    # move_group.set_planner_id("RRTConnectkConfigDefault")
    # Increase the number of planning attempts
    move_group.set_num_planning_attempts(2)

    # Set goal tolerance
    move_group.set_goal_tolerance(0.01)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # done = False
    # plan = True
    # before = time.time()
    # while not done:
    #     current_pose = self.move_group.get_current_pose().pose
    #     done = all_close(msg, current_pose, 0.01)
    #     print(msg)
    #     print(current_pose)
    #     if time.time() - before > 30 :
    #         plan = False
    #         break


    if not plan:
        print("[ERROR] Planning failed. No motion plan found.")
        return False

    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(msg, current_pose, 0.01)


standby_orientation = [0.675, 0.7373, 0, 0.0000]

launch_point = (0, 0.5, 0.5)
grapsing_orientation = [0.0165891788968, 0.999862372875, 2.35129214586e-05, -9.84286785712e-05]
placing_orientation = [0.994562029839 ,0.104146161736 ,5.14644554126e-05 ,5.99199387464e-06]



class KukaArmGrasper(object):
    def __init__(self):
        self.kuka=MoveGroupPythonIntefaceTutorial()
        #rospy.init_node('tag_name_listener')

        self.tag_position = {}
        self.tag_listener = [rospy.Subscriber('/iiwa/tag_{}'.format(i), Point, callback=self.regist_tag(i)) for i in range(7)]

        #self.kuka.publish_grip_cmd(gripper_reset)
        #self.kuka.publish_grip_cmd(gripper_close)
        #self.kuka.publish_grip_cmd(gripper_open)
        #self.grasper_status = 'open'

        time.sleep(2)
        for i in range(10):
            print( 'waiting for grasper initialization: ',i,' seconds' )
            # time.sleep(1)

    def regist_tag(self, index):
        def closure(msg):
            msg.z = msg.z - 0.09
            self.tag_position[str(index)]=msg # Point.(x,y,z)
        return closure

    def refresh(self):
        self.tag_position = {}
        time.sleep(5)

    def __call__(self, tag_idx):
        tag_idx = str(tag_idx)
        print(self.tag_position)
        self.grasping(tag_idx)
        self.zero_position()

    def grasping(self, tag_idx):
        hand_length= 0.25
        print('moving',{tag_idx})
        ans = self.tag_position[str(tag_idx)]# point.(x,y,z)
        target_pose = np.array([ans.x, ans.y, ans.z])
        if tag_idx == '4':
            target_impulse_pose = target_pose + np.array( [0,0,hand_length + 0.05] )
            target_grasping_pose = target_pose + np.array( [0,0,hand_length - 0.015] )
        else:
            target_impulse_pose = target_pose + np.array( [0,0,hand_length + 0.05] )
            target_grasping_pose = target_pose + np.array( [0,0,hand_length - 0.025 ] )

        self.arm_move(target_impulse_pose, standby_orientation)
        self.arm_move(target_impulse_pose, grapsing_orientation)
        self.arm_move(target_grasping_pose, grapsing_orientation)
        input('enter to confirm')
        #self.grapser_close()
        print('performing a grasping close action')
        self.arm_move(target_impulse_pose + np.array([0, 0, 0.2]), grapsing_orientation)
        self.arm_move(target_impulse_pose + np.array([0, 0, 0.2]), standby_orientation)
        self.arm_move(launch_point, standby_orientation)
        print('performing a place action')
        input('enter to confirm')

        # self.arm_move(launch_point[-1], placing_orientation)
        self.arm_move(np.array([0.5, 0, 0.5]), standby_orientation)
        self.arm_move(np.array([0.5, 0, 0.5]) , placing_orientation)
        self.arm_move(np.array([0.5, 0, target_grasping_pose[-1]+ 0.025 ]), placing_orientation)
        # return
        self.arm_move(np.array([0.5, 0, 0.5]), placing_orientation)
        self.arm_move(launch_point, placing_orientation)
        self.arm_move(launch_point, standby_orientation)

        input('enter to confirm')
        #self.grasper_open()
        print('performing a grasping open action')
        print('done, moving to home')


    def arm_move(self, pose, orientation):
        print('moving to', pose, orientation)
        msg = Pose()
        msg.position.x = pose[0]
        msg.position.y = pose[1]
        msg.position.z = pose[2]
        msg.orientation.x = orientation[0]
        msg.orientation.y = orientation[1]
        msg.orientation.z = orientation[2]
        msg.orientation.w = orientation[3]
        self.kuka.go_to_pose_goal(msg)
        time.sleep(0.05)


    def grapser_close(self):
        print('closing grapser')
        self.grasper_status = 'close'
        self.kuka.gripper_close()
        time.sleep(1)

    def grasper_open(self):
        print('opening grapser')
        self.grasper_status = 'close'
        self.kuka.gripper_open()
        time.sleep(1)

    def zero_position(self):
        zero_position = self.grasping_position['zero_pose']
        self.arm_move(zero_position[0]['position'], zero_position[0]['orientation'])



def main():
  try:

    tutorial = MoveGroupPythonIntefaceTutorial()
    rospy.spin()

    time.sleep(2)
    #input('enter')
    tutorial('1')

    # tutorial.go_to_pose_goal()
    # raw_input("Press Enter to continue...")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
if __name__ == '__main__':
  main()
