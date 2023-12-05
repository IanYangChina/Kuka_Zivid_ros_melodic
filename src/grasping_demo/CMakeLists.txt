cmake_minimum_required(VERSION 3.0.2)
project(grasping_demo)

find_package(catkin REQUIRED message_generation std_msgs)

catkin_python_setup()

add_service_files(FILES
    Sample.srv
    TargetPose.srv
    Reset.srv
    MoveDistance.srv
    TrajectoryOne.srv
    TrajectoryTwo.srv
    TrajectoryThree.srv
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

catkin_install_python(PROGRAMS
    nodes/capture_helper.py
    nodes/pcd_processing.py
    nodes/kuka_controller.py
    nodes/command_prompt.py
    nodes/keyboard_event_node.py
    nodes/kuka_pcd_sampler.py
    nodes/send_pose_cmd.py
    nodes/kuka_state_listener.py
    nodes/kuka_controller_moveit.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    launch/grasping_pcd_processing.launch
    launch/grasping_controller.launch
    launch/kuka_moveit_controller.launch
    launch/kuka_pcd_sampler.launch
    launch/camera.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
