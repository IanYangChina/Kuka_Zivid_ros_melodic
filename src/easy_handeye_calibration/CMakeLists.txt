cmake_minimum_required(VERSION 3.0.2)
project(easy_handeye_calibration)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
    nodes/calibration_cmd_prompt.py
    nodes/sample_capture_2d.py
    nodes/sample_pcd.py
    nodes/extrinsic_fine_tuning_prompt.py
    nodes/aruco_marker.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    launch/my_calibrate.launch
    launch/tracking.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
