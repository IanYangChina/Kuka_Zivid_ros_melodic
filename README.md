### Calibration, Registration-based automatic grasping with Zivid Camera

##### Robotics and Autonomous Systems Lab
##### Robot Perception And Learning (RoPAL) - Cardiff University

<img src="/CardiffUnivLogo.jpg" width="80"/>

This package is presented as a catkin workspace source. It has the following main dependencies:

#### Hardware
- Kuka iiwa lbr
- Zivid one camera
- Robotiq 2f gripper
- Aruco marker

#### Software
- Ubuntu 16.04
- ROS Kinetic
- Python 2.7
- `python -m pip install -r requirements.txt`

The package has 2 main functions: camera calibration and automatic grasping. They are all implemented
as command line prompts to be easily accessible. The following links point you to the detailed instructions.

1. [Camera calibration via iiwa_stack and easy_handeye](https://github.com/IanYangChina/Zivid_project/wiki/Camera-calibration-via-iiwa_stack-and-easy_handeye)

- Get intrinsics from Zivid official API
- Perform eye on base extrinsic calibration with easy_handeye
- Perform extrinsic fine-tuning with open3d
- Obtain a reference point cloud with a ground truth grasping pose
- (Under development) Fusing multiple point cloud of an object via registration

2. [Automatic grasping via fpfh-based fast global registration and icp refinement](https://github.com/IanYangChina/Zivid_project/wiki/Automatic-grasping-via-fpfh-bsed-fast-global-registration-and-icp-refinement)
