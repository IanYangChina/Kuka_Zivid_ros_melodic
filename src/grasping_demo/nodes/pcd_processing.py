#!/usr/bin/env python2

import rospy
import ros_numpy
import open3d as o3d
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from grasping_demo.pcd_registration import get_target_grasp_pose


class PcdProcessing:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("pcd_processing_node", anonymous=True)
        rospy.loginfo("Starting pcd_processing_node")
        rospy.Subscriber("/zivid_camera/points/xyzrgba", PointCloud2, self.on_points, queue_size=1, buff_size=1)
        self.target_pose_pub = rospy.Publisher('TargetGraspPose', PoseStamped, queue_size=1)

    def on_points(self, data):
        rospy.loginfo("PointCloud received")
        cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        points = ros_numpy.point_cloud2.get_xyz_points(cloud_array, remove_nans=True)
        pcd_raw = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(points))

        done = False
        while not done:
            pose = get_target_grasp_pose(pcd_raw)
            ans = raw_input("[ USER INPUT] Is the generated grasping pose satisfactory? [y/n]")
            if ans == 'y':
                rospy.sleep(1)
                self.target_pose_pub.publish(pose)
                rospy.loginfo("Publish the generated grasp pose to be executed by the robot...")
                done = True
            else:
                ans = raw_input('[ USER INPUT] Would you like to rerun the registration algorithm? [y/n]')
                if ans == 'y':
                    continue
                else:
                    rospy.loginfo("Please try another object pose for grasping, and recall the program")
                    done = True


if __name__ == '__main__':
    pcd_processing = PcdProcessing()
    rospy.spin()
