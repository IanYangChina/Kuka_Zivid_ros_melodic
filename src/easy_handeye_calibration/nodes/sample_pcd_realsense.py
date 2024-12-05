#!/usr/bin/env python2

import os
import rospy
import roslaunch
import ros_numpy
import open3d as o3d
from sensor_msgs.msg import PointCloud2

script_path = os.path.dirname(os.path.realpath(__file__))


class Sample:
    def __init__(self):
        rospy.init_node("pcd_sample_node", anonymous=True)
        rospy.loginfo("Starting pcd_sample_node")
        self.processing = False
        self.stop = False
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.on_points, queue_size=2)

    def on_points(self, data):
        rospy.sleep(2)
        if self.processing or self.stop:
            pass
        else:
            self.processing = True
        rospy.loginfo("PointCloud received")
        cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        points = ros_numpy.point_cloud2.get_xyz_points(cloud_array, remove_nans=True)
        pcd_raw = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(points))
        raw_cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        o3d.visualization.draw_geometries([raw_cam_frame, pcd_raw])
        path_to_save_pcd = os.path.join(script_path, '..', 'src', 'pcd_reference_0.ply')
        i = 0
        while True:
            if os.path.exists(path_to_save_pcd):
                i += 1
                path_to_save_pcd = os.path.join(script_path, '..', 'src', 'pcd_reference_'+str(i)+'.ply')
            else:
                break
        o3d.io.write_point_cloud(path_to_save_pcd, pcd_raw)
        rospy.loginfo("Point cloud \'pcd_reference.ply\' has been saved in ../src/")
        ans = raw_input("[USER INPUT] Stop saving point clouds? [y/n]")
        if ans == 'y':
            self.stop = True
        self.processing = False


if __name__ == '__main__':
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_pcd = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(script_path, '..', 'launch', 'tracking_realsense.launch')])
    launch_pcd.start()
    rospy.sleep(5)
    sample = Sample()
    while not sample.stop:
        rospy.sleep(2)

    rospy.sleep(2)
    rospy.loginfo("Exiting program...")
    launch_pcd.shutdown()
    exit()
