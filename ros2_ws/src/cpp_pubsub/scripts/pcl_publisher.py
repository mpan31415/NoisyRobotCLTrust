#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from numpy import asarray
import open3d as o3d

from pcl_msgs.msg import Vertices



class PclPublisher(Node):

    ##############################################################################
    def __init__(self):

        super().__init__('traj_recorder')

        # tcp position subscriber
        self.pcl_sub = self.create_subscription(PointCloud2, 'points2', self.pcl_callback, 10)
        self.pcl_sub  # prevent unused variable warning



    ##############################################################################
    def pcl_callback(self, msg):
        
        pcl = msg.fields
        
        print(pcl[0])




        # print("Load a ply point cloud, print it, and render it")
        # ply_point_cloud = o3d.data.PLYPointCloud()
        # pcd = o3d.io.read_point_cloud(msg.fields)
        # print(pcd)
        # print(asarray(pcd.points))   # this was np.asarray()

        # o3d.visualization.draw_geometries([pcd],
        #                                 zoom=0.3412,
        #                                 front=[0.4257, -0.2125, -0.8795],
        #                                 lookat=[2.6172, 2.0475, 1.532],
        #                                 up=[-0.0694, -0.9768, 0.2024])
        



##############################################################################
def main(args=None):

    rclpy.init(args=args)

    michael = PclPublisher()

    rclpy.spin(michael)

    michael.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()