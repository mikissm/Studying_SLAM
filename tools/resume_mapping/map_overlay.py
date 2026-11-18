#!/usr/bin/env python3
"""Visualize saved voxels and newly accumulated voxels without mapper restart."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from core import MapStore


def voxel_keys(points,resolution):
    cells=np.ascontiguousarray(np.floor(np.asarray(points)/resolution).astype(np.int64))
    return cells.view(np.dtype((np.void,24))).ravel()


def added_mask(points,base_keys,resolution):
    return ~np.isin(voxel_keys(points,resolution),base_keys)


class MapOverlay(Node):
    def __init__(self):
        super().__init__('resume_map_overlay')
        self.declare_parameter('map_path','/home/ssm/maps/test1/map.pcd')
        self.declare_parameter('resolution',.1)
        self.res=self.get_parameter('resolution').value
        base=MapStore(self.get_parameter('map_path').value,self.res).points()
        self.keys=voxel_keys(base,self.res)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.base_pub=self.create_publisher(PointCloud2,'/resume_mapping/saved',qos)
        self.added_pub=self.create_publisher(PointCloud2,'/resume_mapping/added',qos)
        self.sub=self.create_subscription(PointCloud2,'/resume_mapping/map',self.receive,qos)
        h=Header();h.frame_id='map';h.stamp=self.get_clock().now().to_msg()
        self.base_pub.publish(point_cloud2.create_cloud_xyz32(h,base))
        self.get_logger().info('Saved map: gray; new voxel additions: green')

    def receive(self,msg):
        points=point_cloud2.read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True).reshape(-1,3)
        points=points[added_mask(points,self.keys,self.res)]
        self.added_pub.publish(point_cloud2.create_cloud_xyz32(msg.header,points))


def main():
    rclpy.init();node=MapOverlay()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
