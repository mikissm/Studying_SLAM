"""Run after sourcing Jazzy; synthetic ROS stream, no hardware required."""
import os
import tempfile
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from resume_mapper import ResumeMapper
from test_geometry import write


def main():
    # Isolate test services/TF from any running robot.
    os.environ["ROS_DOMAIN_ID"]="178"
    with tempfile.TemporaryDirectory() as tmp:
        src=Path(tmp)/'seed.pcd';out=Path(tmp)/'extended.pcd'
        rng=np.random.default_rng(31);scan=rng.uniform(-3,3,(1200,3)).astype(np.float32);write(src,scan)
        rclpy.init(args=['--ros-args','-p',f'map_path:={src}','-p',f'output_path:={out}','-p','cloud_topic:=/resume_test/cloud','-p','odom_topic:=/resume_test/odom'])
        mapper=ResumeMapper();driver=Node('resume_test_driver');ex=SingleThreadedExecutor();ex.add_node(mapper);ex.add_node(driver)
        clouds=driver.create_publisher(PointCloud2,'/resume_test/cloud',10);odoms=driver.create_publisher(Odometry,'/resume_test/odom',10)
        def pump(points,seconds=.8):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                h=Header();h.frame_id='body';h.stamp=driver.get_clock().now().to_msg()
                clouds.publish(point_cloud2.create_cloud_xyz32(h,points))
                odom=Odometry();odom.header.stamp=h.stamp;odom.header.frame_id='odom';odom.child_frame_id='body';odom.pose.pose.orientation.w=1.;odoms.publish(odom)
                ex.spin_once(timeout_sec=.02)
        def call(name):
            client=driver.create_client(Trigger,name)
            assert client.wait_for_service(timeout_sec=2),name
            future=client.call_async(Trigger.Request());ex.spin_until_future_complete(future,timeout_sec=4)
            assert future.done(),name
            response=future.result();assert response.success,response.message
            driver.destroy_client(client);return response
        try:
            pump(scan);assert mapper.latest is not None
            call('/resume_mapping/start');assert mapper.active
            before=len(mapper.store.cells)
            pump(np.vstack((scan,[[5.,5.,0.]])),1.2)
            assert len(mapper.store.cells)>before
            call('/resume_mapping/pause');assert not mapper.active
            call('/resume_mapping/save');assert out.exists()
            print('ROS PASS: synchronized input → ICP → merge → pause → save')
        finally:
            ex.shutdown();mapper.destroy_node();driver.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
